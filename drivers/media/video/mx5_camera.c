/*
 * V4L2 Driver for i.MXL/i.MXL camera (CSI) host
 *
 * Copyright (C) 2008, Paulius Zaleckas <paulius.zaleckas@teltonika.lt>
 * Copyright (C) 2009, Darius Augulis <augulis.darius@gmail.com>
 *
 * Based on PXA SoC camera driver
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <drm/imx-ipu-v3.h>
#include "../../gpu/drm/imx/ipu-v3/ipu-prv.h"
#include <media/soc_camera.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/soc_mediabus.h>
#include <asm/mach-types.h>
#define DRIVER_NAME "imx-ipuv3-camera"

#define CSI_BUS_FLAGS	(V4L2_MBUS_MASTER | V4L2_MBUS_HSYNC_ACTIVE_HIGH | \
			V4L2_MBUS_VSYNC_ACTIVE_HIGH | V4L2_MBUS_VSYNC_ACTIVE_LOW | \
			V4L2_MBUS_PCLK_SAMPLE_RISING | V4L2_MBUS_PCLK_SAMPLE_FALLING | \
			V4L2_MBUS_DATA_ACTIVE_HIGH | V4L2_MBUS_DATA_ACTIVE_LOW | \
			SOCAM_DATAWIDTH_8)

#define MAX_VIDEO_MEM 16	/* Video memory limit in megabytes */

/* buffer for one video frame */
struct mx5_buffer {
	struct vb2_buffer		vb;
	enum v4l2_mbus_pixelcode	code;
	struct list_head		queue;
	int				init;
};

struct mx5_camera_dev {
	struct soc_camera_host		soc_host;
	struct soc_camera_device	*icd;
	struct mx5_camera_pdata		*pdata;
	struct mx5_buffer		*active;
	struct resource			*res;
	struct clk			*clk;
	struct list_head		capture;

	void __iomem			*base;
	int				dma_chan;
	unsigned int			irq;
	unsigned long			mclk;

	spinlock_t			lock;
	struct vb2_alloc_ctx		*alloc_ctx;
	enum v4l2_field			field;
	int				sequence;
	dma_addr_t			bufs[2];
	struct ipuv3_channel		*ipuch;
	struct ipu_soc			*ipu;
	u32				fourcc;
};

static struct mx5_buffer *to_mx5_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct mx5_buffer, vb);
}

static const struct soc_mbus_pixelfmt mx5_camera_formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_YUV420,
		.name			= "YUV420 planar",
		.bits_per_sample	= 12,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
};

static int ipu_capture_channel = 0;

static irqreturn_t mx5cam_completion_handler(int irq, void *context)
{
	struct mx5_camera_dev *mx5_cam = context;
	struct vb2_buffer *vb;
	struct mx5_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&mx5_cam->lock, flags);

	if (mx5_cam->active) {

		vb = &mx5_cam->active->vb;
		buf = to_mx5_vb(vb);

		if (list_is_last(&buf->queue, &mx5_cam->capture))
			goto out;

		list_del_init(&buf->queue);
		do_gettimeofday(&vb->v4l2_buf.timestamp);
		vb->v4l2_buf.field = mx5_cam->field;
		vb->v4l2_buf.sequence = mx5_cam->sequence++;
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	}

	if (list_empty(&mx5_cam->capture))
		goto out;

	mx5_cam->active = list_first_entry(&mx5_cam->capture,
				   struct mx5_buffer, queue);

	vb = &mx5_cam->active->vb;

	ipu_cpmem_set_buffer(ipu_get_cpmem(mx5_cam->ipuch),
			  0, vb2_dma_contig_plane_dma_addr(vb, 0));
	ipu_idmac_select_buffer(mx5_cam->ipuch, 0);
out:
	spin_unlock_irqrestore(&mx5_cam->lock, flags);

	return IRQ_HANDLED;
}

/*
 *  Videobuf operations
 */
static int mx5_videobuf_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
		unsigned int *count, unsigned int *num_planes,
		unsigned int sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	*num_planes = 1;

	mx5_cam->sequence = 0;
	sizes[0] = bytes_per_line * icd->user_height;
	alloc_ctxs[0] = mx5_cam->alloc_ctx;

	if (!*count)
		*count = 32;

	if (sizes[0] * *count > MAX_VIDEO_MEM * 1024 * 1024)
		*count = MAX_VIDEO_MEM * 1024 * 1024 / sizes[0];

	return 0;
}

static int mx5_videobuf_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	size_t new_size;
	struct mx5_buffer *buf;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	buf = to_mx5_vb(vb);

	new_size = bytes_per_line * icd->user_height;

	if (vb2_plane_size(vb, 0) < new_size) {
		dev_err(icd->parent, "Buffer too small (%lu < %zu)\n",
				vb2_plane_size(vb, 0), new_size);
		return -ENOBUFS;
        }

	vb2_set_plane_payload(vb, 0, new_size);

	return 0;
}

/* Called under spinlock_irqsave(&pcdev->lock, ...) */
static void mx5_videobuf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	struct mx5_buffer *buf = to_mx5_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&mx5_cam->lock, flags);

	list_add_tail(&buf->queue, &mx5_cam->capture);

	spin_unlock_irqrestore(&mx5_cam->lock, flags);
}

static void mx5_videobuf_release(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	struct mx5_buffer *buf = to_mx5_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&mx5_cam->lock, flags);

	if (mx5_cam->active == buf)
		mx5_cam->active = NULL;

	if (buf->init)
		list_del_init(&buf->queue);

	spin_unlock_irqrestore(&mx5_cam->lock, flags);
}

static int mx5_videobuf_init(struct vb2_buffer *vb)
{
	struct mx5_buffer *buf = to_mx5_vb(vb);

	/* This is for locking debugging only */
	INIT_LIST_HEAD(&buf->queue);

	buf->init = 1;

	return 0;
}

static int mx5_videobuf_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	int ret;
	int xres = icd->user_width, yres = icd->user_height;
	struct ipu_ch_param *cpmem = ipu_get_cpmem(mx5_cam->ipuch);
	struct device *dev = icd->dev.parent;
	struct device *dev = icd->parent;
	u32 csipixfmt = V4L2_PIX_FMT_UYVY;

	memset(cpmem, 0, sizeof(*cpmem));

	ret = request_threaded_irq(mx5_cam->irq, NULL, mx5cam_completion_handler, IRQF_ONESHOT,
			"mx5cam", mx5_cam);
	if (ret)
		return ret;

	dev_info(dev, "width: %d height: %d\n", icd->user_width, icd->user_height);

	ipu_cpmem_set_resolution(cpmem, xres, yres);

	switch (mx5_cam->fourcc) {
	case V4L2_PIX_FMT_UYVY:
		dev_info(dev, "IPU_PIX_FMT_UYVY\n");
		ipu_cpmem_set_stride(cpmem, xres * 2);
		ipu_cpmem_set_yuv_interleaved(cpmem, V4L2_PIX_FMT_UYVY);
		break;
	case V4L2_PIX_FMT_YUV420:
		dev_info(dev, "V4L2_PIX_FMT_YUV420\n");
		ipu_cpmem_set_stride(cpmem, xres);
		ipu_cpmem_set_yuv_planar(cpmem, V4L2_PIX_FMT_YUV420, xres, yres);
		break;
	}

	ipu_csi_set_window_size(xres, yres, 0);
	ipu_csi_set_window_pos(0, 0, 0);

	ret = ipu_csi_init_interface(xres, yres, csipixfmt, 0x2400cb08);
	if (ret)
		return ret;

	ipu_idmac_set_double_buffer(mx5_cam->ipuch, 1);
	ipu_idmac_select_buffer(mx5_cam->ipuch, 1);
	ipu_idmac_enable_channel(mx5_cam->ipuch);
	ipu_module_enable(mx5_cam->ipu, IPU_CONF_CSI0_EN);
	ipu_module_enable(mx5_cam->ipu, IPU_CONF_SMFC_EN);

	return 0;
}

static int mx5_videobuf_stop_streaming(struct vb2_queue *vq)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;

	free_irq(mx5_cam->irq, mx5_cam);
	ipu_module_disable(mx5_cam->ipu, IPU_CONF_CSI0_EN);
	ipu_module_disable(mx5_cam->ipu, IPU_CONF_SMFC_EN);
	ipu_idmac_disable_channel(mx5_cam->ipuch);

	return 0;
}

static struct vb2_ops mx5_videobuf_ops = {
	.queue_setup		= mx5_videobuf_setup,
	.buf_prepare		= mx5_videobuf_prepare,
	.buf_queue		= mx5_videobuf_queue,
	.buf_cleanup		= mx5_videobuf_release,
	.buf_init		= mx5_videobuf_init,
	.start_streaming	= mx5_videobuf_start_streaming,
	.stop_streaming		= mx5_videobuf_stop_streaming,
	.wait_prepare		= soc_camera_unlock,
	.wait_finish		= soc_camera_lock,
};

static int mx5_camera_init_videobuf(struct vb2_queue *q,
				     struct soc_camera_device *icd)
{
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = icd;
	q->ops = &mx5_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mx5_buffer);

	return vb2_queue_init(q);
}

static int mx5_camera_get_formats(struct soc_camera_device *icd, unsigned int idx,
				  struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	int formats = 0, ret;
	enum v4l2_mbus_pixelcode code;
	const struct soc_mbus_pixelfmt *fmt;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		/* No more formats */
		return 0;

	fmt = soc_mbus_get_fmtdesc(code);
	if (!fmt) {
		dev_err(icd->parent,
			"Invalid format code #%u: %d\n", idx, code);
		return 0;
	}

	formats++;
	if (xlate) {
		xlate->host_fmt	= &mx5_camera_formats[0];
		xlate->code	= code;
		xlate++;
		dev_dbg(dev, "Providing format %s using code %d\n",
			mx5_camera_formats[0].name, code);
	}

	/* Generic pass-through */
	formats++;
	if (xlate) {
		xlate->host_fmt	= fmt;
		xlate->code	= code;
		dev_dbg(dev, "Providing format %c%c%c%c in pass-through mode\n",
			(fmt->fourcc >> (0*8)) & 0xFF,
			(fmt->fourcc >> (1*8)) & 0xFF,
			(fmt->fourcc >> (2*8)) & 0xFF,
			(fmt->fourcc >> (3*8)) & 0xFF);
		xlate++;
	}

	return formats;
}

static void mx5_camera_activate(struct mx5_camera_dev *mx5_cam)
{
	dev_dbg(mx5_cam->icd->parent, "Activate device\n");
}

static int mx5_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	int ret;

	if (mx5_cam->icd) {
		ret = -EBUSY;
		goto ebusy;
	}

	dev_info(icd->parent, "MX5 Camera driver attached to camera %d\n",
		 icd->devnum);

	mx5_cam->icd = icd;

	mx5_camera_activate(mx5_cam);

	return 0;

ebusy:
	return ret;
}

static void mx5_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *pcdev = ici->priv;

	BUG_ON(icd != pcdev->icd);

	dev_info(icd->parent, "MX5 Camera driver detached from camera %d\n",
		 icd->devnum);

	pcdev->icd = NULL;
}

static int mx5_camera_set_crop(struct soc_camera_device *icd,
			       struct v4l2_crop *a)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

	return v4l2_subdev_call(sd, video, s_crop, a);
}

static int mx5_camera_set_bus_param(struct soc_camera_device *icd)
{
	return 0;
}

static int mx5_camera_set_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mx5_camera_dev *mx5_cam = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(icd->parent, "Format %x not found\n",
			 pix->pixelformat);
		return -EINVAL;
	}

	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	if (mf.code != xlate->code)
		return -EINVAL;

	mx5_cam->fourcc = f->fmt.pix.pixelformat;

	pix->width		= mf.width;
	pix->height		= mf.height;
	pix->field		= mf.field;
	pix->colorspace		= mf.colorspace;
	icd->current_fmt	= xlate;

	return ret;
}

static int mx5_camera_try_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(icd->parent, "Format %x not found\n",
			 pix->pixelformat);
		return -EINVAL;
	}

	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	/* limit to sensor capabilities */
	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width	= mf.width;
	pix->height	= mf.height;
	pix->field	= mf.field;
	pix->colorspace	= mf.colorspace;

	return 0;
}

static int mx5_camera_reqbufs(struct soc_camera_device *icd,
			      struct v4l2_requestbuffers *p)
{
	return 0;
}

static unsigned int mx5_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int mx5_camera_querycap(struct soc_camera_host *ici,
			       struct v4l2_capability *cap)
{
	/* cap->name is set by the friendly caller:-> */
	strlcpy(cap->card, "imx-ipuv3-camera", sizeof(cap->card));
	cap->version = 0;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static struct soc_camera_host_ops mx5_soc_camera_host_ops = {
	.owner		= THIS_MODULE,
	.add		= mx5_camera_add_device,
	.remove		= mx5_camera_remove_device,
	.get_formats	= mx5_camera_get_formats,
	.set_bus_param	= mx5_camera_set_bus_param,
	.set_crop	= mx5_camera_set_crop,
	.set_fmt	= mx5_camera_set_fmt,
	.try_fmt	= mx5_camera_try_fmt,
	.init_videobuf2	= mx5_camera_init_videobuf,
	.reqbufs	= mx5_camera_reqbufs,
	.poll		= mx5_camera_poll,
	.querycap	= mx5_camera_querycap,
};

static u64 camera_mask = DMA_BIT_MASK(32);

static int __devinit mx5_camera_probe(struct platform_device *pdev)
{
	struct ipu_soc *ipu = dev_get_drvdata(pdev->dev.parent);
	struct mx5_camera_dev *mx5_cam;
	int irq, err;

	pdev->dev.dma_mask		= &camera_mask,
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32),

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -EINVAL;

	mx5_cam = kzalloc(sizeof(*mx5_cam), GFP_KERNEL);
	if (!mx5_cam) {
		err = -ENOMEM;
		goto failed_alloc;
	}

	mx5_cam->pdata = pdev->dev.platform_data;
	mx5_cam->irq = irq;
	mx5_cam->ipu = ipu;

	mx5_cam->ipuch = ipu_idmac_get(ipu, ipu_capture_channel);
	if (!mx5_cam->ipuch) {
		err = -EBUSY;
		goto failed_ipu;
	}

	INIT_LIST_HEAD(&mx5_cam->capture);
	spin_lock_init(&mx5_cam->lock);

	mx5_cam->soc_host.drv_name	= DRIVER_NAME;
	mx5_cam->soc_host.ops		= &mx5_soc_camera_host_ops;
	mx5_cam->soc_host.priv		= mx5_cam;
	mx5_cam->soc_host.v4l2_dev.dev	= &pdev->dev;
	mx5_cam->soc_host.nr		= pdev->id;

	mx5_cam->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(mx5_cam->alloc_ctx)) {
		err = PTR_ERR(mx5_cam->alloc_ctx);
		goto failed_vb2;
	}

	err = soc_camera_host_register(&mx5_cam->soc_host);
	if (err)
		goto failed_register;

	_ipu_csi_init(ipu_capture_channel, 0, 15, 0);

	platform_set_drvdata(pdev, mx5_cam);

	dev_info(&pdev->dev, "MX5 Camera driver loaded\n");

	return 0;

failed_register:
	vb2_dma_contig_cleanup_ctx(mx5_cam->alloc_ctx);
failed_vb2:
	ipu_idmac_put(mx5_cam->ipuch);
failed_ipu:
	kfree(mx5_cam);
failed_alloc:
	return err;
}

static int __exit mx5_camera_remove(struct platform_device *pdev)
{
	struct mx5_camera_dev *mx5_cam = platform_get_drvdata(pdev);

	soc_camera_host_unregister(&mx5_cam->soc_host);
	vb2_dma_contig_cleanup_ctx(mx5_cam->alloc_ctx);
	ipu_idmac_put(mx5_cam->ipuch);
	kfree(mx5_cam);

	return 0;
}

static struct platform_driver mx5_camera_driver = {
	.driver 	= {
		.name	= DRIVER_NAME,
	},
	.probe = mx5_camera_probe,
	.remove		= __exit_p(mx5_camera_remove),
};

static int __init mx5_camera_init(void)
{
	return platform_driver_register(&mx5_camera_driver);
}

static void __exit mx5_camera_exit(void)
{
	return platform_driver_unregister(&mx5_camera_driver);
}

module_init(mx5_camera_init);
module_exit(mx5_camera_exit);

MODULE_DESCRIPTION("i.MX51/53 SoC Camera Host driver");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
