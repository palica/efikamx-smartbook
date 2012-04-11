/*
 * i.MX IPUv3 Graphics driver
 *
 * Copyright (C) 2011 Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <linux/module.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <drm/imx-ipu-v3.h>
#include <asm/fb.h>
#include <drm/sdrm_encon.h>
#include <drm/sdrm.h>

#define DRIVER_DESC		"i.MX IPUv3 Graphics"

struct ipu_resource {
	int ipu_channel_bg;
	int dc_channel;
	int dp_channel;
	int display;
	u32 interface_pix_fmt; /* FIXME: move to platform data */
};

static struct ipu_resource ipu_resources[] = {
	{
		.ipu_channel_bg = 23, /* IPUV3_CHANNEL_MEM_BG_SYNC */
		.dc_channel = 5,
		.dp_channel = IPU_DP_FLOW_SYNC_BG,
		.display = 0,
		.interface_pix_fmt = V4L2_PIX_FMT_RGB24,
	} , {
		.ipu_channel_bg = 28, /* IPUV3_CHANNEL_MEM_DC_SYNC */
		.dc_channel = 1,
		.dp_channel = -1,
		.display = 1,
		.interface_pix_fmt = V4L2_PIX_FMT_RGB565,
	},
};

struct ipu_priv;

struct ipu_crtc {
	struct drm_crtc		base;
	struct sdrm_crtc	*sdrm_crtc;
	int			pipe;
	struct ipu_resource	*ipu_res;
	struct ipuv3_channel	*ipu_ch;
	struct ipu_dc		*dc;
	struct ipu_dp		*dp;
	struct dmfc_channel	*dmfc;
	struct ipu_di		*di;
	int			di_no;
	int			enabled;
	struct ipu_priv		*ipu_priv;
	struct drm_pending_vblank_event *page_flip_event;
	struct drm_framebuffer	*newfb;
	int			irq;
};

struct ipu_framebuffer {
	struct drm_framebuffer	base;
	void			*virt;
	dma_addr_t		phys;
	size_t			len;
};

struct ipu_priv {
	struct ipu_crtc		crtc[2];
	struct drm_encoder_connector *encon[2];
	struct drm_fb_helper	fb_helper;
	struct ipu_framebuffer	ifb;
	int			num_crtcs;
	struct device		*dev;
};

#define to_ipu_crtc(x) container_of(x, struct ipu_crtc, base)

static struct ipu_rgb def_rgb_32 = {
	.red	= { .offset = 16, .length = 8, },
	.green	= { .offset =  8, .length = 8, },
	.blue	= { .offset =  0, .length = 8, },
	.transp = { .offset = 24, .length = 8, },
	.bits_per_pixel = 32,
};

static struct ipu_rgb def_rgb_16 = {
	.red	= { .offset = 11, .length = 5, },
	.green	= { .offset =  5, .length = 6, },
	.blue	= { .offset =  0, .length = 5, },
	.transp = { .offset =  0, .length = 0, },
	.bits_per_pixel = 16,
};

static int calc_vref(struct drm_display_mode *mode)
{
	unsigned long htotal, vtotal;

	htotal = mode->htotal;
	vtotal = mode->vtotal;

	if (!htotal || !vtotal)
		return 60;

	return mode->clock * 1000 / vtotal / htotal;
}

static int calc_bandwidth(struct drm_display_mode *mode, unsigned int vref)
{
	return mode->hdisplay * mode->vdisplay * vref;
}

static void ipu_fb_enable(struct ipu_crtc *ipu_crtc)
{
	if (ipu_crtc->enabled)
		return;

	ipu_di_enable(ipu_crtc->di);
	ipu_dmfc_enable_channel(ipu_crtc->dmfc);
	ipu_idmac_enable_channel(ipu_crtc->ipu_ch);
	ipu_dc_enable_channel(ipu_crtc->dc);
	if (ipu_crtc->dp)
		ipu_dp_enable_channel(ipu_crtc->dp);

	ipu_crtc->enabled = 1;
}

static void ipu_fb_disable(struct ipu_crtc *ipu_crtc)
{
	if (!ipu_crtc->enabled)
		return;

	if (ipu_crtc->dp)
		ipu_dp_disable_channel(ipu_crtc->dp);
	ipu_dc_disable_channel(ipu_crtc->dc);
	ipu_idmac_disable_channel(ipu_crtc->ipu_ch);
	ipu_dmfc_disable_channel(ipu_crtc->dmfc);
	ipu_di_disable(ipu_crtc->di);

	ipu_crtc->enabled = 0;
}

static void ipu_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);
	struct ipu_priv *ipu_priv = ipu_crtc->ipu_priv;

	dev_info(ipu_priv->dev, "%s mode: %d\n", __func__, mode);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		ipu_fb_enable(ipu_crtc);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		ipu_fb_disable(ipu_crtc);
		break;
	}
}


int ipu_di_clk_set_rate(struct ipu_di *di, unsigned long rate);

const char *fourcc_to_str(u32 fourcc)
{
	static char buf[5];

	*(u32 *)buf = fourcc;
	buf[4] = 0;

	return buf;
}

static int ipu_fb_set_par(struct ipu_crtc *ipu_crtc,
		struct drm_display_mode *mode)
{
	struct ipu_priv	*ipu_priv = ipu_crtc->ipu_priv;
	struct drm_framebuffer *fb = ipu_crtc->base.fb;
	struct ipu_soc *ipu = dev_get_drvdata(ipu_priv->dev->parent);
	int ret;
	struct ipu_di_signal_cfg sig_cfg;
	u32 out_pixel_fmt;
	struct ipu_ch_param *cpmem = ipu_get_cpmem(ipu_crtc->ipu_ch);
	struct ipu_rgb *rgb;
	int bpp;

	memset(cpmem, 0, sizeof(*cpmem));

	switch (fb->pixel_format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		rgb = &def_rgb_32;
		bpp = 32;
		break;
	case DRM_FORMAT_RGB565:
		rgb = &def_rgb_16;
		bpp = 16;
		break;
	default:
		dev_err(ipu_priv->dev, "unsupported pixel format %s\n",
				fourcc_to_str(fb->pixel_format));
		return -EINVAL;
	}

	memset(&sig_cfg, 0, sizeof(sig_cfg));
	out_pixel_fmt = ipu_crtc->ipu_res->interface_pix_fmt;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		sig_cfg.interlaced = 1;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		sig_cfg.Hsync_pol = 1;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		sig_cfg.Vsync_pol = 1;

	sig_cfg.enable_pol = 1;
	sig_cfg.clk_pol = 0;
	sig_cfg.width = mode->hdisplay;
	sig_cfg.height = mode->vdisplay;
	sig_cfg.pixel_fmt = out_pixel_fmt;
	sig_cfg.h_start_width = mode->htotal - mode->hsync_end;
	sig_cfg.h_sync_width = mode->hsync_end - mode->hsync_start;
	sig_cfg.h_end_width = mode->hsync_start - mode->hdisplay;

	sig_cfg.v_start_width = mode->vtotal - mode->vsync_end;
	sig_cfg.v_sync_width = mode->vsync_end - mode->vsync_start;
	sig_cfg.v_end_width = mode->vsync_start - mode->vdisplay;

	sig_cfg.v_to_h_sync = 0;

	ipu_di_clk_set_rate(ipu_crtc->di, mode->clock * 1000);

	if (ipu_crtc->dp) {
		ret = ipu_dp_setup_channel(ipu_crtc->dp, IPUV3_COLORSPACE_RGB,
				IPUV3_COLORSPACE_RGB);
		if (ret) {
			dev_err(ipu_priv->dev, "initializing display processor failed with %d\n",
				ret);
			return ret;
		}
		ipu_dp_set_global_alpha(ipu_crtc->dp, 1, 0, 1);
	}

	ret = ipu_dc_init_sync(ipu_crtc->dc, ipu_crtc->di_no, sig_cfg.interlaced,
			out_pixel_fmt, mode->hdisplay);
	if (ret) {
		dev_err(ipu_priv->dev, "initializing display controller failed with %d\n",
				ret);
		return ret;
	}

	ret = ipu_di_init_sync_panel(ipu_crtc->di, &sig_cfg);
	if (ret) {
		dev_err(ipu_priv->dev, "initializing panel failed with %d\n", ret);
		return ret;
	}

	ipu_cpmem_set_resolution(cpmem, mode->hdisplay, mode->vdisplay);
	ipu_cpmem_set_format_rgb(cpmem, rgb);
	ipu_cpmem_set_high_priority(ipu, cpmem);

	ret = ipu_dmfc_init_channel(ipu_crtc->dmfc, mode->hdisplay);
	if (ret) {
		dev_err(ipu_priv->dev, "initializing dmfc channel failed with %d\n",
				ret);
		return ret;
	}

	ret = ipu_dmfc_alloc_bandwidth(ipu_crtc->dmfc,
			calc_bandwidth(mode, calc_vref(mode)), 64);
	if (ret) {
		dev_err(ipu_priv->dev, "allocating dmfc bandwidth failed with %d\n",
				ret);
		return ret;
	}

	return ret;
}

static int ipu_page_flip(struct drm_crtc *crtc,
                         struct drm_framebuffer *fb,
                         struct drm_pending_vblank_event *event)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);
	struct ipu_priv	*ipu_priv = ipu_crtc->ipu_priv;
	int ret;

	if (ipu_crtc->newfb)
		return -EBUSY;

	ret = sdrm_crtc_vblank_get(ipu_crtc->sdrm_crtc);
	if (ret) {
		dev_dbg(ipu_priv->dev, "failed to acquire vblank counter\n");
		list_del(&event->base.link);

		return ret;
	}

	ipu_crtc->newfb = fb;
	ipu_crtc->page_flip_event = event;

	return 0;
}

static const struct drm_crtc_funcs ipu_crtc_funcs = {
	.page_flip = ipu_page_flip,
};

static void ipu_put_resources(struct ipu_crtc *ipu_crtc)
{
	if (!IS_ERR_OR_NULL(ipu_crtc->ipu_ch))
		ipu_idmac_put(ipu_crtc->ipu_ch);
	if (!IS_ERR_OR_NULL(ipu_crtc->dmfc))
		ipu_dmfc_put(ipu_crtc->dmfc);
	if (!IS_ERR_OR_NULL(ipu_crtc->dp))
		ipu_dp_put(ipu_crtc->dp);
	if (!IS_ERR_OR_NULL(ipu_crtc->di))
		ipu_di_put(ipu_crtc->di);
}

static int ipu_get_resources(struct ipu_priv *ipu_priv, struct ipu_crtc *ipu_crtc)
{
	struct ipu_soc *ipu = dev_get_drvdata(ipu_priv->dev->parent);
	struct ipu_resource *res = &ipu_resources[ipu_crtc->pipe];
	int ret;
	ipu_crtc->ipu_res = res;

	ipu_crtc->ipu_ch = ipu_idmac_get(ipu, res->ipu_channel_bg);
	if (IS_ERR_OR_NULL(ipu_crtc->ipu_ch)) {
		ret = PTR_ERR(ipu_crtc->ipu_ch);
		goto err_out;
	}

	ipu_crtc->dc = ipu_dc_get(ipu, res->dc_channel);
	if (IS_ERR(ipu_crtc->dc)) {
		ret = PTR_ERR(ipu_crtc->dc);
		goto err_out;
	}

	ipu_crtc->dmfc = ipu_dmfc_get(ipu, res->ipu_channel_bg);
	if (IS_ERR(ipu_crtc->dmfc)) {
		ret = PTR_ERR(ipu_crtc->dmfc);
		goto err_out;
	}

	if (res->dp_channel >= 0) {
		ipu_crtc->dp = ipu_dp_get(ipu, res->dp_channel);
		if (IS_ERR(ipu_crtc->dp)) {
			ret = PTR_ERR(ipu_crtc->ipu_ch);
			goto err_out;
		}
	}

	ipu_crtc->di = ipu_di_get(ipu, res->display);
	if (IS_ERR(ipu_crtc->di)) {
		ret = PTR_ERR(ipu_crtc->di);
		goto err_out;
	}

	return 0;
err_out:
	ipu_put_resources(ipu_crtc);

	return ret;
}

static int ipu_drm_set_base(struct drm_crtc *crtc, int x, int y)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);
	struct ipu_priv *ipu_priv = ipu_crtc->ipu_priv;
	struct sdrm_buf_entry *entry;
	struct drm_framebuffer *fb = crtc->fb;
	unsigned long phys;

	entry = sdrm_fb_get_buf(fb);
	if (!entry) {
		DRM_LOG_KMS("entry is null.\n");
		return -EFAULT;
	}

	phys = entry->paddr;
	phys += x * (fb->bits_per_pixel >> 3);
	phys += y * fb->pitches[0];

	dev_dbg(ipu_priv->dev, "%s: phys: 0x%lx\n", __func__, phys);
	dev_dbg(ipu_priv->dev, "%s: xy: %dx%d\n", __func__, x, y);

	ipu_cpmem_set_stride(ipu_get_cpmem(ipu_crtc->ipu_ch), fb->pitches[0]);
	ipu_cpmem_set_buffer(ipu_get_cpmem(ipu_crtc->ipu_ch),
			  0, phys);

	return 0;
}

static int ipu_crtc_mode_set(struct drm_crtc *crtc,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode,
			       int x, int y,
			       struct drm_framebuffer *old_fb)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);
	struct ipu_priv *ipu_priv = ipu_crtc->ipu_priv;

	dev_dbg(ipu_priv->dev, "%s: mode->hdisplay: %d\n", __func__, mode->hdisplay);
	dev_dbg(ipu_priv->dev, "%s: mode->vdisplay: %d\n", __func__, mode->vdisplay);

	ipu_fb_set_par(ipu_crtc, mode);
	ipu_drm_set_base(crtc, x, y);

	return 0;
}

static void sdrm_handle_pageflip(struct ipu_crtc *ipu_crtc)
{
	struct drm_pending_vblank_event *e;
	struct timeval now;
	unsigned long flags;
	struct drm_device *drm = ipu_crtc->base.dev;

	spin_lock_irqsave(&drm->event_lock, flags);

	e = ipu_crtc->page_flip_event;
	if (!e) {
		spin_unlock_irqrestore(&drm->event_lock, flags);
		return;
	}

	do_gettimeofday(&now);
	e->event.sequence = 0;
	e->event.tv_sec = now.tv_sec;
	e->event.tv_usec = now.tv_usec;
	ipu_crtc->page_flip_event = NULL;

	sdrm_crtc_vblank_put(ipu_crtc->sdrm_crtc);

	list_add_tail(&e->base.link, &e->base.file_priv->event_list);

	wake_up_interruptible(&e->base.file_priv->event_wait);

	spin_unlock_irqrestore(&drm->event_lock, flags);
}

static irqreturn_t ipu_irq_handler(int irq, void *dev_id)
{
	struct ipu_crtc *ipu_crtc = dev_id;

	sdrm_handle_vblank(ipu_crtc->sdrm_crtc);

	if (ipu_crtc->newfb) {
		ipu_crtc->base.fb = ipu_crtc->newfb;
		ipu_crtc->newfb = NULL;
		ipu_drm_set_base(&ipu_crtc->base, 0, 0);
		sdrm_handle_pageflip(ipu_crtc);
	}

	return IRQ_HANDLED;
}

static bool ipu_crtc_mode_fixup(struct drm_crtc *crtc,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void ipu_crtc_prepare(struct drm_crtc *crtc)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	ipu_fb_disable(ipu_crtc);
}

static void ipu_crtc_commit(struct drm_crtc *crtc)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	ipu_fb_enable(ipu_crtc);
}

static void ipu_crtc_load_lut(struct drm_crtc *crtc)
{
}

static struct drm_crtc_helper_funcs ipu_helper_funcs = {
	.dpms = ipu_crtc_dpms,
	.mode_fixup = ipu_crtc_mode_fixup,
	.mode_set = ipu_crtc_mode_set,
	.prepare = ipu_crtc_prepare,
	.commit = ipu_crtc_commit,
	.load_lut = ipu_crtc_load_lut,
};

static int ipu_enable_vblank(struct drm_crtc *crtc)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	enable_irq(ipu_crtc->irq);

	return 0;
}

static void ipu_disable_vblank(struct drm_crtc *crtc)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	disable_irq(ipu_crtc->irq);
}

struct sdrm_crtc_helper_funcs ipu_sdrm_crtc_helper_funcs = {
	.enable_vblank = ipu_enable_vblank,
	.disable_vblank = ipu_disable_vblank,
};

static int ipu_crtc_init(struct ipu_priv *ipu_priv, struct ipu_crtc *ipu_crtc, int num)
{
	int ret;

	ipu_crtc->pipe = num;
	ipu_crtc->di_no = num;

	ret = ipu_get_resources(ipu_priv, ipu_crtc);
	if (ret)
		return ret;

	ipu_crtc->ipu_priv = ipu_priv;
	ipu_crtc->sdrm_crtc = sdrm_add_crtc(dev_name(ipu_priv->dev),
			&ipu_crtc->base, &ipu_crtc_funcs, &ipu_helper_funcs,
			&ipu_sdrm_crtc_helper_funcs);
	if (!ipu_crtc->sdrm_crtc) {
		ret = -EINVAL;
		goto err_put_resources;
	}

	ipu_crtc->irq = platform_get_irq(to_platform_device(ipu_priv->dev), num);
	ret = devm_request_irq(ipu_priv->dev, ipu_crtc->irq, ipu_irq_handler, 0,
			"imx_drm", ipu_crtc);
	if (ret < 0) {
		dev_err(ipu_priv->dev, "irq request failed with %d.\n", ret);
		goto err_put_resources;
	}

	disable_irq(ipu_crtc->irq);

	return 0;

err_put_resources:
	ipu_put_resources(ipu_crtc);

	return ret;
}

static int __devinit ipu_drm_probe(struct platform_device *pdev)
{
	struct ipu_priv *ipu_priv;
	int ret;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	ipu_priv = devm_kzalloc(&pdev->dev, sizeof(*ipu_priv), GFP_KERNEL);
	if (!ipu_priv)
		return -ENOMEM;

	ipu_priv->dev = &pdev->dev;

	ret = ipu_crtc_init(ipu_priv, &ipu_priv->crtc[0], 0);
	ret = ipu_crtc_init(ipu_priv, &ipu_priv->crtc[1], 1);

	platform_set_drvdata(pdev, ipu_priv);

	ret = sdrm_init_drm(dev_name(&pdev->dev), pdev);
	if (ret) {
		dev_err(&pdev->dev, "initializing sdrm failed with %d\n", ret);
		return ret;
	}

	return 0;
}

static int __devexit ipu_drm_remove(struct platform_device *pdev)
{
	struct ipu_priv *ipu_priv = platform_get_drvdata(pdev);
	int ret;

	ret = sdrm_exit_drm(dev_name(&pdev->dev));
	if (ret)
		return ret;

	sdrm_remove_crtc(ipu_priv->crtc[0].sdrm_crtc);
	sdrm_remove_crtc(ipu_priv->crtc[1].sdrm_crtc);

	ipu_put_resources(&ipu_priv->crtc[0]);
	ipu_put_resources(&ipu_priv->crtc[1]);

	return 0;
}

static struct platform_driver ipu_drm_driver = {
	.driver = {
		.name = "imx-drm",
	},
	.probe = ipu_drm_probe,
	.remove = __devexit_p(ipu_drm_remove),
};

int __init ipu_drm_init(void)
{
	return platform_driver_register(&ipu_drm_driver);
}

void __exit ipu_drm_exit(void)
{
	platform_driver_unregister(&ipu_drm_driver);
}

late_initcall(ipu_drm_init);
module_exit(ipu_drm_exit);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
