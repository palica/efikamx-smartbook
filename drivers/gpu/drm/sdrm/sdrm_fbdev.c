/* sdrm_fbdev.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * VA LINUX SYSTEMS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/sdrm.h>

#include "sdrm_fbdev.h"
#include "sdrm_fb.h"
#include "sdrm_buf.h"

#define MAX_CONNECTOR		4
#define PREFERRED_BPP		16

static struct fb_ops sdrm_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_check_var	= drm_fb_helper_check_var,
	.fb_set_par	= drm_fb_helper_set_par,
	.fb_blank	= drm_fb_helper_blank,
	.fb_pan_display	= drm_fb_helper_pan_display,
	.fb_setcmap	= drm_fb_helper_setcmap,
};

static int sdrm_fbdev_update(struct drm_fb_helper *helper,
				     struct drm_framebuffer *fb,
				     unsigned int fb_width,
				     unsigned int fb_height)
{
	struct fb_info *fbi = helper->fbdev;
	struct drm_device *dev = helper->dev;
	struct sdrm_buf_entry *entry;
	unsigned int size = fb_width * fb_height * (fb->bits_per_pixel >> 3);
	unsigned long offset;

	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(fbi, helper, fb_width, fb_height);

	entry = sdrm_fb_get_buf(fb);
	if (!entry) {
		DRM_LOG_KMS("entry is null.\n");
		return -EFAULT;
	}

	offset = fbi->var.xoffset * (fb->bits_per_pixel >> 3);
	offset += fbi->var.yoffset * fb->pitches[0];

	dev->mode_config.fb_base = entry->paddr;
	fbi->screen_base = entry->vaddr + offset;
	fbi->fix.smem_start = entry->paddr + offset;
	fbi->screen_size = size;
	fbi->fix.smem_len = size;
	fbi->flags |= FBINFO_CAN_FORCE_OUTPUT;

	return 0;
}

static int sdrm_fbdev_create(struct drm_fb_helper *helper,
				    struct drm_fb_helper_surface_size *sizes)
{
	struct drm_device *dev = helper->dev;
	struct fb_info *fbi;
	struct drm_framebuffer *fb;
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct platform_device *pdev = dev->platformdev;
	int ret;

	DRM_DEBUG_KMS("surface width(%d), height(%d) and bpp(%d\n",
			sizes->surface_width, sizes->surface_height,
			sizes->surface_bpp);

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
			sizes->surface_depth);

	mutex_lock(&dev->struct_mutex);

	fbi = framebuffer_alloc(0, &pdev->dev);
	if (!fbi) {
		DRM_ERROR("failed to allocate fb info.\n");
		ret = -ENOMEM;
		goto out;
	}

	fb = dev->mode_config.funcs->fb_create(dev, NULL, &mode_cmd);
	if (IS_ERR(fb)) {
		DRM_ERROR("failed to create drm framebuffer.\n");
		ret = PTR_ERR(fb);
		goto out;
	}

	helper->fb = fb;
	helper->fbdev = fbi;

	fbi->par = helper;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->fbops = &sdrm_fb_ops;

	ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (ret) {
		DRM_ERROR("failed to allocate cmap.\n");
		goto out;
	}

	ret = sdrm_fbdev_update(helper, helper->fb, sizes->fb_width,
			sizes->fb_height);
	if (ret < 0)
		fb_dealloc_cmap(&fbi->cmap);

out:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}

static int sdrm_fbdev_probe(struct drm_fb_helper *helper,
				   struct drm_fb_helper_surface_size *sizes)
{
	int ret = 0;

	BUG_ON(helper->fb);

	ret = sdrm_fbdev_create(helper, sizes);
	if (ret < 0) {
		DRM_ERROR("failed to create fbdev.\n");
		return ret;
	}

	/*
	 * fb_helper expects a value more than 1 if succeed
	 * because register_framebuffer() should be called.
	 */
	return 1;
}

static struct drm_fb_helper_funcs sdrm_fb_helper_funcs = {
	.fb_probe = sdrm_fbdev_probe,
};

struct drm_fb_helper *sdrm_fbdev_init(struct drm_device *dev)
{
	struct drm_fb_helper *helper;
	unsigned int num_crtc;
	int ret;

	helper = kzalloc(sizeof(*helper), GFP_KERNEL);
	if (!helper) {
		DRM_ERROR("failed to allocate drm fbdev.\n");
		return NULL;
	}

	helper->funcs = &sdrm_fb_helper_funcs;

	num_crtc = dev->mode_config.num_crtc;

	ret = drm_fb_helper_init(dev, helper, num_crtc, MAX_CONNECTOR);
	if (ret < 0) {
		DRM_ERROR("failed to initialize drm fb helper.\n");
		goto err_init;
	}

	ret = drm_fb_helper_single_add_all_connectors(helper);
	if (ret < 0) {
		DRM_ERROR("failed to register drm_fb_helper_connector.\n");
		goto err_setup;

	}

	ret = drm_fb_helper_initial_config(helper, PREFERRED_BPP);
	if (ret < 0) {
		DRM_ERROR("failed to set up hw configuration.\n");
		goto err_setup;
	}

	return helper;

err_setup:
	drm_fb_helper_fini(helper);

err_init:
	kfree(helper);

	return NULL;
}

void sdrm_fbdev_fini(struct drm_fb_helper *helper)
{
	/* release linux framebuffer */
	if (helper->fbdev) {
		struct fb_info *info;
		int ret;

		info = helper->fbdev;
		ret = unregister_framebuffer(info);
		if (ret < 0)
			pr_err("unregister_framebuffer failed with %d\n", ret);

		if (info->cmap.len)
			fb_dealloc_cmap(&info->cmap);

		framebuffer_release(info);
	}

	drm_fb_helper_fini(helper);

	kfree(helper);
}
