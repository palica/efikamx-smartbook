/* sdrm_fb.c
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
#include <drm/drm_crtc_helper.h>
#include <drm/sdrm.h>

#include "sdrm_fb.h"
#include "sdrm_buf.h"
#include "sdrm_gem.h"

#define to_sdrm_fb(x)	container_of(x, struct sdrm_fb, fb)

/*
 * drm ec specific framebuffer structure.
 *
 * @fb: drm framebuffer obejct.
 * @sdrm_gem_obj: drm ec specific gem object containing a gem object.
 * @entry: pointer to ec drm buffer entry object.
 *	- containing only the information to physically continuous memory
 *	region allocated at default framebuffer creation.
 */
struct sdrm_fb {
	struct drm_framebuffer		fb;
	struct sdrm_gem_obj	*sdrm_gem_obj;
	struct sdrm_buf_entry	*entry;
};

static void sdrm_fb_destroy(struct drm_framebuffer *fb)
{
	struct sdrm_fb *sdrm_fb = to_sdrm_fb(fb);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	drm_framebuffer_cleanup(fb);

	/*
	 * default framebuffer has no gem object so
	 * a buffer of the default framebuffer should be released at here.
	 */
	if (!sdrm_fb->sdrm_gem_obj && sdrm_fb->entry)
		sdrm_buf_destroy(fb->dev, sdrm_fb->entry);

	kfree(sdrm_fb);
}

static int sdrm_fb_create_handle(struct drm_framebuffer *fb,
					struct drm_file *file_priv,
					unsigned int *handle)
{
	struct sdrm_fb *sdrm_fb = to_sdrm_fb(fb);

	return drm_gem_handle_create(file_priv,
			&sdrm_fb->sdrm_gem_obj->base, handle);
}

static int sdrm_fb_dirty(struct drm_framebuffer *fb,
				struct drm_file *file_priv, unsigned flags,
				unsigned color, struct drm_clip_rect *clips,
				unsigned num_clips)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* TODO */

	return 0;
}

static struct drm_framebuffer_funcs sdrm_fb_funcs = {
	.destroy	= sdrm_fb_destroy,
	.create_handle	= sdrm_fb_create_handle,
	.dirty		= sdrm_fb_dirty,
};

static struct drm_framebuffer *sdrm_fb_create(struct drm_device *dev,
					      struct drm_file *file_priv,
					      struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct sdrm_fb *sdrm_fb;
	struct drm_framebuffer *fb;
	struct drm_gem_object *obj;
	unsigned int size;
	int ret;
	u32 bpp, depth;

	drm_fb_get_bpp_depth(mode_cmd->pixel_format, &depth, &bpp);

	mode_cmd->pitches[0] = max(mode_cmd->pitches[0],
			mode_cmd->width * (bpp >> 3));

	DRM_LOG_KMS("drm fb create(%dx%d)\n",
			mode_cmd->width, mode_cmd->height);

	sdrm_fb = kzalloc(sizeof(*sdrm_fb), GFP_KERNEL);
	if (!sdrm_fb) {
		DRM_ERROR("failed to allocate drm framebuffer.\n");
		return ERR_PTR(-ENOMEM);
	}

	fb = &sdrm_fb->fb;
	ret = drm_framebuffer_init(dev, fb, &sdrm_fb_funcs);
	if (ret) {
		DRM_ERROR("failed to initialize framebuffer.\n");
		goto err_init;
	}

	DRM_LOG_KMS("create: fb id: %d\n", fb->base.id);

	size = mode_cmd->pitches[0] * mode_cmd->height;

	/*
	 * without file_priv we are called from sdrm_fbdev_create in which
	 * case we only create a framebuffer without a handle.
	 */
	if (!file_priv) {
		struct sdrm_buf_entry *entry;

		entry = sdrm_buf_create(dev, size);
		if (IS_ERR(entry)) {
			ret = PTR_ERR(entry);
			goto err_buffer;
		}

		sdrm_fb->entry = entry;
	} else {
		obj = drm_gem_object_lookup(dev, file_priv, mode_cmd->handles[0]);
		if (!obj) {
			ret = -EINVAL;
			goto err_buffer;
		}

		sdrm_fb->sdrm_gem_obj = to_sdrm_gem_obj(obj);

		drm_gem_object_unreference_unlocked(obj);

		sdrm_fb->entry = sdrm_fb->sdrm_gem_obj->entry;
	}

	drm_helper_mode_fill_fb_struct(fb, mode_cmd);

	return fb;

err_buffer:
	drm_framebuffer_cleanup(fb);

err_init:
	kfree(sdrm_fb);

	return ERR_PTR(ret);
}

struct sdrm_buf_entry *sdrm_fb_get_buf(struct drm_framebuffer *fb)
{
	struct sdrm_fb *sdrm_fb = to_sdrm_fb(fb);
	struct sdrm_buf_entry *entry;

	entry = sdrm_fb->entry;

	return entry;
}
EXPORT_SYMBOL_GPL(sdrm_fb_get_buf);

static struct drm_mode_config_funcs sdrm_mode_config_funcs = {
	.fb_create = sdrm_fb_create,
};

void sdrm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	/*
	 * set max width and height as default value(4096x4096).
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &sdrm_mode_config_funcs;
}
