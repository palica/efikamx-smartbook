/*
 * Implementation of a 1:1 relationship for drm encoders and connectors
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
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/sdrm_encon.h>
#include <drm/sdrm.h>

#define con_to_encon(x) container_of(x, struct drm_encoder_connector, connector)
#define enc_to_encon(x) container_of(x, struct drm_encoder_connector, encoder)

static enum drm_connector_status connector_detect(struct drm_connector *connector,
		bool force)
{
	struct drm_encoder_connector *encon = con_to_encon(connector);

	if (encon->funcs->detect)
		return encon->funcs->detect(encon);

	return connector_status_connected;
}

static int connector_set_property(struct drm_connector *connector,
		struct drm_property *property, uint64_t val)
{
	struct drm_encoder_connector *encon = con_to_encon(connector);

	if (encon->funcs->set_property)
		return encon->funcs->set_property(encon, property, val);

	return -EINVAL;
}

static void connector_destroy(struct drm_connector *connector)
{
	/* do not free here */
}

static int connector_get_modes(struct drm_connector *connector)
{
	struct drm_encoder_connector *encon = con_to_encon(connector);

	if (encon->funcs->get_modes)
		return encon->funcs->get_modes(encon);

	return 0;
}

static int connector_mode_valid(struct drm_connector *connector,
			  struct drm_display_mode *mode)
{
	struct drm_encoder_connector *encon = con_to_encon(connector);

	if (encon->funcs && encon->funcs->mode_valid)
		return encon->funcs->mode_valid(encon, mode);

	return 0;
}

static struct drm_encoder *connector_best_encoder(struct drm_connector *connector)
{
	struct drm_encoder_connector *encon = con_to_encon(connector);

	return &encon->encoder;
}

static void encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs->dpms)
		encon->funcs->dpms(encon, mode);
}

static bool encoder_mode_fixup(struct drm_encoder *encoder,
			   struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs->mode_fixup)
		return encon->funcs->mode_fixup(encon, mode, adjusted_mode);

	return true;
}

static void encoder_prepare(struct drm_encoder *encoder)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs->prepare)
		encon->funcs->prepare(encon);
}

static void encoder_commit(struct drm_encoder *encoder)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs->commit)
		encon->funcs->commit(encon);
}

static void encoder_mode_set(struct drm_encoder *encoder,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs->mode_set)
		encon->funcs->mode_set(encon, mode, adjusted_mode);
}

static void encoder_disable(struct drm_encoder *encoder)
{
}

static void encoder_destroy(struct drm_encoder *encoder)
{
	/* do not free here */
}

struct drm_connector_funcs connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = connector_detect,
	.set_property = connector_set_property,
	.destroy = connector_destroy,
};

struct drm_connector_helper_funcs connector_helper_funcs = {
	.get_modes = connector_get_modes,
	.mode_valid = connector_mode_valid,
	.best_encoder = connector_best_encoder,
};

static struct drm_encoder_funcs encoder_funcs = {
	.destroy = encoder_destroy,
};

static struct drm_encoder_helper_funcs encoder_helper_funcs = {
	.dpms = encoder_dpms,
	.mode_fixup = encoder_mode_fixup,
	.prepare = encoder_prepare,
	.commit = encoder_commit,
	.mode_set = encoder_mode_set,
	.disable = encoder_disable,
};

int drm_encoder_connector_register(const char *drm_name, struct drm_encoder_connector *encon)
{
	drm_mode_connector_attach_encoder(&encon->connector, &encon->encoder);

	encon->connector.funcs = &connector_funcs;
	encon->encoder.funcs = &encoder_funcs;
	drm_encoder_helper_add(&encon->encoder, &encoder_helper_funcs);
	encon->sdrm_encoder = sdrm_add_encoder(drm_name, &encon->encoder,
			encon->owner);
	if (!encon->sdrm_encoder) {
		pr_err("%s: adding encoder failed\n", __func__);
		return -EINVAL;
	}

	drm_connector_helper_add(&encon->connector, &connector_helper_funcs);

	encon->sdrm_connector = sdrm_add_connector(drm_name, &encon->connector,
			encon->owner);
	if (!encon->sdrm_connector) {
		pr_err("%s: adding connector failed\n", __func__);
		return -EINVAL;
	}

	encon->connector.encoder = &encon->encoder;

	return 0;
}
EXPORT_SYMBOL_GPL(drm_encoder_connector_register);

void drm_encoder_connector_unregister(struct drm_encoder_connector *c)
{
	struct drm_connector *connector = &c->connector;
	struct drm_encoder *encoder = &c->encoder;

	drm_sysfs_connector_remove(connector);
	drm_mode_connector_detach_encoder(connector, encoder);
	drm_encoder_cleanup(encoder);
	drm_connector_cleanup(connector);
}
EXPORT_SYMBOL_GPL(drm_encoder_connector_unregister);

MODULE_DESCRIPTION("drm encoder/connector driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
