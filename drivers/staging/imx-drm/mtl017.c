/*
 * i.MX drm driver - EfikaMX mtl017 driver
 *
 * Copyright (C) 2012 Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <linux/videodev2.h>
#include <linux/of_i2c.h>

#include "imx-drm.h"

#define con_to_mtl017(x) container_of(x, struct mtl017_display, connector)
#define enc_to_mtl017(x) container_of(x, struct mtl017_display, encoder)

struct mtl017_display {
	struct drm_connector connector;
	struct imx_drm_connector *imx_drm_connector;
	struct drm_encoder encoder;
	struct imx_drm_encoder *imx_drm_encoder;
	struct device *dev;
	void *edid;
	int edid_len;
	u32 interface_pix_fmt;
	int mode_valid;
	struct drm_display_mode mode;
	struct i2c_adapter *i2c_adapter;
	struct i2c_client *client;
	const u8 *regs;
};

/*
 * Unfortunately we know nothing about the mtl017 except the following
 * register tables which are derived from the Efika kernel. The displays
 * provide EDID data, but this does not work with the mtl017 or at least
 * not with the below register settings, so we have to provide hardcoded
 * modelines and use the EDID data only to match against the vendor/display.
 */
static u8 mtl017_44_54_tbl[] = {
	/* 44M to 53.9M */
	0x00,0x20,0xAF,0x59,0x2B,0xDE,0x51,0x00,
	0x00,0x04,0x17,0x00,0x58,0x02,0x00,0x00,
	0x00,0x3B,0x01,0x08,0x00,0x1E,0x01,0x05,
	0x00,0x01,0x72,0x05,0x32,0x00,0x00,0x04,
	0x00,0x00,0x20,0xA8,0x02,0x12,0x00,0x58,
	0x02,0x00,0x00,0x02,0x00,0x00,0x02,0x00,
	0x00,0x02,0x10,0x01,0x68,0x03,0xC2,0x01,
	0x4A,0x03,0x46,0x00,0xF1,0x01,0x5C,0x04,
	0x08,0x00,0x10,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x3A,
	0x18,0x4B,0x29,0x5C,0xDE,0xF6,0xE0,0x1C,
	0x03,0xFC,0xE3,0x1F,0xF3,0x75,0x26,0x45,
	0x4A,0x91,0x8A,0xFF,0x3F,0x83,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x20,0x4E,0x48,
	0x00,0x01,0x10,0x01,0x00,0x00,0x10,0x04,
	0x02,0x1F,0x00,0x00,0x00,0x0A,0x00,0x00,
	0x32,0x00,0x00,0x04,0x12,0x00,0x58,0x02,
	0x02,0x7C,0x04,0x98,0x02,0x11,0x78,0x18,
	0x30,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
};

static u8 mtl017_auo_hires_tbl[] = {
	0x00,0x20,0xAF,0x59,0x2B,0xDE,0x3D,0x00,
	0x00,0x05,0x0C,0x00,0xD0,0x02,0x00,0x00,
	0x00,0x05,0x00,0x02,0x00,0x02,0x00,0x0A,
	0x00,0x01,0x70,0x05,0x3D,0x00,0x00,0x05,
	0x00,0x00,0x20,0xF0,0x02,0x0C,0x00,0xD0,
	0x02,0x00,0x00,0x02,0x00,0x00,0x02,0x00,
	0x00,0x02,0x10,0x01,0x68,0x03,0xC2,0x01,
	0x4A,0x03,0x46,0x00,0xF1,0x01,0x5C,0x04,
	0x08,0x00,0x10,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x3A,
	0x18,0x4B,0x29,0x5C,0xDE,0xF6,0xE0,0x1C,
	0x03,0xFC,0xE3,0x1F,0xF3,0x75,0x26,0x45,
	0x4A,0x91,0x8A,0xFF,0x3F,0x83,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x20,0x4E,0x48,
	0x00,0x00,0x06,0x01,0x02,0x00,0x12,0x04,
	0x02,0x1F,0x00,0x00,0x00,0x0A,0x00,0x00,
	0x3D,0x00,0x00,0x05,0x0C,0x00,0xD0,0x02,
	0x02,0x70,0x05,0xE0,0x02,0x11,0x78,0x18,
	0x30,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
};

#define REGMAP_LENGTH (sizeof(mtl017_44_54_tbl) / sizeof(u8))
#define BLOCK_TX_SIZE 32

struct lcd_panel_info {
	char *manufacture;
	char *product_name;
	struct drm_display_mode mode;
	u8  *regs;
};

static struct lcd_panel_info pinfo[] = {
	{
		.manufacture = "AUO",
		.product_name = "B101AW02 V0",
		.mode = {
			.hdisplay = 1024,
			.hsync_start = 1064,
			.hsync_end = 1068,
			.htotal = 1148,
			.vdisplay = 600,
			.vsync_start = 621,
			.vsync_end = 625,
			.vtotal = 649,
			.clock = 43860,
		},
		.regs = mtl017_44_54_tbl,
	}, {
		.manufacture = "CPT",
		.product_name = "CLAA101NB03A",
		.mode = {
			.hdisplay = 1024,
			.hsync_start = 1044,
			.hsync_end = 1048,
			.htotal = 1128,
			.vdisplay = 600,
			.vsync_start = 621,
			.vsync_end = 625,
			.vtotal = 645,
			.clock = 61500,
		},
		.regs = mtl017_44_54_tbl,
	}, {
		.manufacture = "AUO",
		.product_name = "B101EW01",
		.mode = {
			.hdisplay = 1280,
			.hsync_start = 1328,
			.hsync_end = 1360,
			.htotal = 1392,
			.vdisplay = 720,
			.vsync_start = 723,
			.vsync_end = 729,
			.vtotal = 736,
			.clock = 43860,
		},
		.regs = mtl017_auo_hires_tbl,
	}, {
		.manufacture = "CMO",
		.product_name = "N101L6-L0D",
		.mode = {
			.hdisplay = 1024,
			.hsync_start = 1064,
			.hsync_end = 1068,
			.htotal = 1148,
			.vdisplay = 600,
			.vsync_start = 621,
			.vsync_end = 625,
			.vtotal = 645,
			.clock = 43860,
		},
		.regs = mtl017_44_54_tbl,
        },
};

static void mtl017_conf(struct mtl017_display *mtl017, const u8 *reg_tbl)
{
	int i;
        int ret;
        int retry = 5;

	dev_info(&mtl017->client->dev, "Initializing MTL017 LVDS Controller\n");

	/* Write configuration table */
	for (i = 0; i < REGMAP_LENGTH; i+=BLOCK_TX_SIZE) {
        retry:
                msleep(1);
                ret = i2c_smbus_write_i2c_block_data(mtl017->client, i, BLOCK_TX_SIZE, &(reg_tbl[i]));
		if (ret < 0) {
			dev_warn(&mtl017->client->dev, "failed to initialize: %d\n", ret);
                        if (retry-- > 0)
                                goto retry;
			return;
		}
	}
}

static enum drm_connector_status imx_pd_connector_detect(
		struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void imx_pd_connector_destroy(struct drm_connector *connector)
{
	/* do not free here */
}

static int imx_pd_connector_get_modes(struct drm_connector *connector)
{
	struct mtl017_display *mtl017 = con_to_mtl017(connector);
	void *edid = NULL;
	int i;

	if (mtl017->i2c_adapter) {
		edid = drm_get_edid(connector, mtl017->i2c_adapter);
	}

	if (!edid)
		edid = mtl017->edid;

	for (i = 0; i < ARRAY_SIZE(pinfo); i ++) {
		if ((memcmp((u8 *)edid + 0x71, pinfo[i].product_name,
						strlen(pinfo[i].product_name)) == 0)) {
			struct drm_display_mode *mode = drm_mode_create(connector->dev);
			dev_info(mtl017->dev, "found LCD Panel: %s %s\n",
			       pinfo[i].manufacture,
			       pinfo[i].product_name);
			mtl017->regs = pinfo[i].regs;
			drm_mode_copy(mode, &pinfo[i].mode);
			mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
			drm_mode_probed_add(connector, mode);
			return 1;
		}
	}

	return -EINVAL;
}

static int imx_pd_connector_mode_valid(struct drm_connector *connector,
			  struct drm_display_mode *mode)
{
	return 0;
}

static struct drm_encoder *imx_pd_connector_best_encoder(
		struct drm_connector *connector)
{
	struct mtl017_display *mtl017 = con_to_mtl017(connector);

	return &mtl017->encoder;
}

static void imx_pd_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static bool imx_pd_encoder_mode_fixup(struct drm_encoder *encoder,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void imx_pd_encoder_prepare(struct drm_encoder *encoder)
{
	struct mtl017_display *mtl017 = enc_to_mtl017(encoder);

	imx_drm_crtc_panel_format(encoder->crtc, DRM_MODE_ENCODER_NONE,
			mtl017->interface_pix_fmt);
}

static void imx_pd_encoder_commit(struct drm_encoder *encoder)
{
	struct mtl017_display *mtl017 = enc_to_mtl017(encoder);

	mtl017_conf(mtl017, mtl017->regs);
}

static void imx_pd_encoder_mode_set(struct drm_encoder *encoder,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
}

static void imx_pd_encoder_disable(struct drm_encoder *encoder)
{
}

static void imx_pd_encoder_destroy(struct drm_encoder *encoder)
{
	/* do not free here */
}

static struct drm_connector_funcs imx_pd_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = imx_pd_connector_detect,
	.destroy = imx_pd_connector_destroy,
};

static struct drm_connector_helper_funcs imx_pd_connector_helper_funcs = {
	.get_modes = imx_pd_connector_get_modes,
	.best_encoder = imx_pd_connector_best_encoder,
	.mode_valid = imx_pd_connector_mode_valid,
};

static struct drm_encoder_funcs imx_pd_encoder_funcs = {
	.destroy = imx_pd_encoder_destroy,
};

static struct drm_encoder_helper_funcs imx_pd_encoder_helper_funcs = {
	.dpms = imx_pd_encoder_dpms,
	.mode_fixup = imx_pd_encoder_mode_fixup,
	.prepare = imx_pd_encoder_prepare,
	.commit = imx_pd_encoder_commit,
	.mode_set = imx_pd_encoder_mode_set,
	.disable = imx_pd_encoder_disable,
};

static int imx_pd_register(struct mtl017_display *mtl017)
{
	int ret;

	drm_mode_connector_attach_encoder(&mtl017->connector, &mtl017->encoder);

	mtl017->connector.funcs = &imx_pd_connector_funcs;
	mtl017->encoder.funcs = &imx_pd_encoder_funcs;

	mtl017->encoder.encoder_type = DRM_MODE_ENCODER_NONE;
	mtl017->connector.connector_type = DRM_MODE_CONNECTOR_VGA;

	drm_encoder_helper_add(&mtl017->encoder, &imx_pd_encoder_helper_funcs);
	ret = imx_drm_add_encoder(&mtl017->encoder, &mtl017->imx_drm_encoder,
			THIS_MODULE);
	if (ret) {
		dev_err(mtl017->dev, "adding encoder failed with %d\n", ret);
		return ret;
	}

	drm_connector_helper_add(&mtl017->connector,
			&imx_pd_connector_helper_funcs);

	ret = imx_drm_add_connector(&mtl017->connector,
			&mtl017->imx_drm_connector, THIS_MODULE);
	if (ret) {
		imx_drm_remove_encoder(mtl017->imx_drm_encoder);
		dev_err(mtl017->dev, "adding connector failed with %d\n", ret);
		return ret;
	}

	mtl017->connector.encoder = &mtl017->encoder;

	return 0;
}

static int mtl017_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct device_node *adapter_np;
	const u8 *edidp;
	struct mtl017_display *mtl017;
	int ret;
	const char *fmt;

	mtl017 = devm_kzalloc(&client->dev, sizeof(*mtl017), GFP_KERNEL);
	if (!mtl017)
		return -ENOMEM;

	edidp = of_get_property(np, "edid", &mtl017->edid_len);
	if (edidp)
		mtl017->edid = kmemdup(edidp, mtl017->edid_len, GFP_KERNEL);

	adapter_np = of_parse_phandle(np, "edid-i2c", 0);
	if (adapter_np) {
		mtl017->i2c_adapter = of_find_i2c_adapter_by_node(adapter_np);
		if (!mtl017->i2c_adapter) {
			dev_err(&client->dev, "Cannot find parent bus\n");
			return -ENODEV;
		}
	}

	ret = of_property_read_string(np, "interface-pix-fmt", &fmt);
	if (!ret) {
		if (!strcmp(fmt, "rgb24"))
			mtl017->interface_pix_fmt = V4L2_PIX_FMT_RGB24;
		else if (!strcmp(fmt, "rgb565"))
			mtl017->interface_pix_fmt = V4L2_PIX_FMT_RGB565;
	}

	mtl017->dev = &client->dev;
	mtl017->client = client;

	ret = imx_pd_register(mtl017);
	if (ret)
		return ret;

	ret = imx_drm_encoder_add_possible_crtcs(mtl017->imx_drm_encoder, np);

	i2c_set_clientdata(client, mtl017);

	return 0;
}

static int __devexit mtl017_remove(struct i2c_client *client)
{
	struct mtl017_display *mtl017 = i2c_get_clientdata(client);
	struct drm_connector *connector = &mtl017->connector;
	struct drm_encoder *encoder = &mtl017->encoder;

	drm_mode_connector_detach_encoder(connector, encoder);

	imx_drm_remove_connector(mtl017->imx_drm_connector);
	imx_drm_remove_encoder(mtl017->imx_drm_encoder);

	return 0;
}

static const struct i2c_device_id mtl017_ids[] = {
	{ "mtl017", 0 },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(i2c, mtl017_ids);

static struct i2c_driver mtl017_driver = {
	.driver = {
		.name = "mtl017",
		.owner = THIS_MODULE,
	},
	.probe = mtl017_probe,
	.remove = __devexit_p(mtl017_remove),
	.id_table = mtl017_ids,
};

static int __init mtl017_init(void)
{
	return i2c_add_driver(&mtl017_driver);
}
module_init(mtl017_init);

static void __exit mtl017_exit(void)
{
	i2c_del_driver(&mtl017_driver);
}
module_exit(mtl017_exit);

MODULE_DESCRIPTION("i.MX parallel display driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
