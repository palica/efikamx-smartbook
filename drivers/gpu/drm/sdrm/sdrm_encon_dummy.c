#include <linux/module.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/sdrm_encon.h>
#include <drm/sdrm.h>

struct sdrm_encon_dummy_private {
	struct drm_encoder_connector encon;
	struct drm_display_mode *modes;
	int num_modes;
};

static int dummy_get_modes(struct drm_encoder_connector *encon)
{
	struct sdrm_encon_dummy_private *priv = encon->encon_private;
	struct drm_display_mode *mode;
	struct drm_device *drm = encon->connector.dev;
	int i, ret = 0;

	if (!priv->num_modes)
		return 0;

	for (i = 0; i < priv->num_modes; i++) {
		mode = drm_mode_duplicate(drm, &priv->modes[i]);
		if (mode == NULL)
			return 0;

		mode->base.type = DRM_MODE_OBJECT_MODE;

		drm_mode_probed_add(&encon->connector, mode);
		ret++; 
	}
	return ret;
}

static struct drm_encoder_connector_funcs dummy_funcs = {
	.get_modes = dummy_get_modes,
};

/*
 * sdrm_encon_add_dummy - add a dummy encoder/connector
 *
 * All callbacks of a dummy encoder/connector are no-ops, only
 * an array of modes can be provided.
 */
struct drm_encoder_connector *sdrm_encon_add_dummy(struct sdrm_encon_dummy *dummy)
{
	struct sdrm_encon_dummy_private *priv;
	struct drm_encoder_connector *encon;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return NULL;

	encon = &priv->encon;
	priv->modes = dummy->modes;
	priv->num_modes = dummy->num_modes;
	encon->funcs = &dummy_funcs;
	encon->encon_private = priv;
	encon->connector.encoder = &encon->encoder;
	encon->encoder.possible_crtcs = dummy->possible_crtcs;
	encon->encoder.possible_clones = dummy->possible_clones;
	encon->owner = dummy->owner;

	ret = drm_encoder_connector_register(dummy->drm_name, encon);
	if (ret) {
		kfree(priv);
		return NULL;
	}

	return encon;
}
EXPORT_SYMBOL_GPL(sdrm_encon_add_dummy);

int sdrm_encon_remove_dummy(struct drm_encoder_connector *encon)
{
	struct sdrm_encon_dummy_private *priv = encon->encon_private;

	sdrm_remove_connector(encon->sdrm_connector);
	sdrm_remove_encoder(encon->sdrm_encoder);

	kfree(priv);

	return 0;
}
EXPORT_SYMBOL_GPL(sdrm_encon_remove_dummy);

struct sdrm_encon_pdev_priv {
	struct sdrm_encon_dummy dummy;
	struct drm_encoder_connector *encon;
	unsigned int flags;
	int gpio_dpms;
	int gpio_backlight;	
};

static void sdrm_encon_pdev_gpio_dpms(struct sdrm_encon_dummy *dummy, int mode)
{
	struct sdrm_encon_pdev_priv *priv = container_of(dummy,
			struct sdrm_encon_pdev_priv, dummy);
	int gpio_backlight = 0, gpio_dpms = 0;

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		printk("%s: on\n", __func__);
		gpio_backlight = 1;
		gpio_dpms = 1;
	default:
		printk("%s: off\n", __func__);
		break;
	}

	if (priv->flags == DRM_ENCON_DUMMY_BL_GPIO_ACTIVE_LOW)
		gpio_backlight = !gpio_backlight;
	if (priv->flags == DRM_ENCON_DUMMY_DPMS_GPIO_ACTIVE_LOW)
		gpio_dpms = !gpio_dpms;

	if (priv->flags & DRM_ENCON_DUMMY_USE_DPMS_GPIO)
		gpio_direction_output(priv->gpio_dpms, gpio_dpms);

	if (priv->flags & DRM_ENCON_DUMMY_USE_BL_GPIO)
		gpio_direction_output(priv->gpio_backlight, gpio_backlight);
}

static int __devinit sdrm_encon_platform_probe(struct platform_device *pdev)
{
	struct sdrm_encon_dummy_pdata *pdata = pdev->dev.platform_data;
	struct sdrm_encon_pdev_priv *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dummy.owner = THIS_MODULE;
	priv->dummy.drm_name = pdata->drm_name;
	priv->dummy.possible_crtcs = pdata->possible_crtcs;
	priv->dummy.possible_clones = pdata->possible_clones;
	priv->dummy.modes = pdata->modes;
	priv->dummy.num_modes = pdata->num_modes;
	priv->flags = pdata->flags;

	if (pdata->flags & DRM_ENCON_DUMMY_USE_BL_GPIO) {
		priv->gpio_backlight = pdata->gpio_backlight;
		ret = gpio_request(priv->gpio_backlight, "backlight");
		if (ret)
			return ret;
	}

	if (pdata->flags & DRM_ENCON_DUMMY_USE_DPMS_GPIO) {
		priv->gpio_dpms = pdata->gpio_dpms;
		priv->dummy.dpms = sdrm_encon_pdev_gpio_dpms;
		ret = gpio_request(priv->gpio_dpms, "dpms");
		if (ret) {
			if (pdata->flags & DRM_ENCON_DUMMY_USE_BL_GPIO)
				gpio_free(priv->gpio_backlight);
			return ret;
		}
	}

	priv->encon = sdrm_encon_add_dummy(&priv->dummy);
	if (!priv->encon)
		return -EINVAL;

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int __devexit sdrm_encon_platform_remove(struct platform_device *pdev)
{
	struct sdrm_encon_pdev_priv *priv = platform_get_drvdata(pdev);

	sdrm_encon_remove_dummy(priv->encon);

	return 0;
}

static struct platform_driver sdrm_encon_driver = {
	.probe		= sdrm_encon_platform_probe,
	.remove		= __devexit_p(sdrm_encon_platform_remove),
	.driver		= {
		.name	= "drm-encon-dummy",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(sdrm_encon_driver);

MODULE_DESCRIPTION("drm encoder/connector dummy driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
