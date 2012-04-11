#ifndef __DRM_ENCON_H
#define __DRM_ENCON_H

struct drm_encoder_connector;

struct drm_encoder_connector_funcs {
	int (*get_modes)(struct drm_encoder_connector *encon);
	int (*mode_valid)(struct drm_encoder_connector *encon,
			struct drm_display_mode *mode);
	void (*mode_set)(struct drm_encoder_connector *encon,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode);
	void (*dpms)(struct drm_encoder_connector *encon, int mode);
	enum drm_connector_status (*detect)(struct drm_encoder_connector *encon);
	void (*commit)(struct drm_encoder_connector *encon);
	bool (*mode_fixup)(struct drm_encoder_connector *encon,
			struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode);
	void (*prepare)(struct drm_encoder_connector *encon);
	int (*set_property)(struct drm_encoder_connector *encon,
			struct drm_property *property, uint64_t val);
};

struct drm_encoder_connector {
	struct drm_connector connector;
	struct sdrm_connector *sdrm_connector;
	struct drm_encoder encoder;
	struct sdrm_encoder *sdrm_encoder;
	struct drm_encoder_connector_funcs *funcs;
	struct module *owner;
	void *encon_private;
};

void drm_encoder_connector_cleanup(struct drm_device *drm,
		struct drm_encoder_connector *c);
int drm_encoder_connector_register(const char *drm_name,
		struct drm_encoder_connector *encon);
void drm_encoder_connector_unregister(struct drm_encoder_connector *encon);

struct sdrm_encon_dummy {
	char *drm_name;
	u32 possible_crtcs;
	u32 possible_clones;
	struct drm_display_mode *modes;
	int num_modes;
	struct module *owner;
	void (*dpms)(struct sdrm_encon_dummy *dummy, int mode);
	void *driver_priv;
};

struct drm_encoder_connector *sdrm_encon_add_dummy(struct sdrm_encon_dummy *);
int sdrm_encon_remove_dummy(struct drm_encoder_connector *encon);

struct sdrm_encon_dummy_pdata {
	char *drm_name;
	u32 possible_crtcs;
	u32 possible_clones;
	struct drm_display_mode *modes;
	int num_modes;
	int gpio_dpms;
	int gpio_backlight;
#define DRM_ENCON_DUMMY_USE_BL_GPIO		(1 << 0)
#define DRM_ENCON_DUMMY_BL_GPIO_ACTIVE_LOW	(1 << 1)
#define DRM_ENCON_DUMMY_USE_DPMS_GPIO		(1 << 2)
#define DRM_ENCON_DUMMY_DPMS_GPIO_ACTIVE_LOW	(1 << 3)
	unsigned flags;
};

#endif /* __DRM_ENCON_H */
