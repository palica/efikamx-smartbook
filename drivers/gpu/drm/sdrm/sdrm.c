/*
 * simple drm driver template
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
 * You should have rsdrmeived a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <linux/fb.h>
#include <asm/fb.h>
#include <linux/module.h>
#include <drm/sdrm_encon.h>
#include <drm/sdrm.h>
#include "sdrm_gem.h"
#include "sdrm_fb.h"
#include "sdrm_fbdev.h"

#define DRIVER_NAME		"drm-generic"
#define DRIVER_DESC		"drm generic graphics"
#define DRIVER_DATE		"20110604"
#define DRIVER_MAJOR		1
#define DRIVER_MINOR		0
#define DRIVER_PATCHLEVEL	0

struct sdrm_device {
	struct drm_device		*drm;
	struct platform_device		*pdev;
	struct drm_fb_helper		*fb_helper;
	struct list_head		crtc_list;
	struct list_head		encoder_list;
	struct list_head		connector_list;
	struct list_head		list;
	const char			*name;
	int				registered;
	struct mutex			mutex;
	struct drm_driver		driver;
};

struct sdrm_crtc {
	struct drm_crtc			*crtc;
	struct list_head		list;
	struct sdrm_device		*sdrm;
	int				registered;
	int				pipe;
	struct drm_crtc_helper_funcs	crtc_helper_funcs;
	struct drm_crtc_funcs		crtc_funcs;
	struct sdrm_crtc_helper_funcs	sdrm_helper_funcs;
};

struct sdrm_encoder {
	struct drm_encoder		*encoder;
	struct list_head		list;
	struct sdrm_device		*sdrm;
	int				registered;
	struct module			*owner;
};

struct sdrm_connector {
	struct drm_connector		*connector;
	struct list_head		list;
	struct sdrm_device		*sdrm;
	int				registered;
	struct module			*owner;
};

static int sdrm_driver_open(struct drm_device *drm, struct drm_file *file)
{
	struct sdrm_device *sdrm = drm->dev_private;

	if (!try_module_get(sdrm->pdev->dev.driver->owner)) {
		dev_err(drm->dev, "could not get module %s\n",
				module_name(sdrm->pdev->dev.driver->owner));
		return -ENODEV;
	}

	return 0;
}

static void sdrm_driver_lastclose(struct drm_device *drm)
{
	struct sdrm_device *sdrm = drm->dev_private;

	module_put(sdrm->pdev->dev.driver->owner);

	/*
	 * This shouldn't be here. If multiple drm applications (i.e. two
	 * xservers) are running and the active one crashes then we will
	 * only restore fbdev mode when the other one exits aswell. Anyway,
	 * this is what all other drivers so for now we do it aswell.
	 */
	drm_fb_helper_restore_fbdev_mode(sdrm->fb_helper);
}

static int sdrm_driver_unload(struct drm_device *drm)
{
	struct sdrm_device *sdrm = drm->dev_private;

	sdrm_fbdev_fini(sdrm->fb_helper);
	sdrm->fb_helper = NULL;

	drm_mode_config_cleanup(sdrm->drm);
	drm_kms_helper_poll_fini(sdrm->drm);

	return 0;
}

static int sdrm_suspend(struct drm_device *drm, pm_message_t state)
{
	/* TODO */

	return 0;
}

static int sdrm_resume(struct drm_device *drm)
{
	/* TODO */

	return 0;
}

/*
 * We don't care at all for crtc numbers, but the core expects the
 * crtcs to be numbered
 */
static struct sdrm_crtc *sdrm_crtc_by_num(struct sdrm_device *sdrm, int num)
{
	struct sdrm_crtc *sdrm_crtc;

	list_for_each_entry(sdrm_crtc, &sdrm->crtc_list, list)
		if (sdrm_crtc->pipe == num)
			return sdrm_crtc;
	return NULL;
}

int sdrm_crtc_vblank_get(struct sdrm_crtc *sdrm_crtc)
{
	return drm_vblank_get(sdrm_crtc->sdrm->drm, sdrm_crtc->pipe);
}
EXPORT_SYMBOL_GPL(sdrm_crtc_vblank_get);

void sdrm_crtc_vblank_put(struct sdrm_crtc *sdrm_crtc)
{
	drm_vblank_put(sdrm_crtc->sdrm->drm, sdrm_crtc->pipe);
}
EXPORT_SYMBOL_GPL(sdrm_crtc_vblank_put);

void sdrm_handle_vblank(struct sdrm_crtc *sdrm_crtc)
{
	struct sdrm_device *sdrm = sdrm_crtc->sdrm;

	if (sdrm->registered)
		drm_handle_vblank(sdrm_crtc->sdrm->drm, sdrm_crtc->pipe);
}
EXPORT_SYMBOL_GPL(sdrm_handle_vblank);

static int sdrm_enable_vblank(struct drm_device *drm, int crtc)
{
	struct sdrm_device *sdrm = drm->dev_private;
	struct sdrm_crtc *sdrm_crtc;
	int ret;

	sdrm_crtc = sdrm_crtc_by_num(sdrm, crtc);
	if (!sdrm_crtc)
		return -EINVAL;

	if (!sdrm_crtc->sdrm_helper_funcs.enable_vblank)
		return -ENOSYS;

	ret = sdrm_crtc->sdrm_helper_funcs.enable_vblank(sdrm_crtc->crtc);
	return ret;
}

static void sdrm_disable_vblank(struct drm_device *drm, int crtc)
{
	struct sdrm_device *sdrm = drm->dev_private;
	struct sdrm_crtc *sdrm_crtc;

	sdrm_crtc = sdrm_crtc_by_num(sdrm, crtc);
	if (!sdrm_crtc)
		return;

	if (!sdrm_crtc->sdrm_helper_funcs.disable_vblank)
		return;

	sdrm_crtc->sdrm_helper_funcs.disable_vblank(sdrm_crtc->crtc);
}

static struct vm_operations_struct sdrm_gem_vm_ops = {
	.fault = sdrm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static struct drm_ioctl_desc sdrm_ioctls[] = {
	/* none so far */
};

static const struct file_operations sdrm_driver_fops = {
	 .owner = THIS_MODULE,
	 .open = drm_open,
	 .release = drm_release,
	 .unlocked_ioctl = drm_ioctl,
	 .mmap = sdrm_gem_mmap,
	 .poll = drm_poll,
	 .fasync = drm_fasync,
	 .read = drm_read,
	 .llseek = noop_llseek,
};

static int sdrm_get_irq(struct drm_device *dev)
{
	/*
	 * Return an arbitrary number to make the core happy.
	 * We can't return anything meaningful here since drm
	 * devices in general have multiple irqs
	 */
	return 1234;
}

static struct drm_bus drm_platform_bus = {
	.bus_type = DRIVER_BUS_PLATFORM,
	.get_irq = sdrm_get_irq,
};

static struct drm_driver sdrm_driver_template = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM,
	.unload			= sdrm_driver_unload,
	.open			= sdrm_driver_open,
	.lastclose		= sdrm_driver_lastclose,
	.gem_free_object	= sdrm_gem_free_object,
	.gem_vm_ops		= &sdrm_gem_vm_ops,
	.dumb_create		= sdrm_gem_dumb_create,
	.dumb_map_offset	= sdrm_gem_dumb_map_offset,
	.dumb_destroy		= sdrm_gem_dumb_destroy,

	.suspend		= sdrm_suspend,
	.resume			= sdrm_resume,

	.get_vblank_counter	= drm_vblank_count,
	.enable_vblank		= sdrm_enable_vblank,
	.disable_vblank		= sdrm_disable_vblank,
	.reclaim_buffers	= drm_core_reclaim_buffers,
	.ioctls			= sdrm_ioctls,
	.num_ioctls		= ARRAY_SIZE(sdrm_ioctls),
	.bus			= &drm_platform_bus,
	.fops			= &sdrm_driver_fops,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
	.patchlevel		= DRIVER_PATCHLEVEL,
};

static LIST_HEAD(sdrm_device_list);
static DEFINE_MUTEX(sdrm_list_mutex);

/*
 * sdrm_device_get - find or allocate sdrm device with unique name
 *
 * This function returns the sdrm device with the unique name 'name'
 * If this already exists, return it, otherwise allocate a new
 * objsdrmt.
 */
static struct sdrm_device *sdrm_device_get(const char *name)
{
	struct sdrm_device *sdrm;

	mutex_lock(&sdrm_list_mutex);

	list_for_each_entry(sdrm, &sdrm_device_list, list)
		if (!strcmp(sdrm->name, name))
			goto out;

	mutex_unlock(&sdrm_list_mutex);

	sdrm = kzalloc(sizeof(*sdrm), GFP_KERNEL);
        if (!sdrm)
                return NULL;

	mutex_init(&sdrm->mutex);
	INIT_LIST_HEAD(&sdrm->crtc_list);
	INIT_LIST_HEAD(&sdrm->connector_list);
	INIT_LIST_HEAD(&sdrm->encoder_list);

	sdrm->name = kstrdup(name, GFP_KERNEL);
	if (!sdrm->name) {
		kfree(sdrm);
		return NULL;
	}

	mutex_lock(&sdrm_list_mutex);

	list_add_tail(&sdrm->list, &sdrm_device_list);
out:
	mutex_unlock(&sdrm_list_mutex);

	return sdrm;
}

/*
 * sdrm_device_maybe_release - free sdrm device if noone is using it
 *
 * If we have no crtcs, connectors and encoders registered for this
 * device we can free it. Caller must hold sdrm->mutex.
 */
static void sdrm_device_maybe_release(struct sdrm_device *sdrm)
{
	if (!list_empty(&sdrm->crtc_list))
		return;
	if (!list_empty(&sdrm->connector_list))
		return;
	if (!list_empty(&sdrm->encoder_list))
		return;

	mutex_lock(&sdrm_list_mutex);
	list_del(&sdrm->list);
	mutex_unlock(&sdrm_list_mutex);

	kfree(sdrm->name);
	kfree(sdrm);
}

/*
 * I think this legacy modegroup handling should be removed and we
 * have a patch for this which also defines HAVE_MODEGROUP_REMOVED
 * For now ifdef this here to make this work with and without this
 * patch.
 */
#ifndef HAVE_MODEGROUP_REMOVED
static int drm_mode_group_reinit(struct drm_device *dev)
{
	struct drm_mode_group *group = &dev->primary->mode_group;
	uint32_t *id_list = group->id_list;
	int ret;

	ret = drm_mode_group_init_legacy_group(dev, group);
	if (ret < 0)
		return ret;

	kfree(id_list);
	return 0;
}
#else
static int drm_mode_group_reinit(struct drm_device *dev)
{
	return 0;
}
#endif

/*
 * register an encoder to the drm core
 */
static int sdrm_encoder_register(struct sdrm_encoder *sdrm_encoder)
{
	struct sdrm_device *sdrm = sdrm_encoder->sdrm;

	if (!try_module_get(sdrm_encoder->owner))
		return -ENODEV;

	drm_encoder_init(sdrm->drm, sdrm_encoder->encoder,
			sdrm_encoder->encoder->funcs,
			DRM_MODE_ENCODER_TMDS);

	drm_mode_group_reinit(sdrm->drm);

	sdrm_encoder->registered = 1;

	return 0;
}

/*
 * unregister an encoder from the drm core
 */
static void sdrm_encoder_unregister(struct sdrm_encoder *sdrm_encoder)
{
	struct sdrm_device *sdrm = sdrm_encoder->sdrm;

	if (!sdrm_encoder->registered)
		return;

	if (sdrm->registered)
		drm_encoder_cleanup(sdrm_encoder->encoder);

	drm_mode_group_reinit(sdrm->drm);

	sdrm_encoder->registered = 0;

	module_put(sdrm_encoder->owner);
}

/*
 * register a connector to the drm core
 */
static int sdrm_connector_register(struct sdrm_connector *sdrm_connector)
{
	struct sdrm_device *sdrm = sdrm_connector->sdrm;
	int ret;

	if (!try_module_get(sdrm_connector->owner))
		return -ENODEV;

	drm_connector_init(sdrm->drm, sdrm_connector->connector,
			sdrm_connector->connector->funcs,
			DRM_MODE_CONNECTOR_VGA);
	drm_mode_group_reinit(sdrm->drm);
	ret = drm_sysfs_connector_add(sdrm_connector->connector);
	if (ret)
		goto err;

	sdrm_connector->registered = 1;

	return 0;
err:
	module_put(sdrm_connector->owner);

	return ret;
}

/*
 * unregister a connector from the drm core
 */
static void sdrm_connector_unregister(struct sdrm_connector *sdrm_connector)
{
	struct sdrm_device *sdrm = sdrm_connector->sdrm;

	if (!sdrm_connector->registered)
		return;

	if (sdrm->registered) {
		drm_sysfs_connector_remove(sdrm_connector->connector);
		drm_connector_cleanup(sdrm_connector->connector);
	}

	drm_mode_group_reinit(sdrm->drm);

	sdrm_connector->registered = 0;

	module_put(sdrm_connector->owner);
}

/*
 * register a crtc to the drm core
 */
static int sdrm_crtc_register(struct sdrm_crtc *sdrm_crtc)
{
	struct sdrm_device *sdrm = sdrm_crtc->sdrm;
	int ret;

	drm_crtc_init(sdrm->drm, sdrm_crtc->crtc, &sdrm_crtc->crtc_funcs);
	ret = drm_mode_crtc_set_gamma_size(sdrm_crtc->crtc, 256);
#if 0
	/*
	 * drm_mode_crtc_set_gamma_size currently returns a boolean 'true' for
	 * success. A patch has been sent to convert it to use 0 for success and
	 * a proper errno otherwise. Ignore the return value for now until this
	 * has settled.
	 */
	if (ret)
		return -EINVAL;
#endif
	drm_crtc_helper_add(sdrm_crtc->crtc, &sdrm_crtc->crtc_helper_funcs);

	sdrm_crtc->registered = 1;

	return 0;
}

static void sdrm_crtc_unregister(struct sdrm_crtc *sdrm_crtc)
{
	/*
	 * The hard work has already been done in driver unload
	 */
	sdrm_crtc->registered = 0;
}

/*
 * sdrm_uninit - unitialize all crtcs, encoders, connectors
 *
 * This is called in case initialization fails or as a cleanup
 * after unitializing a drm device. Caller must hold sdrm->mutex
 */
static void sdrm_uninit(struct sdrm_device *sdrm)
{
	struct sdrm_crtc *sdrm_crtc;
	struct sdrm_encoder *sdrm_encoder;
	struct sdrm_connector *sdrm_connector;

	list_for_each_entry(sdrm_crtc, &sdrm->crtc_list, list)
		sdrm_crtc_unregister(sdrm_crtc);

	list_for_each_entry(sdrm_connector, &sdrm->connector_list, list)
		sdrm_connector_unregister(sdrm_connector);

	list_for_each_entry(sdrm_encoder, &sdrm->encoder_list, list)
		sdrm_encoder_unregister(sdrm_encoder);

	sdrm->registered = 0;
}

/*
 * Called by the CRTC driver when all CRTCs are registered. This
 * puts all the pisdrmes together and initializes the driver.
 * Once this is called no more CRTCs can be registered since
 * the drm core has hardcoded the number of crtcs in several
 * places.
 */
int sdrm_init_drm(const char *name, struct platform_device *pdev)
{
	struct sdrm_device *sdrm;
	struct drm_device *drm;
	struct sdrm_crtc *sdrm_crtc;
	struct sdrm_encoder *sdrm_encoder;
	struct sdrm_connector *sdrm_connector;
	int ret;
	int num_crtcs = 0;

	sdrm = sdrm_device_get(name);
	if (!sdrm)
		return -ENOMEM;

	drm = kzalloc(sizeof(struct drm_device), GFP_KERNEL);
	if (!drm)
		return -ENOMEM;

	sdrm->drm = drm;

	memcpy(&sdrm->driver, &sdrm_driver_template,
			sizeof(sdrm_driver_template));

	drm->dev_private = sdrm;

	ret = drm_fill_in_dev(drm, NULL, &sdrm->driver);
	if (ret) {
		printk(KERN_ERR "DRM: Fill_in_dev failed.\n");
		goto err_fill_in;
	}

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = 1, we can use the vblank feature.
	 *
	 * P.S. note that we wouldn't use drm irq handler but
	 *      just spsdrmific driver own one instead bsdrmause
	 *      drm framework supports only one irq handler and
	 *      drivers can well take care of their interrupts
	 */
	drm->irq_enabled = 1;

	mutex_lock(&drm_global_mutex);

	ret = drm_get_minor(drm, &drm->control, DRM_MINOR_CONTROL);
	if (ret)
		goto err_get_minor1;

	ret = drm_get_minor(drm, &drm->primary, DRM_MINOR_LEGACY);
	if (ret)
		goto err_get_minor2;

	mutex_unlock(&drm_global_mutex);

	drm_mode_config_init(drm);
	sdrm_mode_config_init(drm);

	mutex_lock(&sdrm->mutex);

	/* we need at least one crtc */
	if (list_empty(&sdrm->crtc_list)) {
		ret = -EINVAL;
		goto err_init;
	}

	/* Register the subdevices we have so far */

	list_for_each_entry(sdrm_encoder, &sdrm->encoder_list, list) {
		ret = sdrm_encoder_register(sdrm_encoder);
		if (ret)
			goto err_init;
	}

	list_for_each_entry(sdrm_connector, &sdrm->connector_list, list) {
		ret = sdrm_connector_register(sdrm_connector);
		if (ret)
			goto err_init;
	}

	list_for_each_entry(sdrm_crtc, &sdrm->crtc_list, list) {
		sdrm_crtc->pipe = num_crtcs;
		num_crtcs++;
		ret = sdrm_crtc_register(sdrm_crtc);
		if (ret)
			goto err_init;
	}

	sdrm->pdev = pdev;
	sdrm->drm->platformdev = pdev;
	sdrm->drm->dev = &pdev->dev;

	sdrm->fb_helper = sdrm_fbdev_init(sdrm->drm);
	if (!sdrm->fb_helper) {
		printk("sdrm_fbdev_init failed\n");
		ret = -ENOMEM;
		goto err_init;
	}

	drm_kms_helper_poll_init(sdrm->drm);

#ifndef HAVE_MODEGROUP_REMOVED
	mutex_lock(&drm_global_mutex);

	/* setup the grouping for the legacy output */
	ret = drm_mode_group_init_legacy_group(sdrm->drm,
			&sdrm->drm->primary->mode_group);
	mutex_unlock(&drm_global_mutex);

	if (ret)
		return ret;
#endif
	ret = drm_vblank_init(sdrm->drm, num_crtcs);
	if (ret)
		goto err_init;

	/*
	 * with vblank_disable_allowed = 1, vblank interrupt will be disabled
	 * by drm timer once a current process gives up ownership of
	 * vblank event.(after drm_vblank_put function is called)
	 */
	sdrm->drm->vblank_disable_allowed = 1;

	sdrm->registered = 1;

	mutex_unlock(&sdrm->mutex);

	return 0;

err_init:
	sdrm_uninit(sdrm);
	mutex_unlock(&sdrm->mutex);
err_get_minor2:
	drm_put_minor(&drm->control);
err_get_minor1:
err_fill_in:
	mutex_unlock(&drm_global_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(sdrm_init_drm);

/*
 * Called by the CRTC driver to uninitialize the driver.
 */
int sdrm_exit_drm(const char *name)
{
	struct sdrm_device *sdrm;

	sdrm = sdrm_device_get(name);
	if (!sdrm)
		return -ENODEV;

	sdrm_uninit(sdrm);
	/*
	 * The drm core never does anything with this list, yet it removes
	 * this entry, so initialize it
	 */
	INIT_LIST_HEAD(&sdrm->drm->driver_item);
	drm_put_dev(sdrm->drm);

	sdrm->drm = NULL;
	sdrm->registered = 0;
	sdrm->pdev = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(sdrm_exit_drm);

/*
 * sdrm_add_crtc - add a new crtc
 *
 * The return value if !NULL is a cookie for the caller to pass to
 * sdrm_remove_crtc later.
 */
struct sdrm_crtc *sdrm_add_crtc(const char *name, struct drm_crtc *crtc,
		const struct drm_crtc_funcs *crtc_funcs,
		const struct drm_crtc_helper_funcs *crtc_helper_funcs,
		const struct sdrm_crtc_helper_funcs *sdrm_helper_funcs)
{
	struct sdrm_device *sdrm;
	struct sdrm_crtc *sdrm_crtc;

	sdrm = sdrm_device_get(name);
	if (!sdrm)
		return NULL; /* -ENOMEM */

	if (sdrm->registered)
		return NULL; /* -EBUSY */

	sdrm_crtc = kzalloc(sizeof(*sdrm_crtc), GFP_KERNEL);
	if (!sdrm_crtc)
		return NULL; /* -ENOMEM */

	sdrm_crtc->crtc_funcs = *crtc_funcs;
	sdrm_crtc->crtc_helper_funcs = *crtc_helper_funcs;
	sdrm_crtc->sdrm_helper_funcs = *sdrm_helper_funcs;

	WARN_ON(crtc_funcs->set_config);
	WARN_ON(crtc_funcs->destroy);

	sdrm_crtc->crtc_funcs.set_config = drm_crtc_helper_set_config;
	sdrm_crtc->crtc_funcs.destroy = drm_crtc_cleanup;

	sdrm_crtc->crtc = crtc;
	sdrm_crtc->sdrm = sdrm;

	mutex_lock(&sdrm->mutex);

	list_add_tail(&sdrm_crtc->list, &sdrm->crtc_list);

	mutex_unlock(&sdrm->mutex);

	return sdrm_crtc;
}
EXPORT_SYMBOL_GPL(sdrm_add_crtc);

/*
 * sdrm_remove_crtc - remove a crtc
 *
 * Can only be called on inactive drm devices since the drm
 * core can't handle dynamic crtcs
 */
int sdrm_remove_crtc(struct sdrm_crtc *sdrm_crtc)
{
	struct sdrm_device *sdrm = sdrm_crtc->sdrm;

	if (sdrm->registered)
		return -EBUSY;

	mutex_lock(&sdrm->mutex);

	sdrm_crtc_unregister(sdrm_crtc);

	list_del(&sdrm_crtc->list);

	kfree(sdrm_crtc);

	sdrm_device_maybe_release(sdrm);

	mutex_unlock(&sdrm->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(sdrm_remove_crtc);

/*
 * sdrm_add_encoder - add a new encoder
 *
 * The return value if !NULL is a cookie for the caller to pass to
 * sdrm_remove_crtc later.
 *
 * Can be called on both active and inactive devices.
 */
struct sdrm_encoder *sdrm_add_encoder(const char *name,
		struct drm_encoder *encoder, struct module *owner)
{
	struct sdrm_device *sdrm;
	struct sdrm_encoder *sdrm_encoder;
	int ret;

	sdrm = sdrm_device_get(name);
	if (!sdrm)
		return NULL;

	sdrm_encoder = kzalloc(sizeof(struct sdrm_encoder), GFP_KERNEL);
	if (!sdrm_encoder)
		return NULL;
	sdrm_encoder->encoder = encoder;
	sdrm_encoder->sdrm = sdrm;
	sdrm_encoder->owner = owner;

	mutex_lock(&sdrm->mutex);

	if (sdrm->registered) {
		/*
		 * If the drm device is up register the encoder,
		 * otherwise it will be done in sdrm_init_drm()
		 */
		ret = sdrm_encoder_register(sdrm_encoder);
		if (ret) {
			kfree(sdrm_encoder);
			goto err;
		}
	}

	list_add_tail(&sdrm_encoder->list, &sdrm->encoder_list);
err:
	mutex_unlock(&sdrm->mutex);

	return sdrm_encoder;
}
EXPORT_SYMBOL_GPL(sdrm_add_encoder);

/*
 * sdrm_remove_encoder - remove an encoder
 *
 * Can be called on both active and inactive devices.
 */
int sdrm_remove_encoder(struct sdrm_encoder *sdrm_encoder)
{
	struct sdrm_device *sdrm = sdrm_encoder->sdrm;

	mutex_lock(&sdrm->mutex);

	sdrm_encoder_unregister(sdrm_encoder);

	list_del(&sdrm_encoder->list);

	kfree(sdrm_encoder);

	sdrm_device_maybe_release(sdrm);

	mutex_unlock(&sdrm->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(sdrm_remove_encoder);

/*
 * sdrm_add_connector - add a connector
 *
 * Can be called on both active and inactive devices.
 */
struct sdrm_connector *sdrm_add_connector(const char *name,
		struct drm_connector *connector, struct module *owner)
{
	struct sdrm_device *sdrm;
	struct sdrm_connector *sdrm_connector;
	int ret;

	sdrm = sdrm_device_get(name);
	if (!sdrm)
		return NULL;

	sdrm_connector = kzalloc(sizeof(struct sdrm_connector), GFP_KERNEL);
	if (!sdrm_connector)
		return NULL;

	sdrm_connector->connector = connector;
	sdrm_connector->sdrm = sdrm;
	sdrm_connector->owner = owner;

	mutex_lock(&sdrm->mutex);

	if (sdrm->registered) {
		/*
		 * If the drm device is up register the connector,
		 * otherwise it will be done in sdrm_init_drm()
		 */
		ret = sdrm_connector_register(sdrm_connector);
		if (ret) {
			kfree(sdrm_connector);
			sdrm_connector = NULL;
			goto err;
		}
	}

	list_add_tail(&sdrm_connector->list, &sdrm->connector_list);
err:
	mutex_unlock(&sdrm->mutex);

	return sdrm_connector;
}
EXPORT_SYMBOL_GPL(sdrm_add_connector);

/*
 * sdrm_remove_connector - remove a connector
 *
 * Can be called on both active and inactive devices.
 */
int sdrm_remove_connector(struct sdrm_connector *sdrm_connector)
{
	struct sdrm_device *sdrm = sdrm_connector->sdrm;

	mutex_lock(&sdrm->mutex);

	sdrm_connector_unregister(sdrm_connector);

	list_del(&sdrm_connector->list);

	kfree(sdrm_connector);

	sdrm_device_maybe_release(sdrm);

	mutex_unlock(&sdrm->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(sdrm_remove_connector);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
