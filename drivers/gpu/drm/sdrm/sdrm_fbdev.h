#ifndef _DRMS_FBDEV_H_
#define _DRMS_FBDEV_H_

struct drm_fb_helper *sdrm_fbdev_init(struct drm_device *dev);
void sdrm_fbdev_fini(struct drm_fb_helper *);

#endif /* _DRMS_FBDEV_H_ */
