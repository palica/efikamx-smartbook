#ifndef __DRM_SDRM_H
#define __DRM_SDRM_H

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

/**
 * User-desired buffer creation information structure.
 *
 * @size: requested size for the object.
 *	- this size value would be page-aligned internally.
 * @flags: user request for setting memory type or cache attributes.
 * @handle: returned handle for the object.
 */
struct sdrm_gem_create {
	unsigned int size;
	unsigned int flags;
	unsigned int handle;
};

/**
 * A structure for getting buffer offset.
 *
 * @handle: a pointer to gem object created.
 * @pad: just padding to be 64-bit aligned.
 * @offset: relatived offset value of the memory region allocated.
 *	- this value should be set by user.
 */
struct sdrm_gem_map_off {
	unsigned int handle;
	unsigned int pad;
	uint64_t offset;
};

/**
 * A structure for mapping buffer.
 *
 * @handle: a handle to gem object created.
 * @size: memory size to be mapped.
 * @mapped: having user virtual address mmaped.
 *	- this variable would be filled by exynos gem module
 *	of kernel side with user virtual address which is allocated
 *	by do_mmap().
 */
struct sdrm_gem_mmap {
	unsigned int handle;
	unsigned int size;
	uint64_t mapped;
};

struct sdrm_device;
struct sdrm_crtc;

struct sdrm_crtc_helper_funcs {
	int (*enable_vblank)(struct drm_crtc *crtc);
	void (*disable_vblank)(struct drm_crtc *crtc);
};

struct sdrm_crtc *sdrm_add_crtc(const char *name, struct drm_crtc *crtc,
		const struct drm_crtc_funcs *crtc_funcs,
		const struct drm_crtc_helper_funcs *crtc_helper_funcs,
		const struct sdrm_crtc_helper_funcs *ec_helper_funcs);
int sdrm_remove_crtc(struct sdrm_crtc *);
int sdrm_init_drm(const char *name, struct platform_device *pdev);
int sdrm_exit_drm(const char *name);

int sdrm_crtc_vblank_get(struct sdrm_crtc *sdrm_crtc);
void sdrm_crtc_vblank_put(struct sdrm_crtc *sdrm_crtc);
void sdrm_handle_vblank(struct sdrm_crtc *sdrm_crtc);

/*
 * sdrm drm buffer entry structure.
 *
 * @paddr: physical address of allocated memory.
 * @vaddr: kernel virtual address of allocated memory.
 * @size: size of allocated memory.
 */
struct sdrm_buf_entry {
	dma_addr_t paddr;
	void __iomem *vaddr;
	unsigned int size;

	dma_addr_t r_paddr;
	void __iomem *r_vaddr;
	unsigned int r_size;
};

/* get physical memory information of a drm framebuffer. */
struct sdrm_buf_entry *sdrm_fb_get_buf(struct drm_framebuffer *fb);

struct sdrm_encoder;
struct sdrm_encoder *sdrm_add_encoder(const char *drmname,
		struct drm_encoder *encoder, struct module *owner);
int sdrm_remove_encoder(struct sdrm_encoder *);

struct sdrm_connector;
struct sdrm_connector *sdrm_add_connector(const char *drmname,
		struct drm_connector *connector, struct module *owner);
int sdrm_remove_connector(struct sdrm_connector *);

#endif /* __DRM_SDRM_H */
