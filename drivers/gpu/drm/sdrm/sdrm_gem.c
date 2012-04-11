/* sdrm_gem.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Author: Inki Dae <inki.dae@samsung.com>
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
#include <drm/drm.h>

#include <drm/sdrm.h>

#include "sdrm_gem.h"

static int lowlevel_buffer_allocate(struct drm_device *dev,
		struct sdrm_buf_entry *entry)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	entry->vaddr = dma_alloc_writecombine(dev->dev, entry->size,
			(dma_addr_t *)&entry->paddr, GFP_KERNEL);
	if (!entry->vaddr) {
		DRM_ERROR("failed to allocate buffer.\n");
		return -ENOMEM;
	}

	DRM_DEBUG_KMS("allocated : vaddr(0x%x), paddr(0x%x), size(0x%x)\n",
			(unsigned int)entry->vaddr, entry->paddr, entry->size);

	return 0;
}

static void lowlevel_buffer_deallocate(struct drm_device *dev,
		struct sdrm_buf_entry *entry)
{
	DRM_DEBUG_KMS("%s.\n", __FILE__);

	if (entry->paddr && entry->vaddr && entry->size)
		dma_free_writecombine(dev->dev, entry->size, entry->vaddr,
				entry->paddr);
	else
		DRM_DEBUG_KMS("entry data is null.\n");
}

struct sdrm_buf_entry *sdrm_buf_create(struct drm_device *dev,
		unsigned int size)
{
	struct sdrm_buf_entry *entry;

	DRM_DEBUG_KMS("%s.\n", __FILE__);

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry) {
		DRM_ERROR("failed to allocate sdrm_buf_entry.\n");
		return ERR_PTR(-ENOMEM);
	}

	entry->size = size;

	/*
	 * allocate memory region with size and set the memory information
	 * to vaddr and paddr of a entry object.
	 */
	if (lowlevel_buffer_allocate(dev, entry) < 0) {
		kfree(entry);
		return ERR_PTR(-ENOMEM);
	}

	return entry;
}

void sdrm_buf_destroy(struct drm_device *dev,
		struct sdrm_buf_entry *entry)
{
	DRM_DEBUG_KMS("%s.\n", __FILE__);

	if (!entry) {
		DRM_DEBUG_KMS("entry is null.\n");
		return;
	}

	lowlevel_buffer_deallocate(dev, entry);

	kfree(entry);
	entry = NULL;
}

static unsigned int convert_to_vm_err_msg(int msg)
{
	unsigned int out_msg;

	switch (msg) {
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
		out_msg = VM_FAULT_NOPAGE;
		break;

	case -ENOMEM:
		out_msg = VM_FAULT_OOM;
		break;

	default:
		out_msg = VM_FAULT_SIGBUS;
		break;
	}

	return out_msg;
}

static unsigned int get_gem_mmap_offset(struct drm_gem_object *obj)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	return (unsigned int)obj->map_list.hash.key << PAGE_SHIFT;
}

struct sdrm_gem_obj *sdrm_gem_create(struct drm_device *dev, unsigned int size)
{
	struct sdrm_gem_obj *sdrm_gem_obj;
	struct sdrm_buf_entry *entry;
	struct drm_gem_object *obj;
	int ret;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	size = roundup(size, PAGE_SIZE);

	sdrm_gem_obj = kzalloc(sizeof(*sdrm_gem_obj), GFP_KERNEL);
	if (!sdrm_gem_obj) {
		DRM_ERROR("failed to allocate gem object.\n");
		return ERR_PTR(-ENOMEM);
	}

	/* allocate the new buffer object and memory region. */
	entry = sdrm_buf_create(dev, size);
	if (!entry) {
		kfree(sdrm_gem_obj);
		return ERR_PTR(-ENOMEM);
	}

	sdrm_gem_obj->entry = entry;

	obj = &sdrm_gem_obj->base;

	ret = drm_gem_object_init(dev, obj, size);
	if (ret < 0) {
		DRM_ERROR("failed to initailize gem object.\n");
		goto err_obj_init;
	}

	DRM_DEBUG_KMS("created file object = 0x%x\n", (unsigned int)obj->filp);

	ret = drm_gem_create_mmap_offset(obj);
	if (ret < 0) {
		DRM_ERROR("failed to allocate mmap offset.\n");
		goto err_create_mmap_offset;
	}

	return sdrm_gem_obj;

err_create_mmap_offset:
	drm_gem_object_release(obj);

err_obj_init:
	sdrm_buf_destroy(dev, sdrm_gem_obj->entry);

	kfree(sdrm_gem_obj);

	return ERR_PTR(ret);
}

struct sdrm_gem_obj *sdrm_gem_create_with_handle(struct drm_file *file_priv,
		struct drm_device *dev, unsigned int size,
		unsigned int *handle)
{
	struct sdrm_gem_obj *sdrm_gem_obj;
	struct drm_gem_object *obj;
	int ret;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	sdrm_gem_obj = sdrm_gem_create(dev, size);
	if (!sdrm_gem_obj)
		return NULL;
	obj = &sdrm_gem_obj->base;

	/*
	 * allocate a id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file_priv, obj, handle);
	if (ret)
		goto err_handle_create;

	DRM_DEBUG_KMS("gem handle = 0x%x\n", *handle);

	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(obj);

	return sdrm_gem_obj;

err_handle_create:
	sdrm_gem_free_object(obj);

	return ERR_PTR(ret);
}

static int sdrm_gem_mmap_buffer(struct file *filp,
		struct vm_area_struct *vma)
{
	struct drm_gem_object *obj = filp->private_data;
	struct sdrm_gem_obj *sdrm_gem_obj = to_sdrm_gem_obj(obj);
	struct sdrm_buf_entry *entry;
	unsigned long pfn, vm_size;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	vma->vm_flags |= (VM_IO | VM_RESERVED);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_file = filp;

	vm_size = vma->vm_end - vma->vm_start;
	/*
	 * a entry contains information to physically continuous memory
	 * allocated by user request or at framebuffer creation.
	 */
	entry = sdrm_gem_obj->entry;

	/* check if user-requested size is valid. */
	if (vm_size > entry->size)
		return -EINVAL;

	/*
	 * get page frame number to physical memory to be mapped
	 * to user space.
	 */
	pfn = sdrm_gem_obj->entry->paddr >> PAGE_SHIFT;

	DRM_DEBUG_KMS("pfn = 0x%lx\n", pfn);

	if (remap_pfn_range(vma, vma->vm_start, pfn, vm_size,
				vma->vm_page_prot)) {
		DRM_ERROR("failed to remap pfn range.\n");
		return -EAGAIN;
	}

	return 0;
}

static const struct file_operations sdrm_gem_fops = {
	.mmap = sdrm_gem_mmap_buffer,
};

int sdrm_gem_init_object(struct drm_gem_object *obj)
{
	return 0;
}

void sdrm_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct sdrm_gem_obj *sdrm_gem_obj;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	DRM_DEBUG_KMS("handle count = %d\n",
			atomic_read(&gem_obj->handle_count));

	if (gem_obj->map_list.map)
		drm_gem_free_mmap_offset(gem_obj);

	/* release file pointer to gem object. */
	drm_gem_object_release(gem_obj);

	sdrm_gem_obj = to_sdrm_gem_obj(gem_obj);

	sdrm_buf_destroy(gem_obj->dev, sdrm_gem_obj->entry);

	kfree(sdrm_gem_obj);
}

int sdrm_gem_dumb_create(struct drm_file *file_priv,
		struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	struct sdrm_gem_obj *sdrm_gem_obj;

	args->pitch = args->width * args->bpp >> 3;
	args->size = args->pitch * args->height;

	sdrm_gem_obj = sdrm_gem_create_with_handle(file_priv, dev, args->size,
							&args->handle);
	if (IS_ERR(sdrm_gem_obj))
		return PTR_ERR(sdrm_gem_obj);

	return 0;
}

int sdrm_gem_dumb_map_offset(struct drm_file *file_priv,
		struct drm_device *dev, uint32_t handle, uint64_t *offset)
{
	struct sdrm_gem_obj *sdrm_gem_obj;
	struct drm_gem_object *obj;

	mutex_lock(&dev->struct_mutex);

	obj = drm_gem_object_lookup(dev, file_priv, handle);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object.\n");
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}

	sdrm_gem_obj = to_sdrm_gem_obj(obj);

	*offset = get_gem_mmap_offset(&sdrm_gem_obj->base);

	drm_gem_object_unreference(obj);

	DRM_DEBUG_KMS("offset = 0x%lx\n", (unsigned long)*offset);

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

int sdrm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct drm_gem_object *obj = vma->vm_private_data;
	struct sdrm_gem_obj *sdrm_gem_obj = to_sdrm_gem_obj(obj);
	struct drm_device *dev = obj->dev;
	unsigned long pfn;
	pgoff_t page_offset;
	int ret;

	page_offset = ((unsigned long)vmf->virtual_address -
			vma->vm_start) >> PAGE_SHIFT;

	mutex_lock(&dev->struct_mutex);

	pfn = (sdrm_gem_obj->entry->paddr >> PAGE_SHIFT) + page_offset;

	ret = vm_insert_mixed(vma, (unsigned long)vmf->virtual_address, pfn);

	mutex_unlock(&dev->struct_mutex);

	return convert_to_vm_err_msg(ret);
}

int sdrm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	/* set vm_area_struct. */
	ret = drm_gem_mmap(filp, vma);
	if (ret < 0)
		return ret;

	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_flags |= VM_MIXEDMAP;

	return ret;
}


int sdrm_gem_dumb_destroy(struct drm_file *file_priv,
		struct drm_device *dev, unsigned int handle)
{
	return drm_gem_handle_delete(file_priv, handle);
}
