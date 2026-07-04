// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_gem.c
 *
 * Copyright (C) 2019 Samsung Electronics Co.Ltd
 * Authors:
 *	Jiun Yu <jiun.yu@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Mainline bring-up note (felix outer-screen port):
 *   The original AOSP driver backed every GEM buffer with the ION/dma-heap
 *   "system" heap and mapped it through the DPU SYSMMU (iommu_client). The
 *   mainline gs201 port has no sysmmu_dpu node, so buffers must be physically
 *   contiguous and the DPU DMA consumes the physical address directly. This
 *   file is reworked to allocate write-combine coherent (CMA-backed) memory
 *   via dma_alloc_wc(); exynos_gem_obj->dma_addr is the DPU-visible scanout
 *   address. Imported dma-bufs (prime) still go through the sg_table path.
 *   The struct exynos_drm_gem and all ->dma_addr/->vaddr consumers are
 *   unchanged, so fb/dqe scanout code needs no edits.
 */
#define pr_fmt(fmt)  "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/mm_types.h>
#include <linux/dma-mapping.h>

#include "exynos_drm_dsim.h"
#include "exynos_drm_gem.h"

void exynos_drm_gem_free_object(struct drm_gem_object *obj)
{
	struct exynos_drm_gem *exynos_gem_obj = to_exynos_gem(obj);

	if (obj->import_attach) {
		/* imported dma-buf: undo the vmap + sg import */
		if (exynos_gem_obj->vaddr) {
			struct iosys_map map =
				IOSYS_MAP_INIT_VADDR(exynos_gem_obj->vaddr);
			dma_buf_vunmap(obj->import_attach->dmabuf, &map);
		}
		drm_prime_gem_destroy(obj, exynos_gem_obj->sgt);
	} else if (exynos_gem_obj->vaddr) {
		/* locally allocated CMA/coherent buffer */
		dma_free_wc(obj->dev->dev, obj->size, exynos_gem_obj->vaddr,
			    exynos_gem_obj->dma_addr);
	}

	drm_gem_object_release(&exynos_gem_obj->base);
	kfree(exynos_gem_obj);
}

void *exynos_drm_gem_get_vaddr(struct exynos_drm_gem *exynos_gem_obj)
{
	struct dma_buf_attachment *attach = exynos_gem_obj->base.import_attach;
	struct iosys_map map;
	int ret;

	/* local CMA buffers already carry a kernel mapping from dma_alloc_wc */
	if (exynos_gem_obj->vaddr)
		return exynos_gem_obj->vaddr;

	if (WARN_ON(!attach))
		return NULL;

	ret = dma_buf_vmap(attach->dmabuf, &map);
	if (ret) {
		pr_err("Failed to map virtual address\n");
		return NULL;
	}

	exynos_gem_obj->vaddr = map.vaddr;
	pr_debug("mapped vaddr: %pK\n", exynos_gem_obj->vaddr);

	return exynos_gem_obj->vaddr;
}

static int exynos_drm_gem_object_mmap(struct drm_gem_object *obj,
				      struct vm_area_struct *vma)
{
	struct exynos_drm_gem *exynos_gem_obj = to_exynos_gem(obj);
	int ret;

	/*
	 * Turn the DRM fake buffer offset into a real page offset and clear the
	 * VM_PFNMAP flag that drm_gem_mmap() set (mirrors drm_gem_dma_mmap()).
	 */
	vma->vm_pgoff -= drm_vma_node_start(&obj->vma_node);
	vm_flags_mod(vma, VM_DONTEXPAND, VM_PFNMAP);

	if (obj->import_attach)
		ret = dma_buf_mmap(obj->import_attach->dmabuf, vma, 0);
	else
		ret = dma_mmap_wc(obj->dev->dev, vma, exynos_gem_obj->vaddr,
				  exynos_gem_obj->dma_addr,
				  vma->vm_end - vma->vm_start);
	if (ret)
		pr_err("Failed to mmap gem buffer: %d\n", ret);

	return ret;
}

static const struct vm_operations_struct exynos_drm_gem_vm_ops = {
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct drm_gem_object_funcs exynos_drm_gem_object_funcs = {
	.free = exynos_drm_gem_free_object,
	.mmap = exynos_drm_gem_object_mmap,
	.vm_ops = &exynos_drm_gem_vm_ops,
};

struct exynos_drm_gem *exynos_drm_gem_alloc(struct drm_device *dev,
					    size_t size, unsigned int flags)
{
	struct exynos_drm_gem *exynos_gem_obj;

	exynos_gem_obj = kzalloc(sizeof(*exynos_gem_obj), GFP_KERNEL);
	if (!exynos_gem_obj)
		return ERR_PTR(-ENOMEM);

	exynos_gem_obj->flags = flags;
	exynos_gem_obj->base.funcs = &exynos_drm_gem_object_funcs;

	/* no need to release initialized private gem object */
	drm_gem_private_object_init(dev, &exynos_gem_obj->base, size);

	pr_debug("allocated %zu bytes with flags %#x\n", size, flags);

	return exynos_gem_obj;
}

struct drm_gem_object *
exynos_drm_gem_prime_import_sg_table(struct drm_device *dev,
				     struct dma_buf_attachment *attach,
				     struct sg_table *sgt)
{
	const unsigned long size = attach->dmabuf->size;
	struct exynos_drm_gem *exynos_gem_obj =
		exynos_drm_gem_alloc(dev, size, 0);
	dma_addr_t dma_addr;

	if (IS_ERR(exynos_gem_obj))
		return ERR_CAST(exynos_gem_obj);

	exynos_gem_obj->sgt = sgt;
	dma_addr = sg_dma_address(sgt->sgl);
	if (IS_ERR_VALUE(dma_addr)) {
		pr_err("Failed to allocate IOVM (%lld)\n", dma_addr);
		kfree(exynos_gem_obj);
		return ERR_PTR(dma_addr);
	}

	exynos_gem_obj->dma_addr = dma_addr;

	pr_debug("mapped dma_addr: 0x%llx\n", exynos_gem_obj->dma_addr);

	return &exynos_gem_obj->base;
}

static int exynos_drm_gem_create(struct drm_device *dev, struct drm_file *filep,
				 size_t size, unsigned int flags,
				 unsigned int *gem_handle)
{
	struct exynos_drm_gem *exynos_gem_obj;
	void *vaddr;
	dma_addr_t dma_addr;
	int ret;

	if (flags & EXYNOS_DRM_GEM_FLAG_COLORMAP) {
		pr_err("unsupported color map gem creation\n");
		return -EINVAL;
	}

	exynos_gem_obj = exynos_drm_gem_alloc(dev, size, flags);
	if (IS_ERR(exynos_gem_obj))
		return PTR_ERR(exynos_gem_obj);

	vaddr = dma_alloc_wc(dev->dev, size, &dma_addr,
			     GFP_KERNEL | __GFP_NOWARN);
	if (!vaddr) {
		pr_err("Failed to allocate %#zx bytes of contiguous memory\n",
		       size);
		/* nothing mapped yet -> free_object() won't dma_free */
		drm_gem_object_put(&exynos_gem_obj->base);
		return -ENOMEM;
	}

	exynos_gem_obj->vaddr = vaddr;
	exynos_gem_obj->dma_addr = dma_addr;

	ret = drm_gem_handle_create(filep, &exynos_gem_obj->base, gem_handle);
	/* handle now holds a ref (on success); drop our creation ref either way.
	 * On failure the last put frees the buffer via free_object(). */
	drm_gem_object_put(&exynos_gem_obj->base);
	if (ret)
		pr_err("Failed to create a handle of GEM\n");

	return ret;
}

int exynos_drm_gem_dumb_create(struct drm_file *file_priv,
			       struct drm_device *dev,
			       struct drm_mode_create_dumb *args)
{
	unsigned int handle;
	int ret;

	args->pitch = args->width * DIV_ROUND_UP(args->bpp, 8);
	args->size = PAGE_ALIGN(args->pitch * args->height);

	ret = exynos_drm_gem_create(dev, file_priv, args->size,
				    EXYNOS_DRM_GEM_FLAG_DUMB_BUF, &handle);
	if (ret) {
		pr_err("Failed to create dumb of %llu bytes (%ux%u/%ubpp)\n",
			  args->size, args->width, args->height, args->bpp);
		return ret;
	}

	args->handle = handle;

	return 0;
}

struct drm_gem_object *exynos_drm_gem_prime_import(struct drm_device *dev,
						   struct dma_buf *dma_buf)
{
	/* No DPU IOMMU on mainline: attach against the DRM platform device and
	 * consume the direct-mapped physical address for scanout. */
	return drm_gem_prime_import(dev, dma_buf);
}

struct drm_gem_object *exynos_drm_gem_fd_to_obj(struct drm_device *dev, int val)
{
	struct dma_buf *dma_buf;
	struct drm_gem_object *obj;

	dma_buf = dma_buf_get(val);
	if (IS_ERR(dma_buf)) {
		pr_err("failed to get dma buf\n");
		return NULL;
	}
	obj = exynos_drm_gem_prime_import(dev, dma_buf);
	dma_buf_put(dma_buf);

	return obj;
}

int exynos_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	/* drm_gem_mmap() looks up the object by fake offset and dispatches to
	 * obj->funcs->mmap (exynos_drm_gem_object_mmap). */
	ret = drm_gem_mmap(filp, vma);
	if (ret < 0) {
		pr_err("Failed to mmap with offset %lu.\n", vma->vm_pgoff);
		return ret;
	}

	pr_debug("mmaped the offset %lu of size %lu to %#lx\n",
			 vma->vm_pgoff, vma->vm_end - vma->vm_start,
			 vma->vm_start);

	return 0;
}

static int exynos_drm_gem_offset(struct drm_device *dev, struct drm_file *filep,
				 unsigned int handle, uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret = 0;

	obj = drm_gem_object_lookup(filep, handle);
	if (!obj) {
		pr_err("Failed to lookup gem object from handle %u.\n",
			  handle);
		return -EINVAL;
	}

	ret = drm_gem_create_mmap_offset(obj);
	if (ret) {
		pr_err("Failed to create mmap fake offset for handle %u\n",
			  handle);
		goto out;
	}

	*offset = drm_vma_node_offset_addr(&obj->vma_node);
out:
	drm_gem_object_put(obj);

	return ret;
}

int exynos_drm_gem_dumb_map_offset(struct drm_file *file_priv,
				   struct drm_device *dev, uint32_t handle,
				   uint64_t *offset)
{
	int ret;

	ret = exynos_drm_gem_offset(dev, file_priv, handle, offset);
	if (!ret)
		pr_debug("obtained fake mmap offset %llu from handle %u\n",
			      *offset, handle);

	return ret;
}
