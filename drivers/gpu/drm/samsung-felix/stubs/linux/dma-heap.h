/* stub: dma-heap allocator — GEM/fb must be ported to CMA/drm_gem_dma in bring-up */
#ifndef __STUB_DMA_HEAP_H
#define __STUB_DMA_HEAP_H
#include <linux/types.h>
#include <linux/err.h>
struct dma_heap;
static inline struct dma_heap *dma_heap_find(const char *name) { return NULL; }
static inline void dma_heap_put(struct dma_heap *h) {}
static inline struct dma_buf *dma_heap_buffer_alloc(struct dma_heap *h, size_t len,
			unsigned int fd_flags, unsigned int heap_flags) { return ERR_PTR(-ENOSYS); }
static inline const char *dma_heap_get_name(struct dma_heap *h) { return ""; }
#endif
