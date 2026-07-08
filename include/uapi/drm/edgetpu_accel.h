/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Userspace ABI for the gs201 Edge TPU DRM `accel` driver.
 *
 * The driver is a thin substrate: it authenticates + runs the secure TPU
 * firmware, brings up the VII (per-context inference) mailbox, and exposes a
 * small buffer + submit surface. All DarwiNN / VII command construction (the
 * FlatBuffer "litebufs", the scratch/exec/run command sequence) lives in
 * userspace (the `finch` runtime), exactly as it does for the AOSP janeiro
 * chardev — only the transport is a DRM ioctl here instead of an mmap'd ring.
 *
 * M2 buffer model: BOs are backed by the firmware carveout's instruction-remap
 * window (the same coherent region the KCI/VII queues live in), so the firmware
 * reaches them without the TPU SysMMU. CREATE_BO returns a `tpu_va` that
 * userspace embeds in the VII command's DMA descriptors; the SysMMU-mapped
 * (arbitrary host page) path is a later milestone.
 */
#ifndef _UAPI_EDGETPU_ACCEL_H_
#define _UAPI_EDGETPU_ACCEL_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*
 * Allocate a device-accessible buffer object from the carveout and return its
 * TPU device virtual address (for VII DMA descriptors) plus a handle.
 */
struct drm_edgetpu_create_bo {
	__u64 size;	/* in:  bytes (rounded up to a page) */
	__u64 tpu_va;	/* out: TPU device VA to embed in VII descriptors */
	__u32 handle;	/* out: BO handle for BO_WRITE / BO_READ */
	__u32 pad;
};

/* Copy userspace bytes into a BO (e.g. input tensor, litebuf, executable). */
struct drm_edgetpu_bo_write {
	__u32 handle;	/* in */
	__u32 pad;
	__u64 offset;	/* in:  byte offset into the BO */
	__u64 size;	/* in:  bytes */
	__u64 user_ptr;	/* in:  source userspace address */
};

/* Copy BO bytes back out to userspace (e.g. output tensor). */
struct drm_edgetpu_bo_read {
	__u32 handle;	/* in */
	__u32 pad;
	__u64 offset;	/* in:  byte offset into the BO */
	__u64 size;	/* in:  bytes */
	__u64 user_ptr;	/* in:  destination userspace address */
};

/*
 * Submit one 48-byte VII command element to the inference mailbox and return
 * the 24-byte response. The command's DMA descriptor addresses are TPU-VAs
 * obtained from CREATE_BO; the kernel copies the element verbatim, rings the
 * doorbell and waits for the matching response.
 */
struct drm_edgetpu_submit {
	__u8  command[48];	/* in:  VII command element */
	__u8  response[24];	/* out: VII response element */
	__u32 timeout_ms;	/* in:  0 => driver default (~1s) */
	__u32 pad;
};

#define DRM_EDGETPU_CREATE_BO	0x00
#define DRM_EDGETPU_BO_WRITE	0x01
#define DRM_EDGETPU_BO_READ	0x02
#define DRM_EDGETPU_SUBMIT	0x03

/*
 * The DRM_IOCTL_EDGETPU_* values live in an enum (not #define) so that the
 * kernel's bindgen reliably emits them as Rust constants — bindgen const-folds
 * enum initialisers but skips object-like macros that expand to _IOC(...). This
 * mirrors panthor_drm.h.
 */
#define DRM_IOCTL_EDGETPU(__access, __id, __type)                              \
	DRM_IO##__access(DRM_COMMAND_BASE + DRM_EDGETPU_##__id,                 \
			 struct drm_edgetpu_##__type)

enum {
	DRM_IOCTL_EDGETPU_CREATE_BO =
		DRM_IOCTL_EDGETPU(WR, CREATE_BO, create_bo),
	DRM_IOCTL_EDGETPU_BO_WRITE =
		DRM_IOCTL_EDGETPU(W, BO_WRITE, bo_write),
	DRM_IOCTL_EDGETPU_BO_READ =
		DRM_IOCTL_EDGETPU(W, BO_READ, bo_read),
	DRM_IOCTL_EDGETPU_SUBMIT =
		DRM_IOCTL_EDGETPU(WR, SUBMIT, submit),
};

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI_EDGETPU_ACCEL_H_ */
