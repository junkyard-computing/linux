/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GSA secure-firmware glue for the gs201 Edge TPU accel driver.
 *
 * The Rust edgetpu driver (drivers/accel/edgetpu, module "edgetpu") drives all
 * of the silicon (power, PSM, SSMT, CSRs) itself, but the secure-world plumbing
 * — resolving the "gsa-device" phandle, the DMA-coherent bounce for the signed
 * 4K header, the write-combining copy of the firmware body into the no-map
 * carveout, and the GSA authenticate/start calls — has no safe Rust
 * abstraction yet and cannot live in the built-in rust/helpers (GSA is a
 * module). It is provided here as a tiny companion module ("edgetpu_gsa") whose
 * exported symbols the Rust module links against.
 */
#ifndef __EDGETPU_GSA_H
#define __EDGETPU_GSA_H

#include <linux/device.h>
#include <linux/types.h>

/*
 * Resolve the "gsa-device" phandle on @etdev and return the GSA &struct device
 * with a reference held (release with edgetpu_gsa_put()). Returns NULL on
 * failure (no phandle / device not yet probed).
 */
struct device *edgetpu_gsa_get(struct device *etdev);
void edgetpu_gsa_put(struct device *gsa);

/*
 * Copy @body_len bytes of the firmware body into the physically-contiguous
 * no-map carveout at @carveout_phys (size @carveout_sz) via a temporary
 * write-combining mapping. Returns 0 or a negative errno.
 */
int edgetpu_fw_copy_body(phys_addr_t carveout_phys, size_t carveout_sz,
			 const void *body, size_t body_len);

/*
 * Authenticate + load the TPU firmware: bounce the signed 4K header through a
 * DMA-coherent buffer on the GSA device and call gsa_load_tpu_fw_image() with
 * that DMA address and the (already populated) carveout base @body_phys.
 * Returns 0 or a negative errno.
 */
int edgetpu_gsa_load_fw(struct device *gsa, const void *hdr4k,
			phys_addr_t body_phys);

/*
 * Issue a GSA TPU management command (values match enum gsa_tpu_cmd:
 * 0=GET_STATE, 1=START, 2=SUSPEND, 3=RESUME, 4=SHUTDOWN). Returns the new TPU
 * state (enum gsa_tpu_state, 2=RUNNING) on success or a negative errno.
 */
int edgetpu_gsa_send_cmd(struct device *gsa, u32 cmd);

/* Unlock/unload a previously loaded TPU firmware image. */
int edgetpu_gsa_unload(struct device *gsa);

/*
 * Put a gs201 S2MPU (V1) at @base_phys into allow-all (clears CTRL0.ENABLE).
 * Used for both s2mpu_gsa@17c60000 and s2mpu_tpu@1cc60000.
 */
void edgetpu_s2mpu_allow_all(phys_addr_t base_phys);

/* Enable TPU IO-coherency (writes the SysReg shareability register). */
void edgetpu_enable_coherency(void);

/*
 * Read/write a physically-contiguous no-map region (the TPU firmware carveout)
 * via a temporary write-combining mapping. Used for the KCI queues + FW_INFO
 * buffer that live in the carveout's remapped data region.
 */
int edgetpu_mem_write(phys_addr_t phys, const void *src, size_t len);
int edgetpu_mem_read(phys_addr_t phys, void *dst, size_t len);

/*
 * Module-lifetime mapping of the main TPU CSR block (0x1ce00000), used by the
 * runtime VII mailbox path which outlives the Rust driver's probe-time IoMem.
 * edgetpu_csr_init() is idempotent; call it once from probe. Offsets are from
 * the block base and bounds-checked.
 */
int edgetpu_csr_init(void);
u32 edgetpu_csr_read32(u32 off);
void edgetpu_csr_write32(u32 off, u32 val);

/*
 * Map/unmap a physically-contiguous buffer into the TPU SysMMU domain at @iova
 * (in the DT `dma-window`, 0x18000000+). The firmware reaches VII inference
 * buffers through these translations. @dev is the edgetpu platform device.
 */
int edgetpu_iommu_map(struct device *dev, u64 iova, phys_addr_t paddr, size_t size);
void edgetpu_iommu_unmap(struct device *dev, u64 iova, size_t size);

#endif /* __EDGETPU_GSA_H */
