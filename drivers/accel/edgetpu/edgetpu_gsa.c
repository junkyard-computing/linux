// SPDX-License-Identifier: GPL-2.0
/*
 * GSA secure-firmware glue for the gs201 Edge TPU accel driver.
 *
 * See edgetpu_gsa.h for the rationale. This is a thin C shim around the GSA
 * TPU firmware-management API; the Rust "edgetpu" module orchestrates it.
 *
 * Copyright 2026 Junkyard Computing.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include <linux/gsa/gsa_tpu.h>

#include "edgetpu_gsa.h"

/* The signed firmware header is always 4K (MOBILE_FW_HEADER_SIZE). */
#define EDGETPU_FW_HEADER_SIZE SZ_4K

/*
 * felix GSA S2MPU (s2mpu_gsa@17c60000, gs201 == S2MPU_V1). Left enforcing with
 * a blocking table by the bootloader (off-at-boot); no pKVM s2mpu module claims
 * it in this build, so GSA cannot DMA the firmware header/body and rejects
 * LOAD_TPU_FW_IMG with INVALID_ARGS. Clearing CTRL0.ENABLE (offset 0, bit 0)
 * puts it in allow-all — the hardware does no table walk and passes all traffic.
 */
#define GSA_S2MPU_BASE		0x17c60000UL
#define S2MPU_MMIO_SIZE		0x10000
#define S2MPU_REG_NS_CTRL0	0x0

/*
 * Put a gs201 (S2MPU_V1) S2MPU into allow-all by clearing CTRL0.ENABLE. Safe as
 * a raw host write: no pkvm-s2mpu module has registered these MMIO ranges with
 * EL2, so under kvm-arm.mode=protected they stay host-owned and the store does
 * not trap.
 */
/*
 * Enable TPU IO-coherency (janeiro `janeiro_mmu_set_shareability`). Without it
 * the TPU's memory accesses are non-shareable and do not snoop the CPU caches,
 * so the firmware reads stale (zero) cache lines for the AP-written KCI command
 * and errors instead of responding. The register block is the TPU SysReg at
 * `edgetpu,shareability` (gs201: 0x1cc20000); write SHAREABLE_WRITE(1<<13) |
 * SHAREABLE_READ(1<<12) | INNER_SHAREABLE(1) to offset 0x700.
 */
#define EDGETPU_SYSREG_BASE		0x1cc20000UL
#define EDGETPU_SYSREG_TPU_SHAREABILITY	0x700
#define EDGETPU_TPU_SHAREABILITY_VAL	((1u << 13) | (1u << 12) | 1u)

void edgetpu_enable_coherency(void)
{
	void __iomem *base = ioremap(EDGETPU_SYSREG_BASE, S2MPU_MMIO_SIZE);

	if (!base) {
		pr_warn("edgetpu_gsa: failed to map TPU sysreg@%lx\n",
			EDGETPU_SYSREG_BASE);
		return;
	}
	writel(EDGETPU_TPU_SHAREABILITY_VAL,
	       base + EDGETPU_SYSREG_TPU_SHAREABILITY);
	pr_info("edgetpu_gsa: TPU IO-coherency enabled (shareability=%#x)\n",
		EDGETPU_TPU_SHAREABILITY_VAL);
	iounmap(base);
}
EXPORT_SYMBOL_GPL(edgetpu_enable_coherency);

void edgetpu_s2mpu_allow_all(phys_addr_t base_phys)
{
	void __iomem *base = ioremap(base_phys, S2MPU_MMIO_SIZE);

	if (!base) {
		pr_warn("edgetpu_gsa: failed to map s2mpu@%pa\n", &base_phys);
		return;
	}
	writel(0, base + S2MPU_REG_NS_CTRL0);
	pr_info("edgetpu_gsa: S2MPU@%pa allow-all (CTRL0=0)\n", &base_phys);
	iounmap(base);
}
EXPORT_SYMBOL_GPL(edgetpu_s2mpu_allow_all);

struct device *edgetpu_gsa_get(struct device *etdev)
{
	struct device_node *np;
	struct platform_device *gsa_pdev;

	if (!etdev || !etdev->of_node)
		return NULL;

	np = of_parse_phandle(etdev->of_node, "gsa-device", 0);
	if (!np) {
		dev_warn(etdev, "edgetpu_gsa: no gsa-device phandle\n");
		return NULL;
	}

	gsa_pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!gsa_pdev) {
		dev_warn(etdev, "edgetpu_gsa: gsa device not found/probed\n");
		return NULL;
	}

	/* Reference on gsa_pdev is held; released via put_device() below. */
	return &gsa_pdev->dev;
}
EXPORT_SYMBOL_GPL(edgetpu_gsa_get);

void edgetpu_gsa_put(struct device *gsa)
{
	if (gsa)
		put_device(gsa);
}
EXPORT_SYMBOL_GPL(edgetpu_gsa_put);

int edgetpu_fw_copy_body(phys_addr_t carveout_phys, size_t carveout_sz,
			 const void *body, size_t body_len)
{
	void *va;

	if (body_len > carveout_sz)
		return -EINVAL;

	va = memremap(carveout_phys, carveout_sz, MEMREMAP_WC);
	if (!va)
		return -ENOMEM;

	memcpy(va, body, body_len);
	/* Ensure the body lands in DRAM before GSA reads/authenticates it. */
	wmb();
	memunmap(va);
	return 0;
}
EXPORT_SYMBOL_GPL(edgetpu_fw_copy_body);

int edgetpu_gsa_load_fw(struct device *gsa, const void *hdr4k,
			phys_addr_t body_phys)
{
	dma_addr_t hdr_dma;
	void *hdr_va;
	u64 saved_mask;
	int ret;

	if (!gsa)
		return -ENODEV;

	/*
	 * GSA can only reach the low DRAM bank (below 4 GiB). felix splits its
	 * 12 GiB as a small <4 GiB bank plus a high bank at 0x8_80000000+, and
	 * the GSA driver sets a 36-bit coherent mask — so dma_alloc_coherent()
	 * happily lands the signed 4K header in the high bank, which GSA then
	 * rejects with GSA_MB_ERR_INVALID_ARGS. Constrain this one allocation to
	 * 32 bits so the header lands where GSA can DMA it, then restore.
	 */
	saved_mask = gsa->coherent_dma_mask;
	ret = dma_set_coherent_mask(gsa, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	hdr_va = dma_alloc_coherent(gsa, EDGETPU_FW_HEADER_SIZE, &hdr_dma,
				    GFP_KERNEL);
	if (!hdr_va) {
		ret = -ENOMEM;
		goto out_restore;
	}

	memcpy(hdr_va, hdr4k, EDGETPU_FW_HEADER_SIZE);

	dev_info(gsa, "edgetpu_gsa: load fw hdr_dma=%pad body_phys=%pa\n",
		 &hdr_dma, &body_phys);

	/* Open the GSA S2MPU so GSA can DMA the header + body. */
	edgetpu_s2mpu_allow_all(GSA_S2MPU_BASE);

	ret = gsa_load_tpu_fw_image(gsa, hdr_dma, body_phys);

	dma_free_coherent(gsa, EDGETPU_FW_HEADER_SIZE, hdr_va, hdr_dma);

out_restore:
	dma_set_coherent_mask(gsa, saved_mask);
	return ret;
}
EXPORT_SYMBOL_GPL(edgetpu_gsa_load_fw);

int edgetpu_gsa_send_cmd(struct device *gsa, u32 cmd)
{
	if (!gsa)
		return -ENODEV;
	return gsa_send_tpu_cmd(gsa, (enum gsa_tpu_cmd)cmd);
}
EXPORT_SYMBOL_GPL(edgetpu_gsa_send_cmd);

int edgetpu_gsa_unload(struct device *gsa)
{
	if (!gsa)
		return -ENODEV;
	return gsa_unload_tpu_fw_image(gsa);
}
EXPORT_SYMBOL_GPL(edgetpu_gsa_unload);

/*
 * Read/write a physically-contiguous region (the TPU firmware carveout, which
 * is no-map so it has no kernel linear alias). Used for the KCI/VII cmd/resp
 * queues, FW_INFO buffer, and the BO heap — all in the carveout "remapped data
 * region" the TPU reaches through the GSA-configured instruction remap (no
 * IOMMU/SysMMU mapping involved).
 *
 * Runtime inference rings the VII mailbox thousands of times per model forward,
 * two of these accesses per submit (write the command element, read the
 * response) plus BO reads. A memremap()+memunmap() per access was ~half the
 * per-submit cost measured against the AOSP chardev (which mmaps the ring once
 * from userspace). So the whole data region is memremap'd ONCE at probe
 * (edgetpu_data_init) and every access memcpy's through that persistent mapping;
 * the per-call memremap is kept only as a fallback for addresses outside the
 * window or before init.
 */
#define EDGETPU_DATA_PHYS	0x93100000UL /* KCI queues + VII queues + BO heap */
#define EDGETPU_DATA_SIZE	0x100000     /* up to secure_data_start @0x93200000 */

static void *edgetpu_data_va;

int edgetpu_data_init(void)
{
	if (edgetpu_data_va)
		return 0;
	edgetpu_data_va = memremap(EDGETPU_DATA_PHYS, EDGETPU_DATA_SIZE, MEMREMAP_WC);
	if (!edgetpu_data_va) {
		pr_err("edgetpu_gsa: failed to map carveout data region@%lx\n",
		       EDGETPU_DATA_PHYS);
		return -ENOMEM;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(edgetpu_data_init);

/* Persistent kernel VA for @phys if it lies within the mapped data window. */
static void *edgetpu_data_ptr(phys_addr_t phys, size_t len)
{
	if (edgetpu_data_va && phys >= EDGETPU_DATA_PHYS &&
	    phys + len <= EDGETPU_DATA_PHYS + EDGETPU_DATA_SIZE)
		return edgetpu_data_va + (phys - EDGETPU_DATA_PHYS);
	return NULL;
}

int edgetpu_mem_write(phys_addr_t phys, const void *src, size_t len)
{
	void *va = edgetpu_data_ptr(phys, len);

	if (va) {
		memcpy(va, src, len);
		/* Ensure the write lands before the caller rings the doorbell. */
		wmb();
		return 0;
	}
	va = memremap(phys, len, MEMREMAP_WC);
	if (!va)
		return -ENOMEM;
	memcpy(va, src, len);
	wmb();
	memunmap(va);
	return 0;
}
EXPORT_SYMBOL_GPL(edgetpu_mem_write);

int edgetpu_mem_read(phys_addr_t phys, void *dst, size_t len)
{
	void *va = edgetpu_data_ptr(phys, len);

	if (va) {
		/* Observe the TPU's writes (e.g. the response element). */
		rmb();
		memcpy(dst, va, len);
		return 0;
	}
	va = memremap(phys, len, MEMREMAP_WC);
	if (!va)
		return -ENOMEM;
	rmb();
	memcpy(dst, va, len);
	memunmap(va);
	return 0;
}
EXPORT_SYMBOL_GPL(edgetpu_mem_read);

/*
 * Persistent TPU CSR mapping for the runtime (post-probe) mailbox path.
 *
 * The Rust driver's `IoMem` mapping of the main TPU CSR block only lives for
 * the duration of probe (it drives the boot-time power/PSM/KCI sequence). The
 * VII inference mailbox, by contrast, is rung from ioctls long after probe, so
 * we keep a private module-lifetime `ioremap()` of the same window and expose
 * simple 32-bit accessors. Plain `ioremap()` (no request_mem_region) coexists
 * with the Rust driver's transient exclusive mapping of the same physical
 * range. Set up once from the single-threaded probe path via edgetpu_csr_init().
 */
#define EDGETPU_TPU_CSR_PHYS	0x1ce00000UL
#define EDGETPU_TPU_CSR_SIZE	0x200000

static void __iomem *edgetpu_tpu_csr;

int edgetpu_csr_init(void)
{
	if (edgetpu_tpu_csr)
		return 0;
	edgetpu_tpu_csr = ioremap(EDGETPU_TPU_CSR_PHYS, EDGETPU_TPU_CSR_SIZE);
	if (!edgetpu_tpu_csr) {
		pr_err("edgetpu_gsa: failed to map TPU CSR block@%lx\n",
		       EDGETPU_TPU_CSR_PHYS);
		return -ENOMEM;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(edgetpu_csr_init);

u32 edgetpu_csr_read32(u32 off)
{
	if (!edgetpu_tpu_csr || off >= EDGETPU_TPU_CSR_SIZE)
		return 0;
	return readl(edgetpu_tpu_csr + off);
}
EXPORT_SYMBOL_GPL(edgetpu_csr_read32);

void edgetpu_csr_write32(u32 off, u32 val)
{
	if (!edgetpu_tpu_csr || off >= EDGETPU_TPU_CSR_SIZE)
		return;
	writel(val, edgetpu_tpu_csr + off);
}
EXPORT_SYMBOL_GPL(edgetpu_csr_write32);

/*
 * Raise the MIF (memory interface) and INT (interconnect) buses to their top
 * frequencies *while a client is attached*.
 *
 * The firmware DMAs every VII mailbox op's executable, weights, scratch, and I/O
 * through memory, but that traffic is invisible to the exynos-bus devfreq
 * governor (its gs201-ppc counters watch only CCI/CPU ports), so it idles the
 * buses at their floor (MIF ~421 MHz of 3172, INT 100 MHz of 533 — ~13%) and
 * every op runs ~5x slower. The AOSP driver requests these buses via
 * exynos_pm_qos during inference; the mainline GPU does the equivalent by voting
 * bandwidth through the interconnect framework, load-gated. We do the same with
 * a dev_pm_qos MIN_FREQUENCY request on each exynos-bus devfreq, toggled by a
 * client refcount (raise on first open, drop on last close) so an idle TPU
 * doesn't pin DRAM at max. Values in kHz. Best-effort: if a bus isn't ready the
 * TPU still runs, just slower.
 */
#include <linux/devfreq.h>
#include <linux/pm_qos.h>

#define EDGETPU_MIF_MAX_KHZ	3172000
#define EDGETPU_INT_MAX_KHZ	533000

static struct dev_pm_qos_request edgetpu_mif_qos;
static struct dev_pm_qos_request edgetpu_int_qos;
static atomic_t edgetpu_bus_users = ATOMIC_INIT(0);

/* Add an inactive (MIN_FREQUENCY 0) qos request on a bus's exynos-bus devfreq. */
static int edgetpu_bus_add(const char *name, struct dev_pm_qos_request *req)
{
	struct device_node *np;
	struct devfreq *df;
	int ret;

	/*
	 * These requests are file-static but probe can run more than once: on an
	 * EPROBE_DEFER retry, or on a module reload while edgetpu_gsa itself
	 * stays resident. Re-adding an already-active request trips the WARN in
	 * __dev_pm_qos_add_request() ("called for already added request") and
	 * corrupts the qos list, so treat that as success and keep the existing
	 * request.
	 */
	if (dev_pm_qos_request_active(req))
		return 0;

	np = of_find_node_by_name(NULL, name);
	if (!np)
		return -ENODEV;
	df = devfreq_get_devfreq_by_node(np);
	of_node_put(np);
	if (IS_ERR(df))
		return PTR_ERR(df);
	/* devfreq aggregates MIN_FREQUENCY qos on the bus's parent device (kHz). */
	ret = dev_pm_qos_add_request(df->dev.parent, req, DEV_PM_QOS_MIN_FREQUENCY, 0);
	return ret < 0 ? ret : 0;
}

int edgetpu_bus_qos_init(void)
{
	int ret;

	ret = edgetpu_bus_add("bus-mif", &edgetpu_mif_qos);
	if (ret)
		pr_warn("edgetpu_gsa: MIF bus qos unavailable (%d); inference will be slow\n", ret);
	ret = edgetpu_bus_add("bus-int", &edgetpu_int_qos);
	if (ret)
		pr_warn("edgetpu_gsa: INT bus qos unavailable (%d); inference will be slow\n", ret);
	return 0;
}
EXPORT_SYMBOL_GPL(edgetpu_bus_qos_init);

/* First attached client raises MIF/INT to max; subsequent clients just refcount. */
void edgetpu_bus_qos_get(void)
{
	if (atomic_inc_return(&edgetpu_bus_users) != 1)
		return;
	if (dev_pm_qos_request_active(&edgetpu_mif_qos))
		dev_pm_qos_update_request(&edgetpu_mif_qos, EDGETPU_MIF_MAX_KHZ);
	if (dev_pm_qos_request_active(&edgetpu_int_qos))
		dev_pm_qos_update_request(&edgetpu_int_qos, EDGETPU_INT_MAX_KHZ);
}
EXPORT_SYMBOL_GPL(edgetpu_bus_qos_get);

/* Last client detaching drops the buses back to the governor's floor. */
void edgetpu_bus_qos_put(void)
{
	if (atomic_dec_if_positive(&edgetpu_bus_users) != 0)
		return;
	if (dev_pm_qos_request_active(&edgetpu_mif_qos))
		dev_pm_qos_update_request(&edgetpu_mif_qos, 0);
	if (dev_pm_qos_request_active(&edgetpu_int_qos))
		dev_pm_qos_update_request(&edgetpu_int_qos, 0);
}
EXPORT_SYMBOL_GPL(edgetpu_bus_qos_put);

/*
 * Runtime TPU DVFS knob (`/sys/kernel/debug/edgetpu/tpu_clk_hz`, rw).
 *
 * The Rust driver brings the TPU up at the DVFS *floor* (UUD/226 MHz) so the
 * firmware start doesn't trip the IF-PMIC UVLO at boot (see driver.rs). Inference
 * wants a higher state, so expose the ACPM "tpu" clock rate to userspace: write
 * a frequency in Hz to raise/lower it, read back the current rate. This is a
 * second consumer handle on the same clock the Rust `Clk` holds — the common
 * clock framework shares the underlying clk_core, so clk_set_rate() here changes
 * the one HW rate. We don't prepare/enable it (the Rust handle keeps the rail
 * powered); we only set/get the rate.
 *
 * Because the boot clock is the safe floor, sweeping this knob up to NOM is
 * self-recovering: a rate that trips UVLO just resets the phone, which boots
 * again at the floor. Diagnostic-only; failures here never affect bring-up.
 */
#include <linux/clk.h>
#include <linux/debugfs.h>

static struct clk *edgetpu_dvfs_clk;
static struct dentry *edgetpu_dvfs_dir;
/*
 * Last rate we successfully asked for, 0 until the first write.
 *
 * We cannot use clk_get_rate() to report the state of this clock. The common
 * clock framework caches core->rate and refreshes it from the provider's
 * ->recalc_rate, which for the ACPM clocks is acpm_clk_recalc_rate() ->
 * ACPM dvfs.get_rate(). For the "tpu" domain that query does not reflect a
 * completed set: after raising the rate to NOM the HW is measurably at NOM
 * (0.240 vs 0.662 ms per mailbox op) while ACPM keeps reporting the boot floor,
 * so the cache stays at 226 MHz forever.
 *
 * That single stale value produced two bugs, one of them dangerous:
 *
 *   - the knob read back 226000000 no matter the real rate; and
 *   - clk_set_rate() bails out early when the requested rate equals the cached
 *     rate ("nothing to do"), so `echo 226000000` was a SILENT NO-OP whenever
 *     the part was actually at NOM. Anyone lowering the clock that way was left
 *     running at NOM — exactly the state the boot-time UVLO fix exists to avoid.
 *
 * So track what we asked for and report that, and force the set past the stale
 * cache. Fixing this here rather than in clk-acpm.c keeps the workaround next to
 * the evidence and out of a clock driver shared with every other ACPM consumer.
 */
static unsigned long edgetpu_dvfs_req_hz;

static int edgetpu_dvfs_get(void *data, u64 *val)
{
	if (!edgetpu_dvfs_clk)
		return -ENODEV;
	/* Report what we last asked for; fall back to the CCF cache before any
	 * write, when it is still the rate the Rust driver set at bring-up.
	 */
	*val = edgetpu_dvfs_req_hz ?: clk_get_rate(edgetpu_dvfs_clk);
	return 0;
}

static int edgetpu_dvfs_set(void *data, u64 val)
{
	unsigned long rate = (unsigned long)val;
	int ret;

	if (!edgetpu_dvfs_clk)
		return -ENODEV;

	/*
	 * Defeat the early-bail described above: if the (possibly stale) cached
	 * rate already equals the target, nudge to a neighbouring rate first so
	 * the provider's ->set_rate actually runs. ACPM rounds to the nearest
	 * supported DVFS point, so +1 Hz lands on a real neighbour rather than
	 * an illegal rate, and the second call then pins the target.
	 */
	if (rate == clk_get_rate(edgetpu_dvfs_clk)) {
		ret = clk_set_rate(edgetpu_dvfs_clk, rate + 1);
		if (ret)
			return ret;
	}

	ret = clk_set_rate(edgetpu_dvfs_clk, rate);
	if (ret)
		return ret;

	edgetpu_dvfs_req_hz = rate;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(edgetpu_dvfs_fops, edgetpu_dvfs_get, edgetpu_dvfs_set,
			 "%llu\n");

void edgetpu_dvfs_debugfs_init(struct device *dev)
{
	if (edgetpu_dvfs_clk)
		return;
	edgetpu_dvfs_clk = clk_get(dev, "tpu");
	if (IS_ERR(edgetpu_dvfs_clk)) {
		dev_warn(dev, "edgetpu_gsa: DVFS knob: clk_get(tpu) failed (%ld)\n",
			 PTR_ERR(edgetpu_dvfs_clk));
		edgetpu_dvfs_clk = NULL;
		return;
	}
	edgetpu_dvfs_dir = debugfs_create_dir("edgetpu", NULL);
	debugfs_create_file_unsafe("tpu_clk_hz", 0644, edgetpu_dvfs_dir, NULL,
				   &edgetpu_dvfs_fops);
	dev_info(dev, "edgetpu_gsa: DVFS knob at /sys/kernel/debug/edgetpu/tpu_clk_hz (now %lu Hz)\n",
		 clk_get_rate(edgetpu_dvfs_clk));
}
EXPORT_SYMBOL_GPL(edgetpu_dvfs_debugfs_init);

static void __exit edgetpu_gsa_exit(void)
{
	debugfs_remove_recursive(edgetpu_dvfs_dir);
	if (edgetpu_dvfs_clk)
		clk_put(edgetpu_dvfs_clk);
	if (dev_pm_qos_request_active(&edgetpu_mif_qos))
		dev_pm_qos_remove_request(&edgetpu_mif_qos);
	if (dev_pm_qos_request_active(&edgetpu_int_qos))
		dev_pm_qos_remove_request(&edgetpu_int_qos);
	if (edgetpu_tpu_csr) {
		iounmap(edgetpu_tpu_csr);
		edgetpu_tpu_csr = NULL;
	}
	if (edgetpu_data_va) {
		memunmap(edgetpu_data_va);
		edgetpu_data_va = NULL;
	}
}
module_exit(edgetpu_gsa_exit);

/*
 * TPU SysMMU mapping. The VII firmware validates inference-buffer addresses
 * against the non-secure device window (the DT `dma-window` at 0x18000000+),
 * so the driver maps each carveout-backed buffer object into the TPU's IOMMU
 * domain at a chosen IOVA and hands the firmware that IOVA. The domain is the
 * device's default (unmanaged) domain, attached by the IOMMU core via the
 * `iommus = <&sysmmu_tpu>` DT link. @dev is the edgetpu platform device.
 */
int edgetpu_iommu_map(struct device *dev, u64 iova, phys_addr_t paddr, size_t size)
{
	struct iommu_domain *dom = iommu_get_domain_for_dev(dev);

	if (!dom) {
		dev_err(dev, "edgetpu: no IOMMU domain (sysmmu not attached?)\n");
		return -ENODEV;
	}
	return iommu_map(dom, iova, paddr, size,
			 IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE, GFP_KERNEL);
}
EXPORT_SYMBOL_GPL(edgetpu_iommu_map);

void edgetpu_iommu_unmap(struct device *dev, u64 iova, size_t size)
{
	struct iommu_domain *dom = iommu_get_domain_for_dev(dev);

	if (dom)
		iommu_unmap(dom, iova, size);
}
EXPORT_SYMBOL_GPL(edgetpu_iommu_unmap);

/*
 * System-memory buffer objects (see edgetpu_gsa.h). Back a BO with
 * physically-contiguous kernel pages and SysMMU-map them at @iova, so BOs are no
 * longer bounded by the ~704 KiB carveout. @size is page-aligned by the caller.
 * The TPU is IO-coherent, so the CPU's cacheable pages are snooped — no cache
 * maintenance needed, only a barrier around host access (write/read helpers).
 */
void *edgetpu_bo_sysmem_alloc(struct device *dev, u64 iova, size_t size)
{
	unsigned int order = get_order(size);
	struct page *pg;
	void *va;
	int ret;

	pg = alloc_pages(GFP_KERNEL | __GFP_ZERO, order);
	if (!pg) {
		dev_err(dev, "edgetpu: BO alloc_pages(order=%u) failed\n", order);
		return NULL;
	}
	va = page_address(pg);
	/*
	 * Map exactly @size (the caller's page-aligned request), NOT the
	 * order-rounded allocation: the driver's IOVA bump advances by @size, so
	 * mapping the larger 2^order block would overlap the next BO's IOVA and
	 * iommu_map would reject it (-EADDRINUSE). The extra tail pages of the
	 * allocation are simply left unmapped. @size is a multiple of PAGE_SIZE.
	 */
	ret = edgetpu_iommu_map(dev, iova, page_to_phys(pg), size);
	if (ret) {
		dev_err(dev, "edgetpu: BO SysMMU map iova=%#llx failed (%d)\n", iova, ret);
		__free_pages(pg, order);
		return NULL;
	}
	return va;
}
EXPORT_SYMBOL_GPL(edgetpu_bo_sysmem_alloc);

void edgetpu_bo_sysmem_free(struct device *dev, void *cpu_va, u64 iova, size_t size)
{
	/* Unmap the same @size that was mapped in alloc, then free the pages. */
	edgetpu_iommu_unmap(dev, iova, size);
	if (cpu_va)
		__free_pages(virt_to_page(cpu_va), get_order(size));
}
EXPORT_SYMBOL_GPL(edgetpu_bo_sysmem_free);

void edgetpu_bo_write_bytes(void *cpu_va, u64 offset, const void *src, size_t len)
{
	memcpy((u8 *)cpu_va + offset, src, len);
	/* Ensure the write is visible before the caller rings the doorbell. */
	wmb();
}
EXPORT_SYMBOL_GPL(edgetpu_bo_write_bytes);

void edgetpu_bo_read_bytes(const void *cpu_va, u64 offset, void *dst, size_t len)
{
	/* Observe the TPU's writes (IO-coherent, so a barrier suffices). */
	rmb();
	memcpy(dst, (const u8 *)cpu_va + offset, len);
}
EXPORT_SYMBOL_GPL(edgetpu_bo_read_bytes);

MODULE_DESCRIPTION("GSA secure-firmware glue for the gs201 Edge TPU accel driver");
MODULE_LICENSE("GPL");
