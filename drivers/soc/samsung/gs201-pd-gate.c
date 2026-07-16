// SPDX-License-Identifier: GPL-2.0
/*
 * gs201 idle-power: gate never-used SoC power domains.
 *
 * felix's FlexPMU leaves every SoC power domain powered at boot; a headless
 * compute (no camera, no hw video codec) mainline felix never uses the
 * imaging/video cluster, so those blocks just leak. Power them off once at
 * late_initcall. The PMU CONFIGURATION register (bit0) gates the block;
 * writes go through the BL31 tensor SMC (PMU regs are EL3-write-only). Status
 * is CONFIG+4. Verified on felix: the SMC RMW is accepted and the blocks stay
 * off; isp-thermal drops ~2C once the ISP cluster is gated.
 *
 * Deliberately NOT gated: g3d/embedded_g3d (GPU, in use), hsi0 (USB/dongle),
 * hsi2 (UFS), nocl* (interconnect), aoc, tpu (edgetpu), dpu/disp (display
 * bring-up). Only the compute-irrelevant camera/imaging/video domains.
 */
#include <linux/init.h>
#include <linux/arm-smccc.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/array_size.h>

#define TENSOR_SMC_PMU_SEC_REG	0x82000504
#define TENSOR_PMUREG_RMW	2
#define GS201_PMU_BASE		0x18060000UL

static const struct { u32 cfg; const char *name; } gs201_unused_pd[] = {
	{ 0x2380, "mfc"  }, { 0x2400, "csis" }, { 0x2480, "pdp"  },
	{ 0x2500, "dns"  }, { 0x2580, "g3aa" }, { 0x2600, "ipp"  },
	{ 0x2680, "itp"  }, { 0x2700, "mcsc" }, { 0x2780, "gdc"  },
	{ 0x2800, "tnr"  },
};

static int __init gs201_pd_gate_init(void)
{
	struct arm_smccc_res res;
	int i, gated = 0;

	if (!of_machine_is_compatible("google,gs201"))
		return 0;

	for (i = 0; i < ARRAY_SIZE(gs201_unused_pd); i++) {
		arm_smccc_smc(TENSOR_SMC_PMU_SEC_REG,
			      GS201_PMU_BASE + gs201_unused_pd[i].cfg,
			      TENSOR_PMUREG_RMW, 0x1, 0x0, 0, 0, 0, &res);
		if (!res.a0)
			gated++;
		else
			pr_warn("gs201-pd-gate: %s SMC failed: %ld\n",
				gs201_unused_pd[i].name, (long)res.a0);
	}
	pr_info("gs201-pd-gate: powered off %d/%d never-used imaging/video domains\n",
		gated, (int)ARRAY_SIZE(gs201_unused_pd));
	return 0;
}
late_initcall(gs201_pd_gate_init);
