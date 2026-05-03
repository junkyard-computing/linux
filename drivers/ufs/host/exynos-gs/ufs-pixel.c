// SPDX-License-Identifier: GPL-2.0
/*
 * Stub of AOSP's ufs-pixel.c for the felix vendor-graft on mainline.
 *
 * The original AOSP ufs-pixel.c is ~2k lines of Pixel-specific stats,
 * observability, manual GC, BKOPS control, FFU, and slow-IO detection.
 * It exercises a swath of UFS-core internals that have drifted significantly
 * in upstream (struct ufshcd_lrb member layout, MASK_RSP_UPIU_RESULT removal,
 * ufshcd_query_flag_retry / ufshcd_bkops_ctrl unexported, hrtimer_init →
 * hrtimer_setup, request_desc_header.dword_2 rename, UFS_CMD_SEND/COMP enum
 * rename, etc.). None of it sits on the link-startup path that ufs-exynos.c
 * owns — which is the *only* part we need to validate that HS-Rate-A/B works.
 *
 * Stub the entry points that ufs-exynos.c calls so the module links; the
 * cosmetic features can be ported back once the boot path is proven.
 */

#include <linux/types.h>
#include <ufs/ufshcd.h>
#include "ufs-exynos-gs.h"
#include "ufs-pixel.h"

int pixel_init(struct ufs_hba *hba, struct device *pdev,
	       const struct pixel_crypto_ops *crypto_ops)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);

	ufs->pixel_ufs.crypto_ops = crypto_ops;
	return 0;
}

void pixel_exit(struct ufs_hba *hba)
{
}

void pixel_print_cmd_log(struct ufs_hba *hba)
{
}

void pixel_ufs_record_hibern8(struct ufs_hba *hba, bool is_enter_h8)
{
}

void pixel_update_power_event(struct ufs_hba *hba,
			      enum pixel_power_event_type event)
{
}
