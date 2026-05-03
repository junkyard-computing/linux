# SPDX-License-Identifier: GPL-2.0
#
# Pixel/GS UFS host controller driver, ported as-is from AOSP's
# private/google-modules/soc/gs/drivers/ufs/. Coexists with the upstream
# drivers/ufs/host/ufs-exynos.c — runtime selection is by DT compatible
# string, build selection is by CONFIG_SCSI_UFS_EXYNOS_GS.

obj-$(CONFIG_SCSI_UFS_EXYNOS_GS) += ufs-exynos-gs.o
ufs-exynos-gs-y += ufs-exynos.o ufs-exynos-dbg.o ufs-pixel.o
# Per-SoC cal-if files all share global symbol names (ufs_cal_init,
# ufs_cal_pre_pmc, ...), so exactly one must be linked. Gate by SOC_*.
ufs-exynos-gs-$(CONFIG_SOC_GS101) += gs101/ufs-cal-if.o
ufs-exynos-gs-$(CONFIG_SOC_GS201) += gs201/ufs-cal-if.o
ufs-exynos-gs-$(CONFIG_SOC_ZUMA)  += zuma/ufs-cal-if.o
# ufs-exynos-crypto.o + ufs-pixel-crypto.o pull in linux/gsa/gsa_kdn.h and the
# Pixel inline-encryption stack; only build them when SCSI_UFS_CRYPTO is on.
ufs-exynos-gs-$(CONFIG_SCSI_UFS_CRYPTO) += ufs-exynos-crypto.o ufs-pixel-crypto.o

ccflags-y += -I $(srctree)/drivers/ufs
