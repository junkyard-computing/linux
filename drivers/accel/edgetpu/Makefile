# SPDX-License-Identifier: GPL-2.0

# The Rust accel driver ("edgetpu") plus its GSA secure-firmware glue module
# ("edgetpu_gsa"). The Rust module links against the glue's exported symbols;
# load order is gsa -> edgetpu_gsa -> edgetpu (resolved via module deps).
obj-$(CONFIG_DRM_ACCEL_EDGETPU) += edgetpu.o
obj-$(CONFIG_DRM_ACCEL_EDGETPU) += edgetpu_gsa.o
