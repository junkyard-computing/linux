/* SPDX-License-Identifier: GPL-2.0 */
/* mainline 7.1 compatibility shim for the AOSP samsung display graft.
 * Force-included (-include) before all driver TUs. */
#ifndef __SAMSUNG_FELIX_COMPAT_H
#define __SAMSUNG_FELIX_COMPAT_H

/* commit 5164f7e7ff8e "drm: Rename struct drm_atomic_state to
 * drm_atomic_commit" — the AOSP 6.1 driver predates the rename. */
#define drm_atomic_state		drm_atomic_commit
#define drm_atomic_state_alloc		drm_atomic_commit_alloc
#define drm_atomic_state_clear		drm_atomic_commit_clear
#define drm_atomic_state_get		drm_atomic_commit_get
#define drm_atomic_state_put		drm_atomic_commit_put

#endif /* __SAMSUNG_FELIX_COMPAT_H */
