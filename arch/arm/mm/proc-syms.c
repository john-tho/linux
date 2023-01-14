// SPDX-License-Identifier: GPL-2.0-only
/*
 *  linux/arch/arm/mm/proc-syms.c
 *
 *  Copyright (C) 2000-2002 Russell King
 */
#include <linux/module.h>
#include <linux/mm.h>

#include <asm/cacheflush.h>
#include <asm/proc-fns.h>
#include <asm/tlbflush.h>
#include <asm/page.h>

#ifndef MULTI_CPU
EXPORT_SYMBOL(cpu_dcache_clean_area);
#ifdef CONFIG_MMU
EXPORT_SYMBOL(cpu_set_pte_ext);
#endif
#else
EXPORT_SYMBOL(processor);
#endif

#ifndef MULTI_CACHE
EXPORT_SYMBOL(__cpuc_flush_kern_all);
EXPORT_SYMBOL(__cpuc_flush_user_all);
EXPORT_SYMBOL(__cpuc_flush_user_range);
EXPORT_SYMBOL(__cpuc_coherent_kern_range);
EXPORT_SYMBOL(__cpuc_flush_dcache_area);
#else
EXPORT_SYMBOL(cpu_cache);
#endif

#ifdef CONFIG_CPU_CACHE_V7
extern void v7_dma_unmap_area(const void *, size_t, int);
EXPORT_SYMBOL(v7_dma_flush_range);
void v7_dma_inv_range(void *start, void *end);
EXPORT_SYMBOL(v7_dma_inv_range);
void v7_dma_inv_range_nosync(void *start, void *end);
EXPORT_SYMBOL(v7_dma_inv_range_nosync);
void v7_dma_flush_range_nosync(void *start, void *end);
EXPORT_SYMBOL(v7_dma_flush_range_nosync);
EXPORT_SYMBOL(v7_dma_unmap_area);
#endif

#ifdef CONFIG_MMU
#ifndef MULTI_USER
EXPORT_SYMBOL(__cpu_clear_user_highpage);
EXPORT_SYMBOL(__cpu_copy_user_highpage);
#else
EXPORT_SYMBOL(cpu_user);
#endif
#endif

/*
 * No module should need to touch the TLB (and currently
 * no modules do.  We export this for "loadkernel" support
 * (booting a new kernel from within a running kernel.)
 */
#ifdef MULTI_TLB
EXPORT_SYMBOL(cpu_tlb);
#endif
