/*
 * Copyright 2010 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 */

#ifndef _ASM_TILE_MMU_CONTEXT_H
#define _ASM_TILE_MMU_CONTEXT_H

#include <linux/compat.h>
#include <linux/smp.h>
#include <linux/mm_types.h>

#include <asm/setup.h>
#include <asm/page.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>
#include <asm/homecache.h>
#include <asm-generic/mm_hooks.h>

static inline int
init_new_context(struct task_struct *tsk, struct mm_struct *mm)
{
	memset(mm->context.asid, 0, sizeof(unsigned long) * NR_CPUS);
	return 0;
}

/*
 * Note that arch/tile/kernel/head_NN.S and arch/tile/mm/migrate_NN.S
 * also call hv_install_context().
 */
static inline void __install_page_table(pgd_t *pgdir, int asid,
					pgprot_t prot, int is_32bit)
{
	/* FIXME: DIRECTIO should not always be set. FIXME. */
	int flags = HV_CTX_DIRECTIO | CTX_PAGE_FLAG;
	int rc;

#ifdef CONFIG_COMPAT
	/*
	 * Set the HV_CTX_32BIT context flag for the m32 process,
	 * so that hypervisor can do a mirror flush in the top 2GB of VA space
	 * when we flush in the 2-4GB range.
	 */
	if (is_32bit)
		flags |= HV_CTX_32BIT;
#endif
	rc = hv_install_context(__pa(pgdir), prot, asid, flags);
	if (rc < 0)
		panic("hv_install_context failed: %d", rc);
}

static inline void install_page_table(pgd_t *pgdir, int asid, int is_32bit)
{
	pte_t *ptep = virt_to_kpte((unsigned long)pgdir);
	__install_page_table(pgdir, asid, *ptep, is_32bit);
}

/*
 * "Lazy" TLB mode is entered when we are switching to a kernel task,
 * which borrows the mm of the previous task.  The goal of this
 * optimization is to avoid having to install a new page table.  On
 * early x86 machines (where the concept originated) you couldn't do
 * anything short of a full page table install for invalidation, so
 * handling a remote TLB invalidate required doing a page table
 * re-install.  Someone clearly decided that it was silly to keep
 * doing this while in "lazy" TLB mode, so the optimization involves
 * installing the swapper page table instead the first time one
 * occurs, and clearing the cpu out of cpu_vm_mask, so the cpu running
 * the kernel task doesn't need to take any more interrupts.  At that
 * point it's then necessary to explicitly reinstall it when context
 * switching back to the original mm.
 *
 * On Tile, we have to do a page-table install whenever DMA is enabled,
 * so in that case lazy mode doesn't help anyway.  And more generally,
 * we have efficient per-page TLB shootdown, and don't expect to spend
 * that much time in kernel tasks in general, so just leaving the
 * kernel task borrowing the old page table, but handling TLB
 * shootdowns, is a reasonable thing to do.  And importantly, this
 * lets us use the hypervisor's internal APIs for TLB shootdown, which
 * means we don't have to worry about having TLB shootdowns blocked
 * when Linux is disabling interrupts; see the page migration code for
 * an example of where it's important for TLB shootdowns to complete
 * even when interrupts are disabled at the Linux level.
 */
static inline void enter_lazy_tlb(struct mm_struct *mm, struct task_struct *t)
{
#if CHIP_HAS_TILE_DMA()
	/*
	 * We have to do an "identity" page table switch in order to
	 * clear any pending DMA interrupts.
	 */
	if (current->thread.tile_dma_state.enabled)
		install_page_table(mm->pgd, __this_cpu_read(current_asid), 0);
#endif
}

/* Update mm->context.asid[cpu] entries and return new ASID to use. */
static inline int manage_asids(struct mm_struct *prev, struct mm_struct *next,
			       int cpu)
{
	unsigned long asid_gen;
	unsigned long gen = __this_cpu_read(current_asid_gen);
	int asid;

retry:

	/* Read ASID and generation count from the new mm. */
	asid_gen = READ_ONCE(next->context.asid[cpu]);
	BUG_ON(asid_gen & ASID_ACTIVE);

	/* Check if a valid ASID for mm on this cpu in current generation. */
	if ((asid_gen >> GEN_SHIFT) == gen) {
		asid = asid_gen & ASID_MASK;
	} else {
		/* No valid ASID in this generation; allocate a new ASID. */
		asid = __this_cpu_read(next_asid);
		if (asid > max_asid) {
			/* No new ASIDs; flush TLB, reset ASID, bump gen. */
			local_flush_tlb();
			asid = min_asid;
			gen = ((gen + 1) & GEN_MASK) ?: 1; /* zero invalid */
			__this_cpu_write(current_asid_gen, gen);
		}
		__this_cpu_write(next_asid, asid + 1);
	}

	/*
	 * Record ASID/generation; if we fail, we're probably racing
	 * with flush_tlb_mm_begin(), so go back and retry.
	 */
	if (cmpxchg(&next->context.asid[cpu], asid_gen,
		    asid | (gen << GEN_SHIFT) | ASID_ACTIVE) != asid_gen)
		goto retry;

	__this_cpu_write(current_asid, asid);

	/* Mark old mm's asid as no longer active. */
	clear_bit(ASID_ACTIVE_BIT, &prev->context.asid[cpu]);

	return asid;
}


static inline void switch_mm(struct mm_struct *prev, struct mm_struct *next,
			     struct task_struct *tsk)
{
	if (likely(prev != next)) {

		int cpu = smp_processor_id();
		int asid = manage_asids(prev, next, cpu);

		/* Clear cpu from the old mm, and set it in the new one. */
		cpumask_clear_cpu(cpu, mm_cpumask(prev));
		cpumask_set_cpu(cpu, mm_cpumask(next));

		/* Re-load page tables */
#ifdef CONFIG_COMPAT
		install_page_table(next->pgd, asid, next->context.is_32bit);
#else
		install_page_table(next->pgd, asid, 0);
#endif

#ifdef __tilepro__ /* priority caching */
		/* See how we should set the red/black cache info */
		check_mm_caching(prev, next);
#endif

		/*
		 * Since we're changing to a new mm, we have to flush
		 * the icache in case some physical page now being mapped
		 * has subsequently been repurposed and has new code.
		 */
		__flush_icache();

	}
}

static inline void activate_mm(struct mm_struct *prev_mm,
			       struct mm_struct *next_mm)
{
	switch_mm(prev_mm, next_mm, NULL);
}

#define destroy_context(mm)		do { } while (0)
#define deactivate_mm(tsk, mm)          do { } while (0)

#endif /* _ASM_TILE_MMU_CONTEXT_H */
