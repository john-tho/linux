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
 *
 */

#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/hugetlb.h>
#include <asm/tlbflush.h>
#include <asm/homecache.h>
#include <hv/hypervisor.h>

/* From tlbflush.h */
DEFINE_PER_CPU(int, current_asid);

/* Initialized in set_init_asid(). */
DEFINE_PER_CPU(int, next_asid);

/* Start with generation 1 so all the mm.context.asid[] entries are invalid. */
DEFINE_PER_CPU(unsigned long, current_asid_gen) = 1;

int min_asid, max_asid;

/*
 * Note that we flush the L1I (for VM_EXEC pages) as well as the TLB
 * so that when we are unmapping an executable page, we also flush it.
 * Combined with flushing the L1I at context switch time, this means
 * we don't have to do any other icache flushes.
 */

/*
 * Walk over all the cpus and collect the remote ASIDs corresponding
 * to this address space from all cores where the address space is
 * running or has run recently.
 *
 * If the "flush_all" argument is true, the caller is planning to
 * flush all translations for the given ASID.  In this case, as an
 * optimization, we can simply zero the stored ASID on any cores not
 * currently running the address space.  We use cmpxchg() here and in
 * switch_mm() to ensure that we atomically decide whether or not to
 * flush a particular mm on a particular cpu.  We might end up issuing
 * the flush to a cpu that isn't actually running the mm any more, but
 * that's OK (just slightly more expensive than zeroing the asid[]
 * entry would have been).
 */
int flush_tlb_mm_begin(struct mm_struct *mm, HV_Remote_ASID *asids,
		       bool flush_all)
{
	int cpu, i = 0;
	unsigned long asid_and_gen;
	HV_Remote_ASID *asid;

	for (cpu = 0; cpu < NR_CPUS; ++cpu) {
	retry:
		asid_and_gen = READ_ONCE(mm->context.asid[cpu]);

		/* If task has never run on this core, skip it. */
		if (asid_and_gen == 0)
			continue;

		/*
		 * If we are flushing all the entries for this ASID
		 * and it's not currently running, try to clear the
		 * switched-out task's ASID on this core.  If we fail,
		 * the task is probably context-switching in right
		 * now, so back up and retry.
		 */
		if (flush_all && !(asid_and_gen & ASID_ACTIVE)) {
			if (cmpxchg(&mm->context.asid[cpu], asid_and_gen, 0) !=
			    asid_and_gen)
				goto retry;
			continue;
		}

		/* Capture asid and core to TLB flush list. */
		asid = &asids[i++];
		asid->x = cpu_x(cpu);
		asid->y = cpu_y(cpu);
		asid->asid = asid_and_gen & ASID_MASK;
	}

	return i;
}

void flush_tlb_mm(struct mm_struct *mm)
{
	HV_Remote_ASID asids[NR_CPUS];
	int nasids = flush_tlb_mm_begin(mm, asids, true);
	flush_remote(0, HV_FLUSH_EVICT_L1I, mm_cpumask(mm),
		     0, 0, 0, NULL, asids, nasids);
}

static unsigned long flush_remote_control(struct vm_area_struct *vma)
{
	unsigned long control = 0;
	if (vma->vm_flags & VM_EXEC)
		control |= HV_FLUSH_EVICT_L1I;
#ifdef __tilegx__
	control |= HV_FLUSH_ASID_VA_RANGE;
# ifdef CONFIG_COMPAT
	if (vma->vm_mm->context.is_32bit)
		control |= HV_FLUSH_TLB_32BIT;
# endif
#endif
	return control;
}

void flush_tlb_page(struct vm_area_struct *vma, unsigned long va)
{
	HV_Remote_ASID asids[NR_CPUS];
	int nasids = flush_tlb_mm_begin(vma->vm_mm, asids, false);
	unsigned long size = vma_kernel_pagesize(vma);
	struct cpumask *mask = mm_cpumask(vma->vm_mm);
	unsigned long control = flush_remote_control(vma);
	flush_remote(0, control, mask, va & -size, size, size,
		     NULL, asids, nasids);
}
EXPORT_SYMBOL(flush_tlb_page);

void flush_tlb_range(struct vm_area_struct *vma,
		     unsigned long start, unsigned long end)
{
	HV_Remote_ASID asids[NR_CPUS];
	int nasids = flush_tlb_mm_begin(vma->vm_mm, asids, false);
	unsigned long size = vma_kernel_pagesize(vma);
	struct cpumask *mask = mm_cpumask(vma->vm_mm);
	unsigned long control = flush_remote_control(vma);
	flush_remote(0, control, mask, start, end - start, size,
		     NULL, asids, nasids);
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	flush_remote(0, control, mask, start, end - start, HPAGE_SIZE,
		     NULL, asids, nasids);
#endif
}

void flush_tlb_all(void)
{
	int i;
	for (i = 0; ; ++i) {
		HV_VirtAddrRange r = hv_inquire_virtual(i);
		if (r.size == 0)
			break;
		flush_remote(0, HV_FLUSH_EVICT_L1I, cpu_online_mask,
			     r.start, r.size, PAGE_SIZE, cpu_online_mask,
			     NULL, 0);
		flush_remote(0, 0, NULL,
			     r.start, r.size, HPAGE_SIZE, cpu_online_mask,
			     NULL, 0);
	}
}

/*
 * Callers need to flush the L1I themselves if necessary, e.g. for
 * kernel module unload.  Otherwise we assume callers are not using
 * executable pgprot_t's.  Using EVICT_L1I means that dataplane cpus
 * will get an unnecessary interrupt otherwise.
 */
void flush_tlb_kernel_range(unsigned long start, unsigned long end)
{
	flush_remote(0, 0, NULL,
		     start, end - start, PAGE_SIZE, cpu_online_mask, NULL, 0);
}
