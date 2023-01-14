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

#ifndef _ASM_TILE_MMU_H
#define _ASM_TILE_MMU_H

/* Capture any arch- and mm-specific information. */
struct mm_context {
#ifdef __tilepro__ /* priority caching */
	/*
	 * Written under the mmap_sem semaphore; read without the
	 * semaphore but atomically, but it is conservatively set.
	 */
	unsigned long priority_cached;
#endif
	unsigned long vdso_base;

#ifdef CONFIG_COMPAT
	/* Track whether this is a 32-bit address space. */
	bool is_32bit;
#endif

	/* Per-cpu ASID/generation info; see macros below. */
	unsigned long asid[NR_CPUS];
};

/* ASIDs are in the low 8 bits of each asid[] array entry. */
#define ASID_SIZE	8
#define ASID_MASK	((1 << ASID_SIZE) - 1)

/* The high bit indicates if the mm is currently in use on that cpu. */
#define ASID_ACTIVE_BIT	(BITS_PER_LONG - 1)
#define ASID_ACTIVE	(1UL << ASID_ACTIVE_BIT)

/* All of the remaining bits are a generation count. */
#define GEN_SHIFT	ASID_SIZE
#define GEN_MASK	((1UL << (BITS_PER_LONG - 1 - ASID_SIZE)) - 1)

typedef struct mm_context mm_context_t;

void leave_mm(int cpu);

#endif /* _ASM_TILE_MMU_H */
