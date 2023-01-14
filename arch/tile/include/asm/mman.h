/*
 * Copyright 2014 Tilera Corporation. All Rights Reserved.
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

#ifndef _ASM_TILE_MMAN_H
#define _ASM_TILE_MMAN_H

#include <uapi/asm/mman.h>

#ifdef CONFIG_HOMECACHE

#include <linux/sched.h>

static inline pgprot_t arch_vm_get_page_prot(unsigned long vm_flags)
{
	return (vm_flags & VM_DONTMERGE) ? \
		__pgprot(current->thread.homecache_prot) : __pgprot(0);
}
#define arch_vm_get_page_prot(vm_flags) arch_vm_get_page_prot(vm_flags)

#endif /* CONFIG_HOMECACHE */
#endif /* _ASM_TILE_MMAN_H */
