/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
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

#ifndef _UAPI_ASM_TILE_MMAN_H
#define _UAPI_ASM_TILE_MMAN_H

#include <arch/chip.h>

/* Standard Linux flags */

#define PROT_READ	0x1		/* page can be read */
#define PROT_WRITE	0x2		/* page can be written */
#define PROT_EXEC	0x4		/* page can be executed */
#define PROT_SEM	0x8		/* page may be used for atomic ops */
#define PROT_NONE	0x0		/* page can not be accessed */
#define PROT_GROWSDOWN	0x01000000	/* mprotect flag: extend change to start of growsdown vma */
#define PROT_GROWSUP	0x02000000	/* mprotect flag: extend change to end of growsup vma */

#define MAP_SHARED	0x01		/* Share changes */
#define MAP_PRIVATE	0x02		/* Changes are private */
#define MAP_TYPE	0x0f		/* Mask for type of mapping */
#define MAP_FIXED	0x10		/* Interpret addr exactly */
#define MAP_ANONYMOUS	0x20		/* don't use a file */
#ifdef CONFIG_MMAP_ALLOW_UNINITIALIZED
# define MAP_UNINITIALIZED 0x4000000	/* For anonymous mmap, memory could be uninitialized */
#else
# define MAP_UNINITIALIZED 0x0		/* Don't support this flag */
#endif

#define MAP_POPULATE	0x0040		/* populate (prefault) pagetables */
#define MAP_NONBLOCK	0x0080		/* do not block on IO */
#define MAP_GROWSDOWN	0x0100		/* stack-like segment */
#define MAP_STACK	MAP_GROWSDOWN	/* provide convenience alias */
#define MAP_LOCKED	0x0200		/* pages are locked */
#define MAP_NORESERVE	0x0400		/* don't check for reservations */
#define MAP_DENYWRITE	0x0800		/* ETXTBSY */
#define MAP_EXECUTABLE	0x1000		/* mark it as an executable */
#define MAP_HUGETLB	0x4000		/* create a huge page mapping */
#define MAP_FIXED_NOREPLACE	0x8000	/* MAP_FIXED which doesn't unmap underlying mapping */

/*
 * Flags for mlock
 */
#define MLOCK_ONFAULT	0x01		/* Lock pages in range after they are faulted in, do not prefault */

#define MS_ASYNC	1		/* sync memory asynchronously */
#define MS_INVALIDATE	2		/* invalidate the caches */
#define MS_SYNC		4		/* synchronous memory sync */

#define MADV_NORMAL	0		/* no further special treatment */
#define MADV_RANDOM	1		/* expect random page references */
#define MADV_SEQUENTIAL	2		/* expect sequential page references */
#define MADV_WILLNEED	3		/* will need these pages */
#define MADV_DONTNEED	4		/* don't need these pages */

/* common parameters: try to keep these consistent across architectures */
#define MADV_FREE	8		/* free pages only if memory pressure */
#define MADV_REMOVE	9		/* remove these pages & resources */
#define MADV_DONTFORK	10		/* don't inherit across fork */
#define MADV_DOFORK	11		/* do inherit across fork */
#define MADV_HWPOISON	100		/* poison a page for testing */
#define MADV_SOFT_OFFLINE 101		/* soft offline page for testing */

#define MADV_MERGEABLE   12		/* KSM may merge identical pages */
#define MADV_UNMERGEABLE 13		/* KSM may not merge identical pages */

#define MADV_HUGEPAGE	14		/* Worth backing with hugepages */
#define MADV_NOHUGEPAGE	15		/* Not worth backing with hugepages */

#define MADV_DONTDUMP   16		/* Explicity exclude from the core dump,
					   overrides the coredump filter bits */
#define MADV_DODUMP	17		/* Clear the MADV_DONTDUMP flag */

#define MADV_WIPEONFORK 18		/* Zero memory on fork, child only */
#define MADV_KEEPONFORK 19		/* Undo MADV_WIPEONFORK */

#define MADV_COLD      20              /* deactivate these pages */
#define MADV_PAGEOUT   21              /* reclaim these pages */

/* compatibility flags */
#define MAP_FILE	0

/*
 * Specify the "home cache" for the page explicitly.  The home cache is
 * the cache of one particular "home" cpu, which is used as a coherence
 * point for normal cached operations.  Normally the kernel chooses for
 * you, but you can use the MAP_CACHE_HOME_xxx flags to override.
 *
 * User code should not use any symbols with a leading "_" as they are
 * implementation specific and may change from release to release
 * without warning.
 *
 * See the Tilera mmap(2) man page for more details (e.g. "tile-man mmap").
 */

/* Implementation details; do not use directly. */
#define _MAP_CACHE_INCOHERENT   0x40000
#define _MAP_CACHE_HOME         0x80000
#define _MAP_CACHE_HOME_SHIFT   20
#define _MAP_CACHE_HOME_MASK    0x3ff
#define _MAP_CACHE_MKHOME(n) \
  (_MAP_CACHE_HOME | (((n) & _MAP_CACHE_HOME_MASK) << _MAP_CACHE_HOME_SHIFT))

/* Set the home cache to the specified cpu. */
#define MAP_CACHE_HOME(n)       _MAP_CACHE_MKHOME(n)

/* Set the home cache to the current cpu. */
#define _MAP_CACHE_HOME_HERE    (_MAP_CACHE_HOME_MASK - 0)
#define MAP_CACHE_HOME_HERE     _MAP_CACHE_MKHOME(_MAP_CACHE_HOME_HERE)

/*
 * Request no on-chip home, i.e. read/write direct to memory.  Invalid if both
 * PROT_WRITE and any local caching are specified; see MAP_CACHE_NONE below.
 */
#define _MAP_CACHE_HOME_NONE    (_MAP_CACHE_HOME_MASK - 1)
#define MAP_CACHE_HOME_NONE     _MAP_CACHE_MKHOME(_MAP_CACHE_HOME_NONE)

/* Request no on-chip home, and allow incoherent cached PROT_WRITE mappings. */
#define MAP_CACHE_INCOHERENT    (_MAP_CACHE_INCOHERENT | MAP_CACHE_HOME_NONE)

/* Force the system to choose a single home cache, on a cpu of its choice. */
#define _MAP_CACHE_HOME_SINGLE  (_MAP_CACHE_HOME_MASK - 2)
#define MAP_CACHE_HOME_SINGLE   _MAP_CACHE_MKHOME(_MAP_CACHE_HOME_SINGLE)

/* Create a mapping that follows the task when it migrates. */
#define _MAP_CACHE_HOME_TASK    (_MAP_CACHE_HOME_MASK - 3)
#define MAP_CACHE_HOME_TASK     _MAP_CACHE_MKHOME(_MAP_CACHE_HOME_TASK)

/* Create a hash-for-home mapping. */
#define _MAP_CACHE_HOME_HASH    (_MAP_CACHE_HOME_MASK - 4)
#define MAP_CACHE_HOME_HASH     _MAP_CACHE_MKHOME(_MAP_CACHE_HOME_HASH)

/*
 * Specify local caching attributes for the mapping.  Normally the kernel
 * chooses whether to use the local cache, but these flags can be used
 * to override the kernel.
 */

/* Disable use of local L2. */
#define MAP_CACHE_NO_L2         0x20000

/* Disable use of local L1. */
#define MAP_CACHE_NO_L1         0x08000

/* Convenience alias that should be used for forward compatibility. */
#define MAP_CACHE_NO_LOCAL      (MAP_CACHE_NO_L1 | MAP_CACHE_NO_L2)

/* Convenience alias for direct-to-RAM mappings. */
#define MAP_CACHE_NONE          (MAP_CACHE_HOME_NONE | MAP_CACHE_NO_LOCAL)

/* Arrange for this mapping to take priority in the cache. */
/* NOTE: not yet implemented for TILE-Gx and will return EINVAL. */
#define MAP_CACHE_PRIORITY      0x02000

/*
 * Environment variable that controls hash-for-home in user programs.
 */
#define MAP_CACHE_HASH_ENV_VAR "LD_CACHE_HASH"

/*
 * Flags for mlockall
 */
#define MCL_CURRENT	1		/* lock all current mappings */
#define MCL_FUTURE	2		/* lock all future mappings */
#define MCL_ONFAULT	4		/* lock all pages that are faulted in */

#endif /* _UAPI_ASM_TILE_MMAN_H */
