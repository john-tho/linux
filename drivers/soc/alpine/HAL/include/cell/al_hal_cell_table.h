/*
 * Copyright 2017, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

/**
 * @defgroup group_table HAL generic table
 * Includes Alpine generic table API
 *  @{
 * @file   al_hal_cell_table.h
 *
 */

#ifndef __AL_HAL_CELL_TABLE_H__
#define __AL_HAL_CELL_TABLE_H__

/* *INDENT-OFF* */
#ifdef __cplusplus
extern "C" {
#endif
/* *INDENT-ON* */

#include "al_hal_plat_types.h"

/* Forward declaration */
struct al_cell_table_ctrl_regs;

struct al_cell_table_params {
	/** Size of a single entry (in bytes) */
	unsigned int entry_size;
	/** Number of entries in table */
	unsigned int entries;
};

/**
 * Table entry head
 *
 * A table entry wrapper is expected to be of this structure:
 *	struct al_example_table_entry_wrap {
 *		// Index
 *		uint32_t idx;
 *		// Actual entry that's provided part of the registers
 *		struct al_example_table_entry_regs entry;
 *		// Nothing else should be after actual entry
 *	};
 *
 * This structure reflects the head of such wrapper.
 * This structure used internally, and not expected to be used by user,
 * but here for a documentation reasons.
 */
struct al_cell_table_entry_head {
	/** Table entry index */
	uint32_t idx;

	/**
	 * Pointer to actual table entry content
	 *
	 * Doesn't consume space from this structure, but actually points
	 * to the data that comes after this structure.
	 */
	uint32_t entry_start[];
};

/**
 * Get table parameters
 *
 * @param table_ctrl_regs
 *	Pointer to table control registers
 * @param params
 *	Fill be filled with table parameters
 */
void al_cell_table_params_get(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	struct al_cell_table_params *params);

/**
 * Write a single entry to a table
 *
 * @param table_ctrl_regs
 *	Pointer to table control registers
 * @param index
 *	Table entry index
 * @param entry
 *	Pointer to entry start to write entry from
 * @param entry_size
 *	Size of table entry in bytes
 *	(use with 'sizeof' and table entry struct)
 */
void al_cell_table_entry_write(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	unsigned int index,
	const void *entry,
	unsigned int entry_size);

/**
 * Read a single entry from a table
 *
 * @param table_ctrl_regs
 *	Pointer to table control registers
 * @param index
 *	Table entry index
 * @param entry
 *	Pointer to entry start to read entry into
 * @param entry_size
 *	Size of table entry in bytes
 *	(use with 'sizeof' and table entry struct)
 */
void al_cell_table_entry_read(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	unsigned int index,
	void *entry,
	unsigned int entry_size);

/**
 * Write a batch of entries to a table.
 *
 * Actual entries are expected to be wrapped in a format declared above.
 * See example above on how this wrapper looks like.
 *
 * @param table_ctrl_regs
 *	Pointer to table control registers
 * @param wrapped_entries
 *	Pointer to start of array of wrapped entries.
 * @param wrapped_entry_size
 *	Size of wrapped table entry in bytes
 *	(use with 'sizeof' and wrapped table entry struct)
 * @param entries_num
 *	Number of entries in the array
 */
void al_cell_table_entries_write(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	void *wrapped_entries,
	unsigned int wrapped_entry_size,
	unsigned int entries_num);


/**
 * Write one entrie multipal time into range inside the table.
 *
 *
 * @param table_ctrl_regs
 *	Pointer to table control registers
 * @param index
 *	Pointer to start row inside the table.
 * @param repeat
 *	How many rows from start point "index" to write
 * @param entry
 *	Pointer to entry start to read entry into
 * @param entry_size
 *	Size of table entry in bytes
 *	(use with 'sizeof' and table entry struct)
 */
void al_cell_table_entry_write_range(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	unsigned int index,
	unsigned int repeat,
	const void *entry,
	unsigned int entry_size);

#ifdef __cplusplus
}
#endif
/* *INDENT-ON* */
#endif /* __AL_HAL_CELL_TABLE_H__ */
/** @} */
