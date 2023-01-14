/*
 * Copyright 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 */

#include "samples.h"

#include "al_hal_cell_table.h"

/**
 * @defgroup group_table_samples Code Samples
 * @ingroup group_table
 * @{
 * table.c: this file can be found in samples directory.
 *
 */


/**
 * An example of registers of a table entry.
 * Such entries are generated from register files, and not created by hand.
 */
struct al_example_table_entry_regs {
	/* Value 1 */
	uint32_t val1;
	/* Value 2 */
	uint32_t val2;
	/* Reserved */
	uint32_t reserved;
};

#define EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_1_MASK		0x000000FF
#define EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_1_SHIFT		0
#define EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_2_MASK		0x0000FF00
#define EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_2_SHIFT		8
#define EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_3_MASK		0x00FF0000
#define EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_3_SHIFT		16

#define EXAMPLE_TABLE_ENTRY_VAL_2_FIELD_1_MASK		0x000000FF
#define EXAMPLE_TABLE_ENTRY_VAL_2_FIELD_1_SHIFT		0
#define EXAMPLE_TABLE_ENTRY_VAL_2_FIELD_1		0x00010000

/**
 * An example of table entry wrapper for SW usage.
 * Such wrappers are generated from register files, and not created by hand.
 */
struct al_example_table_entry_wrap {
	/* Table entry index */
	uint32_t idx;
	/* Actual table entry as defined by HW */
	struct al_example_table_entry_regs entry;
};

al_static_assert(sizeof(struct al_example_table_entry_wrap) == (4*4),
	"Table entry wrap size is not as expected");

static void table_entry_write(struct al_cell_table_ctrl_regs *table_ctrl_regs)
{
	unsigned int entry_idx = 2;
	struct al_example_table_entry_regs entry = {0};

	/* Fill values */

	/* val1 */
	AL_REG_FIELD_SET(entry.val1,
		EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_1_MASK,
		EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_1_SHIFT,
		5);

	AL_REG_FIELD_SET(entry.val1,
		EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_2_MASK,
		EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_2_SHIFT,
		6);

	AL_REG_FIELD_SET(entry.val1,
		EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_3_MASK,
		EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_3_SHIFT,
		7);

	/* val2 */
	AL_REG_FIELD_SET(entry.val2,
		EXAMPLE_TABLE_ENTRY_VAL_2_FIELD_1_MASK,
		EXAMPLE_TABLE_ENTRY_VAL_2_FIELD_1_SHIFT,
		12);

	AL_REG_FIELD_BIT_SET(entry.val2,
		EXAMPLE_TABLE_ENTRY_VAL_2_FIELD_1_MASK,
		AL_TRUE);

	/* Write */
	al_cell_table_entry_write(table_ctrl_regs, entry_idx, &entry, sizeof(entry));
}

static void table_entry_read(struct al_cell_table_ctrl_regs *table_ctrl_regs)
{
	unsigned int entry_idx = 2;
	struct al_example_table_entry_regs entry = {0};
	unsigned int interesting_value;

	/* Read */
	al_cell_table_entry_read(table_ctrl_regs, entry_idx, &entry, sizeof(entry));

	/* Use values */

	/* val1 */
	interesting_value = AL_REG_FIELD_GET(entry.val1,
		EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_1_MASK,
		EXAMPLE_TABLE_ENTRY_VAL_1_FIELD_1_SHIFT);

	al_info("val_1_field_1 is %u\n", interesting_value);

	/* val2 */
	interesting_value = AL_REG_FIELD_BIT_GET(entry.val2,
		EXAMPLE_TABLE_ENTRY_VAL_2_FIELD_1);

	al_info("val_2_field_1 is %u\n", interesting_value);
}

static void table_entries_write(struct al_cell_table_ctrl_regs *table_ctrl_regs)
{
	struct al_example_table_entry_wrap rows[] = {
		/* Entry */
		{
			.idx = 4,
			.entry = {
				.val1 = 0xaabb,
			},
		},
		/* Entry */
		{
			.idx = 7,
			.entry = {
				.val2 = 0xccdd,
			},
		}
	};

	al_cell_table_entries_write(table_ctrl_regs,
		rows, sizeof(rows[0]), AL_ARR_SIZE(rows));
}

int main(void)
{
	/* For running the sample make sure it's not NULL, so we won't fail assertions */
	struct al_cell_table_ctrl_regs *table_ctrl_regs =
		(struct al_cell_table_ctrl_regs *)0x1000;

	table_entry_write(table_ctrl_regs);
	table_entry_read(table_ctrl_regs);

	table_entries_write(table_ctrl_regs);

	return 0;
}

/** @endcode */

/** @} */
