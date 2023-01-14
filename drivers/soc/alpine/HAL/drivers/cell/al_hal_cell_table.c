/*
 * Copyright 2017, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

#include "al_hal_cell_table.h"

#include "al_hal_common.h"
#include "al_hal_cell_table_ctrl_regs.h"

/**
 * Open debug table debug checks
 *
 * Can affect access time to each table
 */
#ifndef TABLE_CTRL_DEBUG
#define TABLE_CTRL_DEBUG	0
#endif

static void debug_assert_table_params(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	unsigned int index,
	unsigned int entry_size)
{
#if TABLE_CTRL_DEBUG
	struct al_cell_table_params params = {0};

	al_cell_table_params_get(table_ctrl_regs, &params);

	al_assert_msg(index < params.entries,
		"Table: provided entry index %u, while table has %u entries\n",
		index, params.entries);

	al_assert_msg(entry_size == params.entry_size,
		"Table: provided entry of size %u, while table entry is of size %u\n",
		index, params.entry_size);
#else
	(void)table_ctrl_regs;
	(void)index;
	(void)entry_size;
#endif
}

static void al_cell_table_default_config_set(struct al_cell_table_ctrl_regs *table_ctrl_regs)
{
	uint32_t reg = 0;

	/**
	 * - Auto-increment index on every data write
	 * - Auto-increment address after every full entry write
	 * - Trigger write on last data write (and not trigger register)
	 * - Trigger read on first data read (and not trigger register)
	 */
	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TBL_CPU_CTRL_IDX_AUTO_INC, AL_TRUE);
	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TBL_CPU_CTRL_ADDR_AUTO_INC, AL_TRUE);
	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TBL_CPU_CTRL_WR_TRG_SEL, AL_FALSE);
	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TBL_CPU_CTRL_RD_TRG_SEL, AL_FALSE);

	al_reg_write32(&table_ctrl_regs->tbl_cpu_ctrl, reg);

	/**
	 * Point to start of entry
	 */
	al_reg_write32(&table_ctrl_regs->tbl_idx, 0);
}

static void table_entry_write(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	unsigned int index,
	const void *entry,
	unsigned int entry_size)
{
	const uint32_t *entry_u32 = (const uint32_t *)entry;
	unsigned int i;

	al_reg_write32(&table_ctrl_regs->tbl_addr, index);

	for (i = 0; i < (entry_size / sizeof(uint32_t)); i++)
		al_reg_write32(&table_ctrl_regs->tbl_data, entry_u32[i]);
}

void al_cell_table_params_get(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	struct al_cell_table_params *params)
{
	al_assert(table_ctrl_regs);
	al_assert(params);

	params->entries = al_reg_read32(&table_ctrl_regs->tbl_param0);

	params->entry_size = al_reg_read32(&table_ctrl_regs->tbl_param1);
	params->entry_size *= sizeof(uint32_t);
}

void al_cell_table_entry_write(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	unsigned int index,
	const void *entry,
	unsigned int entry_size)
{
	al_assert(table_ctrl_regs);
	al_assert(entry);
	debug_assert_table_params(table_ctrl_regs, index, entry_size);

	al_cell_table_default_config_set(table_ctrl_regs);

	table_entry_write(table_ctrl_regs, index, entry, entry_size);
}

void al_cell_table_entry_read(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	unsigned int index,
	void *entry,
	unsigned int entry_size)
{
	uint32_t *entry_u32 = (uint32_t *)entry;
	unsigned int i;

	al_assert(table_ctrl_regs);
	al_assert(entry);
	debug_assert_table_params(table_ctrl_regs, index, entry_size);

	al_cell_table_default_config_set(table_ctrl_regs);

	al_reg_write32(&table_ctrl_regs->tbl_addr, index);

	for (i = 0; i < (entry_size / sizeof(uint32_t)); i++)
		entry_u32[i] = al_reg_read32(&table_ctrl_regs->tbl_data);
}

void al_cell_table_entries_write(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	void *wrapped_entries,
	unsigned int wrapped_entry_size,
	unsigned int entries_num)
{
	unsigned int entry_size = wrapped_entry_size - sizeof(struct al_cell_table_entry_head);
	unsigned int i;

	al_assert(table_ctrl_regs);
	al_assert(wrapped_entries);

	al_cell_table_default_config_set(table_ctrl_regs);

	for (i = 0; i < entries_num; i++) {
		struct al_cell_table_entry_head *entry_head =
			(struct al_cell_table_entry_head *)wrapped_entries;

		debug_assert_table_params(table_ctrl_regs, entry_head->idx, entry_size);

		table_entry_write(table_ctrl_regs,
			entry_head->idx, entry_head->entry_start, entry_size);

		wrapped_entries = (void *)((uintptr_t)wrapped_entries + wrapped_entry_size);
	}
}

static void al_cell_table_range_config_set(struct al_cell_table_ctrl_regs *table_ctrl_regs)
{
	uint32_t reg = 0;

	/**
	 * - Auto-increment index on every data write
	 * - Auto-increment address after every full entry write
	 * - Trigger write on last data write (and not trigger register)
	 * - Trigger read on first data read (and not trigger register)
	 */
	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TBL_CPU_CTRL_IDX_AUTO_INC, AL_TRUE);
	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TBL_CPU_CTRL_ADDR_AUTO_INC, AL_TRUE);
	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TBL_CPU_CTRL_WR_TRG_SEL, AL_TRUE);
	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TBL_CPU_CTRL_RD_TRG_SEL, AL_FALSE);

	al_reg_write32(&table_ctrl_regs->tbl_cpu_ctrl, reg);

	reg = 0;

	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TABLE_GLOBAL_CTRL_BLOCKING_SEL, AL_FALSE);
	al_reg_write32(&table_ctrl_regs->table_global_ctrl, reg);
	/**
	 * Point to start of entry
	 */
	al_reg_write32(&table_ctrl_regs->tbl_idx, 0);
}

static void al_cell_table_range_config_set_back(struct al_cell_table_ctrl_regs *table_ctrl_regs)
{
	uint32_t reg = 0;

	/**
	 * Return value to default
	 */
	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TABLE_GLOBAL_CTRL_BLOCKING_SEL, AL_TRUE);
	al_reg_write32(&table_ctrl_regs->table_global_ctrl, reg);
}

void al_cell_table_entry_write_range(struct al_cell_table_ctrl_regs *table_ctrl_regs,
	unsigned int index,
	unsigned int repeat,
	const void *entry,
	unsigned int entry_size)
{
	uint32_t reg = 0;
	unsigned int timeout;

	al_assert(table_ctrl_regs);
	al_assert(entry);
	debug_assert_table_params(table_ctrl_regs, index, entry_size);

	al_cell_table_range_config_set(table_ctrl_regs);

	/**
	 * Set amount of times to return on the write
	 */
	al_reg_write32(&table_ctrl_regs->write_repeat, repeat);

	table_entry_write(table_ctrl_regs, index, entry, entry_size);

	/**
	 * Trig the write
	 */
	AL_REG_FIELD_BIT_SET(reg, CELL_TABLE_CTRL_TBL_CMD_WR_TRG, 1);
	al_reg_write32(&table_ctrl_regs->tbl_cmd, reg);

	/*
	 * A single row request should take 10 clocks,
	 * where each clock is 1.333 ns.
	 * For calculation simplicity we will round-up 13.33ns to 20ns.
	 *
	 * Align-up calculation to 1000ns quotas that are converted to uSec.
	 */
	timeout = AL_ALIGN_UP(repeat * 20, 1000) / 1000;

	do {
		uint32_t left;

		left = al_reg_read32(&table_ctrl_regs->write_repeat);
		if (left == 0)
			break;

		al_assert_msg(timeout > 0,
			"Timeout reached to 0 writing to table %p\n",
			table_ctrl_regs);

		al_udelay(1);
		timeout--;
	} while (AL_TRUE);

	al_cell_table_range_config_set_back(table_ctrl_regs);
}
