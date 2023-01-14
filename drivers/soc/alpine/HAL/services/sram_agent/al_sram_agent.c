/*
 * Copyright 2019, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */
#include "al_hal_iomap.h"
#include "al_hal_reg_utils.h"
#include "al_hal_unit_adapter_regs.h"
#include "al_sram_agent.h"

#ifndef SZ_1K
#define SZ_1K   (1 << 10)
#endif

#if (AL_DEV_ID != AL_DEV_ID_ALPINE_V2)

int al_sram_agent_run(void)
{
	al_err("\nETH SRAM Service not supported\n");
	return -1;
}

int al_sram_agent_init(enum al_sram_agent_sram_type type, const unsigned char *arr, size_t size)
{
	al_err("\nETH SRAM Service not supported for type %d.\n", type);
	al_err("Cannot copy from %p (%zd bytes).\n", arr, size);
	return -1;
}

#else
#define ETH_SRAM_EN_REG	(AL_ETH_BASE(i) + 0x0000d030)
#define ETH_SRAM_EN_MASK	0x3
#define ETH_SRAM_EN_VAL		0x3

#define AGENT_ENTRY_POINT_OFFSET (0xb00)
#define AGENT_ENTRY_POINT (void *)(uintptr_t)(AL_ETH_SRAM_BASE(0, AL_ETH_SRAM_TYPE_ADV) + AGENT_ENTRY_POINT_OFFSET)

static void sram_arr_copy(void *dst, const unsigned char *src, size_t size)
{
	unsigned int i;
	unsigned char *d = (unsigned char *)dst;
	const unsigned char *s = src;

	for (i = 0; i < size; i++)
		d[i] = s[i];
}

static void al_sram_agent_enable_eth(void)
{
	int i;

	/* ETH SRAM0/2 powerup, if required */
	for (i = 0; i <= 2; i += 2) {
		if (al_reg_read32((uint32_t *)(AL_ETH_BASE(i) + AL_ADAPTER_GENERIC_CONTROL_0)) &
			AL_ADAPTER_GENERIC_CONTROL_0_ADAPTER_DIS) {
			/* Disable clock gating */
			al_reg_write32_masked(
				(uint32_t *)(AL_ETH_BASE(i) + AL_ADAPTER_GENERIC_CONTROL_0),
				AL_ADAPTER_GENERIC_CONTROL_0_CLK_GATE_EN, 0);
			/* Switch to D0 */
			al_reg_write32_masked(
				(uint32_t *)(AL_ETH_BASE(i) + AL_ADAPTER_PM_1),
				AL_ADAPTER_PM_1_PWR_STATE_MASK, AL_ADAPTER_PM_1_PWR_STATE_D0);
			/* Unit specific power-up */
			al_reg_write32_masked(
				(uint32_t *)(AL_ETH_BASE(i) + AL_ADAPTER_GENERIC_CONTROL_3),
				0x8f000000, 0);
			/* Function level reset */
			al_reg_write32_masked(
				(uint32_t *)(AL_ETH_BASE(i) + AL_PCI_EXP_CAP_BASE + AL_PCI_EXP_DEVCTL),
				AL_PCI_EXP_DEVCTL_BCR_FLR, AL_PCI_EXP_DEVCTL_BCR_FLR);
			al_udelay(100000);
		}
		al_reg_write32_masked(
			(uint32_t *)ETH_SRAM_EN_REG,
			ETH_SRAM_EN_MASK,
			ETH_SRAM_EN_VAL);
		al_reg_read32((uint32_t *)ETH_SRAM_EN_REG);
	}
}

static int al_sram_agent_copy_eth(const unsigned char *arr, size_t size)
{
	int err;

	if (size > (AL_ETH_SRAM_SIZE(0, AL_ETH_SRAM_TYPE_ADV) - AGENT_ENTRY_POINT_OFFSET)) {
		size_t delta = AL_ETH_SRAM_SIZE(0, AL_ETH_SRAM_TYPE_ADV) - AGENT_ENTRY_POINT_OFFSET;

		al_dbg("copy to ETH_SRAM0 %zd bytes\n", delta);
		sram_arr_copy(AGENT_ENTRY_POINT, arr, delta);
		al_dbg("copy to ETH_SRAM2 %zd bytes\n", size - delta);
		sram_arr_copy((void *)(uintptr_t)(AL_ETH_SRAM_BASE(2, AL_ETH_SRAM_TYPE_ADV)),
			      &arr[delta], size - delta);

		err = al_memcmp(AGENT_ENTRY_POINT, arr,	delta);
		err = al_memcmp((void *)(uintptr_t)(AL_ETH_SRAM_BASE(2, AL_ETH_SRAM_TYPE_ADV)),
				&arr[delta], size - delta);
	} else {
		sram_arr_copy(AGENT_ENTRY_POINT, arr, size);
		err = al_memcmp(AGENT_ENTRY_POINT, arr,	size);
	}
	if (err) {
		al_err("memcmp failed!!!\n");
		return -ENOMEM;
	}

	return err;
}

/**************************************************************************************************/
int al_sram_agent_init(enum al_sram_agent_sram_type type, const unsigned char *arr, size_t size)
{
	int err;

	if (type != AL_ETH_SRAM_TYPE) {
		al_err("Unsupported SRAM agent type: %d\n", type);
		return -1;
	}

	al_sram_agent_enable_eth();
	err = al_sram_agent_copy_eth(arr, size);

	return err;
}

int al_sram_agent_run(void)
{
	int (*agent_run)(void) = (int (*)(void))AGENT_ENTRY_POINT;
	int err;

	agent_run = (int (*)(void))AGENT_ENTRY_POINT;

	al_dbg("JUMP to %p\n", agent_run);
	err = agent_run();
	al_dbg("Error %d return from SRAM function call\n", err);
	return err;
}

#endif
