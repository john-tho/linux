/*
 * Copyright 2019, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

#ifndef __AL_HAL_UDMA_PUSH_MODE_H__
#define __AL_HAL_UDMA_PUSH_MODE_H__

/**
 *		AXI Address LSB Allocation for Push Mode
 *
 *	+--------------+----------------+-----------------+
 *	|              |                |                 |
 *	| Description  | Number of bits | Address Bits    |
 *	|              |                |                 |
 *	+-------------------------------------------------+
 *	|              |                |                 |
 *	| Tx/Rx        |       1        |       18        |
 *	|              |                |                 |
 *	+-------------------------------------------------+
 *	|              |                |                 |
 *	| Queue ID     |       4        |      17:14      |
 *	|              |                |                 |
 *	+-------------------------------------------------+
 *	|              |                |                 |
 *	| DoorBell/    |                |                 |
 *	| Descriptor   |       1        |       13        |
 *	|              |                |                 |
 *	+-------------------------------------------------+
 *	|              |                |                 |
 *	| Descriptor   |                |                 |
 *	| Index within |       9        |      12:4       |
 *	| Ring         |                |                 |
 *	|              |                |                 |
 *	+-------------------------------------------------+
 *	|              |                |                 |
 *	| Unused since |                |                 |
 *	| Each desc.   |       4        |       3:0       |
 *	| is 16 Byte   |                |                 |
 *	|              |                |                 |
 *	+--------------+----------------+-----------------+
 *
 */

#define AL_UDMA_PUSH_AXI_STREAM_MSB_MASK	0xFFFFFFFFFFF80000UL
#define AL_UDMA_PUSH_DESCRIPTOR_INDEX_MASK	0x00001FF0
#define AL_UDMA_PUSH_DESCR_DOORBELL_SEL_MASK	0x00002000
#define AL_UDMA_PUSH_QUEUE_ID_FIELD_MASK	0x0003C000
#define AL_UDMA_PUSH_TX_RX_FIELD_MASK		0x00040000

#define AL_UDMA_PUSH_DESCRIPTOR_INDEX_OFFSET	4
#define AL_UDMA_PUSH_DESCR_DOORBELL_SEL_OFFSET	13
#define AL_UDMA_PUSH_QUEUE_ID_FIELD_OFFSET	14
#define AL_UDMA_PUSH_TX_RX_FIELD_OFFSET		18

#define AL_UDMA_PUSH_QUEUE_ID_MAX		15

/**
 * maximal number of descriptors in the descriptors ring
 * this value is dictated from the address allocation
 */
#define AL_UDMA_PUSH_MAX_RING_SIZE		512

enum al_udma_push_desc_db_select {
	AL_UDMA_PUSH_DESC_DB_SELECT_DESCRIPTOR = 0,
	AL_UDMA_PUSH_DESC_DB_SELECT_DOORBELL = 1
};

enum al_udma_push_tx_rx_select {
	AL_UDMA_PUSH_TX_RX_SELECT_TX = 0,
	AL_UDMA_PUSH_TX_RX_SELECT_RX = 1
};

/**
 * Get the physical address of the descriptors ring when in push mode.
 *
 * @param queue_id
 * @param tx_rx_select			whether this queue is an tx or rx queue.
 * @param axi_stream_udma_push_base	the base address of this udma unit on the axi_stream bus.
 *                                      please use HAL push mode api to get that base address.
 *
 * @return the physical address that represents the descriptors ring of this queue.
 */
al_phys_addr_t al_udma_push_get_q_ring_address(uint16_t queue_id,
					enum al_udma_push_tx_rx_select tx_rx_select,
					al_phys_addr_t axi_stream_udma_push_base);


#endif /* __AL_HAL_UDMA_PUSH_MODE_H__ */
