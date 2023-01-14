#ifndef _LINUX_IF_SWITCH_H
#define _LINUX_IF_SWITCH_H

#include <linux/types.h>

// only 16 ioctls available
#define SIOCGTYPE	(SIOCDEVPRIVATE + 0)
#define SIOCGSWITCH	(SIOCDEVPRIVATE + 1)
#define SIOCSTXBW	(SIOCDEVPRIVATE + 2)
#define SIOCSRXBW	(SIOCDEVPRIVATE + 3)
#define SIOCSSWITCHED	(SIOCDEVPRIVATE + 4)
#define SIOCSMIRROR	(SIOCDEVPRIVATE + 5)
#define SIOCGREG	(SIOCDEVPRIVATE + 8)
#define SIOCSREG	(SIOCDEVPRIVATE + 9)
#define SIOCGPORT	(SIOCDEVPRIVATE + 10)
#define SIOCGCONFIG	(SIOCDEVPRIVATE + 11)
#define SIOCSCONFIG	(SIOCDEVPRIVATE + 12)
#define SIOCBRIDGE 	(SIOCDEVPRIVATE + 14)
#define SIOCSWICHMAX	(SIOCDEVPRIVATE + 15)

struct switch_params {
    unsigned data;
    unsigned data2;
};

struct switch_fdb_params {
    __u16 vid;
    __u8 mac[6];
    __u8 add;
    __u8 is_static;
};

struct switch_bridge_params {
   __u32 flags;
   __u32 tpid;
};

struct switch_hw_debug_params {
   __u32 in_addr;
   __u32 out_addr;
};

struct switch_hw_generator {
   __u16 start;
   __u16 pkt_len;
   __u16 pkt_count;
   __u16 pkt_multiplier;
   __u8 mac_da[6];
   __u8 mac_sa[6];
};

struct switch_cb_add_port_params {
   __u8 mac[6];
};

struct switch_cb_set_port_params {
   int ifindex;
   int cascLidx;
   __u16 pcid;
};

struct switch_cb_unset_port_params {
   int ifindex;
};

struct switch_bpe_echan_params {
    __u16 pcid;
    __u16 ecid;
};

struct switch_mirror1 {
    __u32 source;
    __u32 target;
};

#define SW_P_MIRR_INGRESS BIT(0)
#define SW_P_MIRR_EGRESS BIT(1)

#define SWITCH_L3HW_FLAGS_TYPE(flags) ((flags) >> 28)
#define SWITCH_L3HW_FLAGS_TYPE_GLOBAL 0
#define SWITCH_L3HW_FLAGS_TYPE_PORT   8
#define SWITCH_L3HW_FLAGS_SET_GLOBAL(flags) ( \
        (SWITCH_L3HW_FLAGS_TYPE_GLOBAL << 28) \
        | ((flags) & 0x0FFFFFFF))
#define SWITCH_L3HW_FLAGS_SET_PORT(portnum, flags) ( \
        (SWITCH_L3HW_FLAGS_TYPE_PORT << 28) \
        | (((portnum) & 0x0FFF) << 16) | (flags) & 0xFFFF)
#define SWITCH_L3HW_FLAGS_GET_GLOBAL(flags) ((flags) & 0x0FFFFFFF)
#define SWITCH_L3HW_FLAGS_GET_PORT_NUM(flags) (((flags) >> 16) & 0x0FFF)
#define SWITCH_L3HW_FLAGS_GET_PORT_FLAGS(flags) ((flags) & 0xFFFF)

#define SWITCH_ADMTEK		0
#define SWITCH_ICPLUS175C	1

#define MIRROR_NONE -1u
#define MIRROR_CPU -2u

#define SWITCH_HW_DEBUG 0
#define SWITCH_BR_SET_STP 1
#define SWITCH_BR_SET_BRIDGE 2
#define SWITCH_BR_FLUSH 3
#define SWITCH_BR_ADD_VLAN 4
#define SWITCH_BR_DEL_VLAN 5
#define SWITCH_BR_SET_VLAN_OPTS 6
#define SWITCH_BR_SET_MSTI 7
#define SWITCH_BR_GET_BOND_ID 8
#define SWITCH_BR_GET_BOND_LID 9
#define SWITCH_BR_ADD_FDB 10
#define SWITCH_BR_SET_PORT_OPTS 11
#define SWITCH_BR_SET_AGING 12
#define SWITCH_BR_SET_FWD_MASK 13
#define SWITCH_BR_SET_EG_RATE 14
#define SWITCH_HW_SET_GENERATOR 15
#define SWITCH_BR_GET_FDB 16
#define SWITCH_HW_L3_CONTROL 17

#define SWITCH_BR_EXT_INIT 20
#define SWITCH_BR_EXT_STOP 21
#define SWITCH_BR_EXT_SET_UPSTREAM 22
#define SWITCH_BR_EXT_ADD_EXTENDED 23
#define SWITCH_BR_EXT_SET_EXTENDED 24
#define SWITCH_BR_EXT_DEL_EXTENDED 25
#define SWITCH_BR_EXT_SET_VLAN 26
#define SWITCH_BR_EXT_ADD_ECHAN 27
#define SWITCH_BR_EXT_DEL_ECHAN 28

#define SWITCH_BR_CB_INIT 30
#define SWITCH_BR_CB_STOP 31
#define SWITCH_BR_CB_SET_CASC 32
#define SWITCH_BR_CB_UNSET_CASC 33
#define SWITCH_BR_CB_ADD_IFACE 34
#define SWITCH_BR_CB_DEL_IFACE 35
#define SWITCH_BR_CB_DEL_ALL 36
#define SWITCH_BR_CB_EXTETH_SET_STATS 37
#define SWITCH_BR_CB_EXTETH_SET_CARRIER 38
#define SWITCH_BR_CB_SET_IFACE 39
#define SWITCH_BR_CB_UNSET_IFACE 40


#define STP_DISABLE 0
#define STP_BLOCK   1
#define STP_LEARN   2
#define STP_FORWARD 3

// see if_bridge.h
#define SW_VLAN_PVID BIT(1)
#define SW_VLAN_UNTAGGED BIT(2)
#define SW_VLAN_FORCE BIT(3)

#define SW_SET_SERDES_LOOPBACK 1
#define SW_SET_PRBS 2
#define SW_GET_PRBS_STATUS 3
#define SW_GET_EOM 4
#define SW_RUN_AUTO_TUNE 5
#define SW_SET_TX_CFG 6
#define SW_GET_SERDES_CFG 7
#define SW_SET_PHY_LOOPBACK 8
#define SW_SET_PHY_PRBS 9
#define SW_GET_PHY_PRBS_STATUS 10
#define SW_SET_RX_CFG 11
#define SW_GET_E1690 12
#define SW_SET_E1690 13
#define SW_GET_E1690_XSMI 14
#define SW_SET_E1690_XSMI 15
#define SW_GET_REG 16
#define SW_SET_REG 17
#define SW_GET_PHY_REG 18
#define SW_SET_PHY_REG 19
#define SW_INJECT_ERROR 20
#define SW_SET_ENH_MODE 21
#define SW_SET_ENH_LF 22

#endif
