#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/console.h>
#include <asm/bootinfo.h>
#include <asm/prom.h>
#include <asm/mips-cps.h>
#include <asm/mips-cpc.h>
#include <asm/smp-ops.h>
#include <asm/rb/boards.h>
#include <asm/rb/booter.h>
#include <linux/ioport.h>
#include <linux/ctype.h>
#include <linux/irq.h>
#include <linux/initrd.h>
#include <linux/libfdt.h>
#include <linux/of_fdt.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>

#define FREQ_TAG   "HZ="
#define HWOPT_TAG  "hw_opt="
#define BOARD_TAG  "board="
#define BOARD_TAG2  "Board="

#define SR_NMI	0x00180000

extern void rb400_setup(void);
extern void rbvm_setup(void);
extern __init void music_setup(void);
extern void ralink_setup(void);

extern char arcs_cmdline[COMMAND_LINE_SIZE];

extern unsigned long mips_hpt_frequency;

unsigned char mips_mac_address[6];
static unsigned hw_opt = 0;
static char board_name[64] = {0};

unsigned long mips_machgroup __read_mostly = 0;
EXPORT_SYMBOL(mips_machgroup);

static int ar9330_serial;

void prom_putchar(char c) {
#if 1    
    extern void serial8250_prom_putchar(char c);
    
    if (ar9330_serial) {
	volatile unsigned *uart = (volatile unsigned *) 0xb8020000;
	while ((*uart & 0x200) == 0);
	*uart = 0x200 | (unsigned) c;
    } else {
	serial8250_prom_putchar(c);
    }
#else
    volatile unsigned *uart = (volatile unsigned *) 0xbe000c00;
    while ((uart[5] & (1 << 6)) == 0);
    uart[0] = (unsigned) c;
#endif
}

static unsigned get_flash_base(void) {
    if (hw_opt & HW_OPT_NO_NAND)
	return 0x1f000000;

    switch (mips_machtype) {
    case MACH_MT_RB951:
    case MACH_MT_RB750r2:
    case MACH_MT_CM2N:
    case MACH_MT_RB962:
    case MACH_MT_RB960:
    case MACH_MT_OMNI_SC:
    case MACH_MT_SXT5N:
    case MACH_MT_mAPL:
    case MACH_MT_LTAP_hb:
    case MACH_MT_PLC_hb:
    case MACH_MT_SXTR_hb:
    case MACH_MT_LHGR_hb:
    case MACH_MT_924_hb:
    case MACH_MT_CRS332:
    case MACH_MT_CRS326:
    case MACH_MT_CRS312:
    case MACH_MT_CRS354:
    case MACH_MT_LHG60_hb:
    case MACH_MT_mAP2:
    case MACH_MT_SXTG5P:
    case MACH_MT_SXTG5C:
    case MACH_MT_CRS210:
    case MACH_MT_CRS212:
    case MACH_MT_CCS226:
    case MACH_MT_GROOVE52:
    case MACH_MT_RB941HL:
	return 0x1f000000;
    default:
	return 0x1fc00000;
    }
}

static void nor_read(void *buf, int count, unsigned long offset) {
    static unsigned char *flash_base = NULL;
    if (!flash_base) flash_base = ioremap(get_flash_base(), 0x20000);
    memcpy(buf, flash_base + offset, count);
}

static unsigned read_unsigned(unsigned *offset) {
    unsigned val = 0;
    nor_read(&val, sizeof(unsigned), *offset);
    *offset += sizeof(unsigned);
    return val;    
}

static int read_booter_cfg(unsigned id, void *buf, int amount) {
    unsigned offset;
    unsigned hcfg_offset;
    unsigned hcfg_size;
    if (mips_machgroup == MACH_GROUP_MT_RB700
	|| mips_machgroup == MACH_GROUP_MT_RB400) {
	offset = 0x24;
    }
    else if (mips_machgroup == MACH_GROUP_MT_MUSIC) {
	offset = 0x14;
    }
    else {
	printk("read_booter_cfg not implemented for this hardware");
	return -1;
    }
    hcfg_offset = read_unsigned(&offset);
    hcfg_size = read_unsigned(&offset);
    printk("hcfg_offset=%08x\n", hcfg_offset);
    printk("hcfg_size=%08x\n", hcfg_size);
    offset = hcfg_offset;
    if (read_unsigned(&offset) != 0x64726148) return 0;
    while (offset < hcfg_offset + hcfg_size) {
	unsigned data = read_unsigned(&offset);
	unsigned tag = data & 0xffff;	
	int len = data >> 16;	
	if (len == 0 || len > hcfg_size || tag == 0) {
	    break;
	} else if (tag == id) {
	    amount = min(len, amount);
	    nor_read(buf, amount, offset);
	    return amount;
	}
	offset = offset + len;
    }
    return 0;
}

unsigned read_hw_opt(void) {
	if (hw_opt == 0) {
		read_booter_cfg(ID_HW_OPTIONS, &hw_opt, 4);
	}
	return hw_opt;
}

static int add_number(char *ver, int len, int next, int n, unsigned *res) {
    int i, value = 0;
    if (next < len && kstrtoint(ver + next, 10, &value) == 0) {
	*res |= (value & 0xff) << n;
	for (i = next; i < len - 1; i++) {
	    if (ver[i] == 0) {
		next = i + 1;
		break;
	    }
	}
    }
    return next;
}

unsigned read_booter_ver(void) {
	char ver[16];
	int i, next = 0;
	unsigned result = 0;
	int len = read_booter_cfg(ID_BIOS_VERSION, ver, 16);
	for (i = 0; i < len; i++) {
	    if (ver[i] == '.' || i == len - 1) ver[i] = 0;
	}
	for (i = 24; i >= 0; i -= 8) {
	    next = add_number(ver, len, next, i, &result);
	}
	return result;
}

char *read_rb_name(void) {
	if (board_name[0] == 0) {
		read_booter_cfg(ID_BOARD_NAME, board_name, sizeof(board_name) - 1);
	}
	return board_name;
}

const char *get_system_type(void)
{
	switch (mips_machgroup) {
	case MACH_GROUP_MT_RB400:
		switch (mips_machtype) {
		case MACH_MT_RB411:
			return "Mikrotik RB411";
		case MACH_MT_RB411U:
			return "Mikrotik RB411U";
		case MACH_MT_RB411L:
			return "Mikrotik RB411L";
		case MACH_MT_RB411UL:
			return "Mikrotik RB411UL";
		case MACH_MT_RB411G:
			return "Mikrotik RB411GL";
		case MACH_MT_RB433:
			return "Mikrotik RB433";
		case MACH_MT_RB433U:
			return "Mikrotik RB433U";
		case MACH_MT_RB433GL:
			return "Mikrotik RB433GL";
		case MACH_MT_RB433UL:
			return "Mikrotik RB433UL";
		case MACH_MT_RB433L:
			return "Mikrotik RB433L";
		case MACH_MT_RB435G:
			return "Mikrotik RB435G";
		case MACH_MT_RB450:
			return "Mikrotik RB450";
		case MACH_MT_RB450G:
			return "Mikrotik RB450G";
		case MACH_MT_RB493:
			return "Mikrotik RB493";
		case MACH_MT_RB750G:
			return "Mikrotik RB750G";
		}
		break;
	case MACH_GROUP_MT_RB700:
		switch (mips_machtype) {
		case MACH_MT_RB711R3:
		case MACH_MT_RB711:
			return "Mikrotik RB711";
		case MACH_MT_RB_SXT5D:
			return "Mikrotik SXT 5-HnD";
		case MACH_MT_RB_GROOVE:
		case MACH_MT_RB_GROOVE_5S:
			return "Mikrotik Groove-5H";
		case MACH_MT_RB751G:
			return "Mikrotik RB751G";
		case MACH_MT_RB912G:
			return "Mikrotik RB912G";
		case MACH_MT_RB711G:
			return "Mikrotik RB711G";
		case MACH_MT_RB_SXTG:
			return "Mikrotik SXT G-5HnD";
		case MACH_MT_RB750GL:
			return "Mikrotik RB750GL";
		case MACH_MT_OMNITIK_WASP:
			return "Mikrotik OmniTIK 5HnD";
		case MACH_MT_SXT2D:
			return "Mikrotik SXT 2nD";
		case MACH_MT_SXT5N:
			return "Mikrotik SXT 5nD";
		case MACH_MT_SXTG5P:
			return "Mikrotik SXT G-5HPnD";
		case MACH_MT_SXTG5C:
			return "Mikrotik SXT G-5HPacD";
		case MACH_MT_SXT_LTE:
			return "Mikrotik SXT LTE";
		case MACH_MT_RB922GS:
			return "Mikrotik RB922UAGS-5HPacD";
		case MACH_MT_951HND:
			return "Mikrotik RB951-2HnD";
		case MACH_MT_RB2011G:
			return "Mikrotik RB2011UAS-2HnD";
		case MACH_MT_RB2011US:
			return "Mikrotik RB2011UAS";
		case MACH_MT_RB2011L:
			return "Mikrotik RB2011L";
		case MACH_MT_RB2011LS:
			return "Mikrotik RB2011LS";
		case MACH_MT_RB2011R5:
			return "Mikrotik RB2011";
		case MACH_MT_RB953GS:
			return "Mikrotik RB953GS";
		case MACH_MT_CRS125G:
			return "Mikrotik CRS125G";
		case MACH_MT_CRS109:
			return "Mikrotik CRS109";
		case MACH_MT_CRS210:
			return "Mikrotik CRS210";
		case MACH_MT_CRS212:
			return "Mikrotik CRS212";
		case MACH_MT_RB951G:
			return "Mikrotik RB951G";
		case MACH_MT_RB951:
			return "Mikrotik RB951-2n";
		case MACH_MT_CM2N:
			return "Mikrotik cAP";
		case MACH_MT_mAP:
			return "Mikrotik mAP";
		case MACH_MT_GROOVE52:
			return "Mikrotik Groove 52HPn";
		case MACH_MT_RB750:
			return "Mikrotik RB750";
		case MACH_MT_RB751:
			return "Mikrotik RB751";
		case MACH_MT_RB_OMNI:
		case MACH_MT_RB_OMNI_5FE:
			return "Mikrotik OmniTIK";
		}
		break;
	case MACH_GROUP_MT_MMIPS:
		return "MikroTIK mmips";
	case MACH_GROUP_MT_VM:
		return "Mikrotik VM";
	}
	return "unknown routerboard";
}

void __init prom_init(void)
{
	int argc = fw_arg0;
	char **argv = (char **) fw_arg1;
	unsigned char board_type[16];

	unsigned i, offset = 0;

	set_io_port_base(KSEG1);

	memset(board_type, 0, sizeof(board_type));

	/* HZ must be parsed here because otherwise is too late */
	for (i = 0; (i < argc && argv[i] != NULL); i++) {
		if (strncmp(argv[i], BOARD_TAG2, sizeof(BOARD_TAG2) - 1) == 0) {
			argv[i][0] = 'b';
		}
		if (strncmp(argv[i], FREQ_TAG, sizeof(FREQ_TAG) - 1) == 0) {
			mips_hpt_frequency = 
			    simple_strtoul(argv[i] + sizeof(FREQ_TAG) - 1, 0, 10);
			continue;
		}
		if (strncmp(argv[i], HWOPT_TAG, sizeof(HWOPT_TAG) - 1) == 0) {
			hw_opt = simple_strtoul(argv[i] + sizeof(HWOPT_TAG) - 1,
						0, 16);
		}
		if (strncmp(argv[i], BOARD_TAG, sizeof(BOARD_TAG) - 1) == 0) {
			strncpy(board_type, argv[i] + sizeof(BOARD_TAG) - 1,
				sizeof(board_type));
		}
		offset += snprintf(arcs_cmdline + offset, sizeof(arcs_cmdline) - offset,
				   "%s ", argv[i]);
	}

	mips_machgroup = MACH_GROUP_MT_MMIPS;
	if (strncmp(board_type, "750g-mt", sizeof(board_type)) == 0) {
		mips_machtype = MACH_MT_RB750Gr3;
	}
	else if (strncmp(board_type, "750gr4-mt", sizeof(board_type)) == 0) {
		mips_machtype = MACH_MT_RB750Gr4;
	}
	else if (strncmp(board_type, "m11", sizeof(board_type)) == 0) {
		mips_machtype = MACH_MT_M11;
	}
	else if (strncmp(board_type, "m33", sizeof(board_type)) == 0) {
		mips_machtype = MACH_MT_M33;
	}
	else if (strncmp(board_type, "ltap-mt", sizeof(board_type)) == 0) {
		mips_machtype = MACH_MT_LtAP;
	} else if (strncmp(board_type, "411", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB411;
	} else if (strncmp(board_type, "411L", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB411L;
	} else if (strncmp(board_type, "411UL", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB411UL;
	} else if (strncmp(board_type, "411G", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB411G;
	} else if (strncmp(board_type, "750G", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB750G;
	} else if (strncmp(board_type, "750i", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB750;
	} else if (strncmp(board_type, "751", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB751;
	} else if (strncmp(board_type, "711r3", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB711R3;
	} else if (strncmp(board_type, "711", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB711;
	} else if (strncmp(board_type, "groove", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB_SXT5D;
	} else if (strncmp(board_type, "751g", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB751G;
	} else if (strncmp(board_type, "711GT", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB711GT;
	} else if (strncmp(board_type, "711G", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB711G;
	} else if (strncmp(board_type, "SXTG", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB_SXTG;
	} else if (strncmp(board_type, "711Gr100", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB912G;
	} else if (strncmp(board_type, "750Gr3", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB750GL;
	} else if (strncmp(board_type, "omni-9344", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_OMNITIK_WASP;
	} else if (strncmp(board_type, "911L", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB911L;
	} else if (strncmp(board_type, "lhg", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_LHG;
	} else if (strncmp(board_type, "952-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB952;
	} else if (strncmp(board_type, "750-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB750r2;
	} else if (strncmp(board_type, "g52c", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_GROOVE_52AC;
	} else if (strncmp(board_type, "962", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB962;
	} else if (strncmp(board_type, "960", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB960;
	} else if (strncmp(board_type, "wsap-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_wsAP;
	} else if (strncmp(board_type, "omni-sc", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_OMNI_SC;
	} else if (strncmp(board_type, "750g-sc", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB750Gv2;
	} else if (strncmp(board_type, "H951L", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB941HL;
	} else if (strncmp(board_type, "931", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB931;
	} else if (strncmp(board_type, "wap-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_wAP;
	} else if (strncmp(board_type, "cap-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_cAP;
	} else if (strncmp(board_type, "lhg-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_LHG_hb;
	} else if (strncmp(board_type, "ltap-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_LTAP_hb;
	} else if (strncmp(board_type, "plc-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_PLC_hb;
	} else if (strncmp(board_type, "lhg60-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_LHG60_hb;
	} else if (strncmp(board_type, "wap-lte", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_wAP_LTE;
	} else if (strncmp(board_type, "map-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_mAPL;
	} else if (strncmp(board_type, "map2-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_mAP2;
	} else if (strncmp(board_type, "wapg-sc", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_wAPG;
	} else if (strncmp(board_type, "sxtr-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_SXTR_hb;
	} else if (strncmp(board_type, "lhgr-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_LHGR_hb;
	} else if (strncmp(board_type, "924-hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_924_hb;
	} else if (strncmp(board_type, "crs332", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_CRS332;
	} else if (strncmp(board_type, "crs326hb", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_CRS326;
	} else if (strncmp(board_type, "crs312", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_CRS312;
	} else if (strncmp(board_type, "crs354", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_CRS354;
	} else if (strncmp(board_type, "sxt-ac", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_SXT_AC;
	} else if (strncmp(board_type, "sxt2d", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_SXT2D;
	} else if (strncmp(board_type, "sxt5n", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_SXT5N;
	} else if (strncmp(board_type, "sxtg5p", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_SXTG5P;
	} else if (strncmp(board_type, "sxtg5c", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_SXTG5C;
	} else if (strncmp(board_type, "sxt-lte", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_SXT_LTE;
	} else if (strncmp(board_type, "922gs", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB922GS;
	} else if (strncmp(board_type, "951HnD", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_951HND;
	} else if (strncmp(board_type, "2011G", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB2011G;
	} else if (strncmp(board_type, "2011US", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB2011US;
	} else if (strncmp(board_type, "2011L", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB2011L;
	} else if (strncmp(board_type, "2011LS", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB2011LS;
	} else if (strncmp(board_type, "2011r5", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB2011R5;
	} else if (strncmp(board_type, "953gs", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB953GS;
	} else if (strncmp(board_type, "crs125g", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_CRS125G;
	} else if (strncmp(board_type, "crs109", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_CRS109;
	} else if (strncmp(board_type, "951G", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB951G;
	} else if (strncmp(board_type, "qrtg", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_QRTG;
	} else if (strncmp(board_type, "cm2n", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_CM2N;
		ar9330_serial = 1;
	} else if (strncmp(board_type, "mAP", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_mAP;
		ar9330_serial = 1;
	} else if (strncmp(board_type, "751UL", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB951;
		ar9330_serial = 1;
	} else if (strncmp(board_type, "groove-52", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_GROOVE52;
	} else if (strncmp(board_type, "groove-5", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB_GROOVE;
	} else if (strncmp(board_type, "groove-5s", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB_GROOVE_5S;
	} else if (strncmp(board_type, "omni", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB_OMNI;
	} else if (strncmp(board_type, "omni-5fe", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB_OMNI_5FE;
	} else if (strncmp(board_type, "411U", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB411U;
	} else if (strncmp(board_type, "433", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB433;
	} else if (strncmp(board_type, "433U", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB433U;
	} else if (strncmp(board_type, "433GL", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB433GL;
	} else if (strncmp(board_type, "433UL", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB433UL;
	} else if (strncmp(board_type, "433L", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB433L;
	} else if (strncmp(board_type, "435G", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB435G;
	} else if (strncmp(board_type, "450", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB450;
	} else if (strncmp(board_type, "450G", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB450G;
	} else if (strncmp(board_type, "493G", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB493G;
	} else if (strncmp(board_type, "493", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_RB400;
		mips_machtype = MACH_MT_RB493;
	} else if (strncmp(board_type, "vm", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_VM;
		mips_machtype = 0;
	} else if (strncmp(board_type, "music", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_MUSIC;
		mips_machtype = MACH_MT_CCS226;
	} else if (strncmp(board_type, "crs210", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_MUSIC;
		mips_machtype = MACH_MT_CRS210;
	} else if (strncmp(board_type, "crs212", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_MUSIC;
		mips_machtype = MACH_MT_CRS212;
	} else if (strncmp(board_type, "ccs112", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_MUSIC;
		mips_machtype = MACH_MT_CCS112;
	} else if (strncmp(board_type, "ccs112r4", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_MUSIC;
		mips_machtype = MACH_MT_CCS112R4;
	} else if (strncmp(board_type, "crs106", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_MUSIC;
		mips_machtype = MACH_MT_CRS106;
	} else if (strncmp(board_type, "crs105", sizeof(board_type)) == 0) {
		mips_machgroup = MACH_GROUP_MT_MUSIC;
		mips_machtype = MACH_MT_CRS105;
	} else {
		mips_machgroup = MACH_GROUP_MT_RB700;
		mips_machtype = MACH_MT_RB750;
	}

	if (mips_machgroup == MACH_GROUP_MT_RB400 ||
	    mips_machgroup == MACH_GROUP_MT_RB700) {
		setup_8250_early_printk_port(CKSEG1ADDR(0x18020000), 2, 0);
	} else if (mips_machgroup == MACH_GROUP_MT_MMIPS) {
		setup_8250_early_printk_port(CKSEG1ADDR(0x1e000c00), 2, 0);
	}

	mips_cm_probe();
	if (mips_machgroup == MACH_GROUP_MT_MMIPS) {
		/* Set L2 CCA Override to WT on lowest 256MB */
		write_gcr_reg3_mask(~(256 * 1024 * 1024 - 1) | 0x11);
		write_gcr_reg3_base(0);
	}
	
	mips_cpc_probe();
	
	register_cps_smp_ops();
}

void __init prom_free_prom_memory(void)
{
}

void __init plat_mem_setup(void)
{
#ifdef CONFIG_BLK_DEV_INITRD
	extern int _end;
	u32 *initrd_header;

	initrd_header = __va(PAGE_ALIGN(__pa_symbol(&_end) + 8)) - 8;
	if (initrd_header[0] == 0x494E5244) {
		initrd_start = (unsigned long) (initrd_header + 2);
                initrd_end = initrd_start + initrd_header[1];
	}
#endif

	switch (mips_machgroup) {
	case MACH_GROUP_MT_RB400:
	case MACH_GROUP_MT_RB700:
		rb400_setup();
		break;
	case MACH_GROUP_MT_MUSIC:
		music_setup();
		break;
	case MACH_GROUP_MT_MMIPS:
		ralink_setup();
		break;
	case MACH_GROUP_MT_VM:
		rbvm_setup();
		break;
	}
}

#ifdef CONFIG_USE_OF

struct device_node *enable_device(const char *path) {
	struct device_node *nd;
	struct property *p;

	nd = of_find_node_by_path(path);
	if (nd) {
		p = of_find_property(nd, "status", NULL);
		if (p) of_remove_property(nd, p);
	}
	return nd;
}

void __init device_tree_init(void)
{
	if (mips_machgroup == MACH_GROUP_MT_MMIPS) {
		extern char __dtb_mt7621_begin;
		static unsigned port_map[] = {
		    cpu_to_be32(0), cpu_to_be32(1), cpu_to_be32(2),
		    cpu_to_be32(3), cpu_to_be32(4), cpu_to_be32(0x120005)
		};
		static struct property ports_prop = {
		    .name = "ports",
		    .value = port_map,
		};
		static unsigned cpu_freq;
		static struct property cpu_clock = {
		    .name = "clock-frequency",
		    .value = &cpu_freq,
		    .length = sizeof(unsigned),
		};
		struct device_node *ether;
		struct device_node *clk;
		struct device_node *nd;
		struct property *p;

		initial_boot_params = &__dtb_mt7621_begin;
		__dt_setup_arch(&__dtb_mt7621_begin);
		unflatten_and_copy_device_tree();

		if (mips_machtype == MACH_MT_RB750Gr3
				|| mips_machtype == MACH_MT_RB750Gr4) {
			unsigned hw_opt = read_hw_opt();

			if (hw_opt & HW_OPT_HAS_USB) enable_device("/xhci@1e1c0000");
			if (hw_opt & HW_OPT_HAS_uSD) enable_device("/sdhci@1e130000");
			enable_device("/palmbus@1e000000/spi@b00/m25p80@0");
			enable_device("/palmbus@1e000000/spi@b00/ts@1");
			enable_device("/beeper");
			enable_device("/rb760-leds");
			ports_prop.length = hw_opt & HW_OPT_HAS_SFP ?
			    6 * sizeof(unsigned) : 5 * sizeof(unsigned);
			nd = enable_device("/poe-simple");
			if (nd && mips_machtype == MACH_MT_RB750Gr3) {
				p = of_find_property(nd, "has_low_current_sense", NULL);
				if (p) of_remove_property(nd, p);
			}
		} else if (mips_machtype == MACH_MT_M33) {
			enable_device("/pcie@1e140000");
			enable_device("/sdhci@1e130000");
			enable_device("/xhci@1e1c0000");
			enable_device("/palmbus@1e000000/uartlite@e00");
			enable_device("/palmbus@1e000000/spi@b00/m25p80_boot@0");
			enable_device("/palmbus@1e000000/spi@b00/m25p80@1");
			enable_device("/m33-leds");
			ports_prop.length = 3 * sizeof(unsigned);
		} else if (mips_machtype == MACH_MT_M11) {
			enable_device("/pcie@1e140000");
			enable_device("/xhci@1e1c0000");
			enable_device("/palmbus@1e000000/spi@b00/m25p80@0");
			enable_device("/m11-leds");
			ports_prop.length = 1 * sizeof(unsigned);
		} else if (mips_machtype == MACH_MT_LtAP) {
			enable_device("/pcie@1e140000");
			enable_device("/palmbus@1e000000/uartlite@e00");
			enable_device("/palmbus@1e000000/spi@b00/m25p80@0");
			enable_device("/palmbus@1e000000/spi@b00/ts@1");
			enable_device("/sdhci@1e130000");
			enable_device("/xhci@1e1c0000");
			enable_device("/ltap-leds");
			ports_prop.length = 1 * sizeof(unsigned);
		}

		ether = of_find_node_by_path("/ethernet@1e100000");
		if (ether) {
			of_add_property(ether, &ports_prop);
		}

		clk = of_find_node_by_path("/cpuclock@0");
		if (clk) {
			cpu_freq = cpu_to_be32(mips_hpt_frequency * 2);
			of_add_property(clk, &cpu_clock);
		}
	}
}

#endif

void __init plat_time_init(void)
{
#ifdef CONFIG_USE_OF
	of_clk_init(NULL);
	timer_probe();
#endif	
}

static int __init setup_kmac(char *s)
{
	int i, j;
	unsigned char result, value;

	for (i = 0; i < 6; i++) {
		if (s[0] == '\0' || s[1] == '\0') return 0;
		if (i != 5 && s[2] != ':') return 0;

		result = 0;
		for (j = 0; j < 2; j++) {
			if (!isxdigit(*s)) return 0;

			value = isdigit(*s) ? *s - '0' :
				toupper(*s) - 'A' + 10;
			if (value >= 16) return 0;

			result = result * 16 + value;
                        s++;
                }

                s++;
                mips_mac_address[i] = result;
        }

        return *s == '\0';
}

__setup("kmac=", setup_kmac);

EXPORT_SYMBOL(mips_mac_address);

int is_crs_type(void) {
    if (mips_machtype == MACH_MT_CRS125G) return 1;
    if (mips_machtype == MACH_MT_CRS109) return 1;
    return 0;
}
EXPORT_SYMBOL(is_crs_type);

int is_music_lite(void) {
	if (mips_machtype == MACH_MT_CRS105
	    || mips_machtype == MACH_MT_CRS106) return true;
	if (mips_machtype == MACH_MT_CCS112R4) return true;
	return mips_machtype == MACH_MT_CCS112;
}
EXPORT_SYMBOL(is_music_lite);
