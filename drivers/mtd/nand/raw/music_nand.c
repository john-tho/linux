#include <linux/init.h>
#include <linux/mtd/rawnand.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <asm/rb/rb400.h>

#define DEBUG_SPEED
#define MUSIC	1

extern int rb_nand_probe(struct nand_chip *nand, int booter);

static struct nand_chip rnand;

static struct spi_device *spi = 0;

#define GPIO_BASE	0x18080000
#define GPIO_OE_REG	    0x0000
#define GPIO_IN_REG	    0x0004
#define GPIO_OUTPUT	    0x0008
#define   GPI_NAND_RDY		(1 << 0)
#define GPIO_SET	    0x000C
#define GPIO_CLEAR	    0x0010

#define GPIO_REG(x)	(*(volatile unsigned *)((unsigned) gpio_base + (x)))

static void __iomem *gpio_base;

#define   CFG_BIT_NAND_nCE	0x80
#define   CFG_BIT_NAND_CLE	0x40
#define   CFG_BIT_NAND_ALE	0x20
#define   CFG_BIT_NAND_READ	0x10
#define	 SPI1_CMD_NAND		0x01

#define  SPI1_CMD_CFG		(0 << 0)
#define   CFG_BIT_INT_RST	(1 << 4)
#define   CFG_BIT_POE_nRST	(1 << 5)
#define   CFG_BIT_LCD_nRST	(1 << 6)
#define   CFG_BIT_LCD_nLED	(1 << 7)

static unsigned char nand_cfg = SPI1_CMD_NAND | CFG_BIT_NAND_nCE;

unsigned music_change_ssr_cfg(unsigned mask, unsigned or) {
	static unsigned char val = (SPI1_CMD_CFG |
				    CFG_BIT_POE_nRST |
				    CFG_BIT_LCD_nRST);
	val &= ~mask;
	val |= or;
	return spi_write(spi, &val, 1);
}
EXPORT_SYMBOL(music_change_ssr_cfg);

/* ------------------ MUSIC SPI nand driver -------------------- */

static int music_dev_ready(struct nand_chip *chip) {
	return GPIO_REG(GPIO_IN_REG) & GPI_NAND_RDY;
}

static void music_flush_nand_cfg(void) {
	spi_write(spi, &nand_cfg, 1);
}

static void music_nandio_write(const uint8_t *data, unsigned cnt) {
	struct spi_transfer t[3] = {
		{
			.tx_buf = &nand_cfg,
			.len = 1,
		},
		{
			.tx_buf = data,
			.len = cnt,
		},
		{
			.len = 1,
		},
	};
	struct spi_message m;

	nand_cfg &= ~CFG_BIT_NAND_READ;

	spi_message_init_with_transfers(&m, t, 3);
	spi_sync(spi, &m);
}

static int music_nandio_read(uint8_t *rdata, unsigned cnt) {
	struct spi_transfer t[3] = {
		{
			.tx_buf = &nand_cfg,
			.len = 1,
		},
		{
			.len = 1,
		},
		{
			.rx_buf = rdata,
			.len = cnt,
		},
	};
	struct spi_message m;

	nand_cfg |= CFG_BIT_NAND_READ;

	spi_message_init_with_transfers(&m, t, 3);
	return spi_sync(spi, &m);
}

static void music_hwcontrol(struct nand_chip *chip, int cmd,
			    unsigned int ctrl) {
	static uint8_t data[8];
	static unsigned dlen = 0;

	if (ctrl & NAND_CTRL_CHANGE) {
		if (dlen) {
			music_nandio_write(data, dlen);
			dlen = 0;
		}
		nand_cfg &= ~(CFG_BIT_NAND_CLE |
			      CFG_BIT_NAND_ALE |
			      CFG_BIT_NAND_nCE);

		if (ctrl & NAND_CLE) {
			nand_cfg |= CFG_BIT_NAND_CLE;
		}
		if (ctrl & NAND_ALE) {
			nand_cfg |= CFG_BIT_NAND_ALE;
		}
		if (!(ctrl & NAND_NCE)) {
			nand_cfg |= CFG_BIT_NAND_nCE;
		}
		if ((ctrl & (NAND_CLE | NAND_ALE)) == 0) {
			music_flush_nand_cfg();
		}
	}

	if (cmd != NAND_CMD_NONE) {
		data[dlen] = cmd;
		++dlen;
		if (dlen == sizeof(data)) {
			music_nandio_write(data, dlen);
			dlen = 0;
		}
	}
}

static uint8_t music_read_byte(struct nand_chip *chip)
{
	uint8_t byte = 0;
	music_nandio_read(&byte, 1);
	return byte;
}

static void music_write_buf(struct nand_chip *chip, const uint8_t *buf, int len)
{
	music_nandio_write(buf, len);
}

static void music_read_buf(struct nand_chip *chip, uint8_t *buf, int len)
{
	music_nandio_read(buf, len);
}

#ifdef DEBUG_SPEED
static void music_nand_test_speed(void) {
	char buf[512];
	unsigned long ticks;
	unsigned kb;

	/* wait for "start of" clock tick */
	kb = 0;
	ticks = jiffies;
	while (ticks == jiffies)
		/* nothing */;
	ticks = jiffies + HZ / 10;

	while ((long)(jiffies - ticks) < 0) {
		music_read_buf(NULL, buf, 512);
		++kb;
	}
	printk("read speed is %u KB/s\n", kb * 10 / 2);

	/* wait for "start of" clock tick */
	kb = 0;
	ticks = jiffies;
	while (ticks == jiffies)
		/* nothing */;
	ticks = jiffies + HZ / 10;

	while ((long)(jiffies - ticks) < 0) {
		music_write_buf(NULL, buf, 512);
		++kb;
	}
	printk("write speed is %u KB/s\n", kb * 10 / 2);
}
#endif

static int music_spi_nand_probe(struct spi_device *_spi)
{
	printk("MUSIC spi nand\n");
	memset(&rnand, 0, sizeof(rnand));

	if (!gpio_base) {
		gpio_base = ioremap(GPIO_BASE, PAGE_SIZE);
		if (!gpio_base)
			return -ENOMEM;
	}

	spi = _spi;

#ifdef DEBUG_SPEED
	music_nand_test_speed();
#endif

	rnand.legacy.cmd_ctrl = music_hwcontrol;
	rnand.legacy.dev_ready = music_dev_ready;
	rnand.legacy.read_byte = music_read_byte;
	rnand.legacy.write_buf = music_write_buf;
	rnand.legacy.read_buf = music_read_buf;

	return rb_nand_probe(&rnand, 1);
}

static struct spi_driver music_spi_nand_driver = {
	.probe	= music_spi_nand_probe,
	.driver	= {
		.name = "music-spi-nand",
		.owner = THIS_MODULE,
	},
};

/* ------------------ common init/exit code -------------------- */

static int __init music_nand_init(void)
{
	return spi_register_driver(&music_spi_nand_driver);
}

static void __exit music_nand_exit(void)
{
	if (gpio_base)
		iounmap(gpio_base);

	spi_unregister_driver(&music_spi_nand_driver);
}

module_init(music_nand_init);
module_exit(music_nand_exit);
