#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/vmalloc.h>
#include <linux/leds-rb.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#ifdef CONFIG_MIPS_MIKROTIK
#include <asm/rb/boards.h>
#include <asm/rb/rb400.h>

#define GPIO_BASE	0x18040000
#define GPIO_BASE_MUSIC	0x18080000
#define GPIO_OE_REG	    0x0000
#define GPIO_IN_REG	    0x0004
#define GPIO_O1_REG	    0x0008
#define GPIO_S1_REG	    0x000c
#define GPIO_S0_REG	    0x0010
#endif

struct rb_led_data {
	struct led_classdev cdev;
	unsigned map;
	struct rb_led_data **colours;
};
static struct rb_led_data rb_leds[] = {
	{ .cdev = { .name = "user-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "led1" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "led2" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "led3" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "led4" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "led5" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "led6" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "usb-power-off" } },
	{ .cdev = { .name = "usb-power-key" } },
	{ .cdev = { .name = "power-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "wlan-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "mpcie-power-off" } },
	{ .cdev = { .name = "mpcie2-power-off" } },
	{ .cdev = { .name = "lcd" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "button" }, .map = PLED_CFG_INPUT },
	{ .cdev = { .name = "pin-hole" }, .map = PLED_CFG_INPUT },
	{ .cdev = { .name = "fan-off" } },
	{ .cdev = { .name = "user-led2" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "sfp-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "link-act-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "all-leds" } },
	{ .cdev = { .name = "omni-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "ap-omni-led" }, .map =  PLED_CFG_DARK },
	{ .cdev = { .name = "ap-dir-led" }, .map =  PLED_CFG_DARK },
	{ .cdev = { .name = "control" } },
	{ .cdev = { .name = "heater" } },
	{ .cdev = { .name = "mode-button" }, .map = PLED_CFG_INPUT },
	{ .cdev = { .name = "sim-select" } },
        { .cdev = { .name = "gps-mux" } },
        { .cdev = { .name = "gps-ant-select" } },
        { .cdev = { .name = "gps-reset" } },
        { .cdev = { .name = "monitor-select" } },
        { .cdev = { .name = "fault-led" }, .map = PLED_CFG_DARK },
        { .cdev = { .name = "psu1-state" } },
        { .cdev = { .name = "psu2-state" } },
        { .cdev = { .name = "lte-reset" } },
        { .cdev = { .name = "w-disable" } },
	{ .cdev = { .name = "lte-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "gps-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "gps-pps" }, .map = PLED_CFG_INPUT },
	{ .cdev = { .name = "gps-timer" }, .map = PLED_CFG_INPUT },
	{ .cdev = { .name = "led-up" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "led-down" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "led-left" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "led-right" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "led-ok" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "eth-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "system-eth-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "poe-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "system-led" }, .map = PLED_CFG_DARK },
	{ .cdev = { .name = "wps-button" }, .map = PLED_CFG_INPUT },
	{ .cdev = { .name = "system-wlan-led" }, .map = PLED_CFG_DARK },
        { .cdev = { .name = "lte-ant-sw1" } },
        { .cdev = { .name = "lte-ant-sw2" } },
        { .cdev = { .name = "fpga-reset" } },
	{ .cdev = { .name = "eth2-led" }, .map = PLED_CFG_DARK },
        { .cdev = { .name = "uart-select" } },
        { .cdev = { .name = "ext-pin1" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin2" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin3" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin4" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin5" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin6" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin7" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin8" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin9" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin10" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin11" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin12" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin13" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin14" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin15" }, .map = PLED_CFG_IO },
        { .cdev = { .name = "ext-pin16" }, .map = PLED_CFG_IO },
	/* name CAN contain '-' and MUST NOT contain '_' */
	/* XXX: keep in sync with enum pled_name in <linux/leds-rb.h> */
};

struct rb_rgb_led_colours {
	unsigned char mask;
	unsigned int shift;
	const char* name;
};

enum rgb7_shift {
	RGB7_red = 0,
	RGB7_green,
	RGB7_blue,
};

struct rb_rgb_led_colours rb_rgb7_led_colours[] = {
	{0xff, 0, "rgb7"},
	{0x1, RGB7_red, "red"},
	{0x2, RGB7_green, "green"},
	{0x4, RGB7_blue, "blue"},
};

static unsigned dark_mode_status;

static void (*set_wifi_gpo)(void *obj, unsigned off, unsigned on) = NULL;
static void *set_wifi_gpo_obj = NULL;
static unsigned wifi_gpo_mask = 0;

unsigned register_wifi_gpo(void *obj,
			   void (*set_gpo)(void *, unsigned, unsigned)) {
	set_wifi_gpo_obj = obj;
	set_wifi_gpo = set_gpo;
	return wifi_gpo_mask;
}
EXPORT_SYMBOL(register_wifi_gpo);

#ifdef CONFIG_MIPS_MIKROTIK
static void __iomem *gpio_base = NULL;

static void gpio_writel(unsigned val, unsigned ofs) {
	if (gpio_base) {
		rb400_writel(val, (unsigned) gpio_base + ofs);
	}
}

static unsigned gpio_readl(unsigned ofs) {
	if (gpio_base) {
		return rb400_readl((unsigned) gpio_base + ofs);
	}
	return 0;
}

static void gpio_rmw(unsigned ofs, unsigned off, unsigned on) {
	if (gpio_base) {
		unsigned val = rb400_readl((unsigned) gpio_base + ofs);
		val = (val | on) & ~off;
		rb400_writel(val, (unsigned) gpio_base + ofs);
	}
}
#endif

static int dark_mode_trig_activate(struct led_classdev *led_cdev)
{
	led_cdev->brightness_set(led_cdev, !dark_mode_status);
	return 0;
}

static struct led_trigger dark_mode_trigger = {
	.name = "dark-mode-trigger",
	.activate = dark_mode_trig_activate,
};

static void rb_led_set(struct led_classdev *led_cdev,
		       enum led_brightness brightness)
{
	struct rb_led_data *rbld = container_of(led_cdev,
				struct rb_led_data, cdev);
	unsigned map = rbld->map;
	unsigned bit = PLED_GET_BIT(map);
	int val = (!brightness) ^ (!(map & PLED_CFG_INV));
	unsigned on = val ? bit : 0;
	unsigned off = val ? 0: bit;
	int i;

	switch(PLED_GET_TYPE(map)) {
#ifdef CONFIG_GPIOLIB
	case PLED_TYPE_GPIOLIB:
		gpio_set_value_cansleep(PLED_GET_BIT_NUM(map), val);
		break;
#endif
#ifdef CONFIG_MIPS_MIKROTIK
	case PLED_TYPE_GPIO:
		gpio_writel(bit, val ? GPIO_S1_REG : GPIO_S0_REG);
		break;
	case PLED_TYPE_GPIO_OE:
		gpio_rmw(GPIO_OE_REG, off, on);
		break;
#endif
#ifdef CONFIG_SPI_RB400
	case PLED_TYPE_SHARED_RB400:
		rb400_change_cfg(on, off);
		break;
	case PLED_TYPE_SSR_RB400:
		rb_change_cfg(off, on);
		break;
#endif
#ifdef CONFIG_MTD_NAND_RB400
	case PLED_TYPE_SHARED_RB700:
		rb700_change_gpo(off, on);
		break;
	case PLED_TYPE_SSR_MUSIC:
		music_change_ssr_cfg(off, on);
		break;
	case PLED_TYPE_SHARED_RB900:
		ar9342_change_gpo(off, on);
		break;
	case PLED_TYPE_SSR_RB900:
		ar9342_change_gpo(off << 20, on << 20);
		break;
#endif
	case PLED_TYPE_WIFI:
		if (set_wifi_gpo) (*set_wifi_gpo)(set_wifi_gpo_obj, on, off);
		wifi_gpo_mask &= ~(bit << 16);
		wifi_gpo_mask |= ((on << 16) | bit);
		break;
	case PLED_TYPE_VIRTUAL:
		/* Do nothing */
		break;
	case PLED_TYPE_RGB7:
		for (i = 0; i < sizeof(rb_rgb7_led_colours) /
				sizeof(struct rb_rgb_led_colours); i++) {
			if (rbld->colours[i] && rbld->colours[i]->cdev.brightness_set) {
				struct rb_rgb_led_colours *col = &rb_rgb7_led_colours[i + 1];
				unsigned val = (brightness & col->mask) >> col->shift;
				rb_led_set(&(rbld->colours[i]->cdev), val);
				rbld->colours[i]->cdev.brightness = val;
			}
		}
		break;
	}
}

static void rb_led_dummy_set(struct led_classdev *led_cdev,
		       enum led_brightness brightness) {
}

static void rb_all_leds_set(struct led_classdev *led_cdev,
		       enum led_brightness brightness) {
	int i;
	led_trigger_event(&dark_mode_trigger, brightness);
	dark_mode_status = !brightness;

	for (i = 0; i < sizeof(rb_leds) / sizeof(rb_leds[0]); ++i) {
		struct rb_led_data *rbled = &rb_leds[i];

		if (!rbled->cdev.brightness_set)
			continue;

		if (rbled->map & PLED_CFG_DARK) {
			void (*br_fn)(struct led_classdev*, enum led_brightness) =
				(brightness == LED_OFF) ? rb_led_dummy_set : rb_led_set;
			enum led_brightness val;

			if (PLED_GET_TYPE(rbled->map) == PLED_TYPE_RGB7) {
				int j;
				for (j = 0; j < sizeof(rb_rgb7_led_colours) /
						sizeof(struct rb_rgb_led_colours) - 1; j++) {
					val = (brightness == LED_OFF) ? brightness :
						rbled->colours[j]->cdev.brightness;
					rbled->colours[j]->cdev.brightness_set = br_fn;
					rb_led_set(&rbled->colours[j]->cdev, val);
				}
				rbled->cdev.brightness_set = br_fn;
			} else {
				val = (brightness == LED_OFF) ?
					brightness : rbled->cdev.brightness;
				rbled->cdev.brightness_set = br_fn;
				rb_led_set(&rbled->cdev, val);
			}
		}
	}
	rb_led_set(led_cdev, brightness);
}

static enum led_brightness rb_led_get(struct led_classdev *led_cdev)
{
	struct rb_led_data *rbld = container_of(led_cdev, struct rb_led_data, cdev);
	unsigned map = rbld->map;
	unsigned bit = PLED_GET_BIT(map);
	unsigned val = 0;
	int i;

#ifdef CONFIG_MIPS_MIKROTIK
	if (!gpio_base) {
		return led_cdev->brightness;
	}
#endif
	switch(PLED_GET_TYPE(map)) {
#ifdef CONFIG_GPIOLIB
	case PLED_TYPE_GPIOLIB:
		val = gpio_get_value_cansleep(PLED_GET_BIT_NUM(map)) ? bit : 0;
		break;
#endif
#ifdef CONFIG_MIPS_MIKROTIK
	case PLED_TYPE_GPIO:
		val = gpio_readl(GPIO_IN_REG);
		break;
	case PLED_TYPE_GPIO_OE:
		val = gpio_readl(GPIO_OE_REG);
		break;
	case PLED_TYPE_SHARED_GPIO:
		// on AR934x OE is actually input-enable
		if (gpio_readl(GPIO_OE_REG) & bit) {
			printk("rb_led_get ERROR: "
			       "PLED_TYPE_SHARED_GPIO on input, bitmask %x\n",
				bit);
			return 1;
		}
		// wait for gpio no to be used before make it as input
		while ((gpio_readl(GPIO_O1_REG) & bit) == 0) {
			// wait for NAND_CE# to be released
			schedule();
		}
		gpio_rmw(GPIO_OE_REG, 0, bit);
		udelay(200);
		val = gpio_readl(GPIO_IN_REG);
		gpio_rmw(GPIO_OE_REG, bit, 0);
		break;
#endif
#ifdef CONFIG_MTD_NAND_RB400
	case PLED_TYPE_SHARED_RB900:
		do {
			val = ar9342_change_gpo(0, 0);
			if (val == 0) schedule();
		} while (val == 0);
		break;
#endif
	case PLED_TYPE_RGB7:
		for (i = 0; i < sizeof(rb_rgb7_led_colours) /
				sizeof(struct rb_rgb_led_colours); i++) {
			if (rbld->colours[i] && rbld->colours[i]->cdev.brightness_get) {
				struct rb_rgb_led_colours *col = &rb_rgb7_led_colours[i + 1];
				unsigned col_val = rb_led_get(&(rbld->colours[i]->cdev));
				val |= col_val << col->shift;
			}
		}
		return val;
	default:
		return led_cdev->brightness;
	}
	return !(val & bit) ^ !(map & PLED_CFG_INV);
}

void rb_beepled(int on) {
	int i;

	for (i = 0; i < sizeof(rb_leds) / sizeof(rb_leds[0]); ++i) {
		struct rb_led_data *rbled = &rb_leds[i];
		unsigned val = (dark_mode_status) ? !LED_OFF 
				: (!rbled->cdev.brightness);
		val ^= !on;
		if (rbled->cdev.brightness_set == NULL) continue;
		if (strstr(rbled->cdev.name, "led") == NULL) continue;
		if (strcmp(rbled->cdev.name, "power-led") == 0) continue;
		if (PLED_GET_TYPE(rbled->map) == PLED_TYPE_RGB7) {
			if (dark_mode_status) continue;
			val = !val << RGB7_blue;
		}
		rb_led_set(&rbled->cdev, val);
	}
}
EXPORT_SYMBOL(rb_beepled);

static int rb_init_led(struct platform_device *pdev, unsigned map) {
	struct rb_led_data *data;
	int colour = PLED_GET_COLOUR(map);
	int name_idx = PLED_GET_NAME_IDX(map);
	if (name_idx >= sizeof(rb_leds) / sizeof(rb_leds[0])) {
		printk("leds-rb ERROR: idx %d too big!\n", name_idx);
		return -EINVAL;
	}
	data = &(rb_leds[name_idx]);
	if ((colour || PLED_GET_TYPE(map) == PLED_TYPE_RGB7) && !data->colours) {
		data->colours = kzalloc((sizeof(rb_rgb7_led_colours) /
			sizeof(struct rb_rgb_led_colours)) - 1, GFP_KERNEL);
	}
	if (colour) {
		struct rb_led_data *cl = kmalloc(sizeof(struct rb_led_data), GFP_KERNEL);
		memcpy(cl, data, sizeof(struct rb_led_data));
		data->colours[colour - 1] = cl;
		data = cl;
		data->map &= ~0x3f;
	}
	data->map |= map;

	if (colour || PLED_GET_TYPE(data->map) == PLED_TYPE_RGB7) {
		char tmp[32];
		char *name = kzalloc(128, GFP_KERNEL);
		char* ch_ptr = strchr(data->cdev.name,':');
		const char* name_str;
		int len;

		/* change name - add colour */
		if (!ch_ptr) {
			name_str = data->cdev.name;
		} else {
			len = ch_ptr - data->cdev.name;
			strncpy(tmp, data->cdev.name, len);
			tmp[len] = '\0';
			name_str = tmp;
		}

		snprintf(name, 127, "%s:%s",
			 name_str, rb_rgb7_led_colours[colour].name);
		data->cdev.name = name;
	}

#ifdef CONFIG_GPIOLIB
	if (PLED_GET_TYPE(data->map) == PLED_TYPE_GPIOLIB) {
		int ret;
		int gpio_num = PLED_GET_BIT_NUM(data->map);

		struct gpio_desc *gpio_desc = gpio_to_desc(gpio_num);
		struct gpio_chip *chip = gpiod_to_chip(gpio_desc);
		if (chip == NULL) {
			printk("leds-rb ERROR: "
			       "gpio_to_chip(%d) failed\n", gpio_num);
			return -ENOENT;
		}

		ret = gpio_request(gpio_num, data->cdev.name);
		if (ret < 0) {
			printk("leds-rb ERROR: "
			       "gpio_request(%d) failed\n", gpio_num);
			return ret;
		}
		if (data->map & PLED_CFG_INV) {
			gpiod_toggle_active_low(gpio_desc);
		}

		if (chip->names == NULL) {
			chip->names = vzalloc(chip->ngpio * sizeof(void*));
			if (chip->names == NULL) {
				printk("leds-rb ERROR: vzalloc failed\n");
				return -ENOMEM;
			}
		}
		((const char**)chip->names)[gpio_num - chip->base] =
			data->cdev.name;

		if (data->map & PLED_CFG_INPUT) {
			gpiod_direction_input(gpio_desc);
		}
		else {
			unsigned val = data->map & PLED_CFG_ON;
			if (data->map & PLED_CFG_KEEP) {
				val = gpiod_get_value_cansleep(gpio_desc);
			}
			gpiod_direction_output(gpio_desc, val);
		}
		gpiod_export(gpio_desc, data->map & PLED_CFG_IO);
		printk("Registered gpio %d as %s\n", gpio_num, data->cdev.name);
		if (data->map & (PLED_CFG_INPUT | PLED_CFG_IO)) {
			return 0;
		}
	}
#endif
	if (name_idx == PLED_NAME_all_leds)
		data->cdev.brightness_set = rb_all_leds_set;
	else
		data->cdev.brightness_set = rb_led_set;

	data->cdev.brightness_get = rb_led_get;
	led_classdev_register(&pdev->dev, &data->cdev);
	if (!(data->map & (PLED_CFG_KEEP | PLED_CFG_INPUT))) {
		data->cdev.brightness = !!(data->map & PLED_CFG_ON);
		rb_led_set(&data->cdev, data->cdev.brightness);
	}
	return 0;
}

#ifdef CONFIG_OF_GPIO
static int rb_led_init_of(struct platform_device *pdev, struct device_node *np,
			const char* name, int name_idx, unsigned colour) {
		enum of_gpio_flags flags;
		const char *state;
		int gpio = -1;
		unsigned cfg = 0;
		const char *type;
		unsigned map;

		gpio = of_get_gpio_flags(np, 0, &flags);
		if (gpio < 0) {
			printk("leds-rb: unknown gpio for '%s' from tree\n",
			       name);
			return -ENODEV;
		}
		if (flags & OF_GPIO_ACTIVE_LOW) cfg |= PLED_CFG_INV;

		state = of_get_property(np, "default-state", NULL);
		if (state) {
			if (!strcmp(state, "keep"))
				cfg |= PLED_CFG_KEEP;
			else if (!strcmp(state, "on"))
				cfg |= PLED_CFG_ON;
			else if (!strcmp(state, "input"))
				cfg |= PLED_CFG_INPUT;
		}
		map = PLEDN(name_idx, gpio, GPIOLIB, cfg, colour);

		type = of_get_property(np, "type", NULL);
		if (type && (strcmp(type, "wifi") == 0)) {
			map = (map & ~PLED_TYPE_MASK) | PLED_TYPE_WIFI;
		}
		rb_init_led(pdev, map);
		return 0;
}

static int rb_leds_probe_of(struct platform_device *pdev) {
	struct device_node *np = pdev->dev.of_node, *child;
	struct device_node *gchild = NULL;

	for_each_child_of_node(np, child) {
		const char *name;
		int name_idx = -1;
		unsigned colour = 0;
		int i;

		name = of_get_property(child, "label", NULL) ? : child->name;
		if (!strcmp(name, "leds-button"))
			name = "wps-button";
		for (i = 0; i < sizeof(rb_leds) / sizeof(rb_leds[0]); ++i) {
			struct rb_led_data *rbled = &rb_leds[i];
			if (strcmp(rbled->cdev.name, name) == 0) {
				name_idx = i;
				break;
			}
		}
		if (name_idx < 0) {
			printk("leds-rb: unknown name '%s' from tree\n", name);
			continue;
		}
		gchild = of_get_next_child(child, gchild);
		if (!gchild) {
			rb_led_init_of(pdev, child, name, name_idx, 0);
			continue;
		} else {
			rb_init_led(pdev, PLEDN(name_idx, 0, RGB7, 0, 0));
		}

		do {
			for (i = 0; i < sizeof(rb_rgb7_led_colours) /
					sizeof(struct rb_rgb_led_colours); i++) {
				if (!strcmp(rb_rgb7_led_colours[i].name, gchild->name)) {
					colour = i;
					break;
				}
			}

			rb_led_init_of(pdev, gchild, name, name_idx, colour);

			gchild = of_get_next_child(child, gchild);
		} while (gchild);
	}

	return 0;
}
static const struct of_device_id rb_of_match[] = {
	{ .compatible = "leds-rb", },
#ifndef CONFIG_LEDS_GPIO
	{ .compatible = "gpio-leds", },
#endif
	{},
};
#endif /* CONFIG_OF_GPIO */

static int rb_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	unsigned *map = (unsigned *)pdev->dev.platform_data;

#ifdef CONFIG_MIPS_MIKROTIK
	unsigned base = GPIO_BASE;

	if (mips_machgroup == MACH_GROUP_MT_MUSIC) {
		base = GPIO_BASE_MUSIC;
	}
	gpio_base = ioremap(base, PAGE_SIZE);
#endif

	led_trigger_register(&dark_mode_trigger);

	if (map == NULL) {
#ifdef CONFIG_OF_GPIO
		ret = rb_leds_probe_of(pdev);
#else
		return -ENODEV;
#endif
	} else {
		for ( ; *map != 0; ++map)
			rb_init_led(pdev, *map);
	}

        if (!rb_leds[PLED_NAME_all_leds].cdev.brightness_set)
		rb_init_led(pdev, PLEDN(PLED_NAME_all_leds, 0, VIRTUAL, 0, 0));

	return ret;
}

static struct platform_driver rb_led_driver = {
	.probe	= rb_led_probe,
	.driver	= {
		.name = "leds-rb",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF_GPIO
		.of_match_table = rb_of_match,
#endif
	},
};

struct led_classdev *leds_rb_request_gpio(char *name) {
	int i;
	for (i = 0; i < sizeof(rb_leds) / sizeof(rb_leds[0]); i++) {
		if (!strcmp(rb_leds[i].cdev.name, name))
			return &rb_leds[i].cdev;
	}
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL(leds_rb_request_gpio);

void leds_rb_set_gpio(struct led_classdev *dev, unsigned val) {
	rb_led_set(dev, (enum led_brightness)val);
}
EXPORT_SYMBOL(leds_rb_set_gpio);

unsigned leds_rb_get_gpio(struct led_classdev *dev) {
	return (unsigned)rb_led_get(dev);
}
EXPORT_SYMBOL(leds_rb_get_gpio);

module_platform_driver(rb_led_driver);
