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

#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/irqflags.h>
#include <linux/printk.h>
#ifdef CONFIG_KVM_GUEST
#include <linux/virtio_console.h>
#include <linux/kvm_para.h>
#include <asm/kvm_virtio.h>
#endif
#include <asm/setup.h>
#include <hv/hypervisor.h>

static void early_hv_write(struct console *con, const char *s, unsigned n)
{
#ifdef CONFIG_KVM_GUEST
	char buf[512];

	if (n > sizeof(buf) - 1)
		n = sizeof(buf) - 1;
	memcpy(buf, s, n);
	buf[n] = '\0';

	hcall_virtio(KVM_VIRTIO_NOTIFY, __pa(buf));
#else
	const char *end = strchr(s, '\n');
	if (end) {
		++end;
		hv_console_write((HV_VirtAddr) s, end - s);
		hv_console_write((HV_VirtAddr) "\r", 1);
		n -= end - s;
		if (n) early_hv_write(con, end, n);
	} else {
		hv_console_write((HV_VirtAddr) s, n);
	}
#endif
}

static struct console early_hv_console = {
	.name =		"earlyhv",
	.write =	early_hv_write,
	.flags =	CON_PRINTBUFFER | CON_BOOT,
	.index =	-1,
};

/* Direct interface for emergencies */

void early_panic(const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	arch_local_irq_disable_all();

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	early_printk("Kernel panic - not syncing: %pV", &vaf);

	va_end(args);

	dump_stack();
	hv_halt();
}

static int __init setup_early_printk(char *str)
{
	if (early_console)
		return 1;

	early_console = &early_hv_console;
	register_console(early_console);

	return 0;
}

early_param("earlyprintk", setup_early_printk);
