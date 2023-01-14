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

#ifndef _ASM_TILE_IRQ_H
#define _ASM_TILE_IRQ_H

#include <linux/hardirq.h>
#include <linux/types.h>

/* The hypervisor interface provides 32 IPI events per core. */
#define NR_IPI_EVENTS 32

#if CHIP_HAS_IPI()
/* 
 * The number of supported IRQs. This can be set to a larger value
 * (at most NR_CPUS*NR_IPI_EVENTS) that may be needed with more
 * peripheral devices.
 */
#define NR_IRQS			256
#else
#define NR_IRQS			32
#endif

/* The IRQ number used for linux IPI reschedule. */
extern int irq_reschedule;
/* The IPI event used for irq_reschedule. */
extern int irq_reschedule_event;

/* Interrupts for dynamic allocation start at 1. Let the core allocate irq0 */
#define NR_IRQS_LEGACY	1

#define irq_canonicalize(irq)   (irq)

/* An IRQ number is bound to a core and an IPI event number. */
struct irq_map {
	int cpu;
	int event;
};

void ack_bad_irq(unsigned int irq);

/*
 * Different ways of handling interrupts.  Tile interrupts are always
 * per-cpu; there is no global interrupt controller to implement
 * enable/disable.  Most onboard devices can send their interrupts to
 * many tiles at the same time, and Tile-specific drivers know how to
 * deal with this.
 *
 * However, generic devices (usually PCIE based, sometimes GPIO)
 * expect that interrupts will fire on a single core at a time and
 * that the irq can be enabled or disabled from any core at any time.
 * We implement this by directing such interrupts to a single core.
 *
 * One added wrinkle is that PCI interrupts can be either
 * hardware-cleared (legacy interrupts) or software cleared (MSI).
 * Other generic device systems (GPIO) are always software-cleared.
 *
 * The enums below are used by drivers for onboard devices, including
 * the internals of PCI root complex and GPIO.  They allow the driver
 * to tell the generic irq code what kind of interrupt is mapped to a
 * particular IRQ number.
 */
enum {
	/* per-cpu interrupt; use enable/disable_percpu_irq() to mask */
	TILE_IRQ_PERCPU,
	/* global interrupt, hardware responsible for clearing. */
	TILE_IRQ_HW_CLEAR,
	/* global interrupt, software responsible for clearing. */
	TILE_IRQ_SW_CLEAR,
};


/*
 * Paravirtualized drivers should call this when they dynamically
 * allocate a new IRQ or discover an IRQ that was pre-allocated by the
 * hypervisor for use with their particular device.  This gives the
 * IRQ subsystem an opportunity to do interrupt-type-specific
 * initialization.
 *
 * ISSUE: We should modify this API so that registering anything
 * except percpu interrupts also requires providing callback methods
 * for enabling and disabling the interrupt.  This would allow the
 * generic IRQ code to proxy enable/disable_irq() calls back into the
 * PCI subsystem, which in turn could enable or disable the interrupt
 * at the PCI shim.
 */
void tile_irq_activate(unsigned int irq, int tile_irq_type);

/* Map an IRQ number to (cpu, event). For globally-valid IRQs, cpu=-1. */
int tile_irq_get_cpu(int irq);
int tile_irq_get_event(int irq);

void setup_irq_regs(void);

#ifdef __tilegx__
void arch_trigger_cpumask_backtrace(const struct cpumask *mask,
				    bool exclude_self);
#define arch_trigger_cpumask_backtrace arch_trigger_cpumask_backtrace
#endif

#endif /* _ASM_TILE_IRQ_H */
