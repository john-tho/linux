# SPDX-License-Identifier: GPL-2.0-only
obj-y				+= alpine_machine.o
obj-$(CONFIG_SMP)		+= platsmp.o alpine_cpu_pm.o
obj-$(CONFIG_PCI)		+= pcie.o

dtb-y				+= rb1100ahx4.dtb
dtb-y				+= rb4011.dtb
obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))

always				:= $(dtb-y)
clean-files			:= *.dtb *.dtb.S
