# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_SMP)	+= platsmp.o machine.o

dtb-y			+= rb3011.dtb
dtb-y			+= lhg60.dtb
dtb-y			+= 450g.dtb
dtb-y			+= d52g.dtb
dtb-y			+= lhgg60.dtb
dtb-y			+= lhg5ac.dtb
dtb-y			+= wap2gr.dtb
dtb-y			+= wapgr.dtb
dtb-y			+= capg5ac.dtb
dtb-y			+= wap_pcie.dtb
dtb-y			+= plc5ac.dtb
dtb-y			+= lhgb5hpa.dtb
dtb-y			+= d23ugs.dtb
dtb-y			+= d53g.dtb
dtb-y			+= d53pk.dtb
dtb-y			+= d53igr.dtb
dtb-y			+= d53ig.dtb
dtb-y			+= d24g.dtb
dtb-y			+= d24gi.dtb
dtb-y			+= d24gir.dtb
dtb-y			+= d24gir60.dtb
obj-y			+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))

always				:= $(dtb-y)
clean-files			:= *.dtb *.dtb.S
