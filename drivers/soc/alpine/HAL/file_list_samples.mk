HAL_SAMPLE_SOURCES = \
	$(HAL_TOP)/samples/dummy_plat_services.c \
	$(HAL_TOP)/samples/crypto.c \
	$(HAL_TOP)/samples/udma_fast.c \
	$(HAL_TOP)/samples/eth.c \
	$(HAL_TOP)/samples/serdes_avg_init.c \
	$(HAL_TOP)/samples/serdes_snps_init.c \
	$(HAL_TOP)/samples/thermal_sensor.c \
	$(HAL_TOP)/samples/pmdt.c \

HAL_SAMPLE_TESTS += \
	$(HAL_TOP)/samples/dummy_plat.c \
	$(HAL_TOP)/samples/pbs.c \
	$(HAL_TOP)/samples/unit_adapter.c \
	$(HAL_TOP)/samples/vt_sensor.c \
	$(HAL_TOP)/samples/fast_crypto.c \
	$(HAL_TOP)/samples/cell.c \
	$(HAL_TOP)/samples/cell_table.c \

ifeq ($(AL_DEV_ID),AL_DEV_ID_ALPINE_V3)
ifeq ($(AL_DEV_REV_ID),1)
HAL_SAMPLE_TESTS += \
	$(HAL_TOP)/samples/pmdt_service.c \
	$(HAL_TOP)/samples/eth_v4_lm.c \

endif
endif

