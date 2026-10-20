# SPDX-License-Identifier: ISC

include $(src)/platform-config.mak

ccflags-y += \
	-I $(src) \
        -I $(src)/include

# Hold the SDIO bus at a stable 25 MHz. The SV6051/SV6051P chip is sensitive
# to the SDIO clock speed and default 37.5 MHz causes CRC / EILSEQ (-84)
# timing errors on the Amlogic (Meson) MMC controller.
ccflags-y += -DSDIO_USE_SLOW_CLOCK

obj-$(CONFIG_SSV6051) += ssv6051.o
ssv6051-objs += \
	ssv6051-generic-wlan.o \
	ssvdevice/ssvdevice.o \
	ssvdevice/ssv_cmd.o \
	hci/ssv_hci.o \
	smac/init.o \
	smac/dev.o \
	smac/ssv_rc.o \
	smac/ssv_ht_rc.o \
	smac/ap.o \
	smac/ampdu.o \
	smac/efuse.o \
	smac/ssv_pm.o \
	smac/sar.o \
	smac/ssv_cfgvendor.o \
	smac/lib.o \
	hwif/sdio/sdio.o

