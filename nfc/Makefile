#
# Makefile for nfc devices
#
ifdef CONFIG_NXP_NFC_I2C
obj-y                     += common.o
else ifdef CONFIG_NXP_NFC_I3C
obj-y                     += common.o
endif
obj-$(CONFIG_NXP_NFC_I2C) += i2c_drv.o
obj-$(CONFIG_NXP_NFC_I3C) += i3c_drv.o
obj-$(CONFIG_NXP_NFC_SN110) += sn110.o
