#
# Makefile for nfc devices
#
obj-$(CONFIG_NXP_NFC_I2C)	+= pn553_i2c.o
pn553_i2c-objs			:= common.o common_ese.o i2c_drv.o recovery_seq.o recovery_fw.o
ccflags-y 			+= -DRECOVERY_ENABLE -UDEBUG
