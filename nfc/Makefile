#
# Makefile for nfc devices
#
obj-$(CONFIG_NXP_NFC_I2C)		+= pn553_i2c.o
pn553_i2c-objs				:= common.o common_ese.o i2c_drv.o
pn553_i2c-$(CONFIG_NXP_NFC_RECOVERY)	+= recovery_seq.o recovery_fw.o
#ccflags-y                      += -DDEBUG

