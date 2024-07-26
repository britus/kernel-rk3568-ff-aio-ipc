# Makefile for the Himax touchscreen drivers.
ccflags-$(TOUCHSCREEN_HIMAX_CHIPSET) += -I$(srctree)/drivers/input/touchscreen/hxchipset/
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_DEBUG) += -DDEBUG

# common platform driver
obj-$(CONFIG_TOUCHSCREEN_HIMAX_COMMON) += himax_common.o himax_platform.o

# for incell chips
obj-$(CONFIG_TOUCHSCREEN_HIMAX_INCELL) += himax_ic_incell_core.o

# static hx83102 chip type
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83102) += himax_ic_HX83102.o 

