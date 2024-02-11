# Makefile for the Himax touchscreen drivers.

ccflags-$(TOUCHSCREEN_HIMAX_CHIPSET) += -I$(srctree)/drivers/input/touchscreen/hxchipset/
ccflags-$(CONFIG_FB) += -DHX_CONFIG_FB
ccflags-$(CONFIG_DRM) += -DHX_CONFIG_DRM
ccflags-$(CONFIG_DEBUG_DRIVER) += -DDEBUG

#NOTE: As module not tested - Please check 
ifneq ($(filter m, $(CONFIG_TOUCHSCREEN_HIMAX_COMMON)),)
ccflags-$(CONFIG_KALLSYMS_ALL) += -D__KERNEL_KALLSYMS_ALL_ENABLED__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_COMMON) += -D__HIMAX_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX852xG) += -D__HIMAX_HX852xG_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX852xH) += -D__HIMAX_HX852xH_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83102) += -D__HIMAX_HX83102_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83103) += -D__HIMAX_HX83103_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83106) += -D__HIMAX_HX83106_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83111) += -D__HIMAX_HX83111_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83112) += -D__HIMAX_HX83112_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83113) += -D__HIMAX_HX83113_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83192) += -D__HIMAX_HX83192_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83191) += -D__HIMAX_HX83191_MOD__
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83191) += -D__HIMAX_HX83191_MOD__
endif

obj-$(CONFIG_TOUCHSCREEN_HIMAX_COMMON)	+= himax_common.o himax_platform.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_INCELL) += himax_ic_incell_core.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_ONCELL) += himax_ic_oncell_core.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX852xG) += himax_ic_HX852xG.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX852xH) += himax_ic_HX852xH.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83102) += himax_ic_HX83102.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83103)	+= himax_ic_HX83103.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83106)	+= himax_ic_HX83106.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83111)	+= himax_ic_HX83111.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83112)	+= himax_ic_HX83112.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83113)	+= himax_ic_HX83113.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83192)	+= himax_ic_HX83192.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83191)	+= himax_ic_HX83191.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_IC_HX83191)	+= himax_ic_HX83191.o

ifneq ($(filter m, $(CONFIG_TOUCHSCREEN_HIMAX_COMMON)),)
obj-$(CONFIG_TOUCHSCREEN_HIMAX_EMBEDDED_FIRMWARE)	+= himax_mmi.o
else
obj-$(CONFIG_TOUCHSCREEN_HIMAX_EMBEDDED_FIRMWARE)	+= Himax_firmware.o
ccflags-$(CONFIG_TOUCHSCREEN_HIMAX_EMBEDDED_FIRMWARE) += -D__EMBEDDED_FW__
endif

obj-$(CONFIG_TOUCHSCREEN_HIMAX_INSPECT)	+= himax_inspection.o
obj-$(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)	+= himax_debug.o

#ifneq ($(filter y, $(CONFIG_TOUCHSCREEN_HIMAX_EMBEDDED_FIRMWARE)),)
#ccflags-y += -D__EMBEDDED_FW__
#ld_array_start_str = _binary_$(srctree)/$(src)_Himax_firmware_bin_start
#ld_array_start_sym = $(subst -,_,$(subst .,_,$(subst /,_,$(ld_array_start_str))))
#obj_array_start_sym = _binary___Himax_firmware_bin_start
#ld_array_size_str = _binary_$(srctree)/$(src)_Himax_firmware_bin_size
#ld_array_size_sym = $(subst -,_,$(subst .,_,$(subst /,_,$(ld_array_size_str))))
#obj_array_size_sym = _binary___Himax_firmware_bin_size
#ld_array_end_str = _binary_$(srctree)/$(src)_Himax_firmware_bin_end
#ld_array_end_sym = $(subst -,_,$(subst .,_,$(subst /,_,$(ld_array_end_str))))
#obj_array_end_sym = _binary___Himax_firmware_bin_end
#$(src)/Himax_firmware.o: $(src)/Himax_firmware.bin FORCE
#$(LD) $(LDFLAGS) -r -b binary $(srctree)/$(src)/Himax_firmware.bin -o $(objtree)/$(obj)/Himax_firmware.o
#$(OBJCOPY) --redefine-sym $(ld_array_start_sym)=$(obj_array_start_sym) $(objtree)/$(obj)/Himax_firmware.o
#$(OBJCOPY) --redefine-sym $(ld_array_size_sym)=$(obj_array_size_sym) $(objtree)/$(obj)/Himax_firmware.o
#$(OBJCOPY) --redefine-sym $(ld_array_end_sym)=$(obj_array_end_sym) $(objtree)/$(obj)/Himax_firmware.o
#endif
