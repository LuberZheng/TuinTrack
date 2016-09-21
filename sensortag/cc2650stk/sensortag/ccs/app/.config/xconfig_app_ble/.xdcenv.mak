#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = D:/TI/tirtos_cc13xx_cc26xx_2_18_00_03/packages;D:/TI/tirtos_cc13xx_cc26xx_2_18_00_03/products/tidrivers_cc13xx_cc26xx_2_16_01_13/packages;D:/TI/tirtos_cc13xx_cc26xx_2_18_00_03/products/bios_6_45_02_31/packages;D:/TI/tirtos_cc13xx_cc26xx_2_18_00_03/products/uia_2_00_05_50/packages;D:/TI/ccsv6/ccs_base;D:/TI/simplelink/ble_sdk_2_02_00_31/examples/cc2650stk/sensortag/ccs/app/.config
override XDCROOT = D:/TI/xdctools_3_32_00_06_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = D:/TI/tirtos_cc13xx_cc26xx_2_18_00_03/packages;D:/TI/tirtos_cc13xx_cc26xx_2_18_00_03/products/tidrivers_cc13xx_cc26xx_2_16_01_13/packages;D:/TI/tirtos_cc13xx_cc26xx_2_18_00_03/products/bios_6_45_02_31/packages;D:/TI/tirtos_cc13xx_cc26xx_2_18_00_03/products/uia_2_00_05_50/packages;D:/TI/ccsv6/ccs_base;D:/TI/simplelink/ble_sdk_2_02_00_31/examples/cc2650stk/sensortag/ccs/app/.config;D:/TI/xdctools_3_32_00_06_core/packages;..
HOSTOS = Windows
endif
