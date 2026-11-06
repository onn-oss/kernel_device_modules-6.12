MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/pmic/include
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/usb20
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply
subdir-ccflags-y += -Werror -I$(srctree)/$(DEVICE_MODULES_REL_DIR)/oem/devinfo
subdir-ccflags-y += -Werror -I$(srctree)/$(DEVICE_MODULES_REL_DIR)/oem/tinno_charger
subdir-ccflags-y += -Werror -I$(srctree)/$(DEVICE_MODULES_REL_DIR)/drivers/misc/mediatek/typec/tcpc
subdir-ccflags-y += -Werror -I$(srctree)/$(DEVICE_MODULES_REL_DIR)/drivers/power/supply
subdir-ccflags-y += -Werror -I$(srctree)/$(DEVICE_MODULES_REL_DIR)/drivers/misc/mediatek/usb/usb20

# Built via kleaf/DDK (BUILD.bazel); traditional make hooks kept for reference:
#obj-$(CONFIG_OEM_TINNO_CHARGER) += oem_tinno_charger.o
#oem_tinno_charger-objs := tinno_charger.o
#obj-$(CONFIG_OEM_SWITCH_CHARGER) += switch_charger/
#obj-$(CONFIG_OEM_TCPC) += typec/
