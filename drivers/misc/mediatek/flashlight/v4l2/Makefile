#obj-$(CONFIG_MTK_VIDEO_LM3643)     += lm3643.o
#obj-$(CONFIG_MTK_VIDEO_LM3644)     += lm3644.o
obj-$(CONFIG_MTK_FLASHLIGHT_OCP81378)    += flashlights-ocp81378.o
MTK_CUSTOM_PATH := $(DEVICE_MODULES_PATH)/drivers/misc/mediatek
subdir-ccflags-y += \
	-I$(MTK_CUSTOM_PATH)/flashlight

ifeq ($(CONFIG_FLASHLIGHT_DEBUG),y)
ccflags-y += -DCONFIG_FLASHLIGHT_DEBUG
endif
