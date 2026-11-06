IMGSENSOR_SUBDRV_OBJ := \
	$(subdrv-rpath)/sc202cs_sj_t405a_mipi_raw_Sensor.o

ifeq ($(CONFIG_MTK_IMGSENSOR_ISP4_C), m)
$(eval $(MTK_PLATFORM_KO)-objs += $(IMGSENSOR_SUBDRV_OBJ))
else ifeq ($(CONFIG_MTK_IMGSENSOR_ISP4_C), y)
obj-$(CONFIG_MTK_IMGSENSOR_ISP4_C) += $(IMGSENSOR_SUBDRV_OBJ)
endif

