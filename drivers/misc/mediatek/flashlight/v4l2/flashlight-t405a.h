/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _FLASHLIGHT_T405A_H
#define _FLASHLIGHT_T405A_H

#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include "flashlight-core.h"

#define FLASHLIGHT_LEVEL_NUM 8
#define FLASHLIGHT_LEVEL_TORCH 0

enum flashlight_led_id {
	FLASHLIGHT_LED0 = 0,
	FLASHLIGHT_LED1,
	FLASHLIGHT_LED_MAX
};

/**
 * struct flashlight_flash
 *
 * @dev: pointer to &struct device
 * @pdata: platform data
 * @regmap: reg. map for i2c
 * @lock: muxtex for serial access.
 * @led_mode: V4L2 LED mode
 * @ctrls_led: V4L2 controls
 * @subdev_led: V4L2 subdev
 */
struct flashlight_flash {
	struct device *dev;
	struct flashlight_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;

	enum v4l2_flash_led_mode led_mode;
	struct v4l2_ctrl_handler ctrls_led[FLASHLIGHT_LED_MAX];
	struct v4l2_subdev subdev_led[FLASHLIGHT_LED_MAX];
	struct device_node *dnode[FLASHLIGHT_LED_MAX];
	struct pinctrl *flashlight_hwen_pinctrl;
	struct pinctrl_state *flashlight_hwen_high;
	struct pinctrl_state *flashlight_hwen_low;
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
	struct flashlight_device_id flash_dev_id[FLASHLIGHT_LED_MAX];
#endif
	int need_cooler;
	unsigned long target_current[FLASHLIGHT_LED_MAX];
	unsigned long ori_current[FLASHLIGHT_LED_MAX];
	unsigned int torch_power;
	unsigned int camera_power;
};

#endif /* _FLASHLIGHT_T405A_H */
