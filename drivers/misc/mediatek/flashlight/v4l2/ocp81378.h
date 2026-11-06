/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _OCP81378_H
#define _OCP81378_H

#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

enum ocp81378_led_id {
	OCP81378_LED0 = 0,
	OCP81378_LED1,
	OCP81378_LED_MAX
};

/**
 * struct ocp81378_flash
 *
 * @dev: pointer to &struct device
 * @pdata: platform data
 * @regmap: reg. map for i2c
 * @lock: muxtex for serial access.
 * @led_mode: V4L2 LED mode
 * @ctrls_led: V4L2 controls
 * @subdev_led: V4L2 subdev
 */
struct ocp81378_flash {
	struct device *dev;
	struct ocp81378_platform_data *pdata;
	struct regmap *regmap;
	struct mutex lock;

	enum v4l2_flash_led_mode led_mode;
	struct v4l2_ctrl_handler ctrls_led[OCP81378_LED_MAX];
	struct v4l2_subdev subdev_led[OCP81378_LED_MAX];
	struct device_node *dnode[OCP81378_LED_MAX];
	struct pinctrl *ocp81378_hwen_pinctrl;
	struct pinctrl_state *ocp81378_hwen_high;
	struct pinctrl_state *ocp81378_hwen_low;
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
	struct flashlight_device_id flash_dev_id[OCP81378_LED_MAX];
#endif
	int need_cooler;
	unsigned long target_current[OCP81378_LED_MAX];
	unsigned long ori_current[OCP81378_LED_MAX];
	unsigned int torch_power;
	unsigned int camera_power;
};

#endif /* _OCP81378_H */
