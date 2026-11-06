// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/videodev2.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/thermal.h>

#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT)
#include "flashlight-core.h"

#include <linux/power_supply.h>
#endif

#include "ocp81378.h"

#define OCP81378_NAME	"ocp81378"
#define OCP81378_I2C_ADDR	(0x63)

#define FLASHLIGHT_SCENARIO_PRE_FLASH	1
#define FLASHLIGHT_SCENARIO_MAIN_FLASH	2

/* registers definitions */
#define REG_ENABLE		0x01
#define REG_LED0_FLASH_BR	0x03
#define REG_LED1_FLASH_BR	0x04
#define REG_LED0_TORCH_BR	0x05
#define REG_LED1_TORCH_BR	0x06
#define REG_FLASH_TOUT		0x08
#define REG_FLAG1		0x0A
#define REG_FLAG2		0x0B
#define REG_CHIP_ID		0x0C

/* fault mask */
#define FAULT_TIMEOUT	(1<<0)
#define FAULT_THERMAL_SHUTDOWN	(1<<2)
#define FAULT_LED0_SHORT_CIRCUIT	(1<<5)
#define FAULT_LED1_SHORT_CIRCUIT	(1<<4)

/*  FLASH TIMEOUT DURATION
 *	min 40ms, step 40ms, max 1600ms
 */
#define OCP81378_FLASH_TOUT_MIN 40
#define OCP81378_FLASH_TOUT_STEP 40
#define OCP81378_FLASH_TOUT_MAX 1600

/*  FLASH Brightness for OCP81378
 *	min 15000uA, step 15650uA, max 2000000uA
 */
#define OCP81378_FLASH_BRT_MIN 15000
#define OCP81378_FLASH_BRT_STEP 15650
#define OCP81378_FLASH_BRT_MAX 2000000
#define OCP81378_FLASH_BRT_uA_TO_REG(a)	\
	((a) < OCP81378_FLASH_BRT_MIN ? 0 :	\
	 (((a) - OCP81378_FLASH_BRT_MIN) / OCP81378_FLASH_BRT_STEP))
#define OCP81378_FLASH_BRT_REG_TO_uA(a)		\
	((a) * OCP81378_FLASH_BRT_STEP + OCP81378_FLASH_BRT_MIN)

/*  TORCH BRT for OCP81378
 *	min 2900uA, step 3840uA, max 490000uA
 */
#define OCP81378_TORCH_BRT_MIN 2900
#define OCP81378_TORCH_BRT_STEP 3840
#define OCP81378_TORCH_BRT_MAX 490000
#define OCP81378_TORCH_BRT_uA_TO_REG(a)	\
	((a) < OCP81378_TORCH_BRT_MIN ? 0 :	\
	 (((a) - OCP81378_TORCH_BRT_MIN) / OCP81378_TORCH_BRT_STEP))
#define OCP81378_TORCH_BRT_REG_TO_uA(a)		\
	((a) * OCP81378_TORCH_BRT_STEP + OCP81378_TORCH_BRT_MIN)

#if !IS_ENABLED(CONFIG_MTK_FLASHLIGHT_THERMAL)
#define FLASHLIGHT_COOLER_MAX_STATE 4
#endif

#define OCP81378_LEVEL_NUM 27
#define OCP81378_LEVEL_TORCH 7
/* define channel, level */
#define OCP81378_CHANNEL_NUM 2
#define OCP81378_CHANNEL_CH1 0
#define OCP81378_CHANNEL_CH2 1

static const int ocp81378_current[OCP81378_LEVEL_NUM] = {
	57, 72, 87, 103, 118, 133, 149,
	406, 438, 469, 500, 531, 563, 594, 625, 657, 688,
	719, 751, 782, 813, 844, 876, 907, 938, 970, 1001
};

static const unsigned char ocp81378_flash_level[OCP81378_LEVEL_NUM] = {
	0x0E, 0x12, 0x16, 0x1A, 0x1E, 0x22, 0x26,
	0x19, 0x1B, 0x1D, 0x1F, 0x21, 0x23, 0x25, 0x27, 0x29, 0x2B,
	0x2D, 0x2F, 0x31, 0x33, 0x35, 0x37, 0x39, 0x3B, 0x3D, 0x3F
};

static int flash_state_to_current_limit[FLASHLIGHT_COOLER_MAX_STATE] = {
	150000, 100000, 50000, 25000
};

/* define mutex and work queue */
static DEFINE_MUTEX(ocp81378_mutex);

/* struct ocp81378_platform_data
 *
 * @max_flash_timeout: flash timeout
 * @max_flash_brt: flash mode led brightness
 * @max_torch_brt: torch mode led brightness
 */
struct ocp81378_platform_data {
	u32 max_flash_timeout;
	u32 max_flash_brt[OCP81378_LED_MAX];
	u32 max_torch_brt[OCP81378_LED_MAX];
};


enum led_enable {
	MODE_SHDN = 0x0,
	MODE_TORCH = 0x08,
	MODE_FLASH = 0x0C,
};

/* define usage count */
static int use_count;

static struct ocp81378_flash *ocp81378_flash_data;

#define to_ocp81378_flash(_ctrl, _no)	\
	container_of(_ctrl->handler, struct ocp81378_flash, ctrls_led[_no])

static int ocp81378_set_driver(int set);

static int ocp81378_flashlight_mode = FLASHLIGHT_SCENARIO_PRE_FLASH;
static int ocp81378_level_ch1 = -1;
static int ocp81378_level_ch2 = -1;

static int ocp81378_verify_level_by_mode(int level, int flash_mode)
{
	int lev = level;

	if (flash_mode == FLASHLIGHT_SCENARIO_MAIN_FLASH)
	{
		/* main flash */
		if (level < OCP81378_LEVEL_TORCH)
			lev = OCP81378_LEVEL_TORCH;
		else if (level >= OCP81378_LEVEL_NUM)
			lev = OCP81378_LEVEL_NUM - 1;
	}
	else
	{
		/* pre-flash */
		if (level < 0)
			lev = 0;
		else if (level >= OCP81378_LEVEL_TORCH)
			lev = OCP81378_LEVEL_TORCH - 1;
	}

	return lev;
}

static int ocp81378_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= OCP81378_LEVEL_NUM)
		level = OCP81378_LEVEL_NUM - 1;

	return level;
}

/* set flashlight level */
static int ocp81378_set_level_ch1(int level)
{
	int ret = 0, verified_level = -1;
	unsigned char reg, val;

	verified_level = ocp81378_verify_level_by_mode(level, ocp81378_flashlight_mode);

	if (ocp81378_flashlight_mode == FLASHLIGHT_SCENARIO_PRE_FLASH)
	{
		/* set torch brightness level */
		reg = REG_LED0_TORCH_BR;
		val = ocp81378_flash_level[verified_level];
		pr_info_ratelimited("Set ch1 torch brightness 0x%02X %dmA", val, ocp81378_current[verified_level]);
		ret = regmap_update_bits(ocp81378_flash_data->regmap, REG_LED0_TORCH_BR, 0x7F, val);
	}
	else
	{
		/* set flash brightness level */
		reg = REG_LED0_FLASH_BR;
		val = ocp81378_flash_level[verified_level];
		pr_info_ratelimited("Set ch1 flash brightness 0x%02X %dmA", val, ocp81378_current[verified_level]);
		ret = regmap_update_bits(ocp81378_flash_data->regmap, REG_LED0_FLASH_BR, 0x7F, val);
	}

	return ret;
}

static int ocp81378_set_level_ch2(int level)
{
	int ret = 0, verified_level = -1;
	unsigned char reg, val;

	verified_level = ocp81378_verify_level_by_mode(level, ocp81378_flashlight_mode);

	if (ocp81378_flashlight_mode == FLASHLIGHT_SCENARIO_PRE_FLASH)
	{
		/* set torch brightness level */
		reg = REG_LED1_TORCH_BR;
		val = ocp81378_flash_level[verified_level];
		pr_info_ratelimited("Set ch2 torch brightness 0x%02X %dmA", val, ocp81378_current[verified_level]);
		ret = regmap_update_bits(ocp81378_flash_data->regmap, REG_LED1_TORCH_BR, 0x7F, val);
	}
	else
	{
		/* set flash brightness level */
		reg = REG_LED1_FLASH_BR;
		val = ocp81378_flash_level[verified_level];
		pr_info_ratelimited("Set ch2 flash brightness 0x%02X %dmA", val, ocp81378_current[verified_level]);
		ret = regmap_update_bits(ocp81378_flash_data->regmap, REG_LED1_FLASH_BR, 0x7F, val);
	}

	return ret;
}

static int ocp81378_set_level(int channel, int level)
{
	if (channel == OCP81378_CHANNEL_CH1) {
		ocp81378_level_ch1 = ocp81378_verify_level(level);
	}
	else if (channel == OCP81378_CHANNEL_CH2) {
		ocp81378_level_ch2 = ocp81378_verify_level(level);
	}
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return 0;
}

/* define pinctrl */
#define OCP81378_PINCTRL_PIN_HWEN 0
#define OCP81378_PINCTRL_PINSTATE_LOW 0
#define OCP81378_PINCTRL_PINSTATE_HIGH 1
#define OCP81378_PINCTRL_STATE_HWEN_HIGH "hwen-high"
#define OCP81378_PINCTRL_STATE_HWEN_LOW  "hwen-low"
/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int ocp81378_pinctrl_init(struct ocp81378_flash *flash)
{
	int ret = 0;

	/* get pinctrl */
	flash->ocp81378_hwen_pinctrl = devm_pinctrl_get(flash->dev);
	if (IS_ERR(flash->ocp81378_hwen_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(flash->ocp81378_hwen_pinctrl);
		return ret;
	}

	/* Flashlight HWEN pin initialization */
	flash->ocp81378_hwen_high = pinctrl_lookup_state(
			flash->ocp81378_hwen_pinctrl,
			OCP81378_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(flash->ocp81378_hwen_high)) {
		pr_info("Failed to init (%s)\n",
			OCP81378_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(flash->ocp81378_hwen_high);
	}
	flash->ocp81378_hwen_low = pinctrl_lookup_state(
			flash->ocp81378_hwen_pinctrl,
			OCP81378_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(flash->ocp81378_hwen_low)) {
		pr_info("Failed to init (%s)\n", OCP81378_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(flash->ocp81378_hwen_low);
	}

	return ret;
}

#ifdef CONFIG_FLASHLIGHT_DEBUG
int hwen_state = OCP81378_PINCTRL_PINSTATE_LOW;
#endif
static int ocp81378_pinctrl_set(struct ocp81378_flash *flash, int pin, int state)
{
	int ret = 0;

	if (IS_ERR(flash->ocp81378_hwen_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case OCP81378_PINCTRL_PIN_HWEN:
		if (state == OCP81378_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(flash->ocp81378_hwen_low))
		{
			pinctrl_select_state(flash->ocp81378_hwen_pinctrl,
					flash->ocp81378_hwen_low);
			#ifdef CONFIG_FLASHLIGHT_DEBUG
			hwen_state = OCP81378_PINCTRL_PINSTATE_LOW;
			#endif
		}
		else if (state == OCP81378_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(flash->ocp81378_hwen_high))
		{
			pinctrl_select_state(flash->ocp81378_hwen_pinctrl,
					flash->ocp81378_hwen_high);
			#ifdef CONFIG_FLASHLIGHT_DEBUG
			hwen_state = OCP81378_PINCTRL_PINSTATE_HIGH;
			#endif
		}
		else
			pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	default:
		pr_info("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}

	return ret;
}

/* enable mode control */
static int ocp81378_mode_ctrl(struct ocp81378_flash *flash)
{
	int rval = -EINVAL;

	pr_info_ratelimited("%s mode:%d", __func__, flash->led_mode);
	switch (flash->led_mode) {
	case V4L2_FLASH_LED_MODE_NONE:
		rval = regmap_update_bits(flash->regmap,
					  REG_ENABLE, 0x0C, MODE_SHDN);
		break;
	case V4L2_FLASH_LED_MODE_TORCH:
		rval = regmap_update_bits(flash->regmap,
					  REG_ENABLE, 0x0C, MODE_TORCH);
		break;
	case V4L2_FLASH_LED_MODE_FLASH:
		rval = regmap_update_bits(flash->regmap,
					  REG_ENABLE, 0x0C, MODE_FLASH);
		break;
	}
	return rval;
}

/* led1/2 enable/disable */
static int ocp81378_enable_ctrl(struct ocp81378_flash *flash,
			      enum ocp81378_led_id led_no, bool on)
{
	int rval;

	if (led_no < 0 || led_no >= OCP81378_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}
	pr_info_ratelimited("%s led:%d enable:%d", __func__, led_no, on);

#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT_PT)
	if (flashlight_pt_is_low()) {
		pr_info_ratelimited("pt is low\n");
		return 0;
	}
#endif
	if (led_no == OCP81378_LED0) {
		if (on)
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x01, 0x01);
		else
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x01, 0x00);
	} else {
		if (on)
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x02, 0x02);
		else
			rval = regmap_update_bits(flash->regmap,
						  REG_ENABLE, 0x02, 0x00);
	}
	return rval;
}

/* torch1/2 brightness control */
static int ocp81378_torch_brt_ctrl(struct ocp81378_flash *flash,
				 enum ocp81378_led_id led_no, unsigned int brt)
{
	int rval = 0;
	u8 br_bits;

	if (led_no < 0 || led_no >= OCP81378_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}
	pr_info_ratelimited("%s %d brt:%u\n", __func__, led_no, brt);
	if (brt < OCP81378_TORCH_BRT_MIN)
		return ocp81378_enable_ctrl(flash, led_no, false);

	if (flash->need_cooler == 0) {
		flash->ori_current[led_no] = brt;
	} else {
		if (brt > flash->target_current[led_no]) {
			brt = flash->target_current[led_no];
			pr_info("thermal limit current:%d\n", brt);
		}
	}

	if (led_no == OCP81378_LED0)
	{
		br_bits = OCP81378_TORCH_BRT_uA_TO_REG(brt);
		rval = regmap_update_bits(flash->regmap, REG_LED0_TORCH_BR, 0x7F, br_bits);
	}
	else
	{
		br_bits = OCP81378_TORCH_BRT_uA_TO_REG(brt);
		rval = regmap_update_bits(flash->regmap, REG_LED1_TORCH_BR, 0x7F, br_bits);
	}

	return rval;
}

/* flash1/2 brightness control */
static int ocp81378_flash_brt_ctrl(struct ocp81378_flash *flash,
				 enum ocp81378_led_id led_no, unsigned int brt)
{
	int rval = 0;
	u8 br_bits;

	if (led_no < 0 || led_no >= OCP81378_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}
	pr_info_ratelimited("%s %d brt:%u", __func__, led_no, brt);
	if (brt < OCP81378_FLASH_BRT_MIN)
		return ocp81378_enable_ctrl(flash, led_no, false);

	if (flash->need_cooler == 1 && brt > flash->target_current[led_no]) {
		brt = flash->target_current[led_no];
		pr_info("thermal limit current:%d\n", brt);
	}

	if (led_no == OCP81378_LED0)
	{
		br_bits = OCP81378_FLASH_BRT_uA_TO_REG(brt);
		rval = regmap_update_bits(flash->regmap, REG_LED0_FLASH_BR, 0x7F, br_bits);
	}
	else
	{
		br_bits = OCP81378_FLASH_BRT_uA_TO_REG(brt);
		rval = regmap_update_bits(flash->regmap, REG_LED1_FLASH_BR, 0x7F, br_bits);
	}

	return rval;
}

/* flash1/2 timeout control */
static int ocp81378_flash_tout_ctrl(struct ocp81378_flash *flash,
				unsigned int tout)
{
	int rval;
	u8 tout_bits;

	pr_info_ratelimited("%s tout:%u", __func__, tout);
	if (tout == 50)
		tout_bits = 0x04;
	else
		tout_bits = 0x07 + (tout / OCP81378_FLASH_TOUT_STEP);

	rval = regmap_update_bits(flash->regmap,
				  REG_FLASH_TOUT, 0x0f, tout_bits);

	return rval;
}

/* v4l2 controls  */
static int ocp81378_get_ctrl(struct v4l2_ctrl *ctrl, enum ocp81378_led_id led_no)
{
	struct ocp81378_flash *flash = to_ocp81378_flash(ctrl, led_no);
	int rval = -EINVAL;

	mutex_lock(&flash->lock);

	if (ctrl->id == V4L2_CID_FLASH_FAULT) {
		s32 fault = 0;
		unsigned int reg_val = 0;

		rval = regmap_read(flash->regmap, REG_FLAG1, &reg_val);
		if (rval < 0)
			goto out;
		if (reg_val & FAULT_LED0_SHORT_CIRCUIT)
			fault |= V4L2_FLASH_FAULT_SHORT_CIRCUIT;
		if (reg_val & FAULT_LED1_SHORT_CIRCUIT)
			fault |= V4L2_FLASH_FAULT_SHORT_CIRCUIT;
		if (reg_val & FAULT_THERMAL_SHUTDOWN)
			fault |= V4L2_FLASH_FAULT_OVER_TEMPERATURE;
		if (reg_val & FAULT_TIMEOUT)
			fault |= V4L2_FLASH_FAULT_TIMEOUT;
		ctrl->cur.val = fault;
	}

out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int ocp81378_set_ctrl(struct v4l2_ctrl *ctrl, enum ocp81378_led_id led_no)
{
	struct ocp81378_flash *flash = to_ocp81378_flash(ctrl, led_no);
	int rval = -EINVAL;

	pr_info_ratelimited("%s led:%d ID:%d", __func__, led_no, ctrl->id);
	mutex_lock(&flash->lock);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		flash->led_mode = ctrl->val;
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH)
			rval = ocp81378_mode_ctrl(flash);
		else
			rval = 0;
		if (flash->led_mode == V4L2_FLASH_LED_MODE_NONE)
			ocp81378_enable_ctrl(flash, led_no, false);
		else if (flash->led_mode == V4L2_FLASH_LED_MODE_TORCH)
			rval = ocp81378_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_SOURCE:
		if (ctrl->val == V4L2_FLASH_STROBE_SOURCE_SOFTWARE) {
			pr_info("sw ctrl\n");
			rval = regmap_update_bits(flash->regmap,
					REG_ENABLE, 0x2C, 0x00);
		} else if (ctrl->val == V4L2_FLASH_STROBE_SOURCE_EXTERNAL) {
			pr_info("hw trigger\n");
			rval = regmap_update_bits(flash->regmap,
					REG_ENABLE, 0x2C, 0x24);
			rval = ocp81378_enable_ctrl(flash, led_no, true);
		}
		if (rval < 0)
			goto err_out;
		break;

	case V4L2_CID_FLASH_STROBE:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		flash->led_mode = V4L2_FLASH_LED_MODE_FLASH;
		rval = ocp81378_mode_ctrl(flash);
		rval = ocp81378_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_STOP:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		ocp81378_enable_ctrl(flash, led_no, false);
		flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
		rval = ocp81378_mode_ctrl(flash);
		break;

	case V4L2_CID_FLASH_TIMEOUT:
		rval = ocp81378_flash_tout_ctrl(flash, ctrl->val);
		break;

	case V4L2_CID_FLASH_INTENSITY:
		rval = ocp81378_flash_brt_ctrl(flash, led_no, ctrl->val);
		break;

	case V4L2_CID_FLASH_TORCH_INTENSITY:
		rval = ocp81378_torch_brt_ctrl(flash, led_no, ctrl->val);
		break;
	}

err_out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int ocp81378_led1_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return ocp81378_get_ctrl(ctrl, OCP81378_LED1);
}

static int ocp81378_led1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return ocp81378_set_ctrl(ctrl, OCP81378_LED1);
}

static int ocp81378_led0_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return ocp81378_get_ctrl(ctrl, OCP81378_LED0);
}

static int ocp81378_led0_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return ocp81378_set_ctrl(ctrl, OCP81378_LED0);
}

static const struct v4l2_ctrl_ops ocp81378_led_ctrl_ops[OCP81378_LED_MAX] = {
	[OCP81378_LED0] = {
			.g_volatile_ctrl = ocp81378_led0_get_ctrl,
			.s_ctrl = ocp81378_led0_set_ctrl,
			},
	[OCP81378_LED1] = {
			.g_volatile_ctrl = ocp81378_led1_get_ctrl,
			.s_ctrl = ocp81378_led1_set_ctrl,
			}
};

static int ocp81378_init_controls(struct ocp81378_flash *flash,
				enum ocp81378_led_id led_no)
{
	struct v4l2_ctrl *fault;
	u32 max_flash_brt = flash->pdata->max_flash_brt[led_no];
	u32 max_torch_brt = flash->pdata->max_torch_brt[led_no];
	struct v4l2_ctrl_handler *hdl = &flash->ctrls_led[led_no];
	const struct v4l2_ctrl_ops *ops = &ocp81378_led_ctrl_ops[led_no];

	v4l2_ctrl_handler_init(hdl, 8);

	/* flash mode */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_LED_MODE,
			       V4L2_FLASH_LED_MODE_TORCH, ~0x7,
			       V4L2_FLASH_LED_MODE_NONE);
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;

	/* flash source */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_STROBE_SOURCE,
			       0x1, ~0x3, V4L2_FLASH_STROBE_SOURCE_SOFTWARE);

	/* flash strobe */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE, 0, 0, 0, 0);

	/* flash strobe stop */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE_STOP, 0, 0, 0, 0);

	/* flash strobe timeout */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TIMEOUT,
			  OCP81378_FLASH_TOUT_MIN,
			  flash->pdata->max_flash_timeout,
			  OCP81378_FLASH_TOUT_STEP,
			  flash->pdata->max_flash_timeout);

	/* flash brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_INTENSITY,
			  OCP81378_FLASH_BRT_MIN, max_flash_brt,
			  OCP81378_FLASH_BRT_STEP, max_flash_brt);

	/* torch brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TORCH_INTENSITY,
			  OCP81378_TORCH_BRT_MIN, max_torch_brt,
			  OCP81378_TORCH_BRT_STEP, max_torch_brt);

	/* fault */
	fault = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_FAULT, 0,
				  V4L2_FLASH_FAULT_OVER_VOLTAGE
				  | V4L2_FLASH_FAULT_OVER_TEMPERATURE
				  | V4L2_FLASH_FAULT_SHORT_CIRCUIT
				  | V4L2_FLASH_FAULT_TIMEOUT, 0, 0);
	if (fault != NULL)
		fault->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (hdl->error)
		return hdl->error;

	if (led_no < 0 || led_no >= OCP81378_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}

	flash->subdev_led[led_no].ctrl_handler = hdl;
	return 0;
}

/* initialize device */
static const struct v4l2_subdev_ops ocp81378_ops = {
	.core = NULL,
};

static const struct regmap_config ocp81378_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static void ocp81378_v4l2_i2c_subdev_init(struct v4l2_subdev *sd,
		struct i2c_client *client,
		const struct v4l2_subdev_ops *ops)
{
	int ret = 0;

	v4l2_subdev_init(sd, ops);
	sd->flags |= V4L2_SUBDEV_FL_IS_I2C;
	/* the owner is the same as the i2c_client's driver owner */
	sd->owner = client->dev.driver->owner;
	sd->dev = &client->dev;
	/* i2c_client and v4l2_subdev point to one another */
	v4l2_set_subdevdata(sd, client);
	i2c_set_clientdata(client, sd);
	/* initialize name */
	ret = snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		client->dev.driver->name, i2c_adapter_id(client->adapter),
		client->addr);
	if (ret < 0)
		pr_info("snprintf failed\n");
}

static int ocp81378_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	ocp81378_set_driver(1);

	return 0;
}

static int ocp81378_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	ocp81378_set_driver(0);

	return 0;
}

static const struct v4l2_subdev_internal_ops ocp81378_int_ops = {
	.open = ocp81378_open,
	.close = ocp81378_close,
};

static int ocp81378_subdev_init(struct ocp81378_flash *flash,
			      enum ocp81378_led_id led_no, char *led_name)
{
	struct i2c_client *client = to_i2c_client(flash->dev);
	struct device_node *np = flash->dev->of_node, *child;
	const char *fled_name = "flash";
	int rval;

	// pr_info("%s %d", __func__, led_no);
	if (led_no < 0 || led_no >= OCP81378_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}

	ocp81378_v4l2_i2c_subdev_init(&flash->subdev_led[led_no],
				client, &ocp81378_ops);
	flash->subdev_led[led_no].flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->subdev_led[led_no].internal_ops = &ocp81378_int_ops;
	strscpy(flash->subdev_led[led_no].name, led_name,
		sizeof(flash->subdev_led[led_no].name));

	for (child = of_get_child_by_name(np, fled_name); child;
			child = of_find_node_by_name(child, fled_name)) {
		int rv;
		u32 reg = 0;

		rv = of_property_read_u32(child, "reg", &reg);
		if (rv)
			continue;

		if (reg == led_no) {
			flash->dnode[led_no] = child;
			flash->subdev_led[led_no].fwnode =
				of_fwnode_handle(flash->dnode[led_no]);
		}
	}

	rval = ocp81378_init_controls(flash, led_no);
	if (rval)
		goto err_out;
	rval = media_entity_pads_init(&flash->subdev_led[led_no].entity, 0, NULL);
	if (rval < 0)
		goto err_out;
	flash->subdev_led[led_no].entity.function = MEDIA_ENT_F_FLASH;

	rval = v4l2_async_register_subdev(&flash->subdev_led[led_no]);
	if (rval < 0)
		goto err_out;

	return rval;

err_out:
	v4l2_ctrl_handler_free(&flash->ctrls_led[led_no]);
	return rval;
}

/* flashlight init */
static int ocp81378_init(struct ocp81378_flash *flash)
{
	int rval = 0;
	unsigned int reg_val;

	ocp81378_flashlight_mode = FLASHLIGHT_SCENARIO_PRE_FLASH;

#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT_DLPT)
	flashlight_kicker_pbm_by_device_id(
				&flash->flash_dev_id[OCP81378_LED0],
				flash->torch_power);
	mdelay(1);
#endif
	ocp81378_pinctrl_set(flash, OCP81378_PINCTRL_PIN_HWEN, OCP81378_PINCTRL_PINSTATE_HIGH);

	/* set timeout */
	rval = ocp81378_flash_tout_ctrl(flash, 400);
	if (rval < 0)
		return rval;

	/* output disable */
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
	rval = ocp81378_mode_ctrl(flash);

	ocp81378_torch_brt_ctrl(flash, OCP81378_LED0,
				flash->ori_current[OCP81378_LED0]);
	ocp81378_torch_brt_ctrl(flash, OCP81378_LED1,
				flash->ori_current[OCP81378_LED1]);
	ocp81378_flash_brt_ctrl(flash, OCP81378_LED0,
				flash->ori_current[OCP81378_LED0]);
	ocp81378_flash_brt_ctrl(flash, OCP81378_LED1,
				flash->ori_current[OCP81378_LED1]);

	/* reset faults */
	rval = regmap_read(flash->regmap, REG_FLAG1, &reg_val);
	return rval;
}

/* flashlight uninit */
static int ocp81378_uninit(struct ocp81378_flash *flash)
{
	ocp81378_pinctrl_set(flash,
			OCP81378_PINCTRL_PIN_HWEN, OCP81378_PINCTRL_PINSTATE_LOW);
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT_DLPT)
	flashlight_kicker_pbm_by_device_id(
				&flash->flash_dev_id[OCP81378_LED0],
				0);
#endif
	return 0;
}

static int ocp81378_flash_open(void)
{
	return 0;
}

static int ocp81378_flash_release(void)
{
	return 0;
}

static int ocp81378_cooling_set_cur_state(int channel, unsigned long state)
{
	struct ocp81378_flash *flash = ocp81378_flash_data;
	int ret = 0;

	/* Request state should be less than max_state */
	if (state > FLASHLIGHT_COOLER_MAX_STATE)
		return -EINVAL;

	if (channel < 0 || channel >= OCP81378_LED_MAX) {
		pr_info("channel error\n");
		return -EINVAL;
	}

	pr_info("set thermal current:%lu\n", state);

	if (state == 0) {
		flash->need_cooler = 0;
		flash->target_current[channel] = OCP81378_FLASH_BRT_MAX;
		ret = ocp81378_torch_brt_ctrl(flash, channel,
					OCP81378_TORCH_BRT_MAX);
	} else {
		flash->need_cooler = 1;
		flash->target_current[channel] =
			flash_state_to_current_limit[state - 1];
		ret = ocp81378_torch_brt_ctrl(flash, channel,
					flash->target_current[channel]);
	}

	return ret;
}

static int ocp81378_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel, scenario, ret = 0;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	switch (cmd) {
	case FLASH_IOC_SET_ONOFF:
		pr_info_ratelimited("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if ((int)fl_arg->arg) {
			if(channel == OCP81378_CHANNEL_CH1)
			{
				ret = ocp81378_set_level_ch1(ocp81378_level_ch1);
			}
			else if(channel == OCP81378_CHANNEL_CH2)
			{
				ret = ocp81378_set_level_ch2(ocp81378_level_ch2);
			}
			else
			{
				pr_err("%s Error: unkown channel %d", __func__, channel);
				return -1;
			}
			if (ocp81378_flashlight_mode == FLASHLIGHT_SCENARIO_MAIN_FLASH)
			{
				ocp81378_flash_data->led_mode = V4L2_FLASH_LED_MODE_FLASH;
			} else {
				ocp81378_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
			}
			ret |= ocp81378_mode_ctrl(ocp81378_flash_data);
			ret |= ocp81378_enable_ctrl(ocp81378_flash_data, channel, true);
			if (ocp81378_flashlight_mode == FLASHLIGHT_SCENARIO_MAIN_FLASH)
			{
				ocp81378_flashlight_mode = FLASHLIGHT_SCENARIO_PRE_FLASH;
			}
		} else {
			if (ocp81378_flash_data->led_mode != V4L2_FLASH_LED_MODE_NONE) {
				ocp81378_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
				ret = ocp81378_mode_ctrl(ocp81378_flash_data);
				ret |= ocp81378_enable_ctrl(ocp81378_flash_data, channel, false);
			}
		}
		break;
	case FLASH_IOC_SET_SCENARIO:
		pr_info_ratelimited("FLASH_IOC_SET_SCENARIO(%d): %d\n",
				channel, (int)fl_arg->arg);
		scenario = (int)fl_arg->arg;
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT_DLPT)
		if (scenario & FLASHLIGHT_SCENARIO_CAMERA_MASK) {
			flashlight_kicker_pbm_by_device_id(
				&ocp81378_flash_data->flash_dev_id[OCP81378_LED0],
				ocp81378_flash_data->camera_power);
		} else {
			flashlight_kicker_pbm_by_device_id(
				&ocp81378_flash_data->flash_dev_id[OCP81378_LED0],
				ocp81378_flash_data->torch_power);
		}
#endif
		break;

	case FLASH_IOC_PRE_ON:
		pr_info_ratelimited("FLASH_IOC_PRE_ON(%d): %d\n",
				channel, (int)fl_arg->arg);
		ocp81378_flashlight_mode = FLASHLIGHT_SCENARIO_MAIN_FLASH;
		break;

	case FLASH_IOC_SET_THERMAL_CUR_STATE:
		pr_info_ratelimited("FLASH_IOC_SET_THERMAL_CUR_STATE(%d): %d\n",
				channel, (int)fl_arg->arg);
		ret = ocp81378_cooling_set_cur_state(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info_ratelimited("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		ret = ocp81378_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info_ratelimited("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = OCP81378_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info_ratelimited("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = OCP81378_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = ocp81378_verify_level(fl_arg->arg);
		pr_info_ratelimited("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = ocp81378_current[fl_arg->arg];
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return ret;
}

static int ocp81378_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&ocp81378_mutex);
	if (set) {
		if (!use_count)
			ret = ocp81378_init(ocp81378_flash_data);
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = ocp81378_uninit(ocp81378_flash_data);
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&ocp81378_mutex);

	return ret;
}

static ssize_t ocp81378_strobe_store(struct flashlight_arg arg)
{
	ocp81378_set_driver(1);
	//ocp81378_set_level(arg.channel, arg.level);
	//ocp81378_timeout_ms[arg.channel] = 0;
	//ocp81378_enable(arg.channel);
	ocp81378_torch_brt_ctrl(ocp81378_flash_data, arg.channel,
				arg.level * 25000);
	ocp81378_enable_ctrl(ocp81378_flash_data, arg.channel, true);
	ocp81378_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
	ocp81378_mode_ctrl(ocp81378_flash_data);
	msleep(arg.dur);
	//ocp81378_disable(arg.channel);
	ocp81378_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
	ocp81378_mode_ctrl(ocp81378_flash_data);
	ocp81378_enable_ctrl(ocp81378_flash_data, arg.channel, false);
	ocp81378_set_driver(0);
	return 0;
}

static struct flashlight_operations ocp81378_flash_ops = {
	ocp81378_flash_open,
	ocp81378_flash_release,
	ocp81378_ioctl,
	ocp81378_strobe_store,
	ocp81378_set_driver
};

static int ocp81378_parse_dt(struct ocp81378_flash *flash)
{
	struct device_node *np, *cnp;
	struct device *dev = flash->dev;
	u32 decouple = 0, vin = 3600;
	int i = 0, ret = 0;
	double power = 0;

	if (!dev || !dev->of_node)
		return -ENODEV;

	np = dev->of_node;
	if (of_property_read_u32(np, "vin", &vin))
		pr_info("Parse no dt, vin.\n");

	power = OCP81378_TORCH_BRT_MAX / 1000 * vin / 1000 * 2;
	flash->torch_power = (unsigned int) power;
	power = OCP81378_FLASH_BRT_MAX / 1000 * vin / 1000;
	flash->camera_power = (unsigned int) power;

	if (of_property_read_u32_array(np, "state-current",
			flash_state_to_current_limit,
			FLASHLIGHT_COOLER_MAX_STATE)){
		pr_info("Parse no dt, state-current use default.\n");
	}

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type",
					&flash->flash_dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"ct", &flash->flash_dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp,
					"part", &flash->flash_dev_id[i].part))
			goto err_node_put;
		ret = snprintf(flash->flash_dev_id[i].name,
				FLASHLIGHT_NAME_SIZE, "%s",
				flash->subdev_led[i].name);
		if (ret < 0)
			pr_info("snprintf failed\n");
		flash->flash_dev_id[i].channel = i;
		flash->flash_dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				flash->flash_dev_id[i].type,
				flash->flash_dev_id[i].ct,
				flash->flash_dev_id[i].part,
				flash->flash_dev_id[i].name,
				flash->flash_dev_id[i].channel,
				flash->flash_dev_id[i].decouple);
		if (flashlight_dev_register_by_device_id(&flash->flash_dev_id[i],
			&ocp81378_flash_ops))
			return -EFAULT;
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

#ifdef CONFIG_FLASHLIGHT_DEBUG
u8 RegAddr = 0;
u8 RegVal = 0;
ssize_t flashlight_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (NULL == client)
	{
		return scnprintf(buf, PAGE_SIZE, "Fail to get i2c_client\n");
	}

	if(hwen_state == OCP81378_PINCTRL_PINSTATE_LOW)
	{
		ocp81378_pinctrl_set(ocp81378_flash_data, OCP81378_PINCTRL_PIN_HWEN,
			OCP81378_PINCTRL_PINSTATE_HIGH);
		msleep(10);
	}

	mutex_lock(&ocp81378_mutex);
	ret = i2c_smbus_read_byte_data(client, RegAddr);
	mutex_unlock(&ocp81378_mutex);
	if (ret < 0)
	{
		return scnprintf(buf, PAGE_SIZE, "Fail to read RegAddr 0x%02X: ret %d\n", RegAddr, ret);
	}
	RegVal = (u8)ret;

	return scnprintf(buf, PAGE_SIZE, "0x%02X 0x%02X\n",	RegAddr, RegVal);
};

ssize_t flashlight_reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char delim[] = " ";
	s32 num;
	int count = 0;
	char *token, *cur = (char *)buf;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (NULL == client)
	{
		pr_info("%s: Fail to get i2c_client\n", __func__);
		return -1;
	}

	while (cur) {
		token = strsep(&cur, delim);
		ret = kstrtos32(token, 16, &num);
		if (ret) {
			pr_info("%s: Error argument %d\n", __func__, count);
			return -1;
		}

		if (count == 0)
			RegAddr = (u8)num;
		else if (count == 1)
			RegVal = (u8)num;
		else {
			break;
		}

		count++;
	}

	/*
	 * RegAddr is 0, set HWEN to low for shutdown/reset.
	 */
	if (0 == RegAddr && hwen_state == OCP81378_PINCTRL_PINSTATE_HIGH)
	{
		ocp81378_pinctrl_set(ocp81378_flash_data, OCP81378_PINCTRL_PIN_HWEN,
				OCP81378_PINCTRL_PINSTATE_LOW);
		msleep(10);
	}
	else
	{
		/*
		* 2 arguments, the second one shoule be RegVal, do I2C write,
		* otherwise 1 argument, only update RegAddr.
		*/
		if (2 == count)
		{
			if(hwen_state == OCP81378_PINCTRL_PINSTATE_LOW)
			{
				ocp81378_pinctrl_set(ocp81378_flash_data, OCP81378_PINCTRL_PIN_HWEN,
					OCP81378_PINCTRL_PINSTATE_HIGH);
				msleep(10);
			}

			mutex_lock(&ocp81378_mutex);
			ret = i2c_smbus_write_byte_data(client, RegAddr, RegVal);
			mutex_unlock(&ocp81378_mutex);
			if (ret < 0)
			{
				pr_info("%s: failed writing at 0x%02x: ret %d\n", __func__, RegAddr, ret);
				return ret;
			}

			pr_info("%s: RegAddr 0x%02X, RegVal 0x%02X\n", __func__, RegAddr, RegVal);
		}
	}

	return size;
};

static DEVICE_ATTR(flashlight_reg, 0664, flashlight_reg_show, flashlight_reg_store);
#endif	//ifdef CONFIG_FLASHLIGHT_DEBUG

static int ocp81378_probe(struct i2c_client *client)
{
	struct ocp81378_flash *flash;
	struct ocp81378_platform_data *pdata = dev_get_platdata(&client->dev);
	int rval;

	pr_info("%s:%d", __func__, __LINE__);

	flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	flash->regmap = devm_regmap_init_i2c(client, &ocp81378_regmap);
	if (IS_ERR(flash->regmap)) {
		rval = PTR_ERR(flash->regmap);
		return rval;
	}

	/* if there is no platform data, use chip default value */
	if (pdata == NULL) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			return -ENODEV;
		pdata->max_flash_timeout = OCP81378_FLASH_TOUT_MAX;
		/* led 1 */
		pdata->max_flash_brt[OCP81378_LED0] = OCP81378_FLASH_BRT_MAX;
		pdata->max_torch_brt[OCP81378_LED0] = OCP81378_TORCH_BRT_MAX;
		/* led 2 */
		pdata->max_flash_brt[OCP81378_LED1] = OCP81378_FLASH_BRT_MAX;
		pdata->max_torch_brt[OCP81378_LED1] = OCP81378_TORCH_BRT_MAX;
	}
	flash->pdata = pdata;
	flash->dev = &client->dev;
	mutex_init(&flash->lock);
	ocp81378_flash_data = flash;

	rval = ocp81378_pinctrl_init(flash);
	if (rval < 0)
		return rval;

	rval = ocp81378_subdev_init(flash, OCP81378_LED0, "ocp81378-led0");
	if (rval < 0)
		return rval;

	rval = ocp81378_parse_dt(flash);

	i2c_set_clientdata(client, flash);

	flash->need_cooler = 0;
	flash->target_current[OCP81378_LED0] = OCP81378_FLASH_BRT_MAX;
	flash->target_current[OCP81378_LED1] = OCP81378_FLASH_BRT_MAX;
	flash->ori_current[OCP81378_LED0] = OCP81378_TORCH_BRT_MIN;
	flash->ori_current[OCP81378_LED1] = OCP81378_TORCH_BRT_MIN;

#ifdef CONFIG_FLASHLIGHT_DEBUG
	if (device_create_file(&client->dev, &dev_attr_flashlight_reg)) {
		pr_info("Failed to create device file(flashlight_reg)\n");
		goto err_create_flashlight_reg_device_file;
	}
#endif

	pr_info("%s:%d", __func__, __LINE__);
	return 0;

#ifdef CONFIG_FLASHLIGHT_DEBUG
err_create_flashlight_reg_device_file:
	device_remove_file(&client->dev, &dev_attr_flashlight_reg);
	i2c_set_clientdata(client, NULL);
	for (int i = OCP81378_LED0; i < OCP81378_LED_MAX; i++) {
		v4l2_ctrl_handler_free(&flash->ctrls_led[i]);
		media_entity_cleanup(&flash->subdev_led[i].entity);
		v4l2_device_unregister_subdev(&flash->subdev_led[i]);
	}
	return -1;
#endif
}

static void ocp81378_remove(struct i2c_client *client)
{
	struct ocp81378_flash *flash = i2c_get_clientdata(client);
	unsigned int i;

	for (i = OCP81378_LED0; i < OCP81378_LED_MAX; i++) {
		v4l2_ctrl_handler_free(&flash->ctrls_led[i]);
		media_entity_cleanup(&flash->subdev_led[i].entity);
		v4l2_device_unregister_subdev(&flash->subdev_led[i]);
	}

#ifdef CONFIG_FLASHLIGHT_DEBUG
	device_remove_file(&client->dev, &dev_attr_flashlight_reg);
#endif
}

static const struct i2c_device_id ocp81378_id_table[] = {
	{OCP81378_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ocp81378_id_table);

static const struct of_device_id ocp81378_of_table[] = {
	{ .compatible = "mediatek,ocp81378" },
	{ },
};
MODULE_DEVICE_TABLE(of, ocp81378_of_table);

static struct i2c_driver ocp81378_i2c_driver = {
	.driver = {
		   .name = OCP81378_NAME,
		   .of_match_table = ocp81378_of_table,
		   },
	.probe = ocp81378_probe,
	.remove = ocp81378_remove,
	.id_table = ocp81378_id_table,
};

module_i2c_driver(ocp81378_i2c_driver);

MODULE_AUTHOR("Roger-HY Wang <roger-hy.wang@mediatek.com>");
MODULE_DESCRIPTION("Texas Instruments OCP81378 LED flash driver");
MODULE_LICENSE("GPL");
