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

#include <linux/power_supply.h>
#endif

#include "flashlight-t405a.h"
#include "ocp81378-t405a.h"
#include "aw36515e-t405a.h"

#define FLASHLIGHT_NAME	"flashlight_t405a"
#define FLASHLIGHT_I2C_ADDR	(0x63)

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

/* Chip ID */
#define FLASHLIGHT_CHIP_ID_OCP81378		0x3A
#define FLASHLIGHT_CHIP_ID_AW36515E		0x02

/* fault mask */
#define FAULT_TIMEOUT	(1<<0)
#define FAULT_THERMAL_SHUTDOWN	(1<<2)
#define FAULT_LED0_SHORT_CIRCUIT	(1<<5)
#define FAULT_LED1_SHORT_CIRCUIT	(1<<4)

/*  FLASH TIMEOUT DURATION
 *	min 40ms, step 40ms, max 1600ms
 */
#define FLASHLIGHT_FLASH_TOUT_MIN 40
#define FLASHLIGHT_FLASH_TOUT_STEP 40
#define FLASHLIGHT_FLASH_TOUT_MAX 1600

#if !IS_ENABLED(CONFIG_MTK_FLASHLIGHT_THERMAL)
#define FLASHLIGHT_COOLER_MAX_STATE 4
#endif

/* define channel, level */
#define FLASHLIGHT_CHANNEL_NUM 2
#define FLASHLIGHT_CHANNEL_CH1 0
#define FLASHLIGHT_CHANNEL_CH2 1

static int flash_state_to_current_limit[FLASHLIGHT_COOLER_MAX_STATE] = {
	150000, 100000, 50000, 25000
};

/* define mutex and work queue */
static DEFINE_MUTEX(flashlight_mutex);

/* struct flashlight_platform_data
 *
 * @max_flash_timeout: flash timeout
 * @max_flash_brt: flash mode led brightness
 * @max_torch_brt: torch mode led brightness
 */
struct flashlight_platform_data {
	u32 max_flash_timeout;
	u32 max_flash_brt[FLASHLIGHT_LED_MAX];
	u32 max_torch_brt[FLASHLIGHT_LED_MAX];
};

enum led_enable {
	MODE_SHDN = 0x0,
	MODE_TORCH = 0x08,
	MODE_FLASH = 0x0C,
};

/* define usage count */
static int use_count = 0;

static struct flashlight_flash *flashlight_flash_data;

static unsigned int flash_brt_min = 0;
static unsigned int flash_brt_step = 0;
static unsigned int flash_brt_max = 0;
static unsigned int torch_brt_min = 0;
static unsigned int torch_brt_step = 0;
static unsigned int torch_brt_max = 0;
static u8 brt_reg_mask = 0;
static unsigned int torch_mode_default_current = 0;
const int* current_table = NULL;
const unsigned char* flash_level = NULL;
unsigned char (*pFlashBrtuA2Reg)(unsigned int flash_current);
unsigned char (*pTorchBrtuA2Reg)(unsigned int torch_current);

extern const int ocp81378_current[FLASHLIGHT_LEVEL_NUM];
extern const unsigned char ocp81378_flash_level[FLASHLIGHT_LEVEL_NUM];

extern unsigned char ocp81378_flash_brt_uA_2_reg(unsigned int flash_current);
extern unsigned char ocp81378_torch_brt_uA_2_reg(unsigned int torch_current);

extern const int aw36515e_current[FLASHLIGHT_LEVEL_NUM];
extern const unsigned char aw36515e_flash_level[FLASHLIGHT_LEVEL_NUM];

extern unsigned char aw36515e_flash_brt_uA_2_reg(unsigned int flash_current);
extern unsigned char aw36515e_torch_brt_uA_2_reg(unsigned int torch_current);

#define to_flashlight_flash(_ctrl, _no)	\
	container_of(_ctrl->handler, struct flashlight_flash, ctrls_led[_no])

static int flashlight_set_driver(int set);

static int flashlight_mode = FLASHLIGHT_SCENARIO_PRE_FLASH;
static int flashlight_level_ch1 = -1;
static int flashlight_level_ch2 = -1;

static int flashlight_verify_level_by_mode(int level, int flash_mode)
{
	int lev = level;

	if (flash_mode == FLASHLIGHT_SCENARIO_MAIN_FLASH)
	{
		/* main flash */
		if (level < FLASHLIGHT_LEVEL_TORCH)
			lev = FLASHLIGHT_LEVEL_TORCH;
		else if (level >= FLASHLIGHT_LEVEL_NUM)
			lev = FLASHLIGHT_LEVEL_NUM - 1;
	}

	return lev;
}

static int flashlight_verify_level(int level)
{
	int lev = level;

	if (level < 0)
		lev = 0;
	else if (level >= FLASHLIGHT_LEVEL_NUM)
		lev = FLASHLIGHT_LEVEL_NUM - 1;

	return lev;
}

/* set flashlight level */
static int flashlight_set_level_ch1(int level)
{
	int ret = 0, verified_level = -1;
	unsigned char reg = 0, val = 0;

	if (flashlight_mode == FLASHLIGHT_SCENARIO_PRE_FLASH)
	{
		ret = regmap_update_bits(flashlight_flash_data->regmap, 
					REG_LED0_TORCH_BR, brt_reg_mask, torch_mode_default_current);
	}
	else
	{
		if(current_table == NULL)
		{
			pr_err("%s Error: current_table is NULL", __func__);
			return -1;
		}
		if(flash_level == NULL)
		{
			pr_err("%s Error: flash_level is NULL", __func__);
			return -1;
		}
		verified_level = flashlight_verify_level_by_mode(level, flashlight_mode);
		/* set flash brightness level */
		reg = REG_LED0_FLASH_BR;
		val = flash_level[verified_level];
		pr_info_ratelimited("Set ch1 flash brightness 0x%02X %dmA", 
			val, current_table[verified_level]);
		ret = regmap_update_bits(flashlight_flash_data->regmap, 
				REG_LED0_FLASH_BR, brt_reg_mask, val);
	}

	return ret;
}

static int flashlight_set_level_ch2(int level)
{
	int ret = 0, verified_level = -1;
	unsigned char reg = 0, val = 0;

	if (flashlight_mode == FLASHLIGHT_SCENARIO_PRE_FLASH)
	{
		ret = regmap_update_bits(flashlight_flash_data->regmap, 
					REG_LED1_TORCH_BR, brt_reg_mask, torch_mode_default_current);
	}
	else
	{
		if(current_table == NULL)
		{
			pr_err("%s Error: current_table is NULL", __func__);
			return -1;
		}
		if(flash_level == NULL)
		{
			pr_err("%s Error: flash_level is NULL", __func__);
			return -1;
		}
		verified_level = flashlight_verify_level_by_mode(level, flashlight_mode);
		/* set flash brightness level */
		reg = REG_LED1_FLASH_BR;
		val = flash_level[verified_level];
		pr_info_ratelimited("Set ch2 flash brightness 0x%02X %dmA", 
			val, current_table[verified_level]);
		ret = regmap_update_bits(flashlight_flash_data->regmap, 
				REG_LED1_FLASH_BR, brt_reg_mask, val);
	}

	return ret;
}

static int flashlight_set_level(int channel, int level)
{
	if (channel == FLASHLIGHT_CHANNEL_CH1) {
		flashlight_level_ch1 = flashlight_verify_level(level);
	}
	else if (channel == FLASHLIGHT_CHANNEL_CH2) {
		flashlight_level_ch2 = flashlight_verify_level(level);
	}
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return 0;
}

/* define pinctrl */
#define FLASHLIGHT_PINCTRL_PIN_HWEN 0
#define FLASHLIGHT_PINCTRL_PINSTATE_LOW 0
#define FLASHLIGHT_PINCTRL_PINSTATE_HIGH 1
#define FLASHLIGHT_PINCTRL_STATE_HWEN_HIGH "hwen-high"
#define FLASHLIGHT_PINCTRL_STATE_HWEN_LOW  "hwen-low"
/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int flashlight_pinctrl_init(struct flashlight_flash *flash)
{
	int ret = 0;

	/* get pinctrl */
	flash->flashlight_hwen_pinctrl = devm_pinctrl_get(flash->dev);
	if (IS_ERR(flash->flashlight_hwen_pinctrl)) {
		pr_info("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(flash->flashlight_hwen_pinctrl);
		return ret;
	}

	/* Flashlight HWEN pin initialization */
	flash->flashlight_hwen_high = pinctrl_lookup_state(
			flash->flashlight_hwen_pinctrl,
			FLASHLIGHT_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(flash->flashlight_hwen_high)) {
		pr_info("Failed to init (%s)\n",
			FLASHLIGHT_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(flash->flashlight_hwen_high);
	}
	flash->flashlight_hwen_low = pinctrl_lookup_state(
			flash->flashlight_hwen_pinctrl,
			FLASHLIGHT_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(flash->flashlight_hwen_low)) {
		pr_info("Failed to init (%s)\n", FLASHLIGHT_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(flash->flashlight_hwen_low);
	}

	return ret;
}

#ifdef CONFIG_FLASHLIGHT_DEBUG
int hwen_state = FLASHLIGHT_PINCTRL_PINSTATE_LOW;
#endif
static int flashlight_pinctrl_set(struct flashlight_flash *flash, int pin, int state)
{
	int ret = -EINVAL;

	if (IS_ERR(flash->flashlight_hwen_pinctrl)) {
		pr_info("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case FLASHLIGHT_PINCTRL_PIN_HWEN:
		if (state == FLASHLIGHT_PINCTRL_PINSTATE_LOW &&
				!IS_ERR(flash->flashlight_hwen_low))
		{
			ret = pinctrl_select_state(flash->flashlight_hwen_pinctrl,
					flash->flashlight_hwen_low);
			#ifdef CONFIG_FLASHLIGHT_DEBUG
			hwen_state = FLASHLIGHT_PINCTRL_PINSTATE_LOW;
			#endif
		}
		else if (state == FLASHLIGHT_PINCTRL_PINSTATE_HIGH &&
				!IS_ERR(flash->flashlight_hwen_high))
		{
			ret = pinctrl_select_state(flash->flashlight_hwen_pinctrl,
					flash->flashlight_hwen_high);
			#ifdef CONFIG_FLASHLIGHT_DEBUG
			hwen_state = FLASHLIGHT_PINCTRL_PINSTATE_HIGH;
			#endif
		}
		else
		{
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
			return -1;
		}
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		ret = -1;
		break;
	}

	return ret;
}

/* enable mode control */
static int flashlight_mode_ctrl(struct flashlight_flash *flash)
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
	default:
		pr_err("Unknow flash_mode %d", flash->led_mode);
		rval = -1;
		break;
	}

	return rval;
}

/* led1/2 enable/disable */
static int flashlight_enable_ctrl(struct flashlight_flash *flash,
			      enum flashlight_led_id led_no, bool on)
{
	int rval = -EINVAL;

	if (led_no < 0 || led_no >= FLASHLIGHT_LED_MAX) {
		pr_err("led_no %d error\n", led_no);
		return -1;
	}
	pr_info_ratelimited("%s led:%d enable:%d", __func__, led_no, on);

#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT_PT)
	if (flashlight_pt_is_low()) {
		pr_info_ratelimited("pt is low\n");
		return 0;
	}
#endif
	if (led_no == FLASHLIGHT_LED0) {
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
static int flashlight_torch_brt_ctrl(struct flashlight_flash *flash,
				 enum flashlight_led_id led_no, unsigned int brt)
{
	int rval = 0;
	u8 br_bits = 0;

	if (led_no < 0 || led_no >= FLASHLIGHT_LED_MAX) {
		pr_info("led_no %d error\n", led_no);
		return -1;
	}
	pr_info_ratelimited("%s %d brt:%u\n", __func__, led_no, brt);
	if (brt < torch_brt_min)
		return flashlight_enable_ctrl(flash, led_no, false);

	if (flash->need_cooler == 0) {
		flash->ori_current[led_no] = brt;
	} else {
		if (brt > flash->target_current[led_no]) {
			brt = flash->target_current[led_no];
			pr_info("thermal limit current:%d\n", brt);
		}
	}

	if(pTorchBrtuA2Reg == NULL)
	{
		pr_err("%s Error: pTorchBrtuA2Reg is NULL", __func__);
		return -1;
	}
	br_bits = pTorchBrtuA2Reg(brt);
	if (led_no == FLASHLIGHT_LED0)
	{
		rval = regmap_update_bits(flash->regmap, REG_LED0_TORCH_BR, brt_reg_mask, br_bits);
	}
	else
	{
		rval = regmap_update_bits(flash->regmap, REG_LED1_TORCH_BR, brt_reg_mask, br_bits);
	}

	return rval;
}

/* flash1/2 brightness control */
static int flashlight_flash_brt_ctrl(struct flashlight_flash *flash,
				 enum flashlight_led_id led_no, unsigned int brt)
{
	int rval = 0;
	u8 br_bits = 0;

	if (led_no < 0 || led_no >= FLASHLIGHT_LED_MAX) {
		pr_info("led_no %d error\n", led_no);
		return -1;
	}
	pr_info_ratelimited("%s %d brt:%u", __func__, led_no, brt);
	if (brt < flash_brt_min)
		return flashlight_enable_ctrl(flash, led_no, false);

	if (flash->need_cooler == 1 && brt > flash->target_current[led_no]) {
		brt = flash->target_current[led_no];
		pr_info("thermal limit current:%d\n", brt);
	}

	if(pFlashBrtuA2Reg == NULL)
	{
		pr_err("%s Error: pFlashBrtuA2Reg is NULL", __func__);
		return -1;
	}
	br_bits = pFlashBrtuA2Reg(brt);
	if (led_no == FLASHLIGHT_LED0)
	{
		rval = regmap_update_bits(flash->regmap, REG_LED0_FLASH_BR, brt_reg_mask, br_bits);
	}
	else
	{
		rval = regmap_update_bits(flash->regmap, REG_LED1_FLASH_BR, brt_reg_mask, br_bits);
	}

	return rval;
}

/* flash1/2 timeout control */
static int flashlight_flash_tout_ctrl(struct flashlight_flash *flash,
				unsigned int tout)
{
	int rval = 0;
	u8 tout_bits = 0;

	pr_info_ratelimited("%s tout:%u", __func__, tout);
	if (tout == 50)
		tout_bits = 0x04;
	else
		tout_bits = 0x07 + (tout / FLASHLIGHT_FLASH_TOUT_STEP);

	rval = regmap_update_bits(flash->regmap,
				  REG_FLASH_TOUT, 0x0f, tout_bits);

	return rval;
}

/* v4l2 controls  */
static int flashlight_get_ctrl(struct v4l2_ctrl *ctrl, enum flashlight_led_id led_no)
{
	struct flashlight_flash *flash = to_flashlight_flash(ctrl, led_no);
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

static int flashlight_set_ctrl(struct v4l2_ctrl *ctrl, enum flashlight_led_id led_no)
{
	struct flashlight_flash *flash = to_flashlight_flash(ctrl, led_no);
	int rval = 0;

	pr_info_ratelimited("%s led:%d ID:%d", __func__, led_no, ctrl->id);
	mutex_lock(&flash->lock);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
		flash->led_mode = ctrl->val;
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH)
			rval = flashlight_mode_ctrl(flash);
		else
			rval = 0;
		if (flash->led_mode == V4L2_FLASH_LED_MODE_NONE)
			rval = flashlight_enable_ctrl(flash, led_no, false);
		else if (flash->led_mode == V4L2_FLASH_LED_MODE_TORCH)
			rval = flashlight_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		flash->led_mode = V4L2_FLASH_LED_MODE_FLASH;
		rval = flashlight_mode_ctrl(flash);
		rval |= flashlight_enable_ctrl(flash, led_no, true);
		break;

	case V4L2_CID_FLASH_STROBE_STOP:
		if (flash->led_mode != V4L2_FLASH_LED_MODE_FLASH) {
			rval = -EBUSY;
			goto err_out;
		}
		rval = flashlight_enable_ctrl(flash, led_no, false);
		flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
		rval |= flashlight_mode_ctrl(flash);
		break;

	case V4L2_CID_FLASH_TIMEOUT:
		rval = flashlight_flash_tout_ctrl(flash, ctrl->val);
		break;

	case V4L2_CID_FLASH_INTENSITY:
		rval = flashlight_flash_brt_ctrl(flash, led_no, ctrl->val);
		break;

	case V4L2_CID_FLASH_TORCH_INTENSITY:
		rval = flashlight_torch_brt_ctrl(flash, led_no, ctrl->val);
		break;
	}

err_out:
	mutex_unlock(&flash->lock);
	return rval;
}

static int flashlight_led1_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return flashlight_get_ctrl(ctrl, FLASHLIGHT_LED1);
}

static int flashlight_led1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return flashlight_set_ctrl(ctrl, FLASHLIGHT_LED1);
}

static int flashlight_led0_get_ctrl(struct v4l2_ctrl *ctrl)
{
	return flashlight_get_ctrl(ctrl, FLASHLIGHT_LED0);
}

static int flashlight_led0_set_ctrl(struct v4l2_ctrl *ctrl)
{
	return flashlight_set_ctrl(ctrl, FLASHLIGHT_LED0);
}

static const struct v4l2_ctrl_ops flashlight_led_ctrl_ops[FLASHLIGHT_LED_MAX] = {
	[FLASHLIGHT_LED0] = {
			.g_volatile_ctrl = flashlight_led0_get_ctrl,
			.s_ctrl = flashlight_led0_set_ctrl,
			},
	[FLASHLIGHT_LED1] = {
			.g_volatile_ctrl = flashlight_led1_get_ctrl,
			.s_ctrl = flashlight_led1_set_ctrl,
			}
};

static int flashlight_init_controls(struct flashlight_flash *flash,
				enum flashlight_led_id led_no)
{
	struct v4l2_ctrl *fault;
	u32 max_flash_brt = flash->pdata->max_flash_brt[led_no];
	u32 max_torch_brt = flash->pdata->max_torch_brt[led_no];
	struct v4l2_ctrl_handler *hdl = &flash->ctrls_led[led_no];
	const struct v4l2_ctrl_ops *ops = &flashlight_led_ctrl_ops[led_no];

	v4l2_ctrl_handler_init(hdl, 8);
	if (hdl->error)
	{
		pr_err("v4l2_ctrl_handler_init error, ret %d", hdl->error);
		return hdl->error;
	}

	/* flash mode */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_LED_MODE,
			       V4L2_FLASH_LED_MODE_TORCH, ~0x7,
			       V4L2_FLASH_LED_MODE_NONE);
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
	if (hdl->error)
	{
		pr_err("v4l2_ctrl_new_std_menu(V4L2_CID_FLASH_LED_MODE) error, ret %d", hdl->error);
		return hdl->error;
	}

	/* flash source */
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_FLASH_STROBE_SOURCE,
			       0x1, ~0x3, V4L2_FLASH_STROBE_SOURCE_SOFTWARE);
	if (hdl->error)
	{
		pr_err("v4l2_ctrl_new_std_menu(V4L2_CID_FLASH_STROBE_SOURCE) error, ret %d", hdl->error);
		return hdl->error;
	}

	/* flash strobe */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE, 0, 0, 0, 0);
	if (hdl->error)
	{
		pr_err("v4l2_ctrl_new_std(V4L2_CID_FLASH_STROBE) error, ret %d", hdl->error);
		return hdl->error;
	}

	/* flash strobe stop */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_STROBE_STOP, 0, 0, 0, 0);
	if (hdl->error)
	{
		pr_err("v4l2_ctrl_new_std(V4L2_CID_FLASH_STROBE_STOP) error, ret %d", hdl->error);
		return hdl->error;
	}

	/* flash strobe timeout */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TIMEOUT,
			  FLASHLIGHT_FLASH_TOUT_MIN,
			  flash->pdata->max_flash_timeout,
			  FLASHLIGHT_FLASH_TOUT_STEP,
			  flash->pdata->max_flash_timeout);
	if (hdl->error)
	{
		pr_err("v4l2_ctrl_new_std(V4L2_CID_FLASH_TIMEOUT) error, ret %d", hdl->error);
		return hdl->error;
	}

	/* flash brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_INTENSITY,
			  flash_brt_min, max_flash_brt,
			  flash_brt_step, max_flash_brt);
	if (hdl->error)
	{
		pr_err("v4l2_ctrl_new_std(V4L2_CID_FLASH_INTENSITY) error, ret %d", hdl->error);
		return hdl->error;
	}

	/* torch brt */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_TORCH_INTENSITY,
			  torch_brt_min, max_torch_brt,
			  torch_brt_step, max_torch_brt);
	if (hdl->error)
	{
		pr_err("v4l2_ctrl_new_std(V4L2_CID_FLASH_TORCH_INTENSITY) error, ret %d", hdl->error);
		return hdl->error;
	}

	/* fault */
	fault = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FLASH_FAULT, 0,
				  V4L2_FLASH_FAULT_OVER_VOLTAGE
				  | V4L2_FLASH_FAULT_OVER_TEMPERATURE
				  | V4L2_FLASH_FAULT_SHORT_CIRCUIT
				  | V4L2_FLASH_FAULT_TIMEOUT, 0, 0);
	if (fault != NULL)
		fault->flags |= V4L2_CTRL_FLAG_VOLATILE;
	if (hdl->error)
	{
		pr_err("v4l2_ctrl_new_std(V4L2_CID_FLASH_FAULT) error, ret %d", hdl->error);
		return hdl->error;
	}

	if (led_no < 0 || led_no >= FLASHLIGHT_LED_MAX) {
		pr_info("led_no error\n");
		return -1;
	}

	flash->subdev_led[led_no].ctrl_handler = hdl;

	return 0;
}

/* initialize device */
static const struct v4l2_subdev_ops flashlight_ops = {
	.core = NULL,
};

static const struct regmap_config flashlight_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static void flashlight_v4l2_i2c_subdev_init(struct v4l2_subdev *sd,
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

static int flashlight_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	flashlight_set_driver(1);

	return 0;
}

static int flashlight_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	flashlight_set_driver(0);

	return 0;
}

static const struct v4l2_subdev_internal_ops flashlight_int_ops = {
	.open = flashlight_open,
	.close = flashlight_close,
};

static int flashlight_subdev_init(struct flashlight_flash *flash,
			      enum flashlight_led_id led_no, char *led_name)
{
	struct i2c_client *client = to_i2c_client(flash->dev);
	struct device_node *np = flash->dev->of_node, *child;
	const char *fled_name = "flash";
	int rval = 0;

	// pr_info("%s %d", __func__, led_no);
	if (led_no < 0 || led_no >= FLASHLIGHT_LED_MAX) {
		pr_err("led_no error\n");
		return -1;
	}

	flashlight_v4l2_i2c_subdev_init(&flash->subdev_led[led_no],
				client, &flashlight_ops);
	flash->subdev_led[led_no].flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	flash->subdev_led[led_no].internal_ops = &flashlight_int_ops;
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

	rval = flashlight_init_controls(flash, led_no);
	if (rval)
	{
		pr_err("flashlight_init_controls fail, ret %d", rval);
		goto err_out;
	}
	rval = media_entity_pads_init(&flash->subdev_led[led_no].entity, 0, NULL);
	if (rval < 0)
	{
		pr_err("media_entity_pads_init fail, ret %d", rval);
		goto err_out;
	}
	flash->subdev_led[led_no].entity.function = MEDIA_ENT_F_FLASH;

	rval = v4l2_async_register_subdev(&flash->subdev_led[led_no]);
	if (rval < 0)
	{
		pr_err("v4l2_async_register_subdev fail, ret %d", rval);
		goto err_out;
	}

	return rval;

err_out:
	v4l2_ctrl_handler_free(&flash->ctrls_led[led_no]);
	return rval;
}

/* flashlight init */
static int flashlight_init(struct flashlight_flash *flash)
{
	int rval = 0;
	unsigned int reg_val = 0;

	flashlight_mode = FLASHLIGHT_SCENARIO_PRE_FLASH;

#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT_DLPT)
	flashlight_kicker_pbm_by_device_id(
				&flash->flash_dev_id[FLASHLIGHT_LED0],
				flash->torch_power);
	mdelay(1);
#endif
	flashlight_pinctrl_set(flash, FLASHLIGHT_PINCTRL_PIN_HWEN, FLASHLIGHT_PINCTRL_PINSTATE_HIGH);

	/* set timeout */
	rval = flashlight_flash_tout_ctrl(flash, 400);
	if (rval < 0)
		return rval;

	/* output disable */
	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;
	rval = flashlight_mode_ctrl(flash);

	flashlight_torch_brt_ctrl(flash, FLASHLIGHT_LED0,
				flash->ori_current[FLASHLIGHT_LED0]);
	flashlight_torch_brt_ctrl(flash, FLASHLIGHT_LED1,
				flash->ori_current[FLASHLIGHT_LED1]);
	flashlight_flash_brt_ctrl(flash, FLASHLIGHT_LED0,
				flash->ori_current[FLASHLIGHT_LED0]);
	flashlight_flash_brt_ctrl(flash, FLASHLIGHT_LED1,
				flash->ori_current[FLASHLIGHT_LED1]);

	/* reset faults */
	rval = regmap_read(flash->regmap, REG_FLAG1, &reg_val);
	return rval;
}

/* flashlight uninit */
static int flashlight_uninit(struct flashlight_flash *flash)
{
	int rval = 0;

	rval = flashlight_pinctrl_set(flash,
				FLASHLIGHT_PINCTRL_PIN_HWEN, FLASHLIGHT_PINCTRL_PINSTATE_LOW);
#if IS_ENABLED(CONFIG_MTK_FLASHLIGHT_DLPT)
	flashlight_kicker_pbm_by_device_id(
				&flash->flash_dev_id[FLASHLIGHT_LED0],
				0);
#endif
	return rval;
}

static int flashlight_flash_open(void)
{
	return 0;
}

static int flashlight_flash_release(void)
{
	return 0;
}

static int flashlight_cooling_set_cur_state(int channel, unsigned long state)
{
	struct flashlight_flash *flash = flashlight_flash_data;
	int ret = 0;

	/* Request state should be less than max_state */
	if (state > FLASHLIGHT_COOLER_MAX_STATE)
		return -EINVAL;

	if (channel < 0 || channel >= FLASHLIGHT_LED_MAX) {
		pr_info("channel error\n");
		return -EINVAL;
	}

	pr_info("set thermal current:%lu\n", state);

	if (state == 0) {
		flash->need_cooler = 0;
		flash->target_current[channel] = flash_brt_max;
		ret = flashlight_torch_brt_ctrl(flash, channel,	torch_brt_max);
	} else {
		flash->need_cooler = 1;
		flash->target_current[channel] =
			flash_state_to_current_limit[state - 1];
		ret = flashlight_torch_brt_ctrl(flash, channel,
					flash->target_current[channel]);
	}

	return ret;
}

static int flashlight_ioctl(unsigned int cmd, unsigned long arg)
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
			if(channel == FLASHLIGHT_CHANNEL_CH1)
			{
				ret = flashlight_set_level_ch1(flashlight_level_ch1);
			}
			else if(channel == FLASHLIGHT_CHANNEL_CH2)
			{
				ret = flashlight_set_level_ch2(flashlight_level_ch2);
			}
			else
			{
				pr_err("%s Error: unkown channel %d", __func__, channel);
				return -1;
			}
			if (flashlight_mode == FLASHLIGHT_SCENARIO_MAIN_FLASH)
			{
				flashlight_flash_data->led_mode = V4L2_FLASH_LED_MODE_FLASH;
			} else {
				flashlight_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
			}
			ret |= flashlight_mode_ctrl(flashlight_flash_data);
			ret |= flashlight_enable_ctrl(flashlight_flash_data, channel, true);
			if (flashlight_mode == FLASHLIGHT_SCENARIO_MAIN_FLASH)
			{
				flashlight_mode = FLASHLIGHT_SCENARIO_PRE_FLASH;
			}
		} else {
			if (flashlight_flash_data->led_mode != V4L2_FLASH_LED_MODE_NONE) {
				flashlight_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
				ret = flashlight_mode_ctrl(flashlight_flash_data);
				ret |= flashlight_enable_ctrl(flashlight_flash_data, channel, false);
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
				&flashlight_flash_data->flash_dev_id[FLASHLIGHT_LED0],
				flashlight_flash_data->camera_power);
		} else {
			flashlight_kicker_pbm_by_device_id(
				&flashlight_flash_data->flash_dev_id[FLASHLIGHT_LED0],
				flashlight_flash_data->torch_power);
		}
#endif
		break;

	case FLASH_IOC_PRE_ON:
		pr_info_ratelimited("FLASH_IOC_PRE_ON(%d): %d\n",
				channel, (int)fl_arg->arg);
		flashlight_mode = FLASHLIGHT_SCENARIO_MAIN_FLASH;
		break;

	case FLASH_IOC_SET_THERMAL_CUR_STATE:
		pr_info_ratelimited("FLASH_IOC_SET_THERMAL_CUR_STATE(%d): %d\n",
				channel, (int)fl_arg->arg);
		ret = flashlight_cooling_set_cur_state(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info_ratelimited("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		ret = flashlight_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info_ratelimited("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = FLASHLIGHT_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info_ratelimited("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = FLASHLIGHT_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = flashlight_verify_level(fl_arg->arg);
		pr_info_ratelimited("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		if(current_table == NULL)
		{
			pr_err("%s Error: current_table is NULL", __func__);
			return -1;
		}
		fl_arg->arg = current_table[fl_arg->arg];
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return ret;
}

static int flashlight_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&flashlight_mutex);
	if (set) {
		if (!use_count)
			ret = flashlight_init(flashlight_flash_data);
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = flashlight_uninit(flashlight_flash_data);
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&flashlight_mutex);

	return ret;
}

static ssize_t flashlight_strobe_store(struct flashlight_arg arg)
{
	int ret = 0;

	ret = flashlight_set_driver(1);
	//flashlight_set_level(arg.channel, arg.level);
	//flashlight_timeout_ms[arg.channel] = 0;
	//flashlight_enable(arg.channel);
	ret |= flashlight_torch_brt_ctrl(flashlight_flash_data, arg.channel,
				arg.level * 25000);
	ret |= flashlight_enable_ctrl(flashlight_flash_data, arg.channel, true);
	flashlight_flash_data->led_mode = V4L2_FLASH_LED_MODE_TORCH;
	ret |= flashlight_mode_ctrl(flashlight_flash_data);
	msleep(arg.dur);
	//flashlight_disable(arg.channel);
	flashlight_flash_data->led_mode = V4L2_FLASH_LED_MODE_NONE;
	ret |= flashlight_mode_ctrl(flashlight_flash_data);
	ret |= flashlight_enable_ctrl(flashlight_flash_data, arg.channel, false);
	ret |= flashlight_set_driver(0);

	return ret;
}

static struct flashlight_operations flashlight_flash_ops = {
	flashlight_flash_open,
	flashlight_flash_release,
	flashlight_ioctl,
	flashlight_strobe_store,
	flashlight_set_driver
};

static int flashlight_parse_dt(struct flashlight_flash *flash)
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

	power = torch_brt_max / 1000 * vin / 1000 * 2;
	flash->torch_power = (unsigned int) power;
	power = flash_brt_max / 1000 * vin / 1000;
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
			&flashlight_flash_ops))
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

	if(hwen_state == FLASHLIGHT_PINCTRL_PINSTATE_LOW)
	{
		ret = flashlight_pinctrl_set(flashlight_flash_data, FLASHLIGHT_PINCTRL_PIN_HWEN,
				FLASHLIGHT_PINCTRL_PINSTATE_HIGH);
		if(ret < 0)
		{
			return scnprintf(buf, PAGE_SIZE, "flashlight_pinctrl_set fail\n");
		}
		msleep(10);
	}

	mutex_lock(&flashlight_mutex);
	ret = i2c_smbus_read_byte_data(client, RegAddr);
	mutex_unlock(&flashlight_mutex);
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
	if (0 == RegAddr && hwen_state == FLASHLIGHT_PINCTRL_PINSTATE_HIGH)
	{
		ret = flashlight_pinctrl_set(flashlight_flash_data, FLASHLIGHT_PINCTRL_PIN_HWEN,
				FLASHLIGHT_PINCTRL_PINSTATE_LOW);
		if(ret < 0)
		{
			pr_info("%s: flashlight_pinctrl_set fail: ret %d\n", __func__, ret);
			return ret;
		}
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
			if(hwen_state == FLASHLIGHT_PINCTRL_PINSTATE_LOW)
			{
				ret = flashlight_pinctrl_set(flashlight_flash_data, FLASHLIGHT_PINCTRL_PIN_HWEN,
						FLASHLIGHT_PINCTRL_PINSTATE_HIGH);
				if(ret < 0)
				{
					pr_info("%s: flashlight_pinctrl_set fail: ret %d\n", __func__, ret);
					return ret;
				}
				msleep(10);
			}

			mutex_lock(&flashlight_mutex);
			ret = i2c_smbus_write_byte_data(client, RegAddr, RegVal);
			mutex_unlock(&flashlight_mutex);
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

static int flashlight_probe(struct i2c_client *client)
{
	struct flashlight_flash *flash;
	struct flashlight_platform_data *pdata = dev_get_platdata(&client->dev);
	int rval = 0;
    unsigned int chip_id = 0;

	pr_info("%s:%d", __func__, __LINE__);

	flash = devm_kzalloc(&client->dev, sizeof(*flash), GFP_KERNEL);
	if (flash == NULL)
		return -ENOMEM;

	flash->regmap = devm_regmap_init_i2c(client, &flashlight_regmap);
	if (IS_ERR(flash->regmap)) {
		rval = PTR_ERR(flash->regmap);
		return rval;
	}
	
	/* if there is no platform data, use chip default value */
	if (pdata == NULL) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			return -ENODEV;
		pdata->max_flash_timeout = FLASHLIGHT_FLASH_TOUT_MAX;
	}
	flash->pdata = pdata;
	flash->dev = &client->dev;
	mutex_init(&flash->lock);
	flashlight_flash_data = flash;

	rval = flashlight_pinctrl_init(flash);
	if (rval < 0)
	{
		pr_err("flashlight_pinctrl_init fail");
		return rval;
	}

	rval = flashlight_parse_dt(flash);
	if (rval < 0)
	{
		pr_err("flashlight_parse_dt fail");
		return rval;
	}

	i2c_set_clientdata(client, flash);

    rval = flashlight_pinctrl_set(flash, FLASHLIGHT_PINCTRL_PIN_HWEN, 
                FLASHLIGHT_PINCTRL_PINSTATE_HIGH);
    if (rval < 0)
    {
        pr_err("%s Error: flashlight_pinctrl_set fail, ret %d", __func__, rval);
        goto err;
    }
	msleep(5);
    rval = regmap_read(flash->regmap, REG_CHIP_ID, &chip_id);
    if (rval < 0)
    {
        pr_err("%s Error: Read chip id fail, ret %d", __func__, rval);
        goto err;
    }
	pr_info("%s: Found chip ID 0x%02X", __func__, chip_id);

    if (FLASHLIGHT_CHIP_ID_OCP81378 == chip_id)
	{
		flash_brt_min = OCP81378_FLASH_BRT_MIN;
		flash_brt_step = OCP81378_FLASH_BRT_STEP;
		flash_brt_max = OCP81378_FLASH_BRT_MAX;
		torch_brt_min = OCP81378_TORCH_BRT_MIN;
		torch_brt_step = OCP81378_TORCH_BRT_STEP;
		torch_brt_max = OCP81378_TORCH_BRT_MAX;
		brt_reg_mask = OCP81378_BRT_REG_MASK;
		torch_mode_default_current = OCP81378_TORCH_MODE_DEFAULT_CURRENT;
		current_table = ocp81378_current;
		flash_level = ocp81378_flash_level;
		pFlashBrtuA2Reg = ocp81378_flash_brt_uA_2_reg;
		pTorchBrtuA2Reg = ocp81378_torch_brt_uA_2_reg;
	}
	else if (FLASHLIGHT_CHIP_ID_AW36515E == chip_id)
	{
		flash_brt_min = AW36515E_FLASH_BRT_MIN;
		flash_brt_step = AW36515E_FLASH_BRT_STEP;
		flash_brt_max = AW36515E_FLASH_BRT_MAX;
		torch_brt_min = AW36515E_TORCH_BRT_MIN;
		torch_brt_step = AW36515E_TORCH_BRT_STEP;
		torch_brt_max = AW36515E_TORCH_BRT_MAX;
		brt_reg_mask = AW36515E_BRT_REG_MASK;
		current_table = aw36515e_current;
		flash_level = aw36515e_flash_level;
		torch_mode_default_current = AW36515E_TORCH_MODE_DEFAULT_CURRENT;
		pFlashBrtuA2Reg = aw36515e_flash_brt_uA_2_reg;
		pTorchBrtuA2Reg = aw36515e_torch_brt_uA_2_reg;
	}
	else
	{
		pr_err("%s Error: Unknow chip ID 0x%02X", __func__, chip_id);
		goto err;
	}

	rval = flashlight_pinctrl_set(flash, FLASHLIGHT_PINCTRL_PIN_HWEN, 
                FLASHLIGHT_PINCTRL_PINSTATE_LOW);
    if (rval < 0)
    {
        pr_err("%s Error: flashlight_pinctrl_set fail, ret %d", __func__, rval);
        goto err;
    }
	msleep(5);

	flash->target_current[FLASHLIGHT_LED0] = flash_brt_max;
	flash->target_current[FLASHLIGHT_LED1] = flash_brt_max;
	flash->ori_current[FLASHLIGHT_LED0] = torch_brt_max;
	flash->ori_current[FLASHLIGHT_LED1] = torch_brt_max;
	flash->need_cooler = 0;
	/* led 1 */
	pdata->max_flash_brt[FLASHLIGHT_LED0] = flash_brt_max;
	pdata->max_torch_brt[FLASHLIGHT_LED0] = torch_brt_max;
	/* led 2 */
	pdata->max_flash_brt[FLASHLIGHT_LED1] = flash_brt_max;
	pdata->max_torch_brt[FLASHLIGHT_LED1] = torch_brt_max;

	rval = flashlight_subdev_init(flash, FLASHLIGHT_LED0, "flashlight-led0");
	if (rval < 0)
	{
		pr_err("flashlight_subdev_init fail");
		goto err_flashlight_subdev_init;
	}

#ifdef CONFIG_FLASHLIGHT_DEBUG
	if (device_create_file(&client->dev, &dev_attr_flashlight_reg)) {
		pr_info("Failed to create device file(flashlight_reg)\n");
		goto err_create_flashlight_reg_device_file;
	}
#endif

	return 0;

#ifdef CONFIG_FLASHLIGHT_DEBUG
err_create_flashlight_reg_device_file:
	device_remove_file(&client->dev, &dev_attr_flashlight_reg);
#endif
err_flashlight_subdev_init:
	for (int i = FLASHLIGHT_LED0; i < FLASHLIGHT_LED_MAX; i++) {
		v4l2_ctrl_handler_free(&flash->ctrls_led[i]);
		media_entity_cleanup(&flash->subdev_led[i].entity);
		v4l2_device_unregister_subdev(&flash->subdev_led[i]);
	}
err:
	i2c_set_clientdata(client, NULL);

	return -1;
}

static void flashlight_remove(struct i2c_client *client)
{
	struct flashlight_flash *flash = i2c_get_clientdata(client);
	unsigned int i;

	for (i = FLASHLIGHT_LED0; i < FLASHLIGHT_LED_MAX; i++) {
		v4l2_ctrl_handler_free(&flash->ctrls_led[i]);
		media_entity_cleanup(&flash->subdev_led[i].entity);
		v4l2_device_unregister_subdev(&flash->subdev_led[i]);
	}

#ifdef CONFIG_FLASHLIGHT_DEBUG
	device_remove_file(&client->dev, &dev_attr_flashlight_reg);
#endif
}

static const struct i2c_device_id flashlight_id_table[] = {
	{FLASHLIGHT_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, flashlight_id_table);

static const struct of_device_id flashlight_of_table[] = {
	{ .compatible = "mediatek,flashlight_t405a" },
	{ },
};
MODULE_DEVICE_TABLE(of, flashlight_of_table);

static struct i2c_driver flashlight_i2c_driver = {
	.driver = {
		   .name = FLASHLIGHT_NAME,
		   .of_match_table = flashlight_of_table,
		   },
	.probe = flashlight_probe,
	.remove = flashlight_remove,
	.id_table = flashlight_id_table,
};

module_i2c_driver(flashlight_i2c_driver);

MODULE_AUTHOR("Yangping Dong <yangping.dong@tinno.com>");
MODULE_DESCRIPTION("T405A LED flash driver");
MODULE_LICENSE("GPL");
