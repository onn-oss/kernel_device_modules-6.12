#define pr_fmt(fmt) "[cx2560x] %s: " fmt, __func__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/string.h>

#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/phy.h>

#include "cx2560x.h"
#include "cx2560x_reg.h"

#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
#include "charger_class.h"
#include "mtk_charger.h"
#include "mtk_musb.h"
#else
#include <mt-plat/upmu_common.h>
#include <mt-plat/charger_class.h>
#include <mt-plat/mtk_charger.h>
#include <mt-plat/charger_type.h>
#include "mediatek/charger/mtk_charger_intf.h"
#endif /* LINUX_VERSION_CODE */

extern void Charger_Detect_Init(void);
static int cx2560x_disable_otg(struct cx2560x_device *cx);
//extern void Charger_Detect_Release(void);

static int no_usb_flag = 0;
static int force_dpdm_count = 0;
static bool usb_detect_flag = false;
static bool usb_detect_done = false;
/* add i2c test */
static struct class  *chg_i2c_test_class = NULL;
static dev_t chg_i2c_test_devt;
static struct device *chg_i2c_test_device;
static char opr_flag = -1;
static int opr_ret;
static uint8_t reg;
static uint8_t read_data;
static uint8_t write_data;

bool charge_done_flag = false;
u8 count = 0;
u8 iindpm_flag_count = 0;
u8 vindpm_flag_count = 0;
u8 regulation_status_count = 0;

static void cx2560x_set_hvdcp_off(struct cx2560x_device *cx);

/*log level: 0-err, 1-info, 2-debug*/
static unsigned int cx2560x_log_level = 0;
module_param(cx2560x_log_level, uint, 0644);
MODULE_PARM_DESC(cx2560x_log_level, "Log level (0=error, 1=info, 2=debug)");

#define cx2560x_err(fmt, ...) \
	do { \
		if (cx2560x_log_level >= 0) \
			pr_err(fmt, ##__VA_ARGS__); \
	} while (0)

#define cx2560x_info(fmt, ...) \
	do { \
		if (cx2560x_log_level >= 1) \
			pr_info(fmt, ##__VA_ARGS__); \
	} while (0)

#define cx2560x_dbg(fmt, ...) \
	do { \
		if (cx2560x_log_level >= 2) \
			pr_debug(fmt, ##__VA_ARGS__); \
	} while (0)

enum cx2560x_part_no {
	CX25600 = 0x00,
	CX25601 = 0x02,
};

int cx2560x_vtemp = 250;
EXPORT_SYMBOL(cx2560x_vtemp);

static struct power_supply_desc cx2560x_charger_desc;
static int cx2560x_force_dpdm(struct cx2560x_device *cx);
static int cx2560x_charging_switch(
	struct charger_device *chg_dev, bool enable);
static int cx2560x_set_vindpm(struct cx2560x_device *cx, unsigned int vindpm);
static inline int cx2560x_set_en_timer(struct cx2560x_device *cx);
static inline int cx2560x_set_disable_timer(struct cx2560x_device *cx);

static char *cx2560x_charge_state[] = {
	"charge-disable",
	"pre-charge",
	"fast-charge",
	"charge-terminated",
};

#if !(LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static enum power_supply_usb_type cx2560x_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,

};
#endif

static enum power_supply_usb_type cx2560x_charge_type[] = {
	POWER_SUPPLY_CHARGE_TYPE_NONE,
	POWER_SUPPLY_CHARGE_TYPE_TRICKLE,
	POWER_SUPPLY_CHARGE_TYPE_FAST,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	POWER_SUPPLY_CHARGE_TYPE_TAPER_EXT,
#else
	POWER_SUPPLY_CHARGE_TYPE_TAPER,
#endif /* LINUX_VERSION_CODE */
};

static enum power_supply_property cx2560x_charger_props[] = {
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static enum power_supply_property cx2560x_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

static enum power_supply_property cx2560x_usb_props[] = {
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
};

#define CX2560X_REG_NUM		(0x10)
#define SINGLE_DUMP_LEN		19
#define TOTAL_DUMP_LEN		(SINGLE_DUMP_LEN * (CX2560X_REG_NUM))

static int cx2560x_dump_register(struct charger_device *chg_dev);
static int cx2560x_read_reg_without_retry(struct cx2560x_device *cx, u8 reg, u8 *data)
{
	int ret;

	ret = i2c_smbus_read_byte_data(cx->client, reg);
	if (ret < 0) {
		cx2560x_err("i2c read register 0x%02x failed: %d\n", reg, ret);
		return -EINVAL;
	} else {
		*data = ret;
	}

	return 0;
}

static int cx2560x_read_reg(struct cx2560x_device *cx, u8 reg, u8 *data)
{
	int ret = 0;
	int i, retry_count;
	u8 read_val, tmp_val[2];

	if ((reg == 0x08) || (reg == 0x40) || (reg == 0x0A))
		return cx2560x_read_reg_without_retry(cx, reg, data);

	if (reg == 0x09)
		return 0;

	for (retry_count = 0; retry_count < 3; retry_count++) {
		for (i = 0; i < 2; i++) {
			ret = cx2560x_read_reg_without_retry(cx, reg, &read_val);
			if (ret < 0)
				break;
			else
				tmp_val[i] = read_val;
		}
		if (ret >= 0 && tmp_val[0] == tmp_val[1]) {
			*data = tmp_val[0];
			return ret;
		}
	}

	if (ret < 0)
		cx2560x_err("i2c read register 0x%02x failed after %d retries\n", reg, retry_count);
	return 0;
}

static int cx2560x_write_reg_without_retry(struct cx2560x_device *cx, u8 reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(cx->client, reg, value);
	if (ret < 0) {
		cx2560x_err("i2c write register 0x%02x with value 0x%02x failed: %d\n", reg, value, ret);
		return -EINVAL;
	}

	return 0;
}

static int cx2560x_write_reg(struct cx2560x_device *cx, u8 reg, u8 value)
{
	int ret = 0;
	u8 read_val = 0;
	int retry_count;

	for (retry_count = 0; retry_count < 10; retry_count++) {
		ret = cx2560x_write_reg_without_retry(cx, reg, value);
		if (ret >= 0) {
			ret = cx2560x_read_reg_without_retry(cx, reg, &read_val);
			if (ret >= 0) {
				if (value == read_val)
					break;

				if (retry_count == 9) {
					cx2560x_err("i2c write register 0x%02x failed after %d retries\n", reg, retry_count + 1);
					return -1;
				}
			}
		}
	}
	return 0;
}

static int cx2560x_update_bits(struct cx2560x_device *cx, u8 reg, u8 mask, u8 val)
{
	u8  tmp;
	int ret;

	ret = cx2560x_read_reg(cx, reg, &tmp);
	if (ret) {
		cx2560x_err("read register 0x%02x failed before update\n", reg);
		return ret;
	}

	tmp &= ~mask;
	tmp |= val & mask;

	if ((reg == 0x01 && mask == 0x40) || (reg == 0x07 && mask == 0x80) ||
		(reg == 0x0B && mask == 0x80) || (reg == 0x0D && mask == 0x80) ||
		(reg == 0x07 && mask == 0x20))
		ret = cx2560x_write_reg_without_retry(cx, reg, tmp);
	else
		ret = cx2560x_write_reg(cx, reg, tmp);

	if (ret) {
		cx2560x_err("update register 0x%02x failed\n", reg);
		return ret;
	}

	return 0;
}

static int cx2560x_write_reg40(struct cx2560x_device *cx, bool enable)
{
	int ret, try_count = 10;
	u8 read_val;

	if (enable) {
		while (try_count--) {
			ret = cx2560x_write_reg(cx, 0x40, 0x00);
			ret = cx2560x_write_reg(cx, 0x40, 0x50);
			ret = cx2560x_write_reg(cx, 0x40, 0x57);
			ret = cx2560x_write_reg(cx, 0x40, 0x44);

			cx2560x_read_reg(cx, 0x40, &read_val);
			if (0x03 == read_val) {
				ret = 1;
				break;
			}
		}
	} else {
		ret = cx2560x_write_reg(cx, 0x40, 0x00);
	}

	return ret;
}

static inline int cx2560x_reset_register(struct cx2560x_device *cx)
{
	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_B,
		CX2560X_REG_RESET, CX2560X_REG_RESET);
}

static int cx2560x_disable_watchdog_timer(struct cx2560x_device *cx)
{
	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_5,
		CX2560X_WDT_TIMER_MASK, CX2560X_WDT_TIMER_DISABLE);
}

static void cx2560x_charge_enable_ctrl(struct cx2560x_device *cx, int en)
{
	cx->chg_en = !!en;
	if (en == 1) {
		if (charge_done_flag == true) {
			cancel_delayed_work_sync(&cx->charge_done_detect_work);
			cancel_delayed_work_sync(&cx->recharge_detect_work);
			schedule_delayed_work(&cx->recharge_detect_work, 0);
		}
		cancel_delayed_work_sync(&cx->charge_done_detect_work);
		cancel_delayed_work_sync(&cx->recharge_detect_work);
		cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_1,
					CX2560X_CHRG_EN, en ? CX2560X_CHRG_EN : 0);
		schedule_delayed_work(&cx->charge_done_detect_work, 0);
	} else {
		cancel_delayed_work_sync(&cx->charge_done_detect_work);
		cancel_delayed_work_sync(&cx->recharge_detect_work);
		charge_done_flag = false;
		count = 0;
		iindpm_flag_count = 0;
		vindpm_flag_count = 0;
		regulation_status_count = 0;
		cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_1,
					CX2560X_CHRG_EN, en ? CX2560X_CHRG_EN : 0);
	}
}

static inline int cx2560x_get_charge_enable_status(struct cx2560x_device *cx)
{
	u8 status;

	cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_1, &status);

	return !!(status & CX2560X_CHRG_EN);
}

static void cx2560x_set_usbin_current_limit(struct cx2560x_device *cx , unsigned int iindpm)
{
	u8 reg_val;

	cx2560x_dbg("iindpm=%d\n", iindpm);

	if (iindpm > 3200) {
		cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_0,
			CX2560X_IINDPM_I_MASK, CX2560X_IINDPM_I_MASK);
	} else if (iindpm == 500) {
		cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_0,
			CX2560X_IINDPM_I_MASK, 0x04);
	} else {
		reg_val = (iindpm - CX2560X_IINDPM_I_MIN_MA)
			/ CX2560X_IINDPM_STEP_MA;
		cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_0,
			CX2560X_IINDPM_I_MASK, reg_val);
	}
}

static int cx2560x_set_fast_charge_current(struct cx2560x_device *cx, unsigned int chrg_curr)
{
	u8 reg_val;

	if (chrg_curr < 0)
		chrg_curr = 0;
	else if (chrg_curr > 30475/10)
		chrg_curr = 30475/10;

	if (chrg_curr < 1170)
		reg_val = chrg_curr / 90;
	else
		reg_val = (chrg_curr - 805) / (575 / 10) + 14;

	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_2,
		CX2560X_ICHRG_CUR_MASK, reg_val);
}

static int cx2560x_set_taper_current(struct cx2560x_device *cx, int term_current)
{
	u8 reg_val;

	reg_val = (term_current - CX2560X_TERMCHRG_I_MIN_MA)
				/ CX2560X_TERMCHRG_CURRENT_STEP_MA;
	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_3,
		CX2560X_TERMCHRG_CUR_MASK, reg_val);
}

static int cx2560x_set_prechg_current(struct cx2560x_device *cx, int prechg_current)
{
	u8 reg_val;

	reg_val = (prechg_current - 52) / 52;
	reg_val = reg_val << 4;
	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_3,
		CX2560X_PRECHG_CUR_MASK, reg_val);
}

static int cx2560x_set_battery_voltage(struct cx2560x_device *cx, unsigned int chrg_volt)
{
	u8 reg_val;

	reg_val = (chrg_volt - CX2560X_VREG_V_MIN_MV)
				/ CX2560X_VREG_V_STEP_MV;
	reg_val = reg_val << 3;
	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_4,
		CX2560X_VREG_V_MASK, reg_val);
}

static int cx2560x_recharge_voltage(struct cx2560x_device *cx, unsigned int rechg_mv)
{
	u8 reg_val;

	if (rechg_mv == 100)
		reg_val = CX2560X_RECHARGE_VOLTAGE_100MV;
	else
		reg_val = CX2560X_RECHARGE_VOLTAGE_200MV;

	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_4,
		CX2560X_RECHARGE_VOLTAGE_MASK, reg_val);
}

/*static int cx2560x_vindpm_track_set(struct cx2560x_device *cx, unsigned int vindpm_mv)
{
	u8 reg_val;

	if (vindpm_mv == 200)
		reg_val = CX2560X_VINDPM_TRACK_SET_200MV;
	else if (vindpm_mv == 250)
		reg_val = CX2560X_VINDPM_TRACK_SET_250MV;
	else if (vindpm_mv == 300)
		reg_val = CX2560X_VINDPM_TRACK_SET_300MV;
	else
		reg_val = CX2560X_VINDPM_TRACK_SET_DISABLE;

	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_7,
		CX2560X_VINDPM_TRACK_SET_MASK, reg_val);
}*/

static int cx2560x_set_boost_current(struct cx2560x_device *cx, int boost_current)
{
	u8 reg_val;

	if (boost_current >= 1200)
		reg_val = CX2560X_BOOST_CUR_1A2;
	else
		reg_val = CX2560X_BOOST_CUR_05A;

	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_2,
		CX2560X_BOOST_CUR_MASK, reg_val << 7);
}

static int cx2560x_chipid_detect(struct cx2560x_device *cx)
{
	int ret = 0;
	u8 val = 0;

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_B, &val);
	if (ret < 0) {
		cx2560x_err("read CX2560X_CHRG_CTRL_B failed\n");
		cx->client->addr = 0x0b;
		ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_B, &val);
		if (ret < 0) {
			cx2560x_err("read SGM41543_CHRG_CTRL_B failed\n");
			return -EINVAL;
		}
	}

	val = val & CX2560X_PN_MASK;
	return val;
}

static int cx2560x_set_acovp(struct cx2560x_device *cx, int acovp)
{
	int ret = 0;

	if (12 == acovp) {
		ret = cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_6, 0xc0, 0x3 << 6);
		if (ret < 0) {
			cx2560x_err("write CX2560X_CHRG_CTRL_6 failed\n");
			return -EINVAL;
		}
	} else if (9 == acovp) {
		ret = cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_6, 0xc0, 0x2 << 6);
		if (ret < 0) {
			cx2560x_err("write CX2560X_CHRG_CTRL_6 failed\n");
			return -EINVAL;
		}
	} else if (5 == acovp) {
		ret = cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_6, 0xc0, 0x1 << 6);
		if (ret < 0) {
			cx2560x_err("write CX2560X_CHRG_CTRL_6 failed\n");
			return -EINVAL;
		}
	}

	return ret;
}

static int cx2560x_hardware_init(struct cx2560x_device *cx)
{
	int ret;

	ret = cx2560x_set_acovp(cx, 9);
	if (ret) {
		cx2560x_err("set cx2560x acovp failed\n");
		return -EPERM;
	}

	if (!of_property_read_u32(cx->dev->of_node, "cx,iterm", &cx->init_data.iterm))
		cx2560x_err("iterm=%d\n", cx->init_data.iterm);
	else
		cx->init_data.iterm = 180;
	ret = cx2560x_set_taper_current(cx, cx->init_data.iterm);
	if (ret) {
		cx2560x_err("set cx2560x taper current failed\n");
		return -EPERM;
	}

	if (!of_property_read_u32(cx->dev->of_node, "cx,iprechg", &cx->init_data.iprechg))
		cx2560x_err("iprechg=%d\n", cx->init_data.iprechg);
	else
		cx->init_data.iprechg = 300;
	ret = cx2560x_set_prechg_current(cx, cx->init_data.iprechg);
	if (ret) {
		cx2560x_err("set cx2560x precharge current failed\n");
		return -EPERM;
	}

	ret = cx2560x_set_battery_voltage(cx, 4380);
	if (ret) {
		cx2560x_err("set cx2560x charger CV voltage failed\n");
		return -EPERM;
	}

	if (!of_property_read_u32(cx->dev->of_node, "cx,vrechg-volt", &cx->init_data.vrechg_volt))
		cx2560x_err("vrechg_volt=%d\n", cx->init_data.vrechg_volt);
	else
		cx->init_data.vrechg_volt = 200;
	ret = cx2560x_recharge_voltage(cx, cx->init_data.vrechg_volt);
	if (ret) {
		cx2560x_err("set cx2560x recharge voltage failed\n");
		return -EPERM;
	}

	if (!of_property_read_u32(cx->dev->of_node, "cx,en-chg-saft-timer", &cx->init_data.en_saft_timer))
		cx2560x_err("en_saft_timer=%d\n", cx->init_data.en_saft_timer);
	else
		cx->init_data.en_saft_timer = 1;
	if (!cx->init_data.en_saft_timer)
		ret = cx2560x_set_disable_timer(cx);
	else
		ret = cx2560x_set_en_timer(cx);
	if (ret) {
		cx2560x_err("set cx2560x safety timer failed\n");
		return -EPERM;
	}

	ret = cx2560x_set_fast_charge_current(cx, 2200);
	if (ret) {
		cx2560x_err("set cx2560x fast charge current failed\n");
		return -EPERM;
	}

	ret = cx2560x_set_boost_current(cx, 1200);
	if (ret) {
		cx2560x_err("set cx2560x boost current failed\n");
		return -EPERM;
	}

	ret = cx2560x_disable_watchdog_timer(cx);
	if (ret) {
		cx2560x_err("disable cx2560x watchdog timer failed\n");
		return -EPERM;
	}

	cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_10, 0x01<<3, 0x00<<3);

	return 0;
}


static int cx2560x_get_charge_state(struct cx2560x_device *cx)
{
	int ret;
	u8 status;

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_8, &status);
	if (ret) {
		cx2560x_err("get register failed\n");
		return -EINVAL;
	}

	if (charge_done_flag == true)
		status = status | 0x03;

	return status;
}

static int cx2560x_get_status(struct cx2560x_device *cx, struct cx2560x_state *state)
{
	int ret;
	u8 status, fault = 0, ctrl;

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_8, &status);
	if (ret) {
		cx2560x_err("read cx2560x 0x08 register failed\n");
		return -EINVAL;
	}

	/* ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_9, &fault);
	if (ret) {
		cx2560x_err("read cx2560x 0x09 register failed\n");
		return -EINVAL;
	} */

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_A, &ctrl);
	if (ret) {
		cx2560x_err("read cx2560x 0x0A register failed\n");
		return -EINVAL;
	}

	state->pg_stat = (status & CX2560X_PG_GOOD_MASK) >> 2;
	state->chrg_stat = (status & CX2560X_CHG_STAT_MASK) >> 3;
	if (charge_done_flag == true)
		state->chrg_stat = 0x3;

	state->vbus_stat = (status & CX2560X_VBUS_STAT_MASK) >> 5;
	state->online = !!(status & CX2560X_PG_STAT);
	state->vbus_gd = !!(ctrl & CX2560X_VBUS_GOOD);
	if (state->vbus_gd) {
		cx2560x_dbg("vbus_gd = 1\n");
	} else {
		cx2560x_dbg("vbus_gd = 0\n");
	}

	cx2560x_dbg("[08]: %#x, [09]: %#x, [0A]: %#x\n", status, fault, ctrl);

	return 0;
}

static int cx2560x_get_initial_state(struct cx2560x_device *cx)
{
	int chg_state;

	chg_state = cx2560x_get_charge_state(cx);
	if (chg_state < 0) {
		cx2560x_err("get charger state failed\n");
		return -EINVAL;
	}

	if (chg_state & CX2560X_PG_GOOD_MASK) {
		cx->voltage_max = 5000000;
		cx->current_max = 1000000;
		cx->state.chrg_stat = CX2560X_FAST_CHARGE;
		cx->state.pg_stat = (chg_state & CX2560X_PG_GOOD_MASK) >> 2;
		cx->state.vbus_stat = (chg_state & CX2560X_VBUS_STAT_MASK) >> 5;
		//schedule_delayed_work(&cx->irq_work, msecs_to_jiffies(1000));
		schedule_delayed_work(&cx->charge_work, msecs_to_jiffies(30000));
		cx2560x_info("charger online during power up\n");
	}

	schedule_delayed_work(&cx->irq_work, msecs_to_jiffies(8000));
	return 0;
}

static int cx2560x_get_battery_temperature(struct cx2560x_device *cx)
{
	int ret;
	int charge_state = 0;
	union power_supply_propval psy_prop;

	ret = power_supply_get_property(cx->battery_psy,
			POWER_SUPPLY_PROP_TEMP, &psy_prop);
	if (ret) {
		cx2560x_err("get battery temp failed\n");
		return -ENODEV;
	}

	// set virtual temperature
	if (cx->vtemp != CX2560X_TEMP_INVALID) {
		psy_prop.intval = cx->vtemp;
		cx2560x_info("set vtemp: %d\n", cx->vtemp);
	}

	cx->temp = psy_prop.intval / 10;
	cx2560x_dbg("battery temp: %d\n", cx->temp);

	if (cx->temp <= -18 || cx->temp > 55)
		charge_state = CX2560X_CHG_DISABLE;
	else if (cx->temp > -17 && cx->temp < 0)
		charge_state = CX2560X_STATE_COLD;
	else if (cx->temp >= 0 && cx->temp < 15)
		charge_state = CX2560X_STATE_COOL;
	else if (cx->temp > 15 && cx->temp <= 45)
		charge_state = CX2560X_STATE_NORMAL;
	else if (cx->temp > 45 && cx->temp <= 55)
		charge_state = CX2560X_STATE_WARM;

	return charge_state;
}

static int cx2560x_get_ichg(struct cx2560x_device *cx, int *curr_ma)
{
	int ret;
	u8 reg_val;

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_2, &reg_val);
	if (ret) {
		dev_err(cx->dev, "read 0x02 failed(%d)\n", ret);
		return ret;
	}

	if ((reg_val & 0x3f) <= 0x0d)
		*curr_ma = reg_val * 90 * 1000;
	else
		*curr_ma = ((reg_val - 0x0d) * 575/10 + 805) * 1000;

	cx2560x_dbg("&&&& get cx2560x charge_current_value = %d\n", *curr_ma);

	return ret;
}

static int cx2560x_get_vindpm(struct cx2560x_device *cx, int *volt_mv)
{
	int ret;
	u8 reg_val;

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_6, &reg_val);
	if (ret) {
		dev_err(cx->dev, "read 0x11 failed(%d)\n", ret);
		return ret;
	}

	*volt_mv = (reg_val * 100 + 3900) * 1000;

	cx2560x_dbg("&&&& get cx2560x charge_input_vindpm = %d\n", *volt_mv);

	return ret;
}

static int cx2560x_get_term_curr(struct cx2560x_device *cx, int *curr_ma)
{
	int ret;
	u8 reg_val;

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_3, &reg_val);
	if (ret) {
		dev_err(cx->dev, "read 0x03 failed(%d)\n", ret);
		return ret;
	}

	*curr_ma = ((reg_val & 0x0f) * 60 + 60) * 1000;

	cx2560x_dbg("&&&& get cx2560x charge_vreg = %d\n", *curr_ma);

	return ret;
}

static int cx2560x_plug_in(struct charger_device *chg_dev)
{
	//struct cx2560x_device *cx = charger_get_data(chg_dev);
	int ret = 0;

	cx2560x_info("enter\n");

	return ret;
}

static int cx2560x_plug_out(struct charger_device *chg_dev)
{
	//struct cx2560x_device *cx = charger_get_data(chg_dev);
	int ret = 0;

	cx2560x_info("enter\n");

	return ret;
}

static int cx2560x_set_ichrg_curr(struct charger_device *chg_dev, unsigned int chrg_curr)
{
	u8 reg_val;
	int ret = 0;
	int batt_vol = 0;
	union power_supply_propval psy_prop;

	struct cx2560x_device *cx = charger_get_data(chg_dev);

	chrg_curr = chrg_curr / 1000;

	if (!cx->battery_psy) {
		cx->battery_psy = power_supply_get_by_name("battery");
		if (!cx->battery_psy) {
			cx2560x_err("get battery power supply failed\n");
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(cx->battery_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &psy_prop);
	batt_vol = psy_prop.intval / 1000;

	if (chrg_curr < 0)
		chrg_curr = 0;
	else if (chrg_curr > 30475/10)
		chrg_curr = 30475/10;

	if (chrg_curr < 1170)
		reg_val = chrg_curr / 90;
	else
		reg_val = (chrg_curr - 805) / (575 / 10) + 14;

	cx2560x_dbg("chrg_curr=%d, reg_val=%d\n", chrg_curr, reg_val);
	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_2,
					CX2560X_ICHRG_CUR_MASK, reg_val);
}

static int cx2560x_get_input_curr_lim(struct charger_device *chg_dev, unsigned int *ilim)
{
	int ret;
	u8 reg_val;
	struct cx2560x_device *cx = charger_get_data(chg_dev);

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_0, &reg_val);
	if (ret)
		return -EINVAL;
	if (CX2560X_IINDPM_I_MASK == (reg_val & CX2560X_IINDPM_I_MASK))
		*ilim = CX2560X_IINDPM_I_MAX_MA * 1000;
	else
		*ilim = ((reg_val & CX2560X_IINDPM_I_MASK) * CX2560X_IINDPM_STEP_MA + CX2560X_IINDPM_I_MIN_MA) * 1000;

	return 0;
}

static int cx2560x_set_input_curr_lim(struct charger_device *chg_dev, unsigned int iindpm)
{
	struct cx2560x_device *cx = charger_get_data(chg_dev);

	cx2560x_dbg("%d\n", iindpm);
	cx2560x_set_usbin_current_limit(cx, iindpm / 1000);
	return 0;
}

static int cx2560x_get_chrg_volt(struct charger_device *chg_dev, unsigned int *volt)
{
	int ret;
	u8 vreg_val;
	struct cx2560x_device *cx = charger_get_data(chg_dev);

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_4, &vreg_val);
	if (ret)
		return ret;

	vreg_val = (vreg_val & CX2560X_VREG_V_MASK) >> 3;
	*volt = (vreg_val * CX2560X_VREG_V_STEP_MV + CX2560X_VREG_V_MIN_MV) * 1000;

	return 0;
}

static int cx2560x_set_chrg_volt(struct charger_device *chg_dev, unsigned int chrg_volt)
{
	int ret;
	struct cx2560x_device *cx = charger_get_data(chg_dev);

	cx2560x_info("cx2560x_set_chrg_volt chrg_volt=%d\n", chrg_volt);

	ret = cx2560x_set_battery_voltage(cx, chrg_volt / 1000);

	return 0;
}

static inline int cx2560x_set_cvcharge_5v(struct cx2560x_device *cx)
{
	int ret;
	union power_supply_propval psy_prop;

	ret = power_supply_get_property(cx->battery_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &psy_prop);
	if (ret) {
		cx2560x_err("get battery charge current faile\n");
		return -ENODEV;
	}

	// dpdm should only be controlled in dcp
	// dpdm should be HIZ state in sdp/cdp
	if (cx->state.vbus_stat == CX2560X_USB_DCP) {
		cx2560x_dbg("enable dp dm for dcp 5v\n");
		//cx2560x_set_hvdcp_off(cx);
	}

	return 0;
}

static void cx2560x_charge_online_work(struct work_struct *work)
{
	int ret, err = 0;
	int new_temp;
	int chg_state;
	union power_supply_propval psy_prop;

	struct cx2560x_device *cx = container_of(
		work, struct cx2560x_device, charge_work.work);

	if (!cx->battery_psy) {
		cx->battery_psy = power_supply_get_by_name("battery");
		if (!cx->battery_psy) {
			err = -ENODEV;
			cx2560x_err("get battery power supply failed\n");
			goto out;
		}
	}

	/*get Vbat from gauge*/
	ret = power_supply_get_property(cx->battery_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &psy_prop);
	cx->batt_vol = psy_prop.intval / 1000;
	if (cx->batt_vol > 4300) {
		ret = cx2560x_set_cvcharge_5v(cx);
		if (ret) {
			err = -EPERM;
			cx2560x_err("set constant charge voltage to 5v failed\n");
			goto out;
		}
	}

	/*get Ibat from gauge*/
	ret = power_supply_get_property(cx->battery_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &psy_prop);
	cx->batt_curr = psy_prop.intval / 1000;

	cx2560x_dbg("Vbat = %d mV, Ibat = %d mA\n", cx->batt_vol, cx->batt_curr);
	new_temp = cx2560x_get_battery_temperature(cx);
	if (new_temp < 0) {
		err = -ENODEV;
		cx2560x_err("get charge state failed\n");
		goto out;
	}

	ret = cx2560x_get_charge_state(cx);
	if (ret < 0) {
		err = -EINVAL;
		cx2560x_err("get charge state failed\n");
		goto out;
	}

	chg_state = (ret & CX2560X_CHG_STAT_MASK) >> 3;
	if (charge_done_flag == true)
		chg_state = 0x3;
	if (cx->state.chrg_stat != chg_state) {
		cx->state.chrg_stat = chg_state;
		cx2560x_dbg("charge state: %s\n", cx2560x_charge_state[chg_state]);
		power_supply_changed(cx->charger_psy);
	}

out:
	if (err || !cx->state.pg_stat)
		cancel_delayed_work(&cx->charge_work);
	else
		schedule_delayed_work(&cx->charge_work, 15 * HZ);
}

static void cx2560x_psy_change_work(struct work_struct *work)
{
	struct cx2560x_device *cx = container_of(
		work, struct cx2560x_device, psy_work.work);

	power_supply_changed(cx->charger_psy);
}

#if IS_ENABLED(CONFIG_OEM_HVDCP_ALGO)
static int cx2560x_set_volt_to_reg(u32 volt)
{
	int reg_val = 0;

	if (volt == 0)
		reg_val = 0x1;
	else if (volt == 3300000)
		reg_val = 0x6;
	else if (volt == 600000)
		reg_val = 0x2;
	else
		reg_val = 0x0;

	return reg_val;
}

static int cx2560x_set_dp_volt(struct charger_device *chg_dev, u32 volt)
{
	struct cx2560x_device *cx = charger_get_data(chg_dev);
	int reg_val = 0;
	reg_val = cx2560x_set_volt_to_reg(volt);

	reg_val = reg_val << 4;
	cx2560x_dbg("set_dp=%duV reg_val=0x%x\n", volt, reg_val);
	return 	cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_10,
		CX2560X_DP_VSEL_MASK, reg_val);
}

static int cx2560x_set_dm_volt(struct charger_device *chg_dev, u32 volt)
{
	struct cx2560x_device *cx = charger_get_data(chg_dev);
	int reg_val = 0;
	reg_val = cx2560x_set_volt_to_reg(volt);

	cx2560x_dbg("set_dm = %duV reg_val=0x%x\n", volt, reg_val);
	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_10,
		CX2560X_DM_VSEL_MASK, reg_val);
}
#endif /* CONFIG_OEM_HVDCP_ALGO */

static void cx2560x_set_hvdcp_en(struct cx2560x_device *cx)
{
	//cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_10,0x01<<3,0x01<<3);
}
static void cx2560x_set_hvdcp_off(struct cx2560x_device *cx)
{
	//cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_10,0x01<<3,0x00<<3);
}
/*static void cx2560x_set_dpdm_hiz(struct cx2560x_device *cx)
{
	cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_10,
		CX2560X_DP_VSEL_MASK | CX2560X_DM_VSEL_MASK,
		CX2560X_DP_HIZ | CX2560X_DM_HIZ);
}*/

static int cx2560x_get_vbus(struct cx2560x_device *cx, int *vbus_volt)
{
	int ret = 0;
	int value = 0;

	ret = iio_read_channel_processed(cx->vbus, &value);
	if (ret < 0) {
		cx2560x_err("get vbus voltage failed\n");
		return ret;
	}

	//*vbus_volt = value + R_VBUS_CHARGER_1 * value / R_VBUS_CHARGER_2;
	*vbus_volt = value * 10;//adc read *10

	cx2560x_dbg("vbus voltage: %d\n", *vbus_volt);

	return ret;
}

static irqreturn_t cx2560x_irq_handler_thread(
	int irq, void *private)
{
	bool prev_vbus_gd;
	struct cx2560x_state state;
	struct cx2560x_device *cx = private;
	//int ret = 0;

	cx2560x_info("charger isr\n");
	prev_vbus_gd = cx->state.vbus_gd;
	cx2560x_get_status(cx, &state);

	cx->state = state;

	if (!prev_vbus_gd && cx->state.vbus_gd) {

		if (!cx->charger_wakelock->active)
			__pm_stay_awake(cx->charger_wakelock);

		cx2560x_dbg("adapter/usb inserted\n");
		charge_done_flag = false;
		count = 0;
		iindpm_flag_count = 0;
		vindpm_flag_count = 0;
		regulation_status_count = 0;

		Charger_Detect_Init();
		cx2560x_info("insert: chg_type = %d, psy_usb_type = %d,cx->state.vbus_stat=%d\n", cx->chg_type, cx->psy_usb_type, cx->state.vbus_stat);

		/*if (cx->usb2_phy->otg->gadget){
			//ret = usb_gadget_connect(cx->usb2_phy->otg->gadget);
			cx2560x_info("gadget->state: %d\n", cx->usb2_phy->otg->gadget->state);
		}*/

		force_dpdm_count = 3;
		no_usb_flag = 0;
		usb_detect_flag = false;
		usb_detect_done = false;
		cx->force_detect_count = 0;
		cancel_delayed_work_sync(&cx->charge_done_detect_work);
		cancel_delayed_work_sync(&cx->recharge_detect_work);
		schedule_delayed_work(&cx->charge_done_detect_work, 5 * HZ);
	} else if (prev_vbus_gd && !cx->state.vbus_gd) {
		cx2560x_info("adapter/usb removed\n");
		__pm_relax(cx->charger_wakelock);

		charge_done_flag = false;
		count = 0;
		iindpm_flag_count = 0;
		vindpm_flag_count = 0;
		regulation_status_count = 0;

		//Charger_Detect_Init();
		cx2560x_set_fast_charge_current(cx, 500);

		cx->state.pg_stat = 0;
		cx->state.vbus_stat = CX2560X_USB_NONE;
		cx->state.chrg_stat = 0;
		cx->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		cx->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		cx2560x_charger_desc.type =POWER_SUPPLY_TYPE_UNKNOWN;
		power_supply_changed(cx->charger_psy);
		power_supply_changed(cx->usb_psy);
		power_supply_changed(cx->ac_psy);
		cx2560x_info("no input detected\n");
		cancel_delayed_work_sync(&cx->charge_done_detect_work);
		cancel_delayed_work_sync(&cx->recharge_detect_work);
		//cancel_delayed_work_sync(&cx->charge_usb_detect_work);
		cancel_delayed_work_sync(&cx->force_detect_dwork);
		cancel_delayed_work_sync(&cx->charge_work);
		cancel_delayed_work_sync(&cx->force_detect_dwork);
		no_usb_flag = 0;
	}

	if (cx->state.pg_stat) {
		schedule_delayed_work(&cx->irq_work, 0);
		schedule_delayed_work(&cx->charge_work, msecs_to_jiffies(3000));
		cx2560x_info("schedule irq work\n");
	}

	power_supply_changed(cx->charger_psy);

	return IRQ_HANDLED;
}

static void cx2560x_charge_irq_work(struct work_struct *work)
{
	struct cx2560x_state state;
	struct cx2560x_device *cx = container_of(
		work, struct cx2560x_device, irq_work.work);

	cx2560x_info("irq work\n");
	/*if (!cx->charger_wakelock->active)
		__pm_stay_awake(cx->charger_wakelock);*/

	cx2560x_get_status(cx, &state);
	cx->state.vbus_stat = state.vbus_stat;

	if (state.vbus_stat == CX2560X_USB_NONE) {
		cx->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		cx->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		cx2560x_charger_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		Charger_Detect_Init();
		cx2560x_set_hvdcp_off(cx);
		cx->ibat = 3000;
		cx->voltage_max = 0;
		cx->current_max = 0;
		cx->state.vbus_stat = 0;
		cx->state.chrg_stat = 0;
		cx->temp_state = CX2560X_STATE_NORMAL;
		cancel_delayed_work(&cx->charge_work);
		cancel_delayed_work(&cx->vindpm_work);
		cx2560x_info("no input detected\n");
	} else if (state.vbus_stat == CX2560X_USB_DCP) {
		Charger_Detect_Init();
		cx->voltage_max = 5000000;
		cx->current_max = 2000000;
		cx->chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		cx->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		cx2560x_charger_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		cx2560x_set_usbin_current_limit(cx, 1400);
		cx2560x_info("dcp detected1\n");
	} else if (state.vbus_stat == CX2560X_USB_SDP) {
		//Charger_Detect_Release();
		//if (!usb_detect_flag)
		//	schedule_delayed_work(&cx->charge_usb_detect_work, 5 * HZ);
		if (no_usb_flag == 0) {
			cx2560x_info("CX2589x charger type: SDP\n");
			cx->chg_type = POWER_SUPPLY_TYPE_USB;
			cx->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;

			cx2560x_set_usbin_current_limit(cx, 500);
			cx2560x_info("sdp detected\n");
		}else{
			cx2560x_info("CX2589x charger type: UNKNOWN/FLOAT\n");
			cx->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
			cx->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			cx2560x_set_usbin_current_limit(cx, 1000);
			cx2560x_info("unknown charge detected\n");
		}
		cx->voltage_max = 5000000;
		cx->current_max = 500000;
		cx2560x_charger_desc.type = POWER_SUPPLY_TYPE_USB;
	} else if (cx->state.vbus_stat == CX2560X_USB_CDP) {
		//Charger_Detect_Release();
		cx->voltage_max = 5000000;
		cx->current_max = 1500000;
		cx->chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		cx->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		cx2560x_charger_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		cx2560x_set_usbin_current_limit(cx, 1500);
		cx2560x_info("cdp detected\n");
	} else if (cx->state.vbus_stat == CX2560X_USB_UNKNOWN) {
		//Charger_Detect_Init();
		cx->voltage_max = 5000000;
		cx->current_max = 500000;
		cx->chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		cx->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		cx2560x_charger_desc.type = POWER_SUPPLY_TYPE_USB;
		cx2560x_set_usbin_current_limit(cx, 1000);
		cx2560x_info("unknown charge detected\n");
	} else if (cx->state.vbus_stat == CX2560X_USB_NSTANDA) {
		 //Charger_Detect_Init();
		cx->voltage_max = 5000000;
		cx->current_max = 2000000;
		cx->chg_type = POWER_SUPPLY_TYPE_USB_DCP;
		cx->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		cx2560x_charger_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		cx2560x_set_usbin_current_limit(cx, 2300);
		cx2560x_info("not standard detected\n");
	}

	//cx2560x_get_charger_type(cx);
	cx2560x_info("Update: chg_type=%d, psy_usb_type=%d, cx->state.vbus_stat=%d\n", cx->chg_type, cx->psy_usb_type, cx->state.vbus_stat);
	power_supply_changed(cx->charger_psy);
	/*if (true == usb_detect_done || state.vbus_stat != CX2560X_USB_SDP ) {
		__pm_relax(cx->charger_wakelock);
	}*/
}

/*
static void charger_usb_detect_work_func(struct work_struct *work)
{
	struct delayed_work *charge_usb_detect_work = NULL;
	struct cx2560x_device *cx = NULL;
	struct cx2560x_state state;

	//int ret;

	charge_usb_detect_work = container_of(work, struct delayed_work, work);
	if (charge_usb_detect_work == NULL) {
		cx2560x_err("Cann't get charge_usb_detect_work\n");
		return;
	}

	cx = container_of(charge_usb_detect_work, struct cx2560x_device, charge_usb_detect_work);
	if (cx == NULL) {
		cx2560x_err("Cann't get cx2560x_device\n");
		return;
	}
	usb_detect_flag = true;
	cx2560x_dbg("enter\n");

	if (cx->usb2_phy->otg->gadget) {
		//ret = usb_gadget_connect(cx->usb2_phy->otg->gadget);
		cx2560x_info("gadget->state: %d\n", cx->usb2_phy->otg->gadget->state);
		if (cx->usb2_phy->otg->gadget->state == 0) {
			do {
				cx2560x_info("SDP retry:%d\n", force_dpdm_count);
				cx2560x_force_dpdm(cx);
				msleep(1000);
				cx2560x_get_status(cx, &state);
				if (state.vbus_stat != CX2560X_USB_SDP)
					break;
				msleep(2000);
			} while (force_dpdm_count-- > 0);
			if (state.vbus_stat == CX2560X_USB_SDP)
				no_usb_flag = 1;
		} else {
			no_usb_flag = 0;
		}
		cx2560x_dbg("exit\n");
		schedule_delayed_work(&cx->irq_work, 0);
	}
	usb_detect_done = true;
	return;
}
*/

static int cx2560x_get_chrg_volt(struct charger_device *chg_dev, unsigned int *volt);
static int cx2560x_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	int value = 0;
	int vbus = 0;
	u32 chg_cv = 0;
	struct cx2560x_device *cx = power_supply_get_drvdata(psy);
	struct cx2560x_state state;

	cx2560x_get_status(cx, &state);
	cx->state = state;
	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "CX25601D";
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = cx2560x_charger_desc.type;
		cx2560x_dbg("charger psy prop type: %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (cx->state.pg_stat)
			val->intval = 1;
		else
			val->intval = 0;
		cx2560x_dbg("charger online: %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = cx2560x_charge_type[cx->state.chrg_stat];
		cx2560x_dbg("charger psy charge type: %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		value = cx->state.chrg_stat;
		if (value == CX2560X_PRECHARGE || value == CX2560X_FAST_CHARGE)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (value == CX2560X_USB_NONE || value == CX2560X_USB_OTG)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (value == CX2560X_CHG_DISABLED)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if (value == CX2560X_TERM_CHARGE)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		cx2560x_dbg("charger psy prop status: %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		if (!cx->state.vbus_gd) {
			cx2560x_dbg("vbus not good\n");
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		} else {
			cx2560x_dbg("vbus is good\n");
			if (cx->state.vbus_stat == 0)
				val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			if (cx->state.vbus_stat == 1)
				val->intval = POWER_SUPPLY_USB_TYPE_SDP;
			if (cx->state.vbus_stat == 2)
				val->intval = POWER_SUPPLY_USB_TYPE_CDP;
			if (cx->state.vbus_stat == 3)
				val->intval = POWER_SUPPLY_USB_TYPE_DCP;
			if (cx->state.vbus_stat == 5)
				val->intval = POWER_SUPPLY_USB_TYPE_SDP;
			if (cx->state.vbus_stat == 6)
				val->intval = POWER_SUPPLY_USB_TYPE_DCP;
		}
		cx2560x_info("charger usb type=%d, cx->state.vbus_stat=%d\n", val->intval, cx->state.vbus_stat);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		cx2560x_get_vbus(cx, &vbus);
		val->intval = vbus;
		cx2560x_dbg("charger voltage: %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = cx2560x_get_ichg(cx, &value);
		if (ret)
			break;
		val->intval = value;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = cx2560x_get_input_curr_lim(cx->chg_dev, &value);
		if (ret < 0)
			break;
		val->intval = value;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		cx2560x_get_chrg_volt(cx->chg_dev, &chg_cv);
		val->intval = chg_cv;
		cx2560x_dbg("charger constant voltage:%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = cx2560x_get_vindpm(cx, &value);
		if (ret < 0)
			break;
		val->intval = value;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = cx2560x_get_term_curr(cx, &value);
		if (ret < 0)
			break;
		val->intval = value;
		break;
	default:
		cx2560x_dbg("get charger prop: %u is not supported\n", psp);
		return -EINVAL;
	}

	return 0;
}

static int cx2560x_charger_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	int ret = 0;
	struct cx2560x_device *cx = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		cx2560x_charge_enable_ctrl(cx, !!val->intval);
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = cx2560x_set_ichrg_curr(cx->chg_dev, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = cx2560x_set_chrg_volt(cx->chg_dev, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = cx2560x_set_input_curr_lim(cx->chg_dev, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = cx2560x_set_vindpm(cx, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = cx2560x_set_taper_current(cx, val->intval / 1000);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0))
static struct power_supply_desc cx2560x_charger_desc = {
	.name = "primary_chg",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		BIT(POWER_SUPPLY_USB_TYPE_DCP),
	.properties = cx2560x_charger_props,
	.num_properties = ARRAY_SIZE(cx2560x_charger_props),
	.get_property = cx2560x_charger_get_property,
	.set_property = cx2560x_charger_set_property,
};
#else
static struct power_supply_desc cx2560x_charger_desc = {
	.name = "cx2560x",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = cx2560x_usb_type,
	.num_usb_types = ARRAY_SIZE(cx2560x_usb_type),
	.properties = cx2560x_charger_props,
	.num_properties = ARRAY_SIZE(cx2560x_charger_props),
	.get_property = cx2560x_charger_get_property,
	.set_property = cx2560x_charger_set_property,
};
#endif

static char *cx2560x_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger",
};

static struct power_supply *cx2560x_register_charger_psy(struct cx2560x_device *cx)
{
	struct power_supply_config psy_cfg = {
		.drv_data = cx,
		.of_node = cx->dev->of_node,
	};

	psy_cfg.supplied_to = cx2560x_charger_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(cx2560x_charger_supplied_to);
	return power_supply_register(cx->dev, &cx2560x_charger_desc, &psy_cfg);
}

static int cx2560x_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct cx2560x_device *cx = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (cx->state.vbus_stat == CX2560X_USB_DCP ||
			cx->state.vbus_stat == CX2560X_USB_UNKNOWN ||
			cx->state.vbus_stat == CX2560X_USB_NSTANDA)
			val->intval = 1;
		else
			val->intval = 0;
		cx2560x_dbg("ac online: %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = cx->current_max;
		cx2560x_dbg("ac current maxium: %d\n", cx->current_max);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = cx->voltage_max;
		cx2560x_dbg("ac voltage maxium: %d\n", cx->voltage_max);
		break;
	default:
		cx2560x_dbg("get ac prop: %u is not supported\n", psp);
		return -EINVAL;
	}

	return 0;
}

static struct power_supply_desc cx2560x_ac_desc = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = cx2560x_ac_props,
	.num_properties = ARRAY_SIZE(cx2560x_ac_props),
	.get_property = cx2560x_ac_get_property,
};

static struct power_supply *cx2560x_register_ac_psy(struct cx2560x_device *cx)
{
	struct power_supply_config psy_cfg = {
		.drv_data = cx,
		.of_node = cx->dev->of_node,
	};

	return power_supply_register(cx->dev, &cx2560x_ac_desc, &psy_cfg);
}

static int cx2560x_usb_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct cx2560x_device *cx = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_USB;
		cx2560x_dbg("usb psy prop type: %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (cx->state.vbus_stat == CX2560X_USB_SDP ||
			cx->state.vbus_stat == CX2560X_USB_CDP)
			val->intval = 1;
		else
			val->intval = 0;
		cx2560x_dbg("usb online: %d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = cx->current_max;
		cx2560x_dbg("usb current maxium: %d\n", cx->current_max);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = cx->voltage_max;
		cx2560x_dbg("usb voltage maxium: %d\n", cx->voltage_max);
		break;
	default:
		cx2560x_dbg("get usb prop: %u is not supported\n", psp);
		return -EINVAL;
	}

	return 0;
}

static struct power_supply_desc cx2560x_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = cx2560x_usb_props,
	.num_properties = ARRAY_SIZE(cx2560x_usb_props),
	.get_property = cx2560x_usb_get_property,
};

static struct power_supply *cx2560x_register_usb_psy(struct cx2560x_device *cx)
{
	struct power_supply_config psy_cfg = {
		.drv_data = cx,
		.of_node = cx->dev->of_node,
	};

	return power_supply_register(cx->dev, &cx2560x_usb_desc, &psy_cfg);
}

#if 0
static int cx2560x_enable_vbus(struct regulator_dev *rdev)
{
	struct cx2560x_device *cx = rdev_get_drvdata(rdev);

	cx->otg_mode = 1;
	cx2560x_info("enable otg\n");
	return cx2560x_update_bits(cx,
			CX2560X_CHRG_CTRL_1, CX2560X_OTG_EN, CX2560X_OTG_EN);
}

static int cx2560x_disable_vbus(struct regulator_dev *rdev)
{
	struct cx2560x_device *cx = rdev_get_drvdata(rdev);

	cx->otg_mode = 0;
	cx2560x_info("dibable otg\n");
	return cx2560x_update_bits(cx,
		CX2560X_CHRG_CTRL_1, CX2560X_OTG_EN, 0);
}

static int cx2560x_vbus_state(struct regulator_dev *rdev)
{
	u8 otg_en;
	struct cx2560x_device *cx = rdev_get_drvdata(rdev);

	cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_1, &otg_en);
	otg_en = !!(otg_en & CX2560X_OTG_EN);
	cx2560x_info("otg state: %d\n", otg_en);

	return otg_en;
}

static struct regulator_ops cx2560x_vbus_ops = {
	.enable = cx2560x_enable_vbus,
	.disable = cx2560x_disable_vbus,
	.is_enabled = cx2560x_vbus_state,
};

static const struct regulator_desc cx2560x_otg_rdesc = {
	.name = "cx2560x-vbus",
	.of_match = "cx2560x-otg-vbus",
	.ops = &cx2560x_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
};

static struct regulator_dev *cx2560x_register_regulator(struct cx2560x_device *cx)
{
	struct regulator_config config = {
		.dev = cx->dev,
		.driver_data = cx,
	};

	return regulator_register(&cx2560x_otg_rdesc, &config);
}
#endif

static int cx2560x_gpio_config(struct cx2560x_device *cx)
{
	int ret, irqn;

	cx->en_gpio = of_get_named_gpio(cx->dev->of_node, "cx,chg-en-gpio", 0);
	if (!gpio_is_valid(cx->en_gpio)) {
		cx2560x_err("get cx,chg-en-gpio failed\n");
		return -EINVAL;
	}

	ret = gpio_request_one(cx->en_gpio, GPIOF_OUT_INIT_LOW, "cx_chg_en_pin");
	if (ret) {
		cx2560x_err("request cx,chg-en-gpio failed\n");
		return -EPERM;
	}

	cx->irq_gpio = of_get_named_gpio(cx->dev->of_node, "cx,intr_gpio", 0);
	if (!gpio_is_valid(cx->irq_gpio)) {
		cx2560x_err("get cx,irq-gpio failed\n");
		goto err_out;
	}
	irqn = gpio_to_irq(cx->irq_gpio);
	if (irqn > 0)
		cx->client->irq = irqn;
	else {
		cx2560x_err("map cx,irq-gpio to irq failed\n");
		goto err_out;
	}

	return 0;

err_out:
	gpio_free(cx->en_gpio);
	return -EINVAL;
}

static ssize_t cx2560x_reg_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	int i, ret;
	char *reg_val[2];
	ulong addr, val;
	struct cx2560x_device *cx = dev_get_drvdata(dev);

	if (strstr(buf, ":")) {
		for (i = 0; i < ARRAY_SIZE(reg_val); i++)
			reg_val[i] = strsep((char **)&buf, ":");

		ret = kstrtoul(reg_val[0], 16, &addr);
		if (ret < 0) {
			cx2560x_err("get reg addr failed\n");
			return -EINVAL;
		}

		ret = kstrtoul(reg_val[1], 16, &val);
		if (ret < 0) {
			cx2560x_err("get reg val failed\n");
			return -EINVAL;
		}
		ret = i2c_smbus_write_byte_data(cx->client, addr, val);
		if (ret < 0) {
			cx2560x_err("write reg[%x]: %x failed\n", (uint32_t)addr, (uint32_t)val);
			return -EIO;
		}
	} else {
		ret = kstrtoul(buf, 16, &val);
		if (ret) {
			cx2560x_err("kstrtoul [%s] failed\n", buf);
			return -EINVAL;
		}
		cx->reg_addr = val;
	}

	return size;
}

static struct device_attribute dev_attr_reg = __ATTR(reg, S_IWUSR, NULL, cx2560x_reg_store);

static ssize_t cx2560x_reg_dump(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, ret;
	ssize_t len = 0;
	struct cx2560x_device *cx = dev_get_drvdata(dev);
	u8 reg_index[16] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	};

	for (i = 0; i < ARRAY_SIZE(reg_index); i++) {
		ret = i2c_smbus_read_byte_data(cx->client, reg_index[i]);
		if (ret < 0) {
			cx2560x_err("read reg [%#x] failed\n", cx->reg_addr);
			return -EINVAL;
		}
		len += snprintf(buf + len, PAGE_SIZE, "%#04x: %#04x\n", reg_index[i], ret);
	}

	return len;
}

static struct device_attribute dev_attr_dump = __ATTR(dump, S_IRUSR, cx2560x_reg_dump, NULL);

static ssize_t cx2560x_charge_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct cx2560x_device *cx = dev_get_drvdata(dev);

	if (!strncmp(buf, "1", 1)) {
		cx->qc_force = QC_TYPE_FORCE_1P0;
		cx2560x_dbg("enable qc1.0\n");
	} else if (!strncmp(buf, "2", 1)) {
		cx->qc_force = QC_TYPE_FORCE_2P0;
		cx2560x_dbg("enable qc2.0\n");
	} else if (!strncmp(buf, "3", 1)) {
		cx->qc_force = QC_TYPE_FORCE_3P0;
		cx2560x_dbg("enable qc3.0\n");
	} else {
		cx2560x_err("invalid parameter: %s\n", buf);
		return -EINVAL;
	}

	return size;
}

static struct device_attribute dev_attr_charge = __ATTR(charge, S_IWUSR, NULL, cx2560x_charge_store);

static ssize_t cx2560x_dpdm_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct cx2560x_device *cx = dev_get_drvdata(dev);

	if (!strncmp(buf, "dp", 2)) {
		cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_10,
			CX2560X_DP_VSEL_MASK, CX2560X_DP_3V3);
		usleep_range(100, 100);
		cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_10,
			CX2560X_DP_VSEL_MASK, CX2560X_DP_0V6);
	} else if (!strncmp(buf, "dm", 2)) {
		cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_10,
			CX2560X_DM_VSEL_MASK, CX2560X_DM_0V6);
		usleep_range(1000, 1000);
		cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_10,
			CX2560X_DM_VSEL_MASK, CX2560X_DM_3V3);
	} else {
		cx2560x_err("error params\n");
		return -EINVAL;
	}

	return size;
}

static struct device_attribute dev_attr_dpdm = __ATTR(dpdm, S_IWUSR, NULL, cx2560x_dpdm_store);

static ssize_t cx2560x_qc3v0_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct cx2560x_device *cx = dev_get_drvdata(dev);

	sscanf(buf, "%d %d\n", &cx->chg_volt, &cx->chg_curr);
	cx2560x_info("qc3.0 voltage: %d, current: %d\n", cx->chg_volt, cx->chg_curr);

	return size;
}

static struct device_attribute dev_attr_qc3v0 = __ATTR(qc3v0, S_IWUSR, NULL, cx2560x_qc3v0_store);

static ssize_t cx2560x_vtemp_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t size)
{
	struct cx2560x_device *cx = dev_get_drvdata(dev);

	sscanf(buf, "%d\n", &cx->vtemp);
	cx2560x_vtemp = cx->vtemp;
	schedule_delayed_work(&cx->psy_work, msecs_to_jiffies(100));
	cx2560x_info("virtual temperature %d\n", cx->vtemp);

	return size;
}

static struct device_attribute dev_attr_vtemp = __ATTR(vtemp, S_IWUSR, NULL, cx2560x_vtemp_store);

static ssize_t cx2560x_log_level_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", cx2560x_log_level);
}

static ssize_t cx2560x_log_level_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	unsigned int level;

	if (kstrtouint(buf, 10, &level) == 0 && level <= 2) {
		cx2560x_log_level = level;
		cx2560x_info("set log level to %u\n", level);
		return size;
	}

	cx2560x_err("invalid log level: %s (0-2 allowed)\n", buf);
	return -EINVAL;
}

static struct device_attribute dev_attr_log_level = __ATTR(log_level, 0644, cx2560x_log_level_show, cx2560x_log_level_store);

static int cx2560x_charging_switch(struct charger_device *chg_dev, bool enable)
{
	struct cx2560x_device *cx = charger_get_data(chg_dev);

	if (enable) {
		cx2560x_charge_enable_ctrl(cx, 1);
	} else {
		cx2560x_charge_enable_ctrl(cx, 0);
	}
	return 0;
}

static int cx2560x_is_enabled(struct charger_device *chg_dev, bool *enabled)
{
	struct cx2560x_device *cx = charger_get_data(chg_dev);

	*enabled = cx2560x_get_charge_enable_status(cx);
	cx2560x_info("cx2560x charger is %s\n", *enabled ? "charging" : "not charging");

	cx2560x_dump_register(cx->chg_dev);

	return 0;
}

static int cx2560x_set_wdt_rst(struct cx2560x_device *cx, bool is_rst)
{
	u8 val;

	if (is_rst)
		val = CX2560X_WDT_RST_MASK;
	else
		val = 0;

	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_1,
			CX2560X_WDT_RST_MASK, val);
}

static int cx2560x_reset_watch_dog_timer(struct charger_device *chg_dev)
{
	struct cx2560x_device *cx = charger_get_data(chg_dev);

	return cx2560x_set_wdt_rst(cx, 0x1);
}

static int cx2560x_set_input_volt_lim(struct charger_device *chg_dev, unsigned int vindpm)
{
	return 0;
}

static int cx2560x_get_charging_status(struct charger_device *chg_dev, bool *is_done)
{
	struct cx2560x_device *cx = charger_get_data(chg_dev);

	if (cx->state.chrg_stat == CX2560X_TERM_CHARGE)
		*is_done = true;
	else
		*is_done = false;

	return 0;
}

static inline int cx2560x_set_en_timer(struct cx2560x_device *cx)
{
	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_5,
				CX2560X_SAFETY_TIMER_EN, CX2560X_SAFETY_TIMER_EN);
}

static inline int cx2560x_set_disable_timer(struct cx2560x_device *cx)
{
	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_5,
				CX2560X_SAFETY_TIMER_EN, 0);
}

static int cx2560x_enable_safetytimer(struct charger_device *chg_dev, bool en)
{
	return 0;
}

static int cx2560x_get_is_safetytimer_enable(struct charger_device *chg_dev, bool *en)
{
	int ret = 0;
	u8 val = 0;

	struct cx2560x_device *cx = charger_get_data(chg_dev);

	ret = cx2560x_read_reg(cx,CX2560X_CHRG_CTRL_5,&val);
	if (ret < 0) {
		cx2560x_err("read CX2560X_CHRG_CTRL_5 fail\n");
		return ret;
	} else
		*en = !!(val & CX2560X_SAFETY_TIMER_EN);

	return 0;
}

static int cx2560x_enable_otg(struct cx2560x_device *cx)
{
	int ret;
	u8 tmp;

	ret = cx2560x_read_reg_without_retry(cx, CX2560X_CHRG_CTRL_9, &tmp);
	cx2560x_dbg("cx2560x_enable_otg 0x09 reg 0x%x \n", tmp);

	ret = cx2560x_write_reg40(cx, 1);
	if (ret != 0)
		cx2560x_err("otg write reg40 fail\n");

	ret = cx2560x_write_reg(cx, 0x83, 0x03);
	ret = cx2560x_write_reg(cx, 0x41, 0xA8);
	ret = cx2560x_update_bits(cx, 0x84, 0x80, 0x80);
	ret = cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_1,REG01_OTG_CONFIG_MASK,
					REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT);
	msleep(100);
	ret = cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_2, 0x80, 0x80);

	ret = cx2560x_write_reg(cx, 0x83, 0x01);
	msleep(10);
	ret = cx2560x_write_reg(cx, 0x41, 0x28);

	cx2560x_write_reg40(cx, 0);

	cx2560x_dbg("cx2560x_enable_otg successfully\n");

	return ret;

#if 0
	u8 val,temp,data;
	int ret;

	ret = cx2560x_read_reg(cx, CX2560X_REG_02, &data);
	temp = data & REG02_BOOST_LIM_MASK;
	ret = cx2560x_update_bits(cx, CX2560X_REG_02,REG02_BOOST_LIM_MASK,
			REG02_BOOST_LIM_0P5A << REG02_BOOST_LIM_SHIFT);

	val = REG01_OTG_ENABLE << REG01_OTG_CONFIG_SHIFT;
	cx2560x_info("cx2560x_enable_otg enter\n");
	ret = cx2560x_update_bits(cx, CX2560X_REG_01,REG01_OTG_CONFIG_MASK, val);
	msleep(500);
	ret = cx2560x_update_bits(cx, CX2560X_REG_02,REG02_BOOST_LIM_MASK, temp);

	return ret;
#endif
}

/*static bool cx2560x_is_otg_enable(struct cx2560x_device *cx)
{
	u8 val = 0;

	cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_1, &val);
	val = val & 0x20;
	cx2560x_info("cx2560x_is_otg_enable enter val = %d\n", val);

	if (0 != val) {
		return 1;
	} else {
		return 0;
	}
}*/

static int cx2560x_disable_otg(struct cx2560x_device *cx)
{
	int ret;
	u8 val = REG01_OTG_DISABLE << REG01_OTG_CONFIG_SHIFT;
	struct cx2560x_state state;

	cx2560x_get_status(cx, &state);

	cx2560x_write_reg40(cx, 1);
	cx2560x_write_reg(cx, 0x83, 0x00);
	ret = cx2560x_update_bits(cx, 0x84, 0x80, 0x00);
	cx2560x_write_reg40(cx, 0);

	cx2560x_dbg("cx2560x_disable_otg enter\n");

	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_1,
					REG01_OTG_CONFIG_MASK, val);
}

static int cx2560x_set_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct cx2560x_device *cx = charger_get_data(chg_dev);
	if (en) {
		ret = cx2560x_enable_otg(cx);
		if (ret < 0) {
			cx2560x_err("cx2560x_enable_otg fail\n");
			return ret;
		}
	} else {
		ret = cx2560x_disable_otg(cx);
		if (ret < 0) {
			cx2560x_err("cx2560x_disable_otg fail\n");
			return ret;
		}
	}
	return 0;
}

static int cx2560x_set_boost_current_limit(struct charger_device *chg_dev, u32 uA)
{
	return 0;
}

static int cx2560x_do_event(struct charger_device *chg_dev, u32 event, u32 args)
{
	if (chg_dev == NULL)
		return -EINVAL;

	switch (event) {
	case CHARGER_DEV_NOTIFY_EOC:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_EOC);
		break;
	case CHARGER_DEV_NOTIFY_RECHG:
		charger_dev_notify(chg_dev, CHARGER_DEV_NOTIFY_RECHG);
		break;
	default:
		break;
	}

	return 0;
}

/* for i2c auto test */
static ssize_t fts_rw_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ret = 0;
	struct i2c_client * client = (struct i2c_client *)dev->driver_data;
	struct cx2560x_device *cx = i2c_get_clientdata(client);

	if (opr_flag == 0) {
		if (opr_ret < 0) {
			return sprintf(buf, "Write fail reg=%x: write_data=%x\n", reg, write_data);
		} else {
			return sprintf(buf, "Write success reg=%x: write_data=%x\n", reg, write_data);
		}
	} else if (opr_flag == 1) {
		ret = cx2560x_read_reg(cx, reg, &read_data);
		if (ret < 0) {
			cx2560x_info("i2c read fail!\n");
			return sprintf(buf, "Read fail reg=%x, read_data=%x\n", reg, read_data);
		}
		return sprintf(buf, "Read success reg=%x, read_data=%x\n", reg, read_data);
	}
	return sprintf(buf, "fts_rw_reg_show operation error\n");
}

static ssize_t fts_rw_reg_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client * client = (struct i2c_client *)dev->driver_data;
	struct cx2560x_device *cx = i2c_get_clientdata(client);
	uint32_t data = 0;

	cx2560x_dbg("fts_rw_reg_store\n");
	if (strlen(buf) == 3) {
		sscanf(buf, "%x", &data);
		reg = data & 0xff;
		opr_flag = 1;
		cx2560x_info("fts_rw_reg_store: read reg:%x\n" , reg);
	} else if (strlen(buf) == 5) {
		sscanf(buf, "%x", &data);
		reg = (data >> 8) & 0xff;
		write_data = data & 0xff;
		opr_ret = cx2560x_write_reg(cx, reg, write_data);
		opr_flag = 0;
		cx2560x_info("fts_rw_reg_store: write reg:%x, data:%x\n" , reg, write_data);
	} else {
		opr_flag = -1;
		cx2560x_info("fts_rw_reg_store buf error.\n");
	}
	return count;
}

static DEVICE_ATTR(fts_rw_reg, 0664, fts_rw_reg_show, fts_rw_reg_store);
static struct attribute * chg_i2c_test_attributes[] = {
	&dev_attr_fts_rw_reg.attr,
	NULL,
};

static struct attribute_group chg_i2c_test_attribute_group = {
	.attrs = chg_i2c_test_attributes
};

static const struct attribute_group * chg_i2c_test_groups[] = {
	&chg_i2c_test_attribute_group,
	NULL,
};

static int cx2560x_en_pe_current_partern(struct charger_device *chg_dev, bool is_up)
{
	return 0;
}

static struct charger_ops cx2560x_chg_ops = {
	.plug_in = cx2560x_plug_in,
	.plug_out = cx2560x_plug_out,
	.enable = cx2560x_charging_switch,
	.is_enabled = cx2560x_is_enabled,
	.get_charging_current = NULL,
	.set_charging_current = cx2560x_set_ichrg_curr,
	.get_input_current = cx2560x_get_input_curr_lim,
	.set_input_current = cx2560x_set_input_curr_lim,
	.get_constant_voltage = cx2560x_get_chrg_volt,
	.set_constant_voltage = cx2560x_set_chrg_volt,
	.kick_wdt = cx2560x_reset_watch_dog_timer,
	.set_mivr = cx2560x_set_input_volt_lim,
	.is_charging_done = cx2560x_get_charging_status,
	.enable_safety_timer = cx2560x_enable_safetytimer,
	.is_safety_timer_enabled = cx2560x_get_is_safetytimer_enable,
	.enable_otg = cx2560x_set_otg,
	.set_boost_current_limit = cx2560x_set_boost_current_limit,
	.event = cx2560x_do_event,
	.send_ta_current_pattern = cx2560x_en_pe_current_partern,
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
	.enable_cable_drop_comp = NULL,

#if IS_ENABLED(CONFIG_OEM_HVDCP_ALGO)
	/* DPDM */
	.set_dp = cx2560x_set_dp_volt,
	.set_dm = cx2560x_set_dm_volt,
#endif /* CONFIG_OEM_HVDCP_ALGO */
};

static const struct charger_properties cx2560x_chg_props = {
	.alias_name = "cx2560x",
};

static int cx2560x_create_sysfs_file(struct cx2560x_device *cx)
{
	int ret;

	ret = device_create_file(cx->dev, &dev_attr_reg);
	if (ret) {
		cx2560x_err("device_create_file reg failed\n");
		goto err_create_reg;
	}

	ret = device_create_file(cx->dev, &dev_attr_dump);
	if (ret) {
		cx2560x_err("device_create_file dump failed\n");
		goto err_create_dump;
	}

	ret = device_create_file(cx->dev, &dev_attr_charge);
	if (ret) {
		cx2560x_err("device_create_file hvdcp_enable failed\n");
		goto err_create_charge;
	}

	ret = device_create_file(cx->dev, &dev_attr_dpdm);
	if (ret) {
		cx2560x_err("device_create_file dpdm failed\n");
		goto err_create_dpdm;
	}

	ret = device_create_file(cx->dev, &dev_attr_qc3v0);
	if (ret) {
		cx2560x_err("device_create_file dpdm failed\n");
		goto err_create_qc3v0;
	}

	ret = device_create_file(cx->dev, &dev_attr_vtemp);
	if (ret) {
		cx2560x_err("device_create_file vtemp failed\n");
		goto err_create_vtemp;
	}

	ret = device_create_file(cx->dev, &dev_attr_log_level);
	if (ret) {
		cx2560x_err("device_create_file log_level failed\n");
		goto err_create_log_level;
	}

	return 0;

err_create_vtemp:
	device_remove_file(cx->dev, &dev_attr_qc3v0);
err_create_qc3v0:
	device_remove_file(cx->dev, &dev_attr_dpdm);
err_create_dpdm:
	device_remove_file(cx->dev, &dev_attr_charge);
err_create_charge:
	device_remove_file(cx->dev, &dev_attr_dump);
err_create_dump:
	device_remove_file(cx->dev, &dev_attr_reg);
err_create_reg:
err_create_log_level:
	return -EPERM;
}

static int cx2560x_set_vindpm(struct cx2560x_device *cx, unsigned int vindpm)
{
	u8 reg_val;

	cx2560x_dbg("cx2560x_set_vindpm vindpm=%d\n", vindpm);
	if (vindpm < 3900)
		reg_val = 0x0;
	else if (vindpm > 5400)
		reg_val = 0x0F;
	else
		reg_val = (vindpm - 3900) / 100;

	return cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_6, 0x0F, reg_val);
}

static int cx2560x_force_dpdm(struct cx2560x_device *cx)
{
	int ret = 0;

	cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_7, 0x80, 0x80);

	cx2560x_info("cx2560x_force_dpdm\n");

	return ret;
}

static void cx2560x_force_detection_dwork_handler(struct work_struct *work)
{
	int ret;
	struct cx2560x_device *cx = container_of(work, struct cx2560x_device, force_detect_dwork.work);
	struct cx2560x_state state;


	cx2560x_get_status(cx, &state);
	//Charger_Detect_Init();
	ret = cx2560x_force_dpdm(cx);
	if (ret) {
		cx2560x_err("force dpdm failed(%d)\n", ret);
		return;
	}
	cx2560x_info("force dpdm state.vbus_stat=%d, force_detect_count=%d \n", state.vbus_stat, cx->force_detect_count);
	cx->force_detect_count++;
}

static void cx2560x_charger_auto_set_vindpm_work(struct work_struct *work)
{
	int ret;
	int vbus_volt = 0;
	int batt_vol = 0;
	int vindpm_volt;
	union power_supply_propval psy_prop;
	struct cx2560x_device *cx = container_of(work, struct cx2560x_device, vindpm_work.work);

	cx2560x_get_vbus(cx, &vbus_volt);

	if (!cx->battery_psy) {
		cx->battery_psy = power_supply_get_by_name("battery");
		if (!cx->battery_psy) {
			cx2560x_err("get battery power supply failed\n");
			goto out;
		}
	}

	ret = power_supply_get_property(cx->battery_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &psy_prop);
	if (ret) {
		cx2560x_err("get battery voltage failed\n");
		goto out;
	}
	batt_vol = psy_prop.intval / 1000;

	if (batt_vol < 4100)
		cx2560x_set_vindpm(cx, 4300);
	else {
		vindpm_volt = batt_vol + 250;
		cx2560x_set_vindpm(cx, vindpm_volt);
	}

	if (!ret)
		cx2560x_info("vbus_volt=%dmV, batt_vol=%dmV, set vindpm_volt=%dmV\n", vbus_volt, batt_vol, vindpm_volt);

out:
	schedule_delayed_work(&cx->vindpm_work, msecs_to_jiffies(5000));

	return;
}

static void charge_done_detect_work_func(struct work_struct *work)
{
	int ibat = 0;
	int vbat = 0;
	int vbus = 0;
	int ret = -1;
	int val_term = 0;
	int val_vreg = 0;
	unsigned char val;
	unsigned char iindpm_flag = 0;
	unsigned char vindpm_flag = 0;
	unsigned char regulation_status = 0;
	unsigned int vindpm_value;

	struct delayed_work *charge_done_detect_work = NULL;
	union power_supply_propval psy_prop;
	struct cx2560x_device *cx = container_of(
		work, struct cx2560x_device, charge_done_detect_work.work);

	if (!cx->battery_psy) {
		cx->battery_psy = power_supply_get_by_name("battery");
		if (!cx->battery_psy) {
			cx2560x_err("get battery power supply failed\n");
			goto out;
		}
	}

	/*get Vbat from gauge*/
	ret = power_supply_get_property(cx->battery_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &psy_prop);
	vbat = psy_prop.intval / 1000;

	/*get Ibat from gauge*/
	ret = power_supply_get_property(cx->battery_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &psy_prop);
	ibat = psy_prop.intval / 1000;

	/*get vbus from adc*/
	cx2560x_get_vbus(cx, &vbus);

	charge_done_detect_work = container_of(work, struct delayed_work, work);
	if (charge_done_detect_work == NULL) {
		cx2560x_err("Cann't get charge_detect_delayed_work\n");
		goto out;
	}
	cx = container_of(charge_done_detect_work, struct cx2560x_device, charge_done_detect_work);
	if (cx == NULL) {
		cx2560x_err("Cann't get cx25890_device\n");
		goto out;
	}

	cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_5, 0x80, 0 << 6);
	cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_F, 0xC0, 0 << 6);

	if (vbat < 4100)
		cx2560x_set_vindpm(cx, 4300);
	else {
		vindpm_value = (vbat + 250);
		cx2560x_set_vindpm(cx, vindpm_value);
	}

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_3, &val);
	if (ret)
		goto out;

	val &= REG03_ITERM_MASK;
	val_term = (val * 60) + 60;

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_4, &val);
	if (ret)
		goto out;

	val = (val & REG04_VREG_MASK) >> REG04_VREG_SHIFT;
	val_vreg = val * 32 + 3856;

	ret = cx2560x_read_reg_without_retry(cx, CX2560X_CHRG_CTRL_A, &val);
	if (ret)
		goto out;

	iindpm_flag = (val & REG0A_IINDPM_STAT_MASK) >> REG0A_IINDPM_STAT_SHIFT;
	vindpm_flag = (val & REG0A_VINDPM_STAT_MASK) >> REG0A_VINDPM_STAT_SHIFT;

	ret = cx2560x_read_reg_without_retry(cx, CX2560X_CHRG_CTRL_8, &val);
	if (ret)
		goto out;

	val = (val & REG08_THERM_STAT_MASK) >> REG08_THERM_STAT_SHIFT;
	regulation_status = val;

	cx2560x_info("ibat=%d, vbat=%d, val_vreg=%d, val_term=%d, iindpm_flag=%d, vindpm_flag=%d, regulation_status=%d\n",
			ibat, vbat, val_vreg, val_term, iindpm_flag, vindpm_flag, regulation_status);
	if ((val_vreg - 100) < vbat && ibat < (val_term + 10)) {
		if (iindpm_flag)
			iindpm_flag_count++;

		if (vindpm_flag)
			vindpm_flag_count++;

		if (regulation_status)
			regulation_status_count++;

		count++;
		if (count >= 4 && iindpm_flag_count < 3 && vindpm_flag_count < 3 && regulation_status_count < 3) {
			cx2560x_err("entry disable_charger\n");
			cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_1, CX2560X_CHRG_EN, 0);
			count = 0;
			charge_done_flag = true;
			iindpm_flag_count = 0;
			vindpm_flag_count = 0;
			regulation_status_count = 0;
			schedule_delayed_work(&cx->recharge_detect_work, 0);
			cancel_delayed_work(&cx->charge_done_detect_work);
			return;
		} else if (count >= 4) {
			count = 0;
			iindpm_flag_count = 0;
			vindpm_flag_count = 0;
			regulation_status_count = 0;
		}
	} else {
		count = 0;
		iindpm_flag_count = 0;
		vindpm_flag_count = 0;
		regulation_status_count = 0;
	}

out:
	if ((val_vreg - 100) < vbat  && ibat < 300) {
		schedule_delayed_work(&cx->charge_done_detect_work, 5 * HZ);
	} else {
		schedule_delayed_work(&cx->charge_done_detect_work, 10 * HZ);
	}

	return;
}

static void recharge_detect_work_func(struct work_struct *work)
{
	int vbat;
	int val_vreg;
	unsigned char val;
	int ret = -1;
	struct delayed_work *recharge_detect_work = NULL;
	union power_supply_propval psy_prop;
	struct cx2560x_device *cx = container_of(
		work, struct cx2560x_device, recharge_detect_work.work);

	if (!cx->battery_psy) {
		cx->battery_psy = power_supply_get_by_name("battery");
		if (!cx->battery_psy) {
			cx2560x_err("get battery power supply failed\n");
			goto out;
		}
	}

	/*get Vbat from gauge*/
	ret = power_supply_get_property(cx->battery_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &psy_prop);
	vbat = psy_prop.intval / 1000;

	recharge_detect_work = container_of(work, struct delayed_work, work);
	if (recharge_detect_work == NULL) {
		cx2560x_err("Cann't get charge_detect_delayed_work\n");
		goto out;
	}
	cx = container_of(recharge_detect_work, struct cx2560x_device, recharge_detect_work);
	if (cx == NULL) {
		cx2560x_err("Cann't get cx25890_device\n");
		goto out;
	}

	ret = cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_4, &val);
	if (ret)
		goto out;

	val = (val & REG04_VREG_MASK) >> REG04_VREG_SHIFT;
	val_vreg = val * 32 + 3856;

	cx2560x_info("vbat=%d, val_vreg=%d\n", vbat, val_vreg);
	if (charge_done_flag) {
		if (val_vreg - 100 > vbat) {
			cx2560x_err("entry enable_charger\n");
			cx2560x_update_bits(cx, CX2560X_CHRG_CTRL_1, CX2560X_CHRG_EN, CX2560X_CHRG_EN);
			charge_done_flag = false;
			count = 0;
			iindpm_flag_count = 0;
			vindpm_flag_count = 0;
			regulation_status_count = 0;
			schedule_delayed_work(&cx->charge_done_detect_work, 0);
			cancel_delayed_work(&cx->recharge_detect_work);
			return;
		} else {
			schedule_delayed_work(&cx->recharge_detect_work, 10 * HZ);
			return;
		}
	}
out:
	schedule_delayed_work(&cx->recharge_detect_work, 10 * HZ);
	return;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static int cx2560x_driver_probe(struct i2c_client *client)
#else
static int cx2560x_driver_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
#endif /* LINUX_VERSION_CODE */
{
	int ret = 0;
	struct cx2560x_device *cx;
	u8 data;
	char *name = NULL;

	cx2560x_info("enter\n");

	cx = devm_kzalloc(&client->dev, sizeof(*cx), GFP_KERNEL);
	if (!cx) {
		cx2560x_err("allocate memory for cx2560x failed\n");
		return -ENOMEM;
	}

	cx->chg_en = 1;
	cx->ibat = 3000;
	cx->chg_volt = 5600;
	cx->chg_curr = 3000;
	cx->vtemp = CX2560X_TEMP_INVALID;
	cx->qc_type = QUICK_CHARGE_1P0;
	cx->qc_force = QC_TYPE_FORCE_2P0;
	cx->temp_state = CX2560X_STATE_NORMAL;
	cx->client = client;
	cx->dev = &client->dev;
	cx->chg_desc = &cx2560x_charger_desc;
	i2c_set_clientdata(client, cx);
	INIT_DELAYED_WORK(&cx->irq_work, cx2560x_charge_irq_work);
	INIT_DELAYED_WORK(&cx->psy_work, cx2560x_psy_change_work);
	INIT_DELAYED_WORK(&cx->charge_work, cx2560x_charge_online_work);
	INIT_DELAYED_WORK(&cx->vindpm_work, cx2560x_charger_auto_set_vindpm_work);
	//INIT_DELAYED_WORK(&cx->charge_usb_detect_work, charger_usb_detect_work_func);
	INIT_DELAYED_WORK(&cx->charge_done_detect_work, charge_done_detect_work_func);
	INIT_DELAYED_WORK(&cx->recharge_detect_work, recharge_detect_work_func);

	ret = cx2560x_write_reg40(cx, 1);
	if (ret == 1)
		cx2560x_info("write reg40 successfully\n");

	cx2560x_write_reg(cx, 0x41, 0x28);
	cx2560x_write_reg(cx, 0x44, 0x18);
	cx2560x_read_reg(cx, 0x41, &data);

	cx2560x_info("enter, 0x41=%x\n", data);

	ret = cx2560x_write_reg40(cx, false);
	cx2560x_info("cx2560x init sucess\n");

	ret = cx2560x_chipid_detect(cx);
	if (ret != CX2560X_PN_ID) {
		cx2560x_err("charger ic not found\n");
		devm_kfree(cx->dev, cx);
		return -ENODEV;;
	}

	ret = cx2560x_gpio_config(cx);
	if (ret < 0) {
		cx2560x_err("config gpio failed\n");
		goto err_gpio_config;
	}

	ret = cx2560x_hardware_init(cx);
	if (ret) {
		cx2560x_err("hardware init failed\n");
		goto err_hardware_init;
	}

	name = devm_kasprintf(cx->dev, GFP_KERNEL, "%s", "cx2560x suspend wakelock");
	cx->charger_wakelock = wakeup_source_register(cx->dev, name);

	/* Register charger device */
	cx->chg_dev = charger_device_register("primary_chg",
						&client->dev, cx,
						&cx2560x_chg_ops,
						&cx2560x_chg_props);
	if (IS_ERR_OR_NULL(cx->chg_dev)) {
		cx2560x_err("register charger device failed\n");
		ret = PTR_ERR(cx->chg_dev);
		goto err_hardware_init;
	}

	ret = devm_request_threaded_irq(cx->dev, client->irq,
				NULL, cx2560x_irq_handler_thread,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				dev_name(cx->dev), cx);
	if (ret) {
		cx2560x_err("request interrupt failed\n");
		goto err_request_irq;
	}
	ret = enable_irq_wake(client->irq);
	if (ret) {
		cx2560x_err("enable irq wake failed\n");
		goto err_irq_wake;
	}

	cx->charger_psy = cx2560x_register_charger_psy(cx);
	if (IS_ERR_OR_NULL(cx->charger_psy)) {
		cx2560x_err("register charger power supply failed\n");
		goto err_register_charger_psy;
	}

	cx->usb_psy = cx2560x_register_usb_psy(cx);
	if (IS_ERR_OR_NULL(cx->usb_psy)) {
		cx2560x_err("register usb power supply failed\n");
		goto err_register_usb_psy;
	}

	cx->ac_psy = cx2560x_register_ac_psy(cx);
	if (IS_ERR_OR_NULL(cx->ac_psy)) {
		cx2560x_err("register ac power supply failed\n");
		goto err_register_ac_psy;
	}

#if 0
	cx->otg_rdev = cx2560x_register_regulator(cx);
	if (IS_ERR_OR_NULL(cx->otg_rdev)) {
		cx2560x_err("register regulator failed\n");
		goto err_register_rdev;
	}
#endif

	/* for i2c auto test, and current node is fix */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0))
	chg_i2c_test_class = class_create("mcharger_iic_test");
#else
	chg_i2c_test_class = class_create(THIS_MODULE, "mcharger_iic_test");
#endif /* LINUX_VERSION_CODE */
	if (IS_ERR(chg_i2c_test_class)) {
		ret = PTR_ERR(chg_i2c_test_class);
		cx2560x_err("create chg_i2c_test_class error\n");
	}
	chg_i2c_test_class->dev_groups = chg_i2c_test_groups;
	ret = alloc_chrdev_region(&chg_i2c_test_devt, 0, 1, "chg_i2c_test_devt");
	if (ret < 0) {
		cx2560x_err("alloc_chrdev_region failed!\n");
	}
	chg_i2c_test_device = device_create(chg_i2c_test_class, NULL, chg_i2c_test_devt, client, "chg_i2c_test_node");
	if (IS_ERR(chg_i2c_test_device)) {
		ret = PTR_ERR(chg_i2c_test_device);
		cx2560x_err("chg_i2c_test_device create error: %d\n", ret);
	}

	ret = cx2560x_create_sysfs_file(cx);
	if (ret) {
		cx2560x_err("create sysfs failed\n");
		goto err_create_sysfs;
	}

	INIT_DELAYED_WORK(&cx->force_detect_dwork, cx2560x_force_detection_dwork_handler);

	//msleep(10);
	//cx2560x_set_hvdcp_off(cx);

	//get usb device
	cx->usb2_phy = devm_usb_get_phy(cx->dev, USB_PHY_TYPE_USB2);

	if (IS_ERR_OR_NULL(cx->usb2_phy)) {
		cx2560x_err("usb_get_phy failed\n");
		return ret;
	} else {
		cx2560x_info("usb_get_phy success\n");
	}

	cx->vbus = devm_iio_channel_get(cx->dev, "pmic_vbus");
	if (IS_ERR_OR_NULL(cx->vbus)) {
		cx2560x_err("cx2560x get vbus failed\n");
		goto err_create_sysfs;
	}

	//msleep(500); //add for guanjichongdian
	//cx2560x_force_dpdm(cx);
	//msleep(500); //add for guanjichongdian
	schedule_delayed_work(&cx->force_detect_dwork, 5 * HZ);
	cx2560x_set_hvdcp_en(cx);
	ret = cx2560x_get_initial_state(cx);
	if (ret) {
		cx2560x_err("get initial state failed\n");
		goto err_irq_wake;
	}


	cx2560x_read_reg(cx, CX2560X_CHRG_CTRL_A, &data);
	if (data & 0x80)
		schedule_delayed_work(&cx->charge_done_detect_work, 10 * HZ);

	cx2560x_info("cx2560x_driver_probe finished\n");

	return 0;

err_create_sysfs:
//	regulator_unregister(cx->otg_rdev);
//err_register_rdev:
	power_supply_unregister(cx->usb_psy);
err_register_usb_psy:
	power_supply_unregister(cx->ac_psy);
err_register_ac_psy:
	power_supply_unregister(cx->charger_psy);
err_register_charger_psy:
	disable_irq_wake(client->irq);
err_irq_wake:
	free_irq(client->irq, cx);
err_request_irq:
	charger_device_unregister(cx->chg_dev);
err_hardware_init:
	gpio_free(cx->irq_gpio);
	gpio_free(cx->en_gpio);
err_gpio_config:
	devm_kfree(cx->dev, cx);
	return -EBUSY;
}

static void cx2560x_remove_sysfs_file(struct cx2560x_device *cx)
{
	device_remove_file(cx->dev, &dev_attr_vtemp);
	device_remove_file(cx->dev, &dev_attr_qc3v0);
	device_remove_file(cx->dev, &dev_attr_dpdm);
	device_remove_file(cx->dev, &dev_attr_charge);
	device_remove_file(cx->dev, &dev_attr_dump);
	device_remove_file(cx->dev, &dev_attr_reg);
	device_remove_file(cx->dev, &dev_attr_log_level);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
static void cx2560x_charger_remove(struct i2c_client *client)
#else
static int cx2560x_charger_remove(struct i2c_client *client)
#endif /* LINUX_VERSION_CODE */
{
	struct cx2560x_device *cx = i2c_get_clientdata(client);

	cx2560x_remove_sysfs_file(cx);
	regulator_unregister(cx->otg_rdev);
	power_supply_unregister(cx->usb_psy);
	power_supply_unregister(cx->ac_psy);
	power_supply_unregister(cx->charger_psy);
	disable_irq_wake(client->irq);
	free_irq(client->irq, cx);
	charger_device_unregister(cx->chg_dev);
	gpio_free(cx->irq_gpio);
	gpio_free(cx->en_gpio);
	devm_kfree(cx->dev, cx);

#if !(LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0))
	return 0;
#endif /* LINUX_VERSION_CODE */
}

void cx2560x_charger_shutdown(struct i2c_client *client)
{
	struct cx2560x_device *cx = i2c_get_clientdata(client);
	int ret = 0;

	cx2560x_set_hvdcp_off(cx);
	//cx2560x_set_dpdm_hiz(cx);
	cx2560x_info("ret=%d\n", ret);
	cx2560x_set_fast_charge_current(cx, 500);
	cx2560x_dump_register(cx->chg_dev);

	msleep(50);
}

static int cx2560x_device_suspend(struct device *dev)
{
	struct cx2560x_device *cx = dev_get_drvdata(dev);

	if (cx->state.pg_stat) {
		cancel_delayed_work(&cx->charge_work);
		cx2560x_info("cancel charge work\n");
	}

	return 0;
}

static int cx2560x_device_resume(struct device *dev)
{
	struct cx2560x_device *cx = dev_get_drvdata(dev);

	if (cx->state.pg_stat) {
		schedule_delayed_work(&cx->charge_work, msecs_to_jiffies(100));
		cx2560x_info("resume charge work\n");
	}

	return 0;
}

static int cx2560x_dump_register(struct charger_device *chg_dev)
{
	unsigned char i = 0;
	unsigned int ret = 0;
	unsigned char cx2560x_reg[CX2560X_REG_NUM + 1] = { 0 };
	struct cx2560x_device *cx = charger_get_data(chg_dev);

	for (i = 0; i < CX2560X_REG_NUM + 1; i++) {
		ret = cx2560x_read_reg(cx, i, &cx2560x_reg[i]);
		if (ret != 0) {
			cx2560x_err("i2c transfor error\n");
			return ret;
		}
		cx2560x_info("reg[0x%02x]=0x%02x\n", i, cx2560x_reg[i]);
	}

	return 0;
}

static struct dev_pm_ops cx2560x_pm_osp = {
	.suspend = cx2560x_device_suspend,
	.resume = cx2560x_device_resume,
};

static const struct i2c_device_id cx2560x_i2c_ids[] = {
	{ "cx2560x-charger", CX25601 }, { },
};

static const struct of_device_id cx2560x_of_match[] = {
	{ .compatible = "cx,cx2560x_charger", }, { },
};

static struct i2c_driver cx2560x_driver = {
	.driver = {
		.name = "cx2560x-charger",
		.of_match_table = cx2560x_of_match,
		.pm = &cx2560x_pm_osp,
	},
	.probe = cx2560x_driver_probe,
	.remove = cx2560x_charger_remove,
	.shutdown = cx2560x_charger_shutdown,
	.id_table = cx2560x_i2c_ids,
};

module_i2c_driver(cx2560x_driver);
MODULE_DESCRIPTION("SUNCORE CX2560x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SUNCORE");
