// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/timer.h>
#include <linux/delay.h>

#include <linux/pinctrl/consumer.h>

/*
 * use tcpc dev to detect audio plug in
 */
#include "../inc/tcpci_core.h"
#include "../inc/tcpm.h"

#define USE_POWER_SUPPLY_NOTIFIER    0
#define USE_TCPC_NOTIFIER            1

enum et_function {
	ET_MIC_GND_SWAP,
	ET_USBC_ORIENTATION_CC1,
	ET_USBC_ORIENTATION_CC2,
	ET_USBC_DISPLAYPORT_DISCONNECTED,
	ET_EVENT_MAX,
};

enum SWITCH_STATUS {
	SWITCH_STATUS_INVALID = 0,
	SWITCH_STATUS_NOT_CONNECTED,
	SWITCH_STATUS_USB_MODE,
	SWITCH_STATUS_HEADSET_MODE,
	SWITCH_STATUS_MAX
};

char *switch_status_string[SWITCH_STATUS_MAX] = {
	"switch invalid",
	"switch not connected",
	"switch usb mode",
	"switch headset mode",
};

#define HL5280_I2C_NAME	"hl5280-driver"

#define HL5280_ID              0x00
#define HL5280_SWITCH_SETTINGS 0x04
#define HL5280_SWITCH_CONTROL  0x05
#define HL5280_SWITCH_STATUS0  0x06
#define HL5280_SWITCH_STATUS1  0x07
#define HL5280_SLOW_L          0x08
#define HL5280_SLOW_R          0x09
#define HL5280_SLOW_MIC        0x0A
#define HL5280_SLOW_SENSE      0x0B
#define HL5280_SLOW_GND        0x0C
#define HL5280_DELAY_L_R       0x0D
#define HL5280_DELAY_L_MIC     0x0E
#define HL5280_DELAY_L_SENSE   0x0F
#define HL5280_DELAY_L_AGND    0x10
#define HL5280_FUNCTION_ENABLE 0x12
#define HL5280_JACK_STATUS     0x17
#define HL5280_DETECTION_INT   0x18
#define HL5280_RESET           0x1E
#define HL5280_CURRENT_SOURCE  0x1F

#define ET_DBG_TYPE_MODE          0
#define ET_DBG_REG_MODE           1

#define HL5280_DELAY_INIT_TIME     (10 * HZ)

#define HL5280_ID_VALUE   0x49
#define BCT4480_ID_VALUE  0x09

static struct timer_list hl5280_enable_timer;
static struct work_struct hl5280_enable_switch_work;
static struct workqueue_struct *hl5280_enable_switch_workqueue;

static struct timer_list hl5280_delay_init_timer;
static struct work_struct hl5280_delay_init_work;
static struct workqueue_struct *hl5280_delay_init_workqueue;

struct hl5280_priv {
	struct regmap *regmap;
	struct device *dev;
	struct power_supply *usb_psy;
	struct tcpc_device *tcpc_dev;
	struct notifier_block psy_nb;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;//not used
	struct blocking_notifier_head hl5280_notifier;
	struct mutex notification_lock;
//	struct pinctrl *uart_en_gpio_pinctrl;
//	struct pinctrl_state *pinctrl_state_enable;
//	struct pinctrl_state *pinctrl_state_disable;
	bool hl5280;
};

static int chip_id = 0;

static struct hl5280_priv *global_hl5280_data = NULL;

struct hl5280_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config hl5280_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = HL5280_CURRENT_SOURCE,
};

static const struct hl5280_reg_val et_reg_i2c_defaults[] = {
	{HL5280_SWITCH_SETTINGS, 0x98},
	{HL5280_SWITCH_CONTROL, 0x18},
	{HL5280_SLOW_L, 0x00},
	{HL5280_SLOW_R, 0x00},
	{HL5280_SLOW_MIC, 0x00},
	{HL5280_SLOW_SENSE, 0x00},
	{HL5280_SLOW_GND, 0x00},
	{HL5280_DELAY_L_R, 0x00},
	{HL5280_DELAY_L_MIC, 0x00},
	{HL5280_DELAY_L_SENSE, 0x00},
	{HL5280_DELAY_L_AGND, 0x00},
	{HL5280_FUNCTION_ENABLE, 0x08},
	{HL5280_CURRENT_SOURCE, 0x07},
};

enum SWITCH_STATUS hl5280_get_switch_mode(void);

extern void accdet_eint_callback_wrapper(unsigned int plug_status);
static void dump_register(void);

static void hl5280_usbc_update_settings(struct hl5280_priv *et_priv,u32 switch_control, u32 switch_enable)
{
	if (!et_priv) {
		pr_err("%s: invalid et_priv %p\n", __func__, et_priv);
		return;
	}
	if (!et_priv->regmap) {
		dev_err(et_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}
	regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, 0x80);
	regmap_write(et_priv->regmap, HL5280_SWITCH_CONTROL, switch_control);
	/* HL5280 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, switch_enable);
}

static int hl5280_tcpc_event_changed(struct notifier_block *nb,
				      unsigned long evt, void *ptr)
{
	u32 int_status = 0;
	u32 jack_status = 0;
	struct tcp_notify *noti = ptr;
	struct hl5280_priv *et_priv = container_of(nb, struct hl5280_priv, psy_nb);

	if (NULL == noti) {
		pr_err("%s: data is NULL. \n", __func__);
		return NOTIFY_DONE;
	}

	switch (evt) {
	case TCP_NOTIFY_TYPEC_STATE:
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO) {
			/* Audio Plug in */
			pr_info("%s: Audio Plug In %d\n", __func__,et_priv->hl5280);
//			pinctrl_select_state(et_priv->uart_en_gpio_pinctrl,et_priv->pinctrl_state_disable);
			if (et_priv->hl5280) {
				if (chip_id == BCT4480_ID_VALUE)
					regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, 0x9d);//0x9F
				else 
					regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, 0x9d);
				regmap_write(et_priv->regmap, HL5280_SWITCH_CONTROL, 0x00);
				regmap_read(global_hl5280_data->regmap, 0X04, &int_status);
				regmap_read(global_hl5280_data->regmap, 0X05, &jack_status);
				pr_info("%s REG04:%X REG05%X \n", __func__,int_status,jack_status);
				if (chip_id == BCT4480_ID_VALUE)
					regmap_write(et_priv->regmap, HL5280_FUNCTION_ENABLE, 0x09);
				else
					regmap_write(et_priv->regmap, HL5280_FUNCTION_ENABLE, 0x49);
				mod_timer(&hl5280_enable_timer, jiffies + (int)(0.2 * HZ));
				// regmap_read(global_hl5280_data->regmap, HL5280_DETECTION_INT, &int_status);
				// regmap_read(global_hl5280_data->regmap, HL5280_JACK_STATUS, &jack_status);
				
				// if((int_status & (1 << 2)) && (jack_status & (1 << 1))){
				// 	//3-pole detect
				// 	if (chip_id == BCT4480_ID_VALUE)
				// 		regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, 0x9B);
				// 	else
				// 		regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, 0x9F);
				// 	regmap_write(et_priv->regmap, HL5280_SWITCH_CONTROL, 0x00);
				// 	mod_timer(&hl5280_enable_timer, jiffies + (int)(0.05 * HZ));	
				// }else if((int_status & (1 << 2)) && (jack_status & (1 << 3))){
				// 	if (chip_id == BCT4480_ID_VALUE)
				// 		regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, 0x9B);
				// 	else
				// 		regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, 0x9F);
				// 	regmap_write(et_priv->regmap, HL5280_SWITCH_CONTROL, 0x00);
				// 	mod_timer(&hl5280_enable_timer, jiffies + (int)(0.05 * HZ));
				// }else if((int_status & (1 << 2)) && (jack_status & (1 << 2))){
				// 	if (chip_id == BCT4480_ID_VALUE) {
				// 		regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, 0x9B);
				// 		regmap_write(et_priv->regmap, HL5280_SWITCH_CONTROL, 0x03);
				// 	} else {
				// 		regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, 0x9F);
				// 		regmap_write(et_priv->regmap, HL5280_SWITCH_CONTROL, 0x07);
				// 	}
				// 	mod_timer(&hl5280_enable_timer, jiffies + (int)(0.05 * HZ));
				// }  把这些都可以放在hl5280_enable_switch_work_callback 里
			}
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_AUDIO &&
			noti->typec_state.new_state == TYPEC_UNATTACHED) {
			/* Audio Plug out */
			pr_info("%s: Audio Plug Out \n", __func__);
			if (et_priv->hl5280)  {
				regmap_write(et_priv->regmap, HL5280_RESET, 0x01);
				mdelay(5);
				regmap_write(et_priv->regmap, HL5280_SWITCH_SETTINGS, 0x9D);
				regmap_write(et_priv->regmap, HL5280_SWITCH_CONTROL, 0x18);
				if (chip_id == BCT4480_ID_VALUE) {
					pr_info("%s set 0x0e to 0xff for POP BCT4480\n", __func__);
					regmap_write(et_priv->regmap, HL5280_DELAY_L_MIC, 0xff);//power up set 0x0e to 0xff for pop
				}
			}
	//		pinctrl_select_state(et_priv->uart_en_gpio_pinctrl,et_priv->pinctrl_state_enable);
			//hl5280_usbc_update_settings(et_priv, 0x18, 0x9f);
			accdet_eint_callback_wrapper(0);
			dump_register();
		} else if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state != TYPEC_ATTACHED_AUDIO) {
			if (hl5280_get_switch_mode() != SWITCH_STATUS_USB_MODE) {
				hl5280_usbc_update_settings(et_priv, 0x18, 0x9D);
//				pinctrl_select_state(et_priv->uart_en_gpio_pinctrl,et_priv->pinctrl_state_enable);
			}
		}
		break;
	}
	return NOTIFY_OK;
}

/*
 * hl5280_unreg_notifier - unregister notifier block with et driver
 *
 * @nb - notifier block of hl5280
 * @node - phandle node to hl5280 device
 *
 * Returns 0 on pass, or error code
 */
int hl5280_unreg_notifier(struct notifier_block *nb,
			     struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct hl5280_priv *et_priv;

	if (!client)
		return -EINVAL;

	et_priv = (struct hl5280_priv *)i2c_get_clientdata(client);
	if (!et_priv)
		return -EINVAL;

	hl5280_usbc_update_settings(et_priv, 0x18, 0x98);
	return blocking_notifier_chain_unregister
					(&et_priv->hl5280_notifier, nb);
}
EXPORT_SYMBOL(hl5280_unreg_notifier);

static int hl5280_validate_display_port_settings(struct hl5280_priv *et_priv)
{
	u32 switch_status = 0;

	regmap_read(et_priv->regmap, HL5280_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		pr_err("AUX SBU1/2 switch status is invalid = %u\n",
				switch_status);
		return -EIO;
	}

	return 0;
}
/*
 * hl5280_switch_event - configure ET switch position based on event
 *
 * @node - phandle node to hl5280 device
 * @event - et_function enum
 *
 * Returns int on whether the switch happened or not
 */
int hl5280_switch_event(struct device_node *node,
			 enum et_function event)
{
	int switch_control = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct hl5280_priv *et_priv;

	if (!client)
		return -EINVAL;

	et_priv = (struct hl5280_priv *)i2c_get_clientdata(client);
	if (!et_priv)
		return -EINVAL;
	if (!et_priv->regmap)
		return -EINVAL;

	switch (event) {
	case ET_MIC_GND_SWAP:
		regmap_read(et_priv->regmap, HL5280_SWITCH_CONTROL,
				&switch_control);
		if ((switch_control & 0x07) == 0x07)
			switch_control = 0x0;
		else {
			if (chip_id == BCT4480_ID_VALUE)
				switch_control = 0x3;
			else 
				switch_control = 0x7;
		}
		if (chip_id == BCT4480_ID_VALUE)
			hl5280_usbc_update_settings(et_priv, switch_control, 0x9B);
		else
			hl5280_usbc_update_settings(et_priv, switch_control, 0x9F);
		return 1;
	case ET_USBC_ORIENTATION_CC1:
		if (chip_id == BCT4480_ID_VALUE)
			hl5280_usbc_update_settings(et_priv, 0x18, 0x98);
		else
			hl5280_usbc_update_settings(et_priv, 0x18, 0xF8);
		return hl5280_validate_display_port_settings(et_priv);
	case ET_USBC_ORIENTATION_CC2:
		if (chip_id == BCT4480_ID_VALUE)
			hl5280_usbc_update_settings(et_priv, 0x18, 0x98);
		else
			hl5280_usbc_update_settings(et_priv, 0x18, 0xF8);
		return hl5280_validate_display_port_settings(et_priv);
	case ET_USBC_DISPLAYPORT_DISCONNECTED:
		hl5280_usbc_update_settings(et_priv, 0x18, 0x98);
		break;
	default:
		break;
	}
	dump_register();
	return 0;
}
EXPORT_SYMBOL(hl5280_switch_event);

#if 0
static void hl5280_usbc_analog_work_fn(struct work_struct *work)
{//not used
	struct hl5280_priv *et_priv =
		container_of(work, struct hl5280_priv, usbc_analog_work);

	if (!et_priv) {
		pr_err("%s: et container invalid\n", __func__);
		return;
	}
	hl5280_usbc_analog_setup_switches(et_priv);
	pm_relax(et_priv->dev);
}
#endif

static void hl5280_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	for (i = 0; i < ARRAY_SIZE(et_reg_i2c_defaults); i++)
		regmap_write(regmap, et_reg_i2c_defaults[i].reg,
					et_reg_i2c_defaults[i].val);
}

/* add hl5280 info node */
enum SWITCH_STATUS hl5280_get_switch_mode(void)
{
	uint val = 0;
	enum SWITCH_STATUS state = SWITCH_STATUS_INVALID;
	regmap_read(global_hl5280_data->regmap, HL5280_SWITCH_STATUS0, &val);

	switch (val & 0xf) {
	case 0x0:
		state = SWITCH_STATUS_NOT_CONNECTED;
		break;
	case 0x5:
		state = SWITCH_STATUS_USB_MODE;
		break;
	case 0xA:
		state = SWITCH_STATUS_HEADSET_MODE;
		break;
	default:
		state = SWITCH_STATUS_INVALID;
		break;
	}

	return state;
}

int hl5280_switch_mode(enum SWITCH_STATUS val)
{
	if (val == SWITCH_STATUS_HEADSET_MODE) {
		if (chip_id == BCT4480_ID_VALUE)
			hl5280_usbc_update_settings(global_hl5280_data, 0x00, 0x9F); // switch to headset
		else
			hl5280_usbc_update_settings(global_hl5280_data, 0x00, 0x9F); // switch to headset
	} else if (val == SWITCH_STATUS_USB_MODE) {
		hl5280_usbc_update_settings(global_hl5280_data, 0x18, 0x98); // switch to USB
	}

	return 0;
}

static ssize_t sysfs_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf, u32 type)
{
	int value = 0;
	char *mode = "Unknown mode";
	int i = 0;
	ssize_t ret_size = 0;

	switch (type) {
	case ET_DBG_TYPE_MODE:
		value = hl5280_get_switch_mode();
		mode = switch_status_string[value];
		ret_size += sprintf(buf, "%s: %d \n", mode, value);
		break;

	case ET_DBG_REG_MODE:
		for (i = 0; i <= HL5280_CURRENT_SOURCE; i++) {
			regmap_read(global_hl5280_data->regmap, i, &value);
			ret_size += sprintf(buf + ret_size, "Reg: 0x%x, Value: 0x%x \n", i, value);
		}
	    break;
	default:
		pr_warn("%s: invalid type %d\n", __func__, type);
		break;
	}
	return ret_size;
}

static ssize_t sysfs_set(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count, u32 type)
{
	int err;
	unsigned long value;
	unsigned int reg;
	unsigned int val;

	switch (type) {
		case ET_DBG_TYPE_MODE:
			err = kstrtoul(buf, 10, &value);
			if (err) {
				pr_warn("%s: get data of type %d failed\n", __func__, type);
				return err;
			}
			pr_info("%s: set type %d, data %ld\n", __func__, type, value);
			hl5280_switch_mode((enum SWITCH_STATUS)value);
			break;
		case ET_DBG_REG_MODE:
			err = sscanf(buf, "%x %x", &reg, &val);
			if (err == 2 && reg < 0x20) {
				regmap_write(global_hl5280_data->regmap, reg, val);
			}
			break;
		default:
			pr_warn("%s: invalid type %d\n", __func__, type);
			break;
	}
	return count;
}

#define hl5280_DEVICE_SHOW(_name, _type) static ssize_t \
show_##_name(struct device *dev, \
			  struct device_attribute *attr, char *buf) \
{ \
	return sysfs_show(dev, attr, buf, _type); \
}

#define hl5280_DEVICE_SET(_name, _type) static ssize_t \
set_##_name(struct device *dev, \
			 struct device_attribute *attr, \
			 const char *buf, size_t count) \
{ \
	return sysfs_set(dev, attr, buf, count, _type); \
}

#define hl5280_DEVICE_SHOW_SET(name, type) \
hl5280_DEVICE_SHOW(name, type) \
hl5280_DEVICE_SET(name, type) \
static DEVICE_ATTR(name, S_IWUSR | S_IRUGO, show_##name, set_##name);

hl5280_DEVICE_SHOW_SET(hl5280_switch_mode, ET_DBG_TYPE_MODE);
hl5280_DEVICE_SHOW_SET(hl5280_reg, ET_DBG_REG_MODE);

static struct attribute *hl5280_attrs[] = {
	&dev_attr_hl5280_switch_mode.attr,
	&dev_attr_hl5280_reg.attr,
	NULL
};

static const struct attribute_group hl5280_group = {
	.attrs = hl5280_attrs,
};
/* end of info node */

#if 0
static int hl5280_gpio_pinctrl_init(struct hl5280_priv *et_priv)
{
	int retval = 0;

	pr_info("%s \n", __func__);

	et_priv->uart_en_gpio_pinctrl = devm_pinctrl_get(et_priv->dev);
	if (IS_ERR_OR_NULL(et_priv->uart_en_gpio_pinctrl)) {
		retval = PTR_ERR(et_priv->uart_en_gpio_pinctrl);
		goto gpio_err_handle;
	}

	et_priv->pinctrl_state_disable
		= pinctrl_lookup_state(et_priv->uart_en_gpio_pinctrl, "uart_disable");
	if (IS_ERR_OR_NULL(et_priv->pinctrl_state_disable)) {
		retval = PTR_ERR(et_priv->pinctrl_state_disable);
		goto gpio_err_handle;
	}

	et_priv->pinctrl_state_enable
		= pinctrl_lookup_state(et_priv->uart_en_gpio_pinctrl, "uart_enable");
	if (IS_ERR_OR_NULL(et_priv->pinctrl_state_enable)) {
		retval = PTR_ERR(et_priv->pinctrl_state_enable);
		goto gpio_err_handle;
	}

	return 0;

gpio_err_handle:
	pr_info("%s: init failed \n", __func__);
	devm_pinctrl_put(et_priv->uart_en_gpio_pinctrl);
	et_priv->uart_en_gpio_pinctrl = NULL;

	return retval;
}
#endif

static void hl5280_enable_switch_handler(struct timer_list *t)
{
	int ret = 0;

	ret = queue_work(hl5280_enable_switch_workqueue, &hl5280_enable_switch_work);
	if (!ret)
		pr_info("%s, queue work return: %d!\n", __func__, ret);
}

static void hl5280_enable_switch_work_callback(struct work_struct *work)
{
	u32 int_status = 0;
	u32 jack_status = 0;
	u32 switch_control = 0;
	u32 switch_setting = 0;
	pr_info("%s()\n", __func__);
	regmap_read(global_hl5280_data->regmap, HL5280_DETECTION_INT, &int_status);
	regmap_read(global_hl5280_data->regmap, HL5280_JACK_STATUS, &jack_status);
	pr_info("%s: hl5280 delay 200ms audio_jack_status=%d,audio_int_status=%d\n", __func__,jack_status,int_status);
	if ((global_hl5280_data->hl5280) && (int_status & (1 << 2) ) && (jack_status & (1 << 1))) {
		pr_info("%s: 3-pole detect\n", __func__);

		regmap_read(global_hl5280_data->regmap, HL5280_SWITCH_CONTROL, &switch_control);
		regmap_read(global_hl5280_data->regmap, HL5280_SWITCH_SETTINGS, &switch_setting);
		/* Set Bit 1 to 0, Enable Mic <---> SBU2 */
		regmap_write(global_hl5280_data->regmap, HL5280_SWITCH_CONTROL, switch_control & (~(1 << 1)));
		/* HL5280 chip hardware requirement */
		usleep_range(50, 55);
		/* Enable Bit 1, Enable Mic <---> SBU2 Switch */
		regmap_write(global_hl5280_data->regmap, HL5280_SWITCH_SETTINGS, switch_setting | (1 << 1));
	}
 	if((int_status & (1 << 2)) && (jack_status & (1 << 3))){
		regmap_write(global_hl5280_data->regmap, HL5280_SWITCH_SETTINGS, 0x9F);
		regmap_write(global_hl5280_data->regmap, HL5280_SWITCH_CONTROL, 0x00);
		pr_err("%s: 4-pole 0 detect\n", __func__);
	}else if((int_status & (1 << 2)) && (jack_status & (1 << 2))){
		regmap_write(global_hl5280_data->regmap, HL5280_SWITCH_SETTINGS, 0x9F);
		regmap_write(global_hl5280_data->regmap, HL5280_SWITCH_CONTROL, 0x07);
		pr_err("%s: 4-pole 1 detect\n", __func__);
	}
	//mdelay(500);
	accdet_eint_callback_wrapper(1);
	dump_register();
}

//static void delay_init_timer_callback(unsigned long data)
static void delay_init_timer_callback(struct timer_list *t)
{//(struct timer_list *t)
	int ret = 0;

	ret = queue_work(hl5280_delay_init_workqueue, &hl5280_delay_init_work);
	pr_info("%s \n", __func__);

	if (!ret)
		pr_info("%s, queue work return: %d!\n", __func__, ret);
}

static void hl5280_delay_init_work_callback(struct work_struct *work)
{
#if USE_TCPC_NOTIFIER
	int rc = 0;
	pr_info("%s() \n", __func__);

	if (global_hl5280_data && global_hl5280_data->tcpc_dev) {
		/* check tcpc status at startup */
		if (TYPEC_ATTACHED_AUDIO == tcpm_inquire_typec_attach_state(global_hl5280_data->tcpc_dev)){
			if (global_hl5280_data->hl5280){
				regmap_write(global_hl5280_data->regmap, HL5280_RESET, 0x01);
				mdelay(1);
				if (chip_id == BCT4480_ID_VALUE)
					regmap_write(global_hl5280_data->regmap, HL5280_SWITCH_SETTINGS, 0x9f);
				else
					regmap_write(global_hl5280_data->regmap, HL5280_SWITCH_SETTINGS, 0x9f);
				regmap_write(global_hl5280_data->regmap, HL5280_SWITCH_CONTROL, 0x00);

				if (chip_id == BCT4480_ID_VALUE){
					regmap_write(global_hl5280_data->regmap, HL5280_FUNCTION_ENABLE, 0x09);
					mdelay(10);
				}
				accdet_eint_callback_wrapper(1);
			}
		}
		/* register tcpc_event */
		global_hl5280_data->psy_nb.notifier_call = hl5280_tcpc_event_changed;
		global_hl5280_data->psy_nb.priority = 0;
		rc = register_tcp_dev_notifier(global_hl5280_data->tcpc_dev, &global_hl5280_data->psy_nb, TCP_NOTIFY_TYPE_USB);
		if (rc) {
			pr_err("%s: register_tcp_dev_notifier failed\n", __func__);
		}
		dump_register();
	}
#endif
}

static void dump_register(void)
{
	int adr = 0, value = 0;
	pr_info("%s:dump %d reg\n",__func__,global_hl5280_data->hl5280);
	for (adr = 0; adr <= HL5280_CURRENT_SOURCE; adr++) {
		regmap_read(global_hl5280_data->regmap, adr, &value);
		pr_info("(0x%x)=0x%x",adr,value);
	}
}

int get_type_c_hph_direction(void)
{
	u32 jack_status;
	regmap_read(global_hl5280_data->regmap, HL5280_JACK_STATUS, &jack_status);
	pr_info("%s:jack status is 0x%x\n",__func__,jack_status);
	return jack_status == 0x04;
}
EXPORT_SYMBOL(get_type_c_hph_direction);

static int hl5280_probe(struct i2c_client *i2c)
//			 const struct i2c_device_id *id)
{
	struct hl5280_priv *et_priv;
	int rc = 0;
	int reg_val = 0;

	et_priv = devm_kzalloc(&i2c->dev, sizeof(*et_priv),
				GFP_KERNEL);
	if (!et_priv)
		return -ENOMEM;

	global_hl5280_data = et_priv; // add for debug
	et_priv->dev = &i2c->dev;
	et_priv->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!et_priv->tcpc_dev) {
		rc = -EPROBE_DEFER;
		pr_err("%s get tcpc device type_c_port0 fail \n", __func__);
		goto err_data;
	}

	et_priv->regmap = devm_regmap_init_i2c(i2c, &hl5280_regmap_config);
	if (IS_ERR_OR_NULL(et_priv->regmap)) {
		dev_err(et_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!et_priv->regmap) {
			rc = -EINVAL;
			goto err_supply;
		}
		rc = PTR_ERR(et_priv->regmap);
		goto err_supply;
	}


	/*hl5280_gpio_pinctrl_init(et_priv);
	if (et_priv->uart_en_gpio_pinctrl) {
		rc = pinctrl_select_state(et_priv->uart_en_gpio_pinctrl,
					et_priv->pinctrl_state_enable);
		if (rc < 0) {
			pr_err("Failed to select enable pinstate %d\n", rc);
		}
	}*/

	hl5280_update_reg_defaults(et_priv->regmap);

	mutex_init(&et_priv->notification_lock);
	i2c_set_clientdata(i2c, et_priv);
	et_priv->hl5280_notifier.rwsem =
		(struct rw_semaphore)__RWSEM_INITIALIZER
		((et_priv->hl5280_notifier).rwsem);
	et_priv->hl5280_notifier.head = NULL;

	rc = sysfs_create_group(&i2c->dev.kobj, &hl5280_group);
	if (rc) {
		pr_err("%s: create attr error %d\n", __func__, rc);
	}

	hl5280_enable_switch_workqueue = create_singlethread_workqueue("enableSwitchQueue");
	INIT_WORK(&hl5280_enable_switch_work, hl5280_enable_switch_work_callback);
	if (!hl5280_enable_switch_workqueue) {
		rc = -1;
		pr_notice("%s create hl5280_enable_switch workqueue fail.\n", __func__);
		goto err_data;
	}

	pr_info("%s(), setup enable timer", __func__);
	timer_setup(&hl5280_enable_timer, hl5280_enable_switch_handler, (unsigned long)et_priv);

	hl5280_delay_init_workqueue = create_singlethread_workqueue("delayInitQueue");
	INIT_WORK(&hl5280_delay_init_work, hl5280_delay_init_work_callback);

	/* delay 2s to register tcpc event change, after accdet init done */
	timer_setup(&hl5280_delay_init_timer, delay_init_timer_callback, 0);
	mod_timer(&hl5280_delay_init_timer, jiffies + HL5280_DELAY_INIT_TIME);

	regmap_read(et_priv->regmap, HL5280_ID, &reg_val);
	if (reg_val != HL5280_ID_VALUE) {
		et_priv->hl5280 = true;
		pr_info("%s audio switch use dio4480 reg_val is %x", __func__, reg_val);
	} else {
		et_priv->hl5280 = true;
		pr_info("%s audio switch use hl5280 reg_val is %x", __func__, reg_val);
	}

	chip_id = reg_val;
	if (chip_id == BCT4480_ID_VALUE) {
		pr_info("%s audio switch use BCT4480", __func__);
		hl5280_usbc_update_settings(et_priv,HL5280_FUNCTION_ENABLE,0x08);
		regmap_write(et_priv->regmap, HL5280_DELAY_L_MIC, 0xff);//power up set 0x0e to 0xff for pop
	}

	hl5280_usbc_update_settings(et_priv,0x18,0x9D);
	mdelay(5);
	dump_register();
	return 0;

err_supply:
err_data:
	devm_kfree(&i2c->dev, et_priv);
	return rc;
}

static void hl5280_remove(struct i2c_client *i2c)
{
	struct hl5280_priv *et_priv =
			(struct hl5280_priv *)i2c_get_clientdata(i2c);

	if (!et_priv)
		return;

	hl5280_usbc_update_settings(et_priv, 0x18, 0x98);
	mutex_destroy(&et_priv->notification_lock);
	dev_set_drvdata(&i2c->dev, NULL);

	//return 0;
}

static const struct of_device_id hl5280_i2c_dt_match[] = {
	{
		.compatible = "mediatek,hl5280-audioswitch",
	},
	{}
};

static struct i2c_driver hl5280_i2c_driver = {
	.driver = {
		.name = HL5280_I2C_NAME,
		.of_match_table = hl5280_i2c_dt_match,
	},
	.probe = hl5280_probe,
	.remove = hl5280_remove,
};

static int __init hl5280_init(void)
{
	int rc;
	rc = i2c_add_driver(&hl5280_i2c_driver);
	if (rc)
		pr_err("hl5280: Failed to register I2C driver: %d\n", rc);

	return rc;
}
late_initcall(hl5280_init);

static void __exit hl5280_exit(void)
{
	i2c_del_driver(&hl5280_i2c_driver);
}
module_exit(hl5280_exit);

MODULE_DESCRIPTION("HL5280 I2C driver");
MODULE_LICENSE("GPL v2");
