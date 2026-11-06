/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __EEPROM_I2C_CUSTOM_DRIVER_H
#define __EEPROM_I2C_CUSTOM_DRIVER_H
#include <linux/i2c.h>

/************************************************************
 * I2C read function (Custom)
 * Customer's driver can put on here
 * Below is an example
 ************************************************************/

#define MODULE_GROUP_FLAG 0x2000
#define MODULE_GROUP1_INFO_FLAG 0x2008
#define MODULE_GROUP1_CHECKSUM 0x20a8
#define MODULE_GROUP2_INFO_FLAG 0x5CE0
#define MODULE_GROUP2_CHECKSUM 0x5D28
#define MODULE_GROUP3_INFO_FLAG 0x99B8
#define MODULE_GROUP3_CHECKSUM 0x9A00
#define MODULE_INFO_LENGTH 20
#define AWB_GROUP1_INFO_FLAG 0x2348
#define AWB_GROUP1_CHECKSUM 0x2388
#define AWB_GROUP2_INFO_FLAG 0x2390
#define AWB_GROUP2_CHECKSUM 0x23D0
#define AWB_GROUP3_INFO_FLAG 0x23D8
#define AWB_GROUP3_CHECKSUM 0x2418
#define AWB_INFO_LENGTH 8
#define LSC_GROUP1_INFO_FLAG 0x2140
#define LSC_GROUP1_CHECKSUM 0x5BA0
#define LSC_GROUP2_INFO_FLAG 0x5E18
#define LSC_GROUP2_CHECKSUM 0x9878
#define LSC_GROUP3_INFO_FLAG 0x9AF0
#define LSC_GROUP3_CHECKSUM 0xD548
#define LSC_INFO_LENGTH 1868

unsigned int Custom_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

unsigned int gc05a2sub_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

#endif				/* __CAM_CAL_LIST_H */
