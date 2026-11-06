// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include <linux/dma-mapping.h>
#if IS_ENABLED( CONFIG_COMPAT)  
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#include "eeprom_utils.h"
#include "eeprom_i2c_custom_driver.h"

#define EEPROM_I2C_MSG_SIZE_READ 2

/************************************************************
 * I2C read function (Custom)
 * Customer's driver can put on here
 * Below is an example
 ************************************************************/
 #define PAGE_SIZE_ 256

struct gc05a2_otp_t{
	u8  module_flag;
	u8  module_param[9];
	u8  module_checksum;
	u8  awb_param[28];
	u8  awb_checksum;
	u8  lsc_param[1868];
	u8  lsc_checksum;
};

struct gc05a2_otp_t gc05a2_otp_info;
bool gc05a_otp_read_tag = 0;
static void read_gc05a2_otp_data(struct i2c_client *clien);

static int iReadRegI2C(struct i2c_client *client,
		u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData)
{
	int i4RetValue = 0;
	struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue = i2c_transfer(client->adapter, msg,
				EEPROM_I2C_MSG_SIZE_READ);

	if (i4RetValue != EEPROM_I2C_MSG_SIZE_READ) {
		pr_err("I2C read failed!!\n");
		return -1;
	}
	return 0;
}

static int iWriteRegI2C(struct i2c_client *client,
			u8 *a_pSendData, u16 a_sizeSendData)
{
	int  i4RetValue = 0;
  	struct i2c_msg msg;
  
  
  	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = a_sizeSendData;
	msg.buf = a_pSendData;
	i4RetValue = i2c_transfer(client->adapter,
				&msg,
				1);
	if (i4RetValue != 1) {
		pr_debug("I2C write failed!!\n");
		return -1;
	}
	return 0;
}

static u16 read_otp(struct i2c_client *client, u32 addr)
{
	u16 get_byte = 0;
	char pu_send_cmd[2] = { (char)((addr >> 8) & 0xFF),
		(char)(addr & 0xFF) };
	iReadRegI2C(client, pu_send_cmd, 2, (u8 *)&get_byte, 1);
	return get_byte;
}
static void write_otp(struct i2c_client *client, u16 addr, u8 para)
{
	char pu_send_cmd[3] = { (char)((addr >> 8) & 0xFF),
		(char)(addr & 0xFF), (char)(para & 0xFF) };
	iWriteRegI2C(client, pu_send_cmd, 3);
}

static int custom_read_region(struct i2c_client *client,
			      u32 addr, u8 *data, u16 i2c_id, u32 size)
{
	u32 size_to_read = size;

	int ret = 0;

	while (size_to_read > 0) {
		u8 page = addr / PAGE_SIZE_;
		u8 offset = addr % PAGE_SIZE_;
		char *Buff = data;

		if (iReadRegI2C(client, &offset, 1, (u8 *)Buff, 1) < 0) {
			pr_err("fail addr=0x%x 0x%x, P=%d, offset=0x%x",
				addr, *Buff, page, offset);
			break;
		}
		addr++;
		size_to_read--;
		ret++;
	}
	pr_info("addr =%x size %d data read = %d\n", addr, size, ret);
	return ret;
}



unsigned int Custom_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	if (custom_read_region(client, addr, data, client->addr, size) == 0)
		return size;
	else
		return 0;
}

unsigned int gc05a2sub_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size)
{
    unsigned char *dataTmp = data;
    pr_info("gc05a2 otp region addr = 0x%x, size = %d\n", addr, size);

    if(gc05a_otp_read_tag == 0 ){
		pr_debug("read_gc05a2_otp_data start\n");
        read_gc05a2_otp_data(client);
    }

	if (addr == 0x0 && size == 8192) {
       unsigned int totalSize = sizeof(gc05a2_otp_info.module_flag) +
                                 sizeof(gc05a2_otp_info.module_param) +
                                 sizeof(gc05a2_otp_info.module_checksum) +
                                 sizeof(gc05a2_otp_info.awb_param) +
                                 sizeof(gc05a2_otp_info.awb_checksum) +
                                 sizeof(gc05a2_otp_info.lsc_param) +
								 sizeof(gc05a2_otp_info.lsc_checksum);
        pr_info("gc05a2 otp region addr = 0x%x, size = %d  totalSize=%d \n", addr, size, totalSize);
        if (size == totalSize) {
            data[0] = gc05a2_otp_info.module_flag;
            /* dataTmp += sizeof(gc05a2_otp_info.page_flag); */
            /* memcpy(dataTmp, gc05a2_otp_info.module_flag, sizeof(gc05a2_otp_info.module_flag)); */
            dataTmp += sizeof(gc05a2_otp_info.module_flag);
            memcpy(dataTmp, gc05a2_otp_info.module_param, sizeof(gc05a2_otp_info.module_param));
            dataTmp += sizeof(gc05a2_otp_info.module_param);
            /* memcpy(dataTmp, gc05a2_otp_info.module_checksum, sizeof(gc05a2_otp_info.module_checksum)); */
			data[10] = gc05a2_otp_info.module_checksum;
            dataTmp += sizeof(gc05a2_otp_info.module_checksum);
            memcpy(dataTmp, gc05a2_otp_info.awb_param, sizeof(gc05a2_otp_info.awb_param));
            dataTmp += sizeof(gc05a2_otp_info.awb_param);
            /* memcpy(dataTmp, gc05a2_otp_info.awb_checksum, sizeof(gc05a2_otp_info.awb_checksum)); */
			data[39] = gc05a2_otp_info.awb_checksum;
            dataTmp += sizeof(gc05a2_otp_info.awb_checksum);
            memcpy(dataTmp, gc05a2_otp_info.lsc_param, sizeof(gc05a2_otp_info.lsc_param));
            dataTmp += sizeof(gc05a2_otp_info.lsc_param);
            /* memcpy(dataTmp, gc05a2_otp_info.lsc_checksum, sizeof(gc05a2_otp_info.lsc_checksum)); */
			data[totalSize - 1] = gc05a2_otp_info.lsc_checksum;
        } else {
			pr_err("gc05a2 otp size != totalSize");
            size = totalSize;
        }
    } else if (size == 4) { //read single awb data
        switch (addr) {
            case 0x1:
                *(u32 *)data = 0x00000006;
                break;
            case 0x15:
                pr_info("add = 0x%x, read awb_unint\n",addr);
            	memcpy(data,(gc05a2_otp_info.awb_param), size);
                break;
            case 0x19:
                pr_info("add = 0x%x, read golden\n",addr);
            	memcpy(data, (gc05a2_otp_info.awb_param + 4), size);
                break;
        }
    } else if (size >=1868 && size < 2048 && addr == 40) {
		memcpy(data, gc05a2_otp_info.lsc_param, size);
		pr_info("add = 0x%x, read lsc\n",addr);
	} else{
        pr_err("gc05a2 otp add = 0x%x, size = %d ,read error !!!\n",addr,size);
    }
	return size;
}

static void gc05a2_otp_init(struct i2c_client *client)
{
    u8 temp = 0;
	write_otp(client,0x0af6, 0x00);
	write_otp(client,0x0b90, 0x10);
	write_otp(client,0x0b91, 0x00);
	write_otp(client,0x0b92, 0x00);
	write_otp(client,0x0ba0, 0x17);
	write_otp(client,0x0ba1, 0x00);
	write_otp(client,0x0ba2, 0x00);
	write_otp(client,0x0ba4, 0x03);
	write_otp(client,0x0ba5, 0x00);
	write_otp(client,0x0ba6, 0x00);
	write_otp(client,0x0ba8, 0x40);
	write_otp(client,0x0ba9, 0x00);
	write_otp(client,0x0baa, 0x00);
	write_otp(client,0x0bac, 0x40);
	write_otp(client,0x0bad, 0x00);
	write_otp(client,0x0bae, 0x00);
	write_otp(client,0x0bb0, 0x02);
	write_otp(client,0x0bb1, 0x00);
	write_otp(client,0x0bb2, 0x00);
	write_otp(client,0x0bb8, 0x02);
	write_otp(client,0x0bb9, 0x00);
	write_otp(client,0x0bba, 0x00);
	write_otp(client,0x0a70, 0x80);
	write_otp(client,0x0a71, 0x00);
	write_otp(client,0x0a72, 0x00);
	/* write_otp(0x0a66, 0x00); */
	temp = read_otp(client, 0x0a66);
    write_otp(client,0x0a66, temp & 0xfe);
	write_otp(client,0x0a67, 0x80);
	write_otp(client,0x0a4d, 0x0e);
	write_otp(client,0x0a45, 0x02);
	write_otp(client,0x0a47, 0x02);
	write_otp(client,0x0a50, 0x00);
	write_otp(client,0x0a4f, 0x0c);
	mdelay(10);
}

static void gc05a2_otp_group_init(struct i2c_client *client)
{
    u8 temp = 0;
	write_otp(client, 0x0af6, 0x00);
	write_otp(client, 0x0b90, 0x10);
	write_otp(client, 0x0b91, 0x00);
	write_otp(client, 0x0b92, 0x00);
	write_otp(client, 0x0ba0, 0x17);
	write_otp(client, 0x0ba1, 0x00);
	write_otp(client, 0x0ba2, 0x00);
	write_otp(client, 0x0ba4, 0x03);
	write_otp(client, 0x0ba5, 0x00);
	write_otp(client, 0x0ba6, 0x00);
	write_otp(client, 0x0ba8, 0x40);
	write_otp(client, 0x0ba9, 0x00);
	write_otp(client, 0x0baa, 0x00);
	write_otp(client, 0x0bac, 0x40);
	write_otp(client, 0x0bad, 0x00);
	write_otp(client, 0x0bae, 0x00);
	write_otp(client, 0x0bb0, 0x02);
	write_otp(client, 0x0bb1, 0x00);
	write_otp(client, 0x0bb2, 0x00);
	write_otp(client, 0x0bb8, 0x02);
	write_otp(client, 0x0bb9, 0x00);
	write_otp(client, 0x0bba, 0x00);
	write_otp(client, 0x0a70, 0x80);
	write_otp(client, 0x0a71, 0x00);
	write_otp(client, 0x0a72, 0x00);
	/* write_otp(0x0a66, 0x00); */
	temp = read_otp(client, 0x0a66);
    write_otp(client, 0x0a66, temp & 0xfe);
	write_otp(client, 0x0a67, 0x84);
	write_otp(client, 0x0a4d, 0x0e);
	write_otp(client, 0x0a45, 0x02);
	write_otp(client, 0x0a47, 0x02);
	write_otp(client, 0x0a50, 0x00);
	write_otp(client, 0x0a4f, 0x0c);
	mdelay(10);
}

static u8 gc05a2_otp_read_byte(struct i2c_client *client, u16 addr)
{
	u8 val = 0;
	u8 temp = 0;
	write_otp(client, 0x0a69, (addr >> 8) & 0xff);
	write_otp(client, 0x0a6a, addr & 0xff);
	temp = read_otp(client, 0x0a66);
	write_otp(client, 0x0a66, temp|0x20);
	val = read_otp(client, 0x0a6c);

	pr_info("addr = 0x%x, data = 0x%x\n", addr , val);
	return val;
}

static u8 gc05a2_otp_read_group(struct i2c_client *client, u16 addr, u8 *data, u8 length)
{
	u8 i = 0;
	u8 val = 0;
	u8 temp = 0;
	pr_info("gc05a2_otp_read_group ui4_offset = 0x%x, ui4_length = %d \n", addr, length);
	write_otp(client, 0x0a69, (addr >> 8) & 0xff);
	write_otp(client, 0x0a6a, addr & 0xff);
	temp = read_otp(client, 0x0a66);
	write_otp(client, 0x0a66, temp|0x20);
	val = read_otp(client, 0x0a6c);
	write_otp(client, 0x0a66, 0x12);
	for (i = 0; i < length; i++) {
		data[i] = read_otp(client, 0x0a6c);
	    // pr_info("addr = 0x%x, data = 0x%x\n", addr + i * 8, data[i]);
	}
	return 0;
}
static int gc05a2_iReadData(struct i2c_client *client, unsigned int ui4_offset, unsigned int ui4_length, unsigned char *pinputdata)
{
	int i4RetValue = 0;
	int i4ResidueDataLength;
	u32 u4CurrentOffset;
	pr_info("ui4_offset = 0x%x, ui4_length = %d \n", ui4_offset, ui4_length);
	i4ResidueDataLength = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	i4RetValue =gc05a2_otp_read_group(client, (u16) u4CurrentOffset, pinputdata, i4ResidueDataLength);
	if (i4RetValue != 0) {
		pr_err("I2C iReadData failed!!\n");
		return -1;
	}
	return 0;
}
static bool gc05a2_param_checksum(u8 *buf, unsigned int size, u8 checksum)
{
    int i, sum = 0;
    for (i = 0; i < size; i++)
    {
        sum += buf[i];
        //pr_err("buf[%d] = 0x%x %d", i, buf[i], buf[i]);
    }
    if ((sum % 255 + 1) != checksum)
    {
        pr_err("checksum fail size = %d sum=%d sum-in-eeprom=%d", size, sum % 255 + 1, checksum);
        return false;
    }
    pr_info("checksum success size = %d sum=%d sum-in-eeprom=%d", size, sum % 255 + 1, checksum);
    return true;
}
static bool gc05a2_read_module_info(struct i2c_client *client, u8 moduleflag)
{
	bool ret = false;
	pr_info("--------------gc05a2 module info read begin------------\n");
	if (moduleflag == 0x01) {
        gc05a2_iReadData(client, MODULE_GROUP1_INFO_FLAG, MODULE_INFO_LENGTH, &gc05a2_otp_info.module_param[0]);
        /* gc05a2_otp_read_byte(MODULE_GROUP1_CHECKSUM, 1, &gc05a2_otp_info.module_checksum); */
		gc05a2_otp_info.module_checksum = gc05a2_otp_read_byte(client, MODULE_GROUP1_CHECKSUM);
	} else if (moduleflag == 0x07) {
		gc05a2_iReadData(client, MODULE_GROUP2_INFO_FLAG, MODULE_INFO_LENGTH, &gc05a2_otp_info.module_param[0]);
        /* gc05a2_otp_read_byte(MODULE_GROUP2_CHECKSUM, 1, &gc05a2_otp_info.module_checksum); */
		gc05a2_otp_info.module_checksum = gc05a2_otp_read_byte(client, MODULE_GROUP2_CHECKSUM);
	} else if (moduleflag == 0x1f) {
		gc05a2_iReadData(client, MODULE_GROUP3_INFO_FLAG, MODULE_INFO_LENGTH, &gc05a2_otp_info.module_param[0]);
        /* gc05a2_otp_read_byte(MODULE_GROUP3_CHECKSUM, 1, &gc05a2_otp_info.module_checksum); */
		gc05a2_otp_info.module_checksum = gc05a2_otp_read_byte(client, MODULE_GROUP3_CHECKSUM);
	} else {
		pr_err("--------------gc05a2 module info read failed------------\n");
	}
	pr_info("--------------gc05a2 module info read end------------\n");
	ret = gc05a2_param_checksum(&gc05a2_otp_info.module_param[0], MODULE_INFO_LENGTH, gc05a2_otp_info.module_checksum);
	if (ret) {
        pr_info("--------------gc05a2 module info checksum success------------\n");
	}
	return ret;
}
static bool gc05a2_read_awb_info(struct i2c_client *client, u8 moduleflag)
{
	bool ret = false;
	pr_info("--------------gc05a2 awb info read begin------------\n");
	pr_info("AWB moduleflag:%d\n", moduleflag);
	if (moduleflag == 0x01) {
        gc05a2_iReadData(client, AWB_GROUP1_INFO_FLAG, AWB_INFO_LENGTH, &gc05a2_otp_info.awb_param[0]);
        /* gc05a2_otp_read_byte(AWB_GROUP1_CHECKSUM, 1, &gc05a2_otp_info.awb_checksum); */
		gc05a2_otp_info.awb_checksum = gc05a2_otp_read_byte(client, AWB_GROUP1_CHECKSUM);
	} else if (moduleflag == 0x07) {
		gc05a2_iReadData(client, AWB_GROUP2_INFO_FLAG, AWB_INFO_LENGTH, &gc05a2_otp_info.awb_param[0]);
        /* gc05a2_otp_read_byte(AWB_GROUP1_CHECKSUM, 1, &gc05a2_otp_info.awb_checksum); */
		gc05a2_otp_info.awb_checksum = gc05a2_otp_read_byte(client, AWB_GROUP2_CHECKSUM);
	} else if (moduleflag == 0x1f) {
		gc05a2_iReadData(client, AWB_GROUP3_INFO_FLAG, AWB_INFO_LENGTH, &gc05a2_otp_info.awb_param[0]);
        /* gc05a2_otp_read_byte(AWB_GROUP1_CHECKSUM, 1, &gc05a2_otp_info.awb_checksum); */
		gc05a2_otp_info.awb_checksum = gc05a2_otp_read_byte(client, AWB_GROUP3_CHECKSUM);
	} else {
		pr_err("--------------gc05a2 awb info read failed------------\n");
	}
	pr_info("--------------gc05a2 awb info read end------------\n");
	ret = gc05a2_param_checksum(&gc05a2_otp_info.awb_param[0], AWB_INFO_LENGTH, gc05a2_otp_info.awb_checksum);
	if (ret) {
        pr_info("--------------gc05a2 awb info checksum success------------\n");
	}
	return ret;
}
static bool gc05a2_read_lsc_info(struct i2c_client *client, u8 moduleflag)
{
	bool ret = false;
	pr_debug("--------------gc05a2 lsc info read begin------------\n");
	if (moduleflag == 0x40) {
        gc05a2_iReadData(client, LSC_GROUP1_INFO_FLAG, LSC_INFO_LENGTH, &gc05a2_otp_info.lsc_param[0]);
        /* gc05a2_otp_read_byte(LSC_GROUP1_CHECKSUM, 1, &gc05a2_otp_info.lsc_checksum); */
		gc05a2_otp_info.lsc_checksum = gc05a2_otp_read_byte(client, LSC_GROUP1_CHECKSUM);
	} else if (moduleflag == 0x10) {
		gc05a2_iReadData(client, LSC_GROUP2_INFO_FLAG, LSC_INFO_LENGTH, &gc05a2_otp_info.lsc_param[0]);
        /* gc05a2_otp_read_byte(LSC_GROUP1_CHECKSUM, 1, &gc05a2_otp_info.lsc_checksum); */
		gc05a2_otp_info.lsc_checksum = gc05a2_otp_read_byte(client, LSC_GROUP2_CHECKSUM);
	} else if (moduleflag == 0x04) {
		gc05a2_iReadData(client, LSC_GROUP3_INFO_FLAG, LSC_INFO_LENGTH, &gc05a2_otp_info.lsc_param[0]);
        /* gc05a2_otp_read_byte(LSC_GROUP1_CHECKSUM, 1, &gc05a2_otp_info.lsc_checksum); */
		gc05a2_otp_info.lsc_checksum = gc05a2_otp_read_byte(client, LSC_GROUP3_CHECKSUM);
	} else {
		pr_err("--------------gc05a2 lsc info read failed------------\n");
	}
	pr_debug("--------------gc05a2 lsc info read end------------\n");
	ret = gc05a2_param_checksum(&gc05a2_otp_info.lsc_param[0], LSC_INFO_LENGTH, gc05a2_otp_info.lsc_checksum);
	if (ret) {
        pr_info("--------------gc05a2 lsc info checksum success------------\n");
	}
	return ret;
}
static void read_gc05a2_otp_data(struct i2c_client *client)
{
	u8 moduleflag =0;
	bool checksum_module = false;
	bool checksum_awb = false;
	bool checksum_lsc = false;
    gc05a2_otp_init(client);
	moduleflag = gc05a2_otp_read_byte(client, MODULE_GROUP_FLAG);
	pr_info("gc05a2 moduleflag = 0x%x", moduleflag);
    gc05a2_otp_group_init(client);
	checksum_module = gc05a2_read_module_info(client, moduleflag);
	checksum_awb = gc05a2_read_awb_info(client, moduleflag);
	checksum_lsc = gc05a2_read_lsc_info(client, moduleflag);
	if (true == (checksum_module & checksum_awb & checksum_lsc))
	{
		gc05a_otp_read_tag = 1;
		pr_info("----------------gc05a2 otp info check success----------------");
	}
	else
	{
		pr_err("----------------gc05a2 otp info check fail-------------------");
	}
}


