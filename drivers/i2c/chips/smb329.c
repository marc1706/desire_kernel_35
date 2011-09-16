/* drivers/i2c/chips/smb329.c
 *
 * Copyright (C) 2009 HTC Corporation
 * Author: Justin Lin <Justin_Lin@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/htc_battery.h>
#include <asm/mach-types.h>

static const unsigned short normal_i2c[] = { I2C_CLIENT_END };

static int smb329_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
/* static int smb329_detect(struct i2c_client *client, int kind,
			 struct i2c_board_info *info); */
static int smb329_remove(struct i2c_client *client);


/* IncredibleC for Switch charger */
struct smb329_i2c_client {
    struct i2c_client *client;
    u8 address;
    /* max numb of i2c_msg required is for read =2 */
    struct i2c_msg xfer_msg[2];
    /* To lock access to xfer_msg */
    struct mutex xfer_lock;
};
static struct smb329_i2c_client smb329_i2c_module;
/**
Function:smb329_i2c_write
Target:	Write a byte to Switch charger
Timing:	TBD
INPUT: 	value-> write value
		reg  -> reg offset
		num-> number of byte to write
return :TRUE-->OK
		FALSE-->Fail
 */
static int smb329_i2c_write(u8 *value, u8 reg, u8 num_bytes)
{
    int ret;
    struct smb329_i2c_client *smb;
    struct i2c_msg *msg;

    smb = &smb329_i2c_module;

    mutex_lock(&smb->xfer_lock);
    /*
     * [MSG1]: fill the register address data
     * fill the data Tx buffer
     */
    msg = &smb->xfer_msg[0];
    msg->addr = smb->address;
    msg->len = num_bytes + 1;
    msg->flags = 0;
    msg->buf = value;
    /* over write the first byte of buffer with the register address */
    *value = reg;
    ret = i2c_transfer(smb->client->adapter, smb->xfer_msg, 1);
    mutex_unlock(&smb->xfer_lock);

    /* i2cTransfer returns num messages.translate it pls.. */
    if (ret >= 0)
		ret = 0;
    return ret;
}


/**
Function:smb329_i2c_read
Target:	Read a byte from Switch charger
Timing:	TBD
INPUT: 	value-> store buffer
		reg  -> reg offset to read
		num-> number of byte to read
return :TRUE-->OK
		FALSE-->Fail
 */
static int smb329_i2c_read(u8 *value, u8 reg, u8 num_bytes)
{
    int ret;
    u8 val;
    struct smb329_i2c_client *smb;
    struct i2c_msg *msg;

    smb = &smb329_i2c_module;

    mutex_lock(&smb->xfer_lock);
    /* [MSG1] fill the register address data */
    msg = &smb->xfer_msg[0];
    msg->addr = smb->address;
    msg->len = 1;
    msg->flags = 0; /* Read the register value */
    val = reg;
    msg->buf = &val;
    /* [MSG2] fill the data rx buffer */
    msg = &smb->xfer_msg[1];
    msg->addr = smb->address;
    msg->flags = I2C_M_RD;  /* Read the register value */
    msg->len = num_bytes;   /* only n bytes */
    msg->buf = value;
    ret = i2c_transfer(smb->client->adapter, smb->xfer_msg, 2);
    mutex_unlock(&smb->xfer_lock);

    /* i2cTransfer returns num messages.translate it pls.. */
    if (ret >= 0)
		ret = 0;
    return ret;
}


/**
Function:smb329_i2c_write_byte
Target:	Write a byte from Switch charger
Timing:	TBD
INPUT: 	value-> store buffer
		reg  -> reg offset to read
return :TRUE-->OK
		FALSE-->Fail
 */
static int smb329_i2c_write_byte(u8 value, u8 reg)
{
    /* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	int result;
	u8 temp_buffer[2] = { 0 };
    /* offset 1 contains the data */
	temp_buffer[1] = value;
	result = smb329_i2c_write(temp_buffer, reg, 1);
	if (result != 0)
		pr_info("SMB329 I2C write fail = %d\n", result);

    return result;
}

/**
Function:smb329_i2c_read_byte
Target:	Read a byte from Switch charger
Timing:	TBD
INPUT: 	value-> store buffer
		reg  -> reg offset to read
return :TRUE-->OK
		FALSE-->Fail
 */
static int smb329_i2c_read_byte(u8 *value, u8 reg)
{
	int result = 0;
	result = smb329_i2c_read(value, reg, 1);
	if (result != 0)
		pr_info("SMB329 I2C read fail = %d\n", result);

	return result;
}

int set_charger_ctrl(u32 ctl)
{
	int result = 0;
    u8 version;
	u8 reg_00h;
	u8 reg_01h;
	u8 reg_05h;
	u8 reg_31h;
	u8 reg_33h;
	u8 reg_35h;
	u8 reg_36h;
	u8 reg_37h;

	switch (ctl) {
	case DISABLE:
		pr_info("batt: Switch charger OFF\n");
		smb329_i2c_read_byte(&reg_00h, 0x00);
		smb329_i2c_read_byte(&reg_01h, 0x01);
		smb329_i2c_read_byte(&reg_05h, 0x05);
		smb329_i2c_read_byte(&reg_31h, 0x31);
		smb329_i2c_read_byte(&reg_33h, 0x33);
		smb329_i2c_read_byte(&reg_35h, 0x35);
		smb329_i2c_read_byte(&reg_36h, 0x36);
		smb329_i2c_read_byte(&reg_37h, 0x37);
		pr_info("batt: Switch charger ON (SLOW):"
			" reg_05h=%x, reg_31h=%x\n", reg_05h, reg_31h);
		pr_info("batt: Switch charger ON (SLOW) DEBUG:"
			" reg_00h=%x, reg_01h=%x, reg_33h=%x\n"
			, reg_00h, reg_01h, reg_33h);
		pr_info("batt: Switch charger ON (SLOW) DEBUG:"
			" reg_35h=%x, reg_36h=%x, reg_37h=%x\n"
			, reg_35h, reg_36h, reg_37h);
		break;
	case ENABLE_SLOW_CHG:
		pr_info("Switch charger ON (SLOW)\n");
		smb329_i2c_write_byte(0x88, 0x31);
		smb329_i2c_read_byte(&reg_31h, 0x31);
		smb329_i2c_write_byte(0xD0, 0x01);
		smb329_i2c_write_byte(0x08, 0x05);
		smb329_i2c_read_byte(&reg_05h, 0x05);
		smb329_i2c_read_byte(&reg_00h, 0x00);
		smb329_i2c_read_byte(&reg_01h, 0x01);
		smb329_i2c_read_byte(&reg_35h, 0x35);
		smb329_i2c_read_byte(&reg_36h, 0x36);
		smb329_i2c_read_byte(&reg_37h, 0x37);
		pr_info("batt: Switch charger ON (SLOW):"
			" reg_05h=%x, reg_31h=%x\n", reg_05h, reg_31h);
		pr_info("batt: Switch charger ON (SLOW) DEBUG:"
			" reg_00h=%x, reg_01h=%x\n", reg_00h, reg_01h);
		pr_info("batt: Switch charger ON (SLOW) DEBUG:"
			" reg_35h=%x, reg_36h=%x, reg_37h=%x\n"
			, reg_35h, reg_36h, reg_37h);
		break;
	case ENABLE_FAST_CHG:
		pr_info("Switch charger ON (FAST)\n");
		smb329_i2c_write_byte(0x84, 0x31);
		smb329_i2c_read_byte(&reg_31h, 0x31);
		smb329_i2c_write_byte(0xD0, 0x01);
		smb329_i2c_write_byte(0x08, 0x05);
		smb329_i2c_write_byte(0xA9, 0x00);
		smb329_i2c_read_byte(&reg_05h, 0x05);
		smb329_i2c_read_byte(&version, 0x3B);
		smb329_i2c_read_byte(&reg_00h, 0x00);
		smb329_i2c_read_byte(&reg_01h, 0x01);
		smb329_i2c_read_byte(&reg_35h, 0x35);
		smb329_i2c_read_byte(&reg_36h, 0x36);
		smb329_i2c_read_byte(&reg_37h, 0x37);
		pr_info("batt: Switch charger ON (FAST):"
			" reg_05h=%x, reg_31h=%x, version=%x\n"
			, reg_05h, reg_31h, version);
		pr_info("batt: Switch charger ON (FAST) DEBUG:"
			" reg_00h=%x, reg_01h=%x\n", reg_00h, reg_01h);
		pr_info("batt: Switch charger ON (FAST) DEBUG:"
			" reg_35h=%x, reg_36h=%x, reg_37h=%x\n"
			, reg_35h, reg_36h, reg_37h);
		break;
	case CHARGER_CHK:
		pr_info("batt: Switch charger CHECK\n");
		smb329_i2c_read_byte(&reg_00h, 0x00);
		smb329_i2c_read_byte(&reg_01h, 0x01);
		smb329_i2c_read_byte(&reg_05h, 0x05);
		smb329_i2c_read_byte(&reg_31h, 0x31);
		smb329_i2c_read_byte(&reg_33h, 0x33);
		smb329_i2c_read_byte(&reg_35h, 0x35);
		smb329_i2c_read_byte(&reg_36h, 0x36);
		smb329_i2c_read_byte(&reg_37h, 0x37);
		pr_info("batt: Switch charger ON (SLOW):"
			" reg_05h=%x, reg_31h=%x\n", reg_05h, reg_31h);
		pr_info("batt: Switch charger ON (SLOW) DEBUG:"
			" reg_00h=%x, reg_01h=%x, reg_33h=%x\n"
			, reg_00h, reg_01h, reg_33h);
		pr_info("batt: Switch charger ON (SLOW) DEBUG:"
			" reg_35h=%x, reg_36h=%x, reg_37h=%x\n"
			, reg_35h, reg_36h, reg_37h);
		break;
	default:
		pr_info("%s: Not supported battery ctr called.!", __func__);
		result = -EINVAL;
		break;
	}

	return result;
}
EXPORT_SYMBOL(set_charger_ctrl);
static int cable_status_handler_func(struct notifier_block *nfb,
		unsigned long action, void *param)
{
	u32 ctl = (u32)action;
	pr_info("Switch charger set control%d\n", ctl);
	set_charger_ctrl(ctl);

	return NOTIFY_OK;
}

static struct notifier_block cable_status_handler = {
	.notifier_call = cable_status_handler_func,
};

#if 0
static int smb329_detect(struct i2c_client *client, int kind,
			 struct i2c_board_info *info)
{
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
				     I2C_FUNC_SMBUS_BYTE))
		return -ENODEV;

	strlcpy(info->type, "smb329", I2C_NAME_SIZE);

	return 0;
}
#endif

static int smb329_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct smb329_i2c_client   *data = &smb329_i2c_module;

	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		dev_dbg(&client->dev, "[SMB329]:I2C fail\n");
		return -EIO;
	}
	if (machine_is_incrediblec() || (machine_is_supersonic()
	    && (system_rev < 1)))
		register_notifier_cable_status(&cable_status_handler);

	data->address = client->addr;
	data->client = client;
	mutex_init(&data->xfer_lock);
	pr_info("[SMB329]: Driver registration done\n");
	return 0;
}

static int smb329_remove(struct i2c_client *client)
{
	struct smb329_i2c_client   *data = i2c_get_clientdata(client);
	if (data->client && data->client != client)
		i2c_unregister_device(data->client);
	smb329_i2c_module.client = NULL;
	return 0;
}
static const struct i2c_device_id smb329_id[] = {
	{ "smb329", 0 },
	{  },
};
static struct i2c_driver smb329_driver = {
	.driver.name    = "smb329",
	.id_table   = smb329_id,
	.probe      = smb329_probe,
	.remove     = smb329_remove,
};

static int __init sensors_smb329_init(void)
{
    int res;

	res = i2c_add_driver(&smb329_driver);
	if (res) {
		pr_info("[SMB329]: Driver registration failed \n");
		return res;
		}
	return res;
}

static void __exit sensors_smb329_exit(void)
{
	i2c_del_driver(&smb329_driver);
}

MODULE_AUTHOR("Justin Lin <Justin_Lin@htc.com>");
MODULE_DESCRIPTION("smb329 driver");
MODULE_LICENSE("GPL");

module_init(sensors_smb329_init);
module_exit(sensors_smb329_exit);
