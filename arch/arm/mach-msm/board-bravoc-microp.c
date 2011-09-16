/* arch/arm/mach-msm/board-bravoc-microp.c
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
#ifdef CONFIG_MICROP_COMMON
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <mach/atmega_microp.h>
#include <mach/drv_callback.h>
#include <mach/htc_headset_mgr.h>

#include "board-bravoc.h"

#ifdef CONFIG_HTC_HEADSET
#define notify_35mm_headset_insert(insert) \
	htc_35mm_remote_notify_insert_ext_headset(insert)
#else
#define notify_35mm_headset_insert(insert) do {} while (0)
#endif

static int bravoc_microp_function_init(struct i2c_client *client)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;
	uint8_t data[20];
	int i, j;
	int ret;

	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

	/* Headset remote key */
	ret = microp_function_check(client, MICROP_FUNCTION_REMOTEKEY);
	if (ret >= 0) {
		i = ret;
		pdata->function_node[MICROP_FUNCTION_REMOTEKEY] = i;
		cdata->int_pin.int_remotekey =
			pdata->microp_function[i].int_pin;

		for (j = 0; j < 6; j++) {
			data[j] = (uint8_t)(pdata->microp_function[i].levels[j] >> 8);
			data[j + 6] = (uint8_t)(pdata->microp_function[i].levels[j]);
		}
		ret = microp_i2c_write(MICROP_I2C_WCMD_REMOTEKEY_TABLE,
				data, 12);
		if (ret)
			goto exit;
	}

	/* OJ interrupt */
	ret = microp_function_check(client, MICROP_FUNCTION_OJ);
	if (ret >= 0) {
		i = ret;
		cdata->int_pin.int_oj = pdata->microp_function[i].int_pin;

		ret = microp_write_interrupt(client, cdata->int_pin.int_oj, 1);
		if (ret)
			goto exit;
	}

	/* Headset plugin */
	ret = microp_function_check(client, MICROP_FUNCTION_HPIN);
	if (ret >= 0) {
		i = ret;
		cdata->int_pin.int_hpin = pdata->microp_function[i].int_pin;
		cdata->gpio.hpin = pdata->microp_function[i].mask_r[0] << 16
					| pdata->microp_function[i].mask_r[1] << 8
					| pdata->microp_function[i].mask_r[2];

		wake_lock_init(&cdata->hpin_wake_lock,
					WAKE_LOCK_SUSPEND, "microp_35mm_hpin");

		microp_read_gpio_status(data);
		cdata->headset_is_in =
				!((data[0] << 16 | data[1] << 8 | data[2])
				& cdata->gpio.hpin);
		if (cdata->headset_is_in)
			notify_35mm_headset_insert(cdata->headset_is_in);

		ret = microp_write_interrupt(client,
				cdata->int_pin.int_hpin, 1);
		if (ret)
			goto exit;
	}

	/* Reset button interrupt */
	data[0] = 0x08;
	ret = microp_i2c_write(MICROP_I2C_WCMD_MISC, data, 1);
	if (ret)
		goto exit;

	/* SD Card detect */
	ret = microp_function_check(client, MICROP_FUNCTION_SDCARD);
	if (ret >= 0) {
		i = ret;
		cdata->int_pin.int_sdcard = pdata->microp_function[i].int_pin;
		cdata->gpio.sdcard = pdata->microp_function[i].mask_r[0] << 16
					| pdata->microp_function[i].mask_r[1] << 8
					| pdata->microp_function[i].mask_r[2];

		microp_read_gpio_status(data);
		cdata->sdcard_is_in = ((data[0] << 16 | data[1] << 8 | data[2])
						& cdata->gpio.sdcard) ? 1 : 0;
		cnf_driver_event("sdcard_detect", &cdata->sdcard_is_in);

		ret = microp_write_interrupt(client,
				cdata->int_pin.int_sdcard, 1);
		if (ret)
			goto exit;
	}

	return 0;

exit:
	return ret;
}

static struct microp_ops ops = {
	.init_microp_func = bravoc_microp_function_init,
};

void __init bravoc_microp_init(void)
{
	microp_register_ops(&ops);
}

#endif
