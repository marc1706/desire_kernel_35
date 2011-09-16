/* arch/arm/mach-msm/board-bravoc-keypad.c
 *
 * Copyright (C) 2009 Google, Inc
 * Copyright (C) 2009 HTC Corporation.
 *
 * Author: Dima Zavin <dima@android.com>
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

#include <linux/gpio_event.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/keyreset.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>

#include "proc_comm.h"
#include "board-bravoc.h"

struct jog_axis_info {
	struct gpio_event_axis_info	info;
	uint16_t			in_state;
	uint16_t			out_state;
};

static unsigned int bravoc_col_gpios[] = {
	BRAVOC_GPIO_KP_MKOUT0,
	BRAVOC_GPIO_KP_MKOUT1,
	BRAVOC_GPIO_KP_MKOUT2,
};
static unsigned int bravoc_row_gpios[] = {
	BRAVOC_GPIO_KP_MPIN0,
	BRAVOC_GPIO_KP_MPIN1,
	BRAVOC_GPIO_KP_MPIN2,
};

#define KEYMAP_INDEX(col, row)	((col)*ARRAY_SIZE(bravoc_row_gpios) + (row))
#define KEYMAP_SIZE		(ARRAY_SIZE(bravoc_col_gpios) * \
				 ARRAY_SIZE(bravoc_row_gpios))

/* keypad */
static const unsigned short bravoc_keymap[KEYMAP_SIZE] = {
	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(1, 1)] = BTN_MOUSE, /* OJ Action key */
	[KEYMAP_INDEX(1, 0)] = KEY_MENU,
	[KEYMAP_INDEX(1, 2)] = KEY_SEARCH,
	[KEYMAP_INDEX(2, 0)] = KEY_HOME,
	[KEYMAP_INDEX(2, 2)] = KEY_BACK,
};

static struct gpio_event_matrix_info bravoc_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.info.oj_btn = true,
	.keymap = bravoc_keymap,
	.output_gpios = bravoc_col_gpios,
	.input_gpios = bravoc_row_gpios,
	.noutputs = ARRAY_SIZE(bravoc_col_gpios),
	.ninputs = ARRAY_SIZE(bravoc_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,
	.flags = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		  GPIOKPF_REMOVE_PHANTOM_KEYS |
		  GPIOKPF_PRINT_UNMAPPED_KEYS),
	.detect_phone_status = 1,
};

static struct gpio_event_direct_entry bravoc_keypad_key_map[] = {
	{
		.gpio	= BRAVOC_GPIO_POWER_KEY,
		.code	= KEY_POWER,
	},
};

static struct gpio_event_input_info bravoc_keypad_key_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.flags = GPIOEDF_PRINT_KEYS,
	.type = EV_KEY,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.keymap = bravoc_keypad_key_map,
	.keymap_size = ARRAY_SIZE(bravoc_keypad_key_map)
};

static struct gpio_event_info *bravoc_input_info[] = {
	&bravoc_keypad_key_info.info,
	&bravoc_keypad_matrix_info.info,
};

int bravoc_gpio_event_power(const struct gpio_event_platform_data *pdata, bool on){
	return 0;
}

static struct gpio_event_platform_data bravoc_input_data = {
	.names = {
		"bravoc-keypad",
		NULL,
	},
	.info = bravoc_input_info,
	.info_count = ARRAY_SIZE(bravoc_input_info),
	.power = bravoc_gpio_event_power,
};

static struct platform_device bravoc_input_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &bravoc_input_data,
	},
};

static int bravoc_reset_keys_up[] = {
	KEY_VOLUMEUP,
	0,
};

static struct keyreset_platform_data bravoc_reset_keys_pdata = {
	.keys_up	= bravoc_reset_keys_up,
	.keys_down	= {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		BTN_MOUSE,
		0
	},
};

struct platform_device bravoc_reset_keys_device = {
	.name	= KEYRESET_NAME,
	.dev	= {
		.platform_data = &bravoc_reset_keys_pdata,
	},
};

static void bravoc_reset_btn_initialize(void)
{
	static uint32_t reset_btn_setup[] = {
		PCOM_GPIO_CFG(BRAVOC_GPIO_RESET_BTN, 0, GPIO_INPUT,
						GPIO_NO_PULL, GPIO_4MA),
	};
	config_gpio_table(reset_btn_setup, ARRAY_SIZE(reset_btn_setup));

}

static int __init bravoc_init_keypad_jogball(void)
{
	int ret;

	if (!machine_is_bravoc())
		return 0;

	ret = platform_device_register(&bravoc_reset_keys_device);
	if (ret != 0)
		return ret;

	bravoc_reset_btn_initialize();

	if (BOOTMODE_POWERTEST == board_mfg_mode())
		((struct gpio_event_platform_data *)(bravoc_input_device.dev.platform_data))->info_count = 1;

	ret = platform_device_register(&bravoc_input_device);
	if (ret != 0)
		return ret;

	return 0;
}

device_initcall(bravoc_init_keypad_jogball);
