/* linux/arch/arm/mach-msm/board-bravoc-mmc.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>

#include <mach/vreg.h>

#include "board-bravoc.h"
#include "devices.h"
#include "proc_comm.h"

#include <mach/drv_callback.h>

#define DEBUG_SDSLOT_VDD 1

static bool opt_disable_sdcard;
static int __init bravoc_disablesdcard_setup(char *str)
{
	opt_disable_sdcard = (bool)simple_strtol(str, NULL, 0);
	return 1;
}

__setup("board_bravoc.disable_sdcard=", bravoc_disablesdcard_setup);

static uint32_t sdcard_on_gpio_table[] = {
	PCOM_GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
};

static uint32_t sdcard_off_gpio_table[] = {
	PCOM_GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(64, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(65, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(66, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(67, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
};

static uint32_t		sdslot_vdd = 0xffffffff;
static uint32_t		sdslot_vreg_enabled;

#if 0
static struct {
	int mask;
	int level;
} mmc_vdd_table[] = {
	{ MMC_VDD_28_29,	2850 },
	{ MMC_VDD_29_30,	2900 },
};
#endif

static uint32_t bravoc_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = (vdd == 0) ? 0 : 2850;

	if (vdd == 0) {
#if DEBUG_SDSLOT_VDD
		printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
#endif
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		gpio_set_value(100, 0); /* PWR OFF */
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		mdelay(5);
		gpio_set_value(100, 1); /* PWR ON */
		udelay(500);
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
	}

#if DEBUG_SDSLOT_VDD
	printk(KERN_INFO "%s: Setting level to %u\n", __func__, sdslot_vdd);
#endif
	return 0;

}

static unsigned int bravoc_sd_status;

static void (*sdslot_status_cb)(int card_present, void *dev_id);
static void *sdslot_status_cb_devid;

static int microp_check_status(int *st)
{
	bravoc_sd_status = !((unsigned) *st);

	if (sdslot_status_cb)
		sdslot_status_cb(bravoc_sd_status, sdslot_status_cb_devid);

	return 0;
}

static unsigned int bravoc_sdslot_status(struct device *dev)
{
	return 1;  /*bravoc_sd_status;*/
}

#define BRAVOC_MMC_VDD	(MMC_VDD_28_29 | MMC_VDD_29_30)

static int
bravoc_sdslot_register_cb(void (*callback)(int card_present, void *dev_id),
				void *dev_id)
{
	if (sdslot_status_cb)
		return -EAGAIN;

	sdslot_status_cb = callback;
	sdslot_status_cb_devid = dev_id;
	return 0;
}

static struct mmc_platform_data bravoc_sdslot_data = {
	.ocr_mask		= BRAVOC_MMC_VDD,
	.status			= bravoc_sdslot_status,
	.translate_vdd		= bravoc_sdslot_switchvdd,
	.register_status_notify = bravoc_sdslot_register_cb,
};

int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
		 unsigned int stat_irq, unsigned long stat_irq_flags);



/* ---- WIFI ---- */

static uint32_t wifi_on_gpio_table[] = {
	PCOM_GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(152, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] = {
	PCOM_GPIO_CFG(51, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(52, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(53, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(54, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(55, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(56, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(152, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* WLAN IRQ */
};

/* BCM4329 returns wrong sdio_vsn(1) when we read cccr,
 * we use predefined value (sdio_vsn=2) here to initial sdio driver well
 */
static struct embedded_sdio_data bravoc_wifi_emb_data = {
	.cccr	= {
		.sdio_vsn	= 2,
		.multi_block	= 1,
		.low_speed	= 0,
		.wide_bus	= 0,
		.high_power	= 1,
		.high_speed	= 1,
	},
};

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int
bravoc_wifi_status_register(void (*callback)(int card_present, void *dev_id),
				void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int bravoc_wifi_cd;	/* WiFi virtual 'card detect' status */

static unsigned int bravoc_wifi_status(struct device *dev)
{
	return bravoc_wifi_cd;
}

static struct mmc_platform_data bravoc_wifi_data = {
	.ocr_mask		= MMC_VDD_28_29,
	.status			= bravoc_wifi_status,
	.register_status_notify	= bravoc_wifi_status_register,
	.embedded_sdio		= &bravoc_wifi_emb_data,
};

int bravoc_wifi_set_carddetect(int val)
{
	printk(KERN_INFO "%s: %d\n", __func__, val);
	bravoc_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(bravoc_wifi_set_carddetect);

int bravoc_wifi_power(int on)
{
	int rc = 0;

	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table,
				  ARRAY_SIZE(wifi_on_gpio_table));
		mdelay(50);
		if (rc)
			return rc;
	} else {
		config_gpio_table(wifi_off_gpio_table,
				  ARRAY_SIZE(wifi_off_gpio_table));
	}

	mdelay(100);
	gpio_set_value(127, on); /* WIFI_SHUTDOWN */
	mdelay(100);
	return 0;
}
EXPORT_SYMBOL(bravoc_wifi_power);

int bravoc_wifi_reset(int on)
{
	printk(KERN_INFO "%s: do nothing\n", __func__);
	return 0;
}

int __init bravoc_init_mmc(unsigned int sys_rev)
{
	uint32_t id;
	struct cnf_driver *microp_sdcard_detect;

	wifi_status_cb = NULL;

	printk(KERN_INFO "%s()+\n", __func__);

	/* initial WIFI_SHUTDOWN# */
	id = PCOM_GPIO_CFG(127, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	gpio_set_value(127, 0);

	msm_add_sdcc(1, &bravoc_wifi_data, 0, 0);

	if (opt_disable_sdcard) {
		pr_info("%s: sdcard disabled on cmdline\n", __func__);
		goto done;
	}

	sdslot_vreg_enabled = 0;

	microp_sdcard_detect = kzalloc(sizeof(struct cnf_driver), GFP_KERNEL);
	if (microp_sdcard_detect) {
		microp_sdcard_detect->name = "sdcard_detect";
		microp_sdcard_detect->func = (void *) &microp_check_status;
	} else {
		printk(KERN_ERR "%s: Alloc SD callback func error\n", __func__);
		goto done;
	}
	cnf_driver_register(microp_sdcard_detect);

	msm_add_sdcc(2, &bravoc_sdslot_data, 0, 0);

done:
	printk(KERN_INFO "%s()-\n", __func__);
	return 0;
}
