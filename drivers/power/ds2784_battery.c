/* drivers/power/ds2784_battery.c
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
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/android_alarm.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/ds2784_battery.h>
#include <linux/smb329.h>
#include <mach/htc_battery.h>
#include <asm/mach-types.h>
#include "../../arch/arm/mach-msm/proc_comm.h"

#include "../w1/w1.h"
#include "../w1/slaves/w1_ds2784.h"
#include <linux/time.h>
#include <linux/rtc.h>

struct battery_info {
	u8 batt_id;		/* Battery ID from ADC */
	int batt_vol;		/* Battery voltage from ADC */
	int batt_temp;		/* Battery Temperature (C) from formula and ADC */
	int batt_current;	/* Battery current from ADC */
	int batt_current_avg;
	u8 level;		/* formula */
	u8 level_last;
	u8 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u8 charging_enabled;	/* 0: Disable, 1: Slow charge,
							2: Fast charge*/
	u32 full_bat;		/* Full capacity of battery (mAh) */
	u32 full_level;		/* Full level for battery control */
	u8  guage_status_reg;/* guage status*/
	u32 acr;
	u32 active_empty;
	u8 battery_full;
	u8 cooldown;		/* was overtemp */
	u8 last_charge_mode; /* previous charger state */
	u8 charge_mode;
	u8 OT_critical;
	u8 OT_hot;
	u8 OT_warm;
	u8 OT_cold;
	u8 full_acr;
};
struct ds2784_device_info {
	struct device *dev;
#if 0
	struct power_supply bat;
#endif
	struct device *w1_dev;
	struct workqueue_struct *monitor_wqueue;
	struct work_struct monitor_work;
	struct battery_info rep;

	/* lock to protect the battery info */
	struct mutex lock;

	/* DS2784 data, valid after calling ds2784_battery_read_status() */
	unsigned long update_time;	/* jiffies when data read */
	char raw[DS2784_DATA_SIZE];	/* raw DS2784 data */
	int voltage_mV;			/* units of mV */
	int current_mA;			/* units of mA */
	int current_avg_mA;		/* unit of avg mA */
	int temp_C;				/* units of 0.1 C */
	int charge_status;		/* POWER_SUPPLY_STATUS_* */
	int percentage;			/* battery percentage */
	int guage_status_reg;	/* battery status register offset=01h*/
	int full_mah;			/* battery full mah */
    long full_charge_count;  /* Full charge counter */
	int acr;
	int active_empty;
	int full_level;		/* Full level for battery control */
	struct alarm alarm;
	struct wake_lock work_wake_lock;
	u8 slow_poll;
	ktime_t last_poll;
};
static struct wake_lock vbus_wake_lock;
static unsigned int fake_temp = 0x00;
static unsigned int emc_debug = 0x00;

/* Battery ID
PS. 0 or other battery ID use the same parameters*/
#define BATT_NO_SOURCE        (0)  /* 0: No source battery */
#define BATT_FIRST_SOURCE     (1)  /* 1: Main source battery */
#define BATT_SECOND_SOURCE    (2)  /* 2: Second source battery */
#define BATT_THIRD_SOURCE     (3)  /* 3: Third source battery */
#define BATT_FOURTH_SOURCE    (4)  /* 4: Fourth source battery */
#define BATT_FIFTH_SOURCE     (5)  /* 5: Fifth source battery */
#define BATT_UNKNOWN        (255)  /* Other: Unknown battery */

#define BATT_RSNSP			(67)	/*Passion battery source 1*/

#define BATT_CHECK_TIME_CHARGING_FULL  (3600)
/* Battery Parameter */
/* Battery ID = 1: HT-E/Formosa 1400mAh */
#define BATT_ID_A				1
#define BATT_FULL_MAH_A			1400

#define BATT_FULL_MAH_DEFAULT	1500

/* Battery OVP and Charging stage define */
#define BATT_GOOD_STATE			0
#define BATT_OTP_BIT0			1<<0	/*45~50*/
#define BATT_OTP_BIT1			1<<1	/*50~60*/
#define BATT_OTP_BIT2			1<<2	/*60~*/
#define BATT_OTP_BIT3			1<<3	/*~0*/

#define BATT_NON_SLEEP_WAKEUP_INTERVAL	60
#define BATT_SLEEP_WAKEUP_INTERVAL		600
#define BATT_SLOPE_UNIT		61	/* 6Ch = 01h --> 61ppm */

//#ifdef CONFIG_BATTERY_DS2784 && CONFIG_SMB329
//#define BATT_HIGH_TEMP			580
//#define BATT_HIGH_TEMP_SPARE	550
//#define BATT_MID_TEMP			480
//#define BATT_MID_TEMP_SPARE	450
//#else
#define BATT_HIGH_TEMP			600
#define BATT_HIGH_TEMP_SPARE	570
#define BATT_MID_TEMP			500
#define BATT_MID_TEMP_SPARE	470
//#endif
static struct ds2784_device_info htc_batt_info;
static int htc_battery_initial;
static int Full_40;
static int Full_30;
static int Full_20;
static int Full_10;
static int Full_0;

#define SOURCE_NONE	0
#define SOURCE_USB	1
#define SOURCE_AC	2

#define CHARGE_OFF	0
#define CHARGE_SLOW	1
#define CHARGE_FAST	2

#define TEMP_CRITICAL	600 /* no charging at all */
#define TEMP_HOT	500 /* no fast charge, no charge > 4.1v */
#define TEMP_WARM	450 /* no fast charge above this */

#define TEMP_HOT_MAX_MV	4100 /* stop charging here when hot */
#define TEMP_HOT_MIN_MV	3800 /* resume charging here when hot */

#define TEMP_CRITICAL_BC	    570 /* no charging at all */
#define TEMP_CRITICAL_RECHG_BC  550 /* recharge at 1.temp <= 55C and 2. Vb < 3.8V */
#define TEMP_HOT_BC	        470 /* no fast charge, no charge > 4v */
#define TEMP_HOT_RECHG_BC	450 /* recharge at 1.temp < 45C or 2. Vb < 3.8V */
#define TEMP_WARM_BC	    450 /* no fast charge above this */
#define TEMP_WARM_HR_BC  420    /* hysteresis: temp <= 42C */
#define TEMP_COLD_BC     0
#define TEMP_COLD_HR_BC  30

#define TEMP_HOT_MAX_MV_BC	4000 /* stop charging here when hot */
#define TEMP_HOT_MIN_MV_BC	3800 /* resume charging here when hot */

/*AEL for low temperature */
#define AEL_0	60
#define AEL_5	20
#define AEL_10	10
#define AEL_15	0

/* When we're awake or running on wall power, sample the battery
 * gauge every FAST_POLL seconds.  If we're asleep and on battery
 * power, sample every SLOW_POLL seconds
 */
#define FAST_POLL	(1 * 60)
#define SLOW_POLL	(10 * 60)
#define Onehr_POLL	(60 * 60)

static BLOCKING_NOTIFIER_HEAD(ds2784_notifier_list);
int ds2784_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ds2784_notifier_list, nb);
}

int ds2784_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ds2784_notifier_list, nb);
}

static int ds2784_blocking_notify(unsigned long val, void *v)
{
	int chg_ctl;

	if (val == DS2784_CHARGING_CONTROL) {
		chg_ctl = *(int *)v;
		if (htc_batt_info.rep.batt_id == BATT_UNKNOWN) {
			htc_batt_info.rep.charging_enabled = DISABLE;
			chg_ctl = DISABLE;
			v = DISABLE;
			pr_info("[HTC_BATT] Unknow battery\n");
		}

		if (machine_is_passionc() || machine_is_bravoc()) {
			pr_info("[HTC_BATT] Switch charging %d\n",chg_ctl);
			if (chg_ctl <= 2){
				gpio_direction_output(22, !(!!chg_ctl));//PNC
				set_charger_ctrl(chg_ctl);
				htc_batt_info.rep.charging_enabled = chg_ctl;
				htc_batt_info.rep.last_charge_mode = chg_ctl;
			}
			return 0;
		} else if (htc_batt_info.rep.batt_id != BATT_UNKNOWN) {
			/* only notify at changes */
			if (htc_batt_info.rep.charging_enabled == chg_ctl)
				return 0;
			else
				htc_batt_info.rep.charging_enabled = chg_ctl;
		}
	}
	return blocking_notifier_call_chain(&ds2784_notifier_list, val, v);

}

int ds2784_get_battery_info(struct battery_info_reply *batt_info)
{
	batt_info->batt_id = htc_batt_info.rep.batt_id;
	batt_info->batt_vol = htc_batt_info.rep.batt_vol;
	batt_info->batt_temp = htc_batt_info.rep.batt_temp;
	batt_info->batt_current = htc_batt_info.rep.batt_current;
	batt_info->level = htc_batt_info.rep.level;
	batt_info->charging_source = htc_batt_info.rep.charging_source;
	batt_info->charging_enabled = htc_batt_info.rep.charging_enabled;
	batt_info->full_bat = htc_batt_info.rep.acr;
/* DS2784 did not support this over vchg, but we need to have reply */
	batt_info->over_vchg = 0;
		return 0;
}
ssize_t htc_battery_show_attr(struct device_attribute *attr,
					 char *buf)
{
	int len = 0;

	if (!strcmp(attr->attr.name, "batt_attr_text")) {
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"batt_id: %d;\n"
		"batt_vol(mV): %d;\n"
		"batt_temp(C): %d;\n"
		"batt_current(mA): %d;\n"
		"batt_current_avg(mA): %d;\n"
		"level(%%): %d;\n"
		"charging_source: %d;\n"
		"charging_enabled: %d;\n"
		"acr(mAh): %d;\n"
		"active_empty(mAh): %d;\n"
		"guage_status_reg: %x;\n"
		"full_level(%%): %d;\n"
		"fake_temp: %d;\n",
		htc_batt_info.rep.batt_id,
		htc_batt_info.rep.batt_vol,
		htc_batt_info.rep.batt_temp,
		htc_batt_info.rep.batt_current,
		htc_batt_info.rep.batt_current_avg,
		htc_batt_info.rep.level,
		htc_batt_info.rep.charging_source,
		htc_batt_info.rep.charging_enabled,
		htc_batt_info.rep.acr,
		htc_batt_info.rep.active_empty,
		htc_batt_info.rep.guage_status_reg,
		htc_batt_info.rep.full_level,
		fake_temp
		);
	}
	return len;
}

static int cable_status_handler_func(struct notifier_block *nfb,
		unsigned long action, void *param)
{
	u32 cable_type = (u32)action;
	pr_info("[HTC_BATT] cable change to %d\n", cable_type);
/* When the cable plug out, reset all the related flag,
Let algorithm machine to judge latest state */
	if (cable_type == 0){
		htc_batt_info.full_charge_count = 0;
		htc_batt_info.rep.full_acr = 0;
		htc_batt_info.rep.last_charge_mode = 0;
		htc_batt_info.rep.battery_full = 0;
	}else if (cable_type == 0xff){
		if (param)
			htc_batt_info.rep.full_level = *(u32 *)param;
		return NOTIFY_OK;
	}else if (cable_type == 0x10){
		fake_temp = 0x01;
		return NOTIFY_OK;
	}else if (cable_type == 0x11){
		emc_debug = 0x01;
		return NOTIFY_OK;
	}

	if (cable_type <= 2) {
		htc_batt_info.rep.charging_source = cable_type;
		ds2784_blocking_notify(DS2784_CHARGING_CONTROL,
			(void*)&cable_type);
		return NOTIFY_OK;
	}
	return NOTIFY_OK;
}

static struct notifier_block cable_status_handler = {
	.notifier_call = cable_status_handler_func,
};
/*
Function:More_Charge_Extend_Time
Target:	One more hour charge .
Timing:	30 second.
return :TRUE-->one hour go pass
		FALSE-->not yet
*/
static int More_Charge_Extend_Time(void)
{
	struct timespec now;
	getnstimeofday(&now);
		if (htc_batt_info.full_charge_count == 0)
			htc_batt_info.full_charge_count = now.tv_sec;
		else if ((now.tv_sec - htc_batt_info.full_charge_count)
				>= BATT_CHECK_TIME_CHARGING_FULL){
				htc_batt_info.full_charge_count = 0;
				return 1;
		}
		return 0;
	}
#if 0
/*
Function:Read_Gauge_Offset
Target:	Read battery information with offset value
Timing:	30 sec timer
return :0 -->Read fail
		1 -->Read OK
*/
static int Read_Gauge_Offset(struct ds2784_device_info *di,int OFFSET)
{
	int ret, start, count;

	start = OFFSET;
	count = 1;

	mutex_lock(&htc_batt_info.lock);
	ret = w1_ds2784_read(di->w1_dev, di->raw + start, start, count);
		if (ret != count) {
			ret = w1_ds2784_read(di->w1_dev, di->raw + start, start, count);
			pr_info("[HTC_BATT] Read Offset %d again\n",OFFSET);
}
	mutex_unlock(&htc_batt_info.lock);
	if (ret != count) {
		pr_info("[HTC_BATT] Read_Gauge_Offset failed offset %d\n",OFFSET);
		return 0;
}
	return 1;
}
#endif
/*
Function:Calculate_full_mAh
Target:	Calculate the mapping full capacity of temperature.
Timing:	30 sec timer
return : Value of Full capacity

*/
static int Calculate_Full_mAh(struct ds2784_device_info *di)
{
	int result;
	int Full_mAh;
	int Upper_Full = 0;
	int Lower_Full = 0;
	int Delta_temp = 0;

	Full_mAh = BATT_FULL_MAH_DEFAULT;

/* Only calculate at initial */
	if (htc_battery_initial == 0) {
		result = ((di->raw[DS2784_REG_FULL_40_MSB])<<8)|
				(di->raw[DS2784_REG_FULL_40_LSB]);
		result = result *625*67/100000;
	Full_40 = result;
	Full_30 = (Full_40*10-(di->raw[DS2784_REG_FULL_SEG_4_SLOPE]*
			Full_40*BATT_SLOPE_UNIT/10000))/10;
	Full_20 = (Full_30*10-(di->raw[DS2784_REG_FULL_SEG_3_SLOPE]*
			Full_30*BATT_SLOPE_UNIT/10000))/10;
	Full_10 = (Full_20*10-(di->raw[DS2784_REG_FULL_SEG_2_SLOPE]*
			Full_20*BATT_SLOPE_UNIT/10000))/10;
	Full_0 = (Full_10*10-(di->raw[DS2784_REG_FULL_SEG_1_SLOPE]*
			Full_10*BATT_SLOPE_UNIT/10000))/10;
	}
/* Mapping to temperature */
	if ( htc_batt_info.rep.batt_temp >= 400 ) {
		Upper_Full=Full_40;Lower_Full=Full_40;
		Delta_temp=0;}
	else if (( htc_batt_info.rep.batt_temp >= 300 ) &&
		( htc_batt_info.rep.batt_temp < 400 )) {
		Upper_Full=Full_40;Lower_Full=Full_30;
		Delta_temp= 400-htc_batt_info.rep.batt_temp;}
	else if (( htc_batt_info.rep.batt_temp >= 200 ) &&
		( htc_batt_info.rep.batt_temp < 300 )) {
		Upper_Full=Full_30;Lower_Full=Full_20;
		Delta_temp= 300-htc_batt_info.rep.batt_temp;}
	else if (( htc_batt_info.rep.batt_temp >= 100 ) &&
		( htc_batt_info.rep.batt_temp < 200 )) {
		Upper_Full=Full_20;Lower_Full=Full_10;
		Delta_temp= 200-htc_batt_info.rep.batt_temp;}
	else if (( htc_batt_info.rep.batt_temp >= 0 ) &&
		( htc_batt_info.rep.batt_temp < 100 )) {
		Upper_Full=Full_10;Lower_Full=Full_0;
		Delta_temp= 100-htc_batt_info.rep.batt_temp;}

	Full_mAh = Upper_Full - (Upper_Full-Lower_Full)*Delta_temp/100;
	return Full_mAh;
}

static int ds2784_battery_read_status(struct ds2784_device_info *di)
{
	short result;
	int ret, start, count;

	/* The first time we read the entire contents of SRAM/EEPROM,
	 * but after that we just read the interesting bits that change. */
	if (htc_battery_initial == 0) {
		start = 0;
		count = DS2784_DATA_SIZE;
	} else {
		start = DS2784_REG_PORT;
		count = DS2784_REG_STBY_EMPTY_LSB - start + 1;
	}

	ret = w1_ds2784_read(di->w1_dev, di->raw + start, start, count);
	if (ret < 0) {
		pr_info("batt:abort this time to read gauge, ret = %d\n",ret);
		return 1;
	}
	if (ret != count) {
		dev_warn(di->dev, "call to w1_ds2784_read failed (0x%p)\n",
			 di->w1_dev);
		return 1;
	}
/*
Check if dummy battery in.
Workaround for dummy battery
Write ACR MSB to 0x05, ensure there must be 500mAH .
ONLY check when battery driver init.
*/
	if (htc_battery_initial == 0) {
		if (!memcmp(di->raw + 0x20, "DUMMY!", 6)) {
			unsigned char acr[2];

			pr_info("batt: dummy battery detected\n");

			/* reset ACC register to ~500mAh, since it may have zeroed out */
			acr[0] = 0x05;
			acr[1] = 0x06;
			mutex_lock(&htc_batt_info.lock);
			ret = w1_ds2784_write(di->w1_dev, acr,DS2784_REG_ACCUMULATE_CURR_MSB, 2);
			if (ret < 0) {
				msleep(5);
				if (w1_ds2784_write(di->w1_dev, acr,DS2784_REG_ACCUMULATE_CURR_MSB, 2) < 0)
					pr_info("batt: Write dummy ACR fail, ret = %d\n",ret);
			}
			mutex_unlock(&htc_batt_info.lock);
		}
	}
/*
Get Rsns, get from offset 69H . Rsnsp=1/Rsns
Judge if this is supported battery
*/
	mutex_lock(&htc_batt_info.lock);
	if (di->raw[DS2784_REG_RSNSP] != BATT_RSNSP)
		htc_batt_info.rep.batt_id = BATT_UNKNOWN;
	else
		htc_batt_info.rep.batt_id = BATT_FIRST_SOURCE;
	mutex_unlock(&htc_batt_info.lock);

/*
Get status reg
*/
	mutex_lock(&htc_batt_info.lock);
	di->guage_status_reg= di->raw[DS2784_REG_STS];
/*
Get Level
*/
	di->percentage = di->raw[DS2784_REG_RARC];

/*
Get Voltage
Unit=4.886mV, range is 0V to 4.99V
*/
	di->voltage_mV = (((di->raw[DS2784_REG_VOLT_MSB]<<8)
			|(di->raw[DS2784_REG_VOLT_LSB])) >> 5)*4886/1000;

/*
Get Current
Unit= 1.5625uV x Rsnsp(67)=104.68
*/
	result = ((di->raw[DS2784_REG_CURR_MSB]) << 8) |
		di->raw[DS2784_REG_CURR_LSB];
	di->current_mA = (((result * 15625) / 10000) * 67)/1000;

	result = ((di->raw[DS2784_REG_AVG_CURR_MSB]) << 8) |
		di->raw[DS2784_REG_AVG_CURR_LSB];
	di->current_avg_mA = (((result * 15625) / 10000) * 67)/1000;

/*
Get Temperature
Unit=0.125 degree C,therefore, give up LSB ,
just caculate MSB for temperature only.
*/
	result = (((signed char)di->raw[DS2784_REG_TEMP_MSB]) << 3) |
				     (di->raw[DS2784_REG_TEMP_LSB] >> 5);

	if (fake_temp)
		di->temp_C = 30;
	else
		di->temp_C = result + (result / 4);

/*
Get ACR and Active Empty
*/
	result = ((di->raw[DS2784_REG_ACTIVE_EMPTY_MSB]<<8)|
		(di->raw[DS2784_REG_ACTIVE_EMPTY_LSB]));
	di->active_empty = ((result*625/100)*67)/1000;

	/* RAAC is in units of 1.6mAh */
	di->acr = (((di->raw[DS2784_REG_RAAC_MSB] << 8) |
			  di->raw[DS2784_REG_RAAC_LSB]) * 1600)/1000;

/*
Set to local for reply to framework
*/
	htc_batt_info.rep.batt_current = di->current_mA;
	htc_batt_info.rep.batt_current_avg = di->current_avg_mA;
	htc_batt_info.rep.batt_temp = di->temp_C;
	htc_batt_info.rep.batt_vol = di->voltage_mV;
	htc_batt_info.rep.level_last = htc_batt_info.rep.level;
	htc_batt_info.rep.level = di->percentage;
	htc_batt_info.rep.guage_status_reg = di->guage_status_reg;
	htc_batt_info.rep.acr = di->acr;
	htc_batt_info.rep.active_empty = di->active_empty;
	htc_batt_info.rep.full_bat = Calculate_Full_mAh(di);


	/* After battery driver gets initialized, send rpc request to inquiry
	 * the battery status in case of we lost some info
	 */
	if (htc_battery_initial == 0)
	{
		htc_batt_info.full_charge_count = 0;
		htc_battery_initial = 1;
		htc_batt_info.rep.full_level = 100;
	}
pr_info("[HTC_BATT]RSNSP=%d,RARC=%d,Vol=%dmV,Current=%dmA,Temp=%dC(1/10)\n"
		,di->raw[DS2784_REG_RSNSP]
		,di->raw[DS2784_REG_RARC]
		,di->voltage_mV
		,di->current_mA
		,di->temp_C);
	mutex_unlock(&htc_batt_info.lock);
	return 0;
}

#if 0
/*
Function:Chk_charging_sts
Target:	Check charging full and recharging condition.
Timing:	30 second, cable in/out.
*/
static void Chk_Charging_Sts(u32 Full_Vol, u32 Stop_Vol)
{
	int chg_ctl = DISABLE;
	mutex_lock(&htc_batt_info.lock);
if (htc_batt_info.rep.full_level == 100) {
	if (Stop_Vol == 0)/* Could charge full */
	{
		if ((htc_batt_info.rep.batt_current < 80)&&
				(htc_batt_info.rep.charging_sts_flag != 0x03) &&
				(htc_batt_info.rep.level == 100) &&
				((htc_batt_info.rep.guage_status_reg & 0x80) == 0x80)) {
				if (htc_batt_info.rep.charging_sts_flag == 0) {
					if (More_Charge_Extend_Time()) {
				htc_batt_info.rep.charging_sts_flag = 0x01;
				ds2784_blocking_notify(DS2784_CHARGING_CONTROL,
				&chg_ctl);
				pr_info("[HTC_BATT] Charing Full\n");
					}
				}
		}
		else if ((htc_batt_info.rep.charging_sts_flag == 0x03)&&/*Rechg full?*/
				(htc_batt_info.rep.batt_current < 80) &&
				((htc_batt_info.rep.guage_status_reg&0x80) == 0x80)) {
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL, &chg_ctl);
			htc_batt_info.rep.charging_sts_flag = 0x09;/*Full/Recharging Full*/
			pr_info("[HTC_BATT] Recharging Full\n");
		}
		else if ((htc_batt_info.rep.level <= 99)&&/*Recharging*/
				 ((htc_batt_info.rep.charging_sts_flag&0x01) == 0x01)) {
			/* Recharging type? slow or fast? */
			if (htc_batt_info.rep.charging_sts_flag != 0x03)
				ds2784_blocking_notify(DS2784_CHARGING_CONTROL,
				&htc_batt_info.rep.charging_enabled);
			htc_batt_info.rep.charging_sts_flag = 0x03;/*Recharging*/
		} else if ((htc_batt_info.rep.level != 100) &&/* Write ACR full? */
		(htc_batt_info.rep.batt_current < 80) &&
			((htc_batt_info.rep.guage_status_reg&0x80) == 0x80)) {
			htc_batt_info.rep.charging_sts_flag = 0x11;
			}
	}
	else /*Can not charge full*/
	{
		if (htc_batt_info.rep.batt_vol >= Full_Vol) {
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL, &chg_ctl);
			htc_batt_info.rep.charging_sts_flag = 0x04;/*Charging pending*/
			pr_info("[HTC_BATT] Charing Pending\n");
		}
		else if ((htc_batt_info.rep.charging_sts_flag == 0x02)&&
				(htc_batt_info.rep.batt_vol >= Full_Vol)){
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL, &chg_ctl);
			htc_batt_info.rep.charging_sts_flag = 0x04;/*Charging pending*/
			pr_info("[HTC_BATT] ReCharing Pending\n");
		}
		else if ((htc_batt_info.rep.batt_vol < Full_Vol)&&
				 (htc_batt_info.rep.batt_vol >= Stop_Vol)&&
				 (htc_batt_info.rep.charging_sts_flag == 0x04)) {
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL, &htc_batt_info.rep.charging_enabled);
			htc_batt_info.rep.charging_sts_flag = 0x02;/*Recharging*/
		}
	}
} else if (htc_batt_info.rep.full_level < 100) { 
/* For MFG battery control only */
	if (htc_batt_info.rep.level >= htc_batt_info.rep.full_level) {
		chg_ctl = DISABLE;
		if (machine_is_passionc()) {
			pr_info("[HTC_BATT] Set full charge ,Stop charge %d\n",chg_ctl);
			gpio_direction_output(22, 1);//PNC //disbale charge
			set_charger_ctrl(chg_ctl);
		}else {
			battery_charging_ctrl(DISABLE);
		}
		htc_batt_info.rep.charging_enabled = DISABLE;
	}
	else if (htc_batt_info.rep.level <= (htc_batt_info.rep.full_level - 5)) {
		if (machine_is_passionc()) {
			pr_info("[HTC_BATT] Set full charge ,Stop charge %d\n",chg_ctl);
			gpio_direction_output(22, 0);//PNC //enable charge
			set_charger_ctrl(htc_batt_info.rep.charging_source);
		}else {
			battery_charging_ctrl(htc_batt_info.rep.charging_source);
		}
		htc_batt_info.rep.charging_enabled = htc_batt_info.rep.charging_source;
	}
}
	mutex_unlock(&htc_batt_info.lock);
}
#endif
#if 0
/*
Function:Battery temperature algorithm
Target:	Update currentlly temperature corespondding
		charging type to system.
Timing:	30 second, cable in/out.
*/
static void ds2784_overtemp(void)
{
/* In short term, switch by ID, might add support in board files*/
	u32	high_temp = BATT_HIGH_TEMP;
	u32	high_temp_spare = BATT_HIGH_TEMP_SPARE;
	u32	mid_temp = BATT_MID_TEMP;
	u32	mid_temp_spare = BATT_MID_TEMP_SPARE;
	u32	second_mid_temp = 450;	/* 45 C degree */
	u32	second_mid_temp_spare = 420;	/* 42 C degree */
	u32	low_temp_spare = 30;	/* 3 C degree */
	u32	low_temp = 0;			/* 0 C degree */
	u32 Chg_Full = 4200;		/* Can charing full batt */
	u32 Stop_Vol = 4100;		/* Can't charging full batt*/
	u32 Rechg_Vol = 3800;		/* Can't charging full batt*/
	int temp_flag ;			/* temperaly use in this function call */
	u32 T ;						/*Currently temperature */
	int chg_ctl = DISABLE;

	temp_flag = htc_batt_info.rep.OTP_Flag;
	T = htc_batt_info.rep.batt_temp;
	
//--------------------------------------------------------
/* 	Step 1.
	Check over temperature condition currently
*/
	if (temp_flag == BATT_GOOD_STATE) {
  		if (T < low_temp)
			temp_flag |= BATT_OTP_BIT3;/*goto ~0*/
		else if (T > high_temp)
			temp_flag |= BATT_OTP_BIT2;/*goto 60~*/
		else if ((T > mid_temp)&&(T <= high_temp))
			temp_flag |= BATT_OTP_BIT1;/*goto 50~60*/
		else if ((T > second_mid_temp)&&(T <= mid_temp))
			temp_flag |= BATT_OTP_BIT0;/*goto 45~50*/
	} else {
		/* Over temperature state last time */
		if (temp_flag == BATT_OTP_BIT3) {/*~0*/
			temp_flag = 0;/*reset, check state below.*/
			if (( T >= low_temp_spare )&&( T < second_mid_temp ))
				temp_flag |= BATT_GOOD_STATE;/*now ~0,goto 0~45*/
			else if ((T >= second_mid_temp)&&(T <= mid_temp ))
				temp_flag = BATT_OTP_BIT0;/*now ~0,goto 45~50*/
			else if ((T >= mid_temp)&&(T <= high_temp))
				temp_flag = BATT_OTP_BIT1;/*now ~0,goto 50~60*/
			else if (T >= high_temp)
				temp_flag = BATT_OTP_BIT2;/*now ~0,goto 60~*/
		else
				temp_flag = BATT_OTP_BIT3;/*keep in ~0*/
		} else if (temp_flag == BATT_OTP_BIT2) {/*60~*/
			temp_flag = 0;/*reset, check state below.*/
			if ((T <= high_temp_spare) && (T >= mid_temp))
				temp_flag |= BATT_OTP_BIT1;/*now 60~,goto 50~60*/
			else if ((T >= second_mid_temp) && (T <= mid_temp))
				temp_flag = BATT_OTP_BIT0;/*now 60~,goto 45~50*/
			else if ((T >= low_temp) && (T <= second_mid_temp))
				temp_flag = BATT_GOOD_STATE;/*now 60~,goto 0~45*/
			else if(T < low_temp)
				temp_flag = BATT_OTP_BIT3;/*now 60~,goto ~0*/
		else
				temp_flag = BATT_OTP_BIT2;/*keep in 60~*/
		} else if (temp_flag == BATT_OTP_BIT1) {/*50~60*/
			temp_flag = 0;/*reset, check state below.*/
			if ((T <= mid_temp_spare) && (T >= second_mid_temp))
				temp_flag |= BATT_OTP_BIT0;/*now 50~60,goto 45~50*/
			else if ((T > high_temp) || (T < low_temp))
				temp_flag |= BATT_OTP_BIT2;/*now 50~60,goto 60~*/
			else if ((T < second_mid_temp) && (T >= low_temp))
				temp_flag = BATT_GOOD_STATE;/*now 50~60,goto 0~45*/
			else if(T < low_temp)
				temp_flag = BATT_OTP_BIT3;/*now 60~,goto ~0*/
		else
				temp_flag |= BATT_OTP_BIT1;/*keep 50~60*/
		} else if (temp_flag == BATT_OTP_BIT0) {/*45~50*/
			temp_flag = 0;/*reset, check state below.*/
			if ((T <= second_mid_temp_spare) && (T >= low_temp))
				temp_flag = BATT_GOOD_STATE;/*now 45~50,goto 0~45*/
			else if ((T > mid_temp) && (T <= high_temp))
				temp_flag |= BATT_OTP_BIT1;/*now 45~50,goto 50~60*/
			else if (T > high_temp)
				temp_flag |= BATT_OTP_BIT2;/*now 45~50,goto 60~*/
			else if(T < low_temp)
				temp_flag = BATT_OTP_BIT3;/*now 60~,goto ~0*/
			else
				temp_flag |= BATT_OTP_BIT0;/*keep 45~50*/
		} else /* Not normal case */
			temp_flag = htc_batt_info.rep.OTP_Flag;/*Give up this time*/
	}
/*	Step 2.
	Check if need to update temperature condition changed
*/
		mutex_lock(&htc_batt_info.lock);
		htc_batt_info.rep.OTP_Flag = temp_flag;
		mutex_unlock(&htc_batt_info.lock);

/* 	Step 3.
	Set corespondding charger type to match temperature condition 
*/
if ((htc_batt_info.rep.charging_sts_flag != 0x01)&&
	(htc_batt_info.rep.full_level == 100)){
	if (htc_batt_info.rep.charging_source == CHARGER_AC) {
		if (htc_batt_info.rep.OTP_Flag == BATT_GOOD_STATE) {
			chg_ctl = ENABLE_FAST_CHG;
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL, &chg_ctl);
		} else if ((htc_batt_info.rep.OTP_Flag == BATT_OTP_BIT0) ||
				 (htc_batt_info.rep.OTP_Flag == BATT_OTP_BIT1)) {
			chg_ctl = ENABLE_SLOW_CHG;
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL,
				&chg_ctl);
		} else
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL,
				&chg_ctl);
	} else if (htc_batt_info.rep.charging_source == CHARGER_USB) {
		if ((htc_batt_info.rep.OTP_Flag == BATT_GOOD_STATE) ||
			(htc_batt_info.rep.OTP_Flag == BATT_OTP_BIT0) ||
			(htc_batt_info.rep.OTP_Flag == BATT_OTP_BIT1)) {
			chg_ctl = ENABLE_SLOW_CHG;
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL,
				&chg_ctl);
		} else
			ds2784_blocking_notify(DS2784_CHARGING_CONTROL,
				&chg_ctl);
	} else
		ds2784_blocking_notify(DS2784_CHARGING_CONTROL, &chg_ctl);
}
/* Step 4.
   Check Charging status(Charging/Recharging)
*/

	if (htc_batt_info.rep.charging_source == CHARGER_AC) {
		if ((htc_batt_info.rep.OTP_Flag == BATT_GOOD_STATE) ||
			(htc_batt_info.rep.OTP_Flag == BATT_OTP_BIT0))
			Chk_Charging_Sts(Chg_Full, 0);
		else if (htc_batt_info.rep.OTP_Flag == BATT_OTP_BIT1)
			Chk_Charging_Sts(Stop_Vol, Rechg_Vol);
	}/*End of AC*/
	else if (htc_batt_info.rep.charging_source == CHARGER_USB) {
		if ((htc_batt_info.rep.OTP_Flag == BATT_GOOD_STATE) ||
			(htc_batt_info.rep.OTP_Flag == BATT_OTP_BIT0))
			Chk_Charging_Sts(Chg_Full, 0);
		else if (htc_batt_info.rep.OTP_Flag == BATT_OTP_BIT1)
			Chk_Charging_Sts(Stop_Vol, Rechg_Vol);
	}/*End of USB*/
	else {/* Non cable in*/
		/* Reset charging_sts_flag, since gauge status bit7 already clear 
		itself by level is less than 90%, let Chk_Charging_Sts start from 
		clear state*/
		mutex_lock(&htc_batt_info.lock);
	if (htc_batt_info.rep.charging_sts_flag != 0x01) {
		if ((htc_batt_info.rep.level < 90)&&
			(htc_batt_info.rep.charging_sts_flag == 0x01))
			htc_batt_info.rep.charging_sts_flag = 0;/*reset*/
		else if ((htc_batt_info.rep.level < 100)&&
				 (htc_batt_info.rep.level >= 90)&&
			((htc_batt_info.rep.charging_sts_flag&0x01) == 0x01))
			htc_batt_info.rep.charging_sts_flag = 0x03;/*Recharging*/
		mutex_unlock(&htc_batt_info.lock);
}
}
}
#endif
static int battery_adjust_charge_state(struct ds2784_device_info *di)
{
	unsigned source;
	int rc = 0;
	int temp, volt;
	u8 charge_mode;
	int chg_ctl = DISABLE;



	temp = htc_batt_info.rep.batt_temp;
	volt = htc_batt_info.rep.batt_vol;

	source = htc_batt_info.rep.charging_source;

	/* initially our charge mode matches our source:
	 * NONE:OFF, USB:SLOW, AC:FAST
	 */
	charge_mode = source;

	/* shut off charger when full:
	 * - CHGTF flag is set
	 * - battery drawing less than 80mA
	 * - battery at 100% capacity
	 */
if (machine_is_bravoc()) {

	/* We don't move from full to not-full until
	* we drop below 95%, to avoid confusing the
	* user while we're maintaining a full charge
	* (slowly draining to 95 and charging back
	* to 100)
	 */
	if (htc_batt_info.rep.charging_source != 0) {
		if (htc_batt_info.rep.level <= 95)
			htc_batt_info.rep.battery_full = 0;

		if (htc_batt_info.rep.battery_full)
			charge_mode = CHARGE_OFF;
		else if (htc_batt_info.rep.level == 100) {
			htc_batt_info.rep.battery_full = 1;
			charge_mode = CHARGE_OFF;
		}
	} else
		htc_batt_info.rep.battery_full = 0;

	// No charging source
	if (htc_batt_info.rep.charging_source == 0) {
		htc_batt_info.rep.OT_critical = 0;
		htc_batt_info.rep.OT_hot = 0;
		htc_batt_info.rep.OT_warm = 0;
		htc_batt_info.rep.OT_cold = 0;
		htc_batt_info.rep.cooldown = 0;
	} else {
		// 1st stage: OT check
		if (temp < TEMP_COLD_BC) {
			charge_mode = CHARGE_OFF;
			htc_batt_info.rep.OT_critical = 0;
			htc_batt_info.rep.OT_hot = 0;
			htc_batt_info.rep.OT_warm = 0;
			htc_batt_info.rep.OT_cold = 1;
		}
		else if (htc_batt_info.rep.OT_cold == 1) {
			if (temp > TEMP_COLD_HR_BC) {
				htc_batt_info.rep.OT_cold = 0;
				if (!htc_batt_info.rep.battery_full)
					charge_mode = source;
			} else charge_mode = CHARGE_OFF;
		}

		if (temp >= TEMP_CRITICAL_BC) {
			charge_mode = CHARGE_OFF;
			htc_batt_info.rep.OT_critical = 1;
			htc_batt_info.rep.OT_hot = 0;
			htc_batt_info.rep.OT_warm = 0;
			htc_batt_info.rep.OT_cold = 0;
		}
		else if (htc_batt_info.rep.OT_critical == 1) {
			if ((temp < TEMP_CRITICAL_RECHG_BC) && (volt < TEMP_HOT_MIN_MV_BC)) {
				htc_batt_info.rep.OT_critical = 0;
			} else charge_mode = CHARGE_OFF;
		}

		if ((temp >= TEMP_HOT_BC) && (temp < TEMP_CRITICAL_BC) &&
		    (htc_batt_info.rep.OT_critical != 1)) {
			if (charge_mode == CHARGE_FAST)
				charge_mode = CHARGE_SLOW;
			htc_batt_info.rep.OT_critical = 0;
			htc_batt_info.rep.OT_hot = 1;
			htc_batt_info.rep.OT_warm = 0;
			htc_batt_info.rep.OT_cold = 0;

		/* once we charge to max voltage when hot, disable
		 * charging until the temp drops or the voltage drops
		 */
			if (volt >= TEMP_HOT_MAX_MV_BC)
				htc_batt_info.rep.cooldown = 1;
		}
		else if (htc_batt_info.rep.OT_hot == 1) {
			if ((temp < TEMP_HOT_RECHG_BC) || (volt < TEMP_HOT_MIN_MV_BC))
				htc_batt_info.rep.OT_hot = 0;
			else {
				if (charge_mode == CHARGE_FAST)
					charge_mode = CHARGE_SLOW;
			}
	}

		if ((temp >= TEMP_WARM_BC) && (temp < TEMP_HOT_BC) &&
		    (htc_batt_info.rep.OT_critical != 1) &&
		    (htc_batt_info.rep.OT_hot != 1)) {
			if (charge_mode == CHARGE_FAST)
		charge_mode = CHARGE_SLOW;
			htc_batt_info.rep.OT_critical = 0;
			htc_batt_info.rep.OT_hot = 0;
			htc_batt_info.rep.OT_warm = 1;
			htc_batt_info.rep.OT_cold = 0;
		}
		else if (htc_batt_info.rep.OT_warm == 1) {
			if (temp < TEMP_WARM_HR_BC) {
				if (charge_mode == CHARGE_SLOW)
					charge_mode = source;
				htc_batt_info.rep.OT_warm = 0;
			}
			else {
				if (charge_mode == CHARGE_FAST)
					charge_mode = CHARGE_SLOW;
			}
		}

		// 2nd stage: cooldown check
		if (htc_batt_info.rep.cooldown) {
			if ((temp < TEMP_HOT_RECHG_BC) || (volt < TEMP_HOT_MIN_MV_BC))
				htc_batt_info.rep.cooldown = 0;
		else
				charge_mode = CHARGE_OFF;
		}
	}
		} else {
	if (htc_batt_info.rep.level <= 99)
		htc_batt_info.rep.battery_full = 0;

	if (htc_batt_info.rep.battery_full)
		charge_mode = CHARGE_OFF;
	else if ((htc_batt_info.rep.guage_status_reg & 0x80) &&
	    (htc_batt_info.rep.batt_current <= 80) &&
	    (htc_batt_info.rep.level == 100)) {
	    	if (More_Charge_Extend_Time()) {
				htc_batt_info.rep.battery_full = 1;
			charge_mode = CHARGE_OFF;
		}
	}

	if (temp >= TEMP_HOT) {
		if (temp >= TEMP_CRITICAL)
		charge_mode = CHARGE_OFF;

		/* once we charge to max voltage when hot, disable
		 * charging until the temp drops or the voltage drops
	 */
		if (volt >= TEMP_HOT_MAX_MV)
			htc_batt_info.rep.cooldown = 1;
	}

	/* when the battery is warm, only charge in slow charge mode */
	if ((temp >= TEMP_WARM) && (charge_mode == CHARGE_FAST))
		charge_mode = CHARGE_SLOW;

	if (htc_batt_info.rep.cooldown) {
		if ((temp < TEMP_WARM) || (volt <= TEMP_HOT_MIN_MV))
			htc_batt_info.rep.cooldown = 0;
		else
		charge_mode = CHARGE_OFF;
	}
}

	if (htc_batt_info.rep.last_charge_mode == charge_mode)
		goto done;

	htc_batt_info.rep.last_charge_mode = charge_mode;
	htc_batt_info.rep.charge_mode = charge_mode;

	switch (charge_mode) {
	case CHARGE_OFF:
		/* CHARGER_EN is active low.  Set to 1 to disable. */
ds2784_blocking_notify(DS2784_CHARGING_CONTROL,&chg_ctl);
		if ((temp >= TEMP_CRITICAL) ||
            (htc_batt_info.rep.OT_critical == 1) ||
            (htc_batt_info.rep.OT_cold == 1))
			pr_info("batt: charging OFF [OVERTEMP]\n");
		else if (htc_batt_info.rep.cooldown)
			pr_info("batt: charging OFF [COOLDOWN]\n");
		else if (htc_batt_info.rep.battery_full)
			pr_info("batt: charging OFF [FULL]\n");
		else
			pr_info("batt: charging OFF\n");
		break;
	case CHARGE_SLOW:
	chg_ctl = ENABLE_SLOW_CHG;
	ds2784_blocking_notify(DS2784_CHARGING_CONTROL,&chg_ctl);
		pr_info("batt: charging SLOW\n");
		break;
	case CHARGE_FAST:
	chg_ctl = ENABLE_FAST_CHG;
	ds2784_blocking_notify(DS2784_CHARGING_CONTROL,&chg_ctl);
		pr_info("batt: charging FAST\n");
		break;
	}
	rc = 1;
done:
	return rc;
}

static void ds2784_program_alarm(struct ds2784_device_info *di, int seconds)
{
	ktime_t low_interval = ktime_set(seconds - 10, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

	next = ktime_add(di->last_poll, low_interval);

	alarm_start_range(&di->alarm, next, ktime_add(next, slack));
}

static int battery_control_mfg(struct ds2784_device_info *di)
{
	int rc = 0;
	int chg_ctl = DISABLE;

	pr_info("[HTC_BATT]MFG battery control\n");
mutex_lock(&htc_batt_info.lock);
	if (htc_batt_info.rep.level >= htc_batt_info.rep.full_level) {
		chg_ctl = DISABLE;
		if (machine_is_passionc() || machine_is_bravoc()) {
			gpio_direction_output(22, 1);
			set_charger_ctrl(chg_ctl);
		} else
		battery_charging_ctrl(DISABLE);
		htc_batt_info.rep.charging_enabled = CHARGE_OFF;
	} else if (htc_batt_info.rep.level <=
	(htc_batt_info.rep.full_level - 5)) {
		if (machine_is_passionc() || machine_is_bravoc()) {
			gpio_direction_output(22, 0);
			set_charger_ctrl(htc_batt_info.rep.charging_source);
		} else
		battery_charging_ctrl(htc_batt_info.rep.charging_source);
	htc_batt_info.rep.charging_enabled = htc_batt_info.rep.charging_source;
	}
mutex_unlock(&htc_batt_info.lock);
	return rc;
}

/*
Function:Battery Algorithm
Target:	Temperature algorithm
		Current algorithm		TBD
		Voltage algorithm		TBD
		Recharging algorithm	TBD
Timing:	30 second
*/
static void ds2784_battery_algorithm(struct ds2784_device_info *di)
{
	u8 last_level;
	static u32 last_update = 0;
	unsigned char acr[2];
	int temp;
	int ael_temp;
	int ael_A;
	int ael_B;
	int chg_ctl;
	last_level = htc_batt_info.rep.level;
	temp = htc_batt_info.rep.batt_temp / 10;
	chg_ctl = CHARGER_CHK;

	if (machine_is_bravoc()) {
	/*AEL checking for low temperature */
	if ((temp > 0) && (temp < 15)) {
		ael_A = 5 * (int)(temp / 5);
		ael_B = temp - ael_A;
		if (ael_A == 0)
			ael_temp = AEL_0 + ael_B * ((AEL_5 - AEL_0) / 5);
		else if (ael_A == 5)
			ael_temp = AEL_5 + ael_B * ((AEL_10 - AEL_5) / 5);
		else if (ael_A == 10)
			ael_temp = AEL_10 + ael_B * ((AEL_15 - AEL_10) / 5);
	} else if ((temp <= 0))
		ael_temp = AEL_0;
	else if ((temp >= 15))
		ael_temp = AEL_15;

	/*Modify the RARC to MBI base on AEL_TEMP */
	if ((last_level - ael_temp) < 0)
		htc_batt_info.rep.level = 0;
	else if ((last_level - ael_temp) >= 100)
		htc_batt_info.rep.level = 100;
	else
		htc_batt_info.rep.level =
		(u32)(((last_level - ael_temp) * 100) / (100 - ael_temp));

	/*Don't let MBI drop too fast */
	/* pr_info("[HTC_BATT] last MBI = %d,
	   current level(AEL_TEMP) = %d\n",
	   last_update,htc_batt_info.rep.level); */
	if (htc_batt_info.rep.level < last_update) {
		if ((last_update >= 5) &&
		    (htc_batt_info.rep.level <= (last_update - 5)))
			htc_batt_info.rep.level = last_update - 5;
		else
			htc_batt_info.rep.level = last_update - 1;
	}

	last_update = htc_batt_info.rep.level;

	/* Check for charger conditon*/
	if (htc_batt_info.rep.charging_source != 0) {
		if ((htc_batt_info.rep.batt_vol < 3700) &&
			(htc_batt_info.rep.batt_current < 0)) {
			set_charger_ctrl(chg_ctl);
			pr_info("[HTC_BATT] Dump the SMB329 register\n");
		}
	}
	}

	/* Check for charging completely */
	if (htc_batt_info.rep.charging_source != 0) {
		if ((htc_batt_info.rep.level != 100) &&
			(htc_batt_info.rep.guage_status_reg & 0x80) &&
			(htc_batt_info.rep.batt_current <= 80) &&
			(htc_batt_info.rep.full_acr == 0)) {
				acr[0] = 0x0d;
				acr[1] = 0x87;
				w1_ds2784_write(di->w1_dev, acr, DS2784_REG_ACCUMULATE_CURR_MSB, 2);
				htc_batt_info.rep.full_acr = 1;
				pr_info("[HTC_BATT] Current Full ACR = %x %x\n", acr[0], acr[1]);
				pr_info("[HTC_BATT] Recharging should set ACR to 100 percent\n");
		}
	} else
		htc_batt_info.rep.full_acr = 0;

	if (htc_batt_info.rep.level >= 100) {
		if ((htc_batt_info.rep.guage_status_reg & 0x80) &&
		    (htc_batt_info.rep.batt_current <= 80))
			htc_batt_info.rep.level = 100;
		else
			htc_batt_info.rep.level = 99;
	}

	/* Level */
	if (htc_batt_info.rep.level_last != htc_batt_info.rep.level)
		ds2784_blocking_notify(DS2784_LEVEL_UPDATE,
			&htc_batt_info.rep.level);

	/* Check Over Temperature */
//	ds2784_overtemp();
	if (htc_batt_info.rep.full_level < 100)
		battery_control_mfg(di);
	else
		battery_adjust_charge_state(di);
#if 0
	/*
	If system charge from 90% above and CHGTF = 0x80, set ACR to 100%
	when net charging current < 80mA
	*/
	if (htc_batt_info.rep.charging_sts_flag == 0x11) {
		acr_temp = Calculate_Full_mAh(di)*100*1000/625/67;
		acr[1] = acr_temp & 0x00ff;
		acr[0] = (acr_temp >> 8) & 0x00ff;
		if (htc_batt_info.rep.level != 100) {
			w1_ds2784_write(di->w1_dev, acr,DS2784_REG_ACCUMULATE_CURR_MSB, 2);
			pr_info("[HTC_BATT] Current Full ACR = %x %x\n",acr[0],acr[1]);
			pr_info("[HTC_BATT] Recharging should set ACR to 100 percent\n");
		}
		mutex_lock(&htc_batt_info.lock);
		htc_batt_info.rep.charging_sts_flag = 0x01;/*Full*/
		mutex_unlock(&htc_batt_info.lock);
	/*
	Recharging Full, set sts_flag to 0x01 Full stage
	*/
	} else if (htc_batt_info.rep.charging_sts_flag == 0x09) {
		mutex_lock(&htc_batt_info.lock);
		htc_batt_info.rep.charging_sts_flag = 0x01;/*Full*/
		mutex_unlock(&htc_batt_info.lock);
	}
#endif
}
#if 0
/* For PNC XA/XB switch charger reset */
/*
*	Reset switch charger every 2 hours
*	[FIXME] for PNC
*/
static unsigned long switch_chgr_count = 0xffffffff;
static void htc_switch_charger_reset(void)
{
	struct timespec now;
	getnstimeofday(&now);

	/* Check Switch charger time count */
	if ((htc_batt_info.rep.charging_enabled != CHARGER_BATTERY) &&
		(switch_chgr_count == 0)){
		switch_chgr_count = now.tv_sec;
		pr_info("batt: start count for switch charger \n");
		return ;
	}else if (htc_batt_info.rep.charging_enabled == CHARGER_BATTERY){
		switch_chgr_count = 0;
	}

	if (switch_chgr_count == 0)
		return;

	if ((now.tv_sec - switch_chgr_count) > 7200){
		pr_info("Reset switch charger 2 seconds, count = %ld secs\n",(now.tv_sec - switch_chgr_count));
		gpio_direction_output(22, 1);//PNC
		msleep(1000);
		msleep(1000);
		gpio_direction_output(22, 0);//PNC
		switch_chgr_count = 0;
	}

}
#endif
/*[TO DO] need to port Battery Algorithm to meet power spec here.
Only update uevent to upper if percentage changed currently
*/
static void ds2784_battery_update_status(struct ds2784_device_info *di)
{
#if 0
/* For PNC XA/XB switch charger reset */
	if (!htc_battery_initial)
		switch_chgr_count = 0;
#endif

	ds2784_battery_read_status(di);

	/* Go to Battery Algorithm */
	ds2784_battery_algorithm(di);

#if 0
/* For PNC XA/XB switch charger reset */
	if (machine_is_passionc())
		htc_switch_charger_reset();
#endif
}
static void ds2784_battery_work(struct work_struct *work)
{
	struct ds2784_device_info *di = container_of(work,
		struct ds2784_device_info, monitor_work);
	unsigned long flags;

	ds2784_battery_update_status(di);

	di->last_poll = alarm_get_elapsed_realtime();

	/* prevent suspend before starting the alarm */
	local_irq_save(flags);
	wake_unlock(&di->work_wake_lock);
	if (emc_debug)
		ds2784_program_alarm(di, Onehr_POLL);
	else
	ds2784_program_alarm(di, FAST_POLL);
	local_irq_restore(flags);
}

static void ds2784_battery_alarm(struct alarm *alarm)
{
	struct ds2784_device_info *di =
		container_of(alarm, struct ds2784_device_info, alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);
}

static int ds2784_battery_probe(struct platform_device *pdev)
{
	int rc;
	struct ds2784_device_info *di;
	struct ds2784_platform_data *pdata;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		rc = -ENOMEM;
		goto fail_register;
	}

	di->update_time = jiffies;
	platform_set_drvdata(pdev, di);

	pdata = pdev->dev.platform_data;
	di->dev = &pdev->dev;
	di->w1_dev	     = pdev->dev.parent;

	INIT_WORK(&di->monitor_work, ds2784_battery_work);
	di->monitor_wqueue = create_freezeable_workqueue(
		dev_name(&pdev->dev));

	/* init to something sane */
	di->last_poll = alarm_get_elapsed_realtime();

	if (!di->monitor_wqueue) {
		rc = -ESRCH;
		goto fail_workqueue;
	}
	wake_lock_init(&di->work_wake_lock, WAKE_LOCK_SUSPEND,
			"ds2784-battery");
	alarm_init(&di->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			ds2784_battery_alarm);
	wake_lock(&di->work_wake_lock);
	queue_work(di->monitor_wqueue, &di->monitor_work);
	return 0;

fail_workqueue:
fail_register:
	kfree(di);
	return rc;
}

static int ds2784_battery_remove(struct platform_device *pdev)
{
	struct ds2784_device_info *di = platform_get_drvdata(pdev);

	cancel_work_sync(&di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);

	return 0;
}

/* FIXME: power down DQ master when not in use. */
static int ds2784_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ds2784_device_info *di = platform_get_drvdata(pdev);
	unsigned long flags;

	/* If we are on battery, reduce our update rate until
	 * we next resume.
	 */
	if (emc_debug) {
		local_irq_save(flags);
		ds2784_program_alarm(di, Onehr_POLL);
		local_irq_restore(flags);
	} else if (di->rep.charging_source == SOURCE_NONE) {
		local_irq_save(flags);
		ds2784_program_alarm(di, SLOW_POLL);
		di->slow_poll = 1;
		local_irq_restore(flags);
	}
	gpio_direction_output(87, 0);
	return 0;
}
static int ds2784_resume(struct platform_device *pdev)
{
	struct ds2784_device_info *di = platform_get_drvdata(pdev);
	unsigned long flags;

	/* We might be on a slow sample cycle.  If we're
	 * resuming we should resample the battery state
	 * if it's been over a minute since we last did
	 * so, and move back to sampling every minute until
	 * we suspend again.
	 */
	gpio_direction_output(87, 1);
	ndelay(100 * 1000);

	if (di->slow_poll) {
		local_irq_save(flags);
		ds2784_program_alarm(di, FAST_POLL);
		di->slow_poll = 0;
		local_irq_restore(flags);
	}
	return 0;
}


MODULE_ALIAS("platform:ds2784-battery");
static struct platform_driver ds2784_battery_driver = {
	.driver = {
		.name = "ds2784-battery",
	},
	.suspend = ds2784_suspend,
	.resume = ds2784_resume,
	.probe = ds2784_battery_probe,
	.remove   = ds2784_battery_remove,
};

static int __init ds2784_battery_init(void)
{
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	register_notifier_cable_status(&cable_status_handler);
	mutex_init(&htc_batt_info.lock);
	return platform_driver_register(&ds2784_battery_driver);
}

static void __exit ds2784_battery_exit(void)
{
	platform_driver_unregister(&ds2784_battery_driver);
}

module_init(ds2784_battery_init);
module_exit(ds2784_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Justin Lin <Justin_lin@htc.com>");
MODULE_DESCRIPTION("ds2784 battery driver");
