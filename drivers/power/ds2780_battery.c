/* 
 * Driver for batteries with DS2780 chips inside.
 *
 * Copyright (C) 2009 Vladimir Ananiev <vovan888@gmail.com>
 * based on:
 * Driver for batteries with DS2760 chips.
 *
 * Copyright (C) 2007 Anton Vorontsov
 *	       2004-2007 Matt Reimer
 *	       2004 Szabolcs Gyurko
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * Author:  Anton Vorontsov <cbou@mail.ru>
 *	    February 2007
 *
 *	    Matt Reimer <mreimer@vpop.net>
 *	    April 2004, 2005, 2007
 *
 *	    Szabolcs Gyurko <szabolcs.gyurko@tlt.hu>
 *	    September 2004
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include "../w1/w1.h"
#include "../w1/slaves/w1_ds2780.h"

struct ds2780_device_info {
	struct device *dev;

	/* DS2780 data, valid after calling ds2780_battery_read_status() */
	unsigned long update_time;	/* jiffies when data read */
	char raw[DS2780_DATA_SIZE];	/* raw DS2780 data */
	int Rsnsp;			/* current sense resistor prime units of 1/Om */
	int voltage;			/* units of uV */
	int current_now;		/* units of uA */	
	int current_avg;		/* units of uAh */
	int temp;			/* units of 0.1 deg C */
	int rem_active_abs_capacity;	/* RAAC - units of uAh */
	int rem_standby_abs_capacity;	/* RSAC - units of uAh */
	int rem_active_rel_capacity;	/* RARC - in % */
	int rem_standby_rel_capacity;	/* RSRC - in % */

	int charge_status;		/* POWER_SUPPLY_STATUS_* */

	struct power_supply bat;
	struct device *w1_dev;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

static unsigned int cache_time = 1000;
module_param(cache_time, uint, 0644);
MODULE_PARM_DESC(cache_time, "cache time in milliseconds");

static int ds2780_battery_read_status(struct ds2780_device_info *di)
{
	int ret, start, count, temp_raw;

	if (di->update_time && time_before(jiffies, di->update_time +
					   msecs_to_jiffies(cache_time)))
		return 0;

	/* The first time we read the entire contents of SRAM/EEPROM,
	 * but after that we just read the interesting bits that change. */
	if (di->update_time == 0) {
		start = 0;
		count = DS2780_DATA_SIZE;
	} else {
		start = DS2780_STATUS;
		count = DS2780_CURRENT_LSB - start + 1;
	}

	ret = w1_ds2780_read(di->w1_dev, di->raw + start, start, count);
	if (ret != count) {
		dev_warn(di->dev, "call to w1_ds2780_read failed (0x%p)\n",
			 di->w1_dev);
		return 1;
	}

	di->update_time = jiffies;

	/* DS2780 reports voltage in units of 4.88mV, but the battery class
	 * reports in units of uV, so convert by multiplying by 4880. */
	di->voltage = ((di->raw[DS2780_VOLT_MSB] << 3) |
			  (di->raw[DS2780_VOLT_LSB] >> 5)) * 4880;

	/* DS2780 reports temperature in signed units of 0.125°C, but the
	 * battery class reports in units of 1/10 °C, so we convert by
	 * multiplying by .125 * 10 = 1.25. */
	temp_raw = (((signed char)di->raw[DS2780_TEMP_MSB]) << 3) |
				     (di->raw[DS2780_TEMP_LSB] >> 5);
	di->temp = temp_raw + (temp_raw / 4);

	/* DS2780 reports current in signed units of 1.5625uV/Rsns, but the battery
	 * class reports in units of µA, so convert it. */
	di->Rsnsp = di->raw[DS2780_PARAM_RSNSP];
	di->current_now = ((((signed char)di->raw[DS2780_CURRENT_MSB]) << 8) |
			(di->raw[DS2780_CURRENT_LSB])) * 15625 / 10000 * di->Rsnsp;

	/* DS2780 reports average current in signed units of 1.5625uV/Rsns. */
	di->current_avg = ((((signed char)di->raw[DS2780_IAVG_MSB]) << 8) |
			di->raw[DS2780_IAVG_LSB]) * 15625 / 10000 * di->Rsnsp;

	/* DS2780 reports Remaining Active Absolute Capacity in units of 1.6mAh. */
	di->rem_active_abs_capacity = ((di->raw[DS2780_RAAC_MSB] << 8) |
			   di->raw[DS2780_RAAC_LSB]) * 1600;

	/* DS2780 reports Remaining Standby Absolute Capacity in units of 1.6mAh. */
	di->rem_standby_abs_capacity = (((di->raw[DS2780_RSAC_MSB]) << 8) |
			   di->raw[DS2780_RSAC_LSB]) * 1600;

	/* DS2780 reports Remaining Active Relative Capacity in units of 1%. */
	di->rem_active_rel_capacity = di->raw[DS2780_RARC];

	/* DS2780 reports Remaining Standby Relative Capacity in units of 1%. */
	di->rem_standby_rel_capacity = di->raw[DS2780_RSRC];

	return 0;
}

static void ds2780_battery_update_status(struct ds2780_device_info *di)
{
	int old_charge_status = di->charge_status;

	ds2780_battery_read_status(di);

	if (power_supply_am_i_supplied(&di->bat)) {
		if (di->raw[DS2780_STATUS] & DS2780_STATUS_CHGTF)
			di->charge_status = POWER_SUPPLY_STATUS_FULL;
		else if (di->current_now > 10000)
			di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		else if (di->current_now < -5000) {
			if (di->charge_status != POWER_SUPPLY_STATUS_NOT_CHARGING)
				dev_notice(di->dev, "not enough power to "
					   "charge\n");
			di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	} else {
		di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (di->charge_status != old_charge_status)
		power_supply_changed(&di->bat);
}

static void ds2780_battery_work(struct work_struct *work)
{
	struct ds2780_device_info *di = container_of(work,
		struct ds2780_device_info, monitor_work.work);
	const int interval = HZ * 60;

	dev_dbg(di->dev, "%s\n", __func__);

	ds2780_battery_update_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

#define to_ds2780_device_info(x) container_of((x), struct ds2780_device_info, \
					      bat);

static void ds2780_battery_external_power_changed(struct power_supply *psy)
{
	struct ds2780_device_info *di = to_ds2780_device_info(psy);

	dev_dbg(di->dev, "%s\n", __func__);

	cancel_delayed_work(&di->monitor_work);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ/10);
}

static int ds2780_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct ds2780_device_info *di = to_ds2780_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->charge_status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = di->current_avg;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->rem_active_abs_capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->rem_active_rel_capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property ds2780_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int ds2780_battery_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct ds2780_device_info *di;
//	struct ds2780_platform_data *pdata;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

	platform_set_drvdata(pdev, di);

//	pdata = pdev->dev.platform_data;
	di->dev			= &pdev->dev;
	di->w1_dev		= pdev->dev.parent;
	di->bat.name		= pdev->dev.bus_id;
	di->bat.type		= POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties	= ds2780_battery_props;
	di->bat.num_properties	= ARRAY_SIZE(ds2780_battery_props);
	di->bat.get_property	= ds2780_battery_get_property;
	di->bat.external_power_changed =
				ds2780_battery_external_power_changed;

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register battery\n");
		goto batt_failed;
	}

	INIT_DELAYED_WORK(&di->monitor_work, ds2780_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(pdev->dev.bus_id);
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 1);

	goto success;

workqueue_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	kfree(di);
di_alloc_failed:
success:
	return retval;
}

static int ds2780_battery_remove(struct platform_device *pdev)
{
	struct ds2780_device_info *di = platform_get_drvdata(pdev);

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
					  &di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);
	power_supply_unregister(&di->bat);

	return 0;
}

#ifdef CONFIG_PM

static int ds2780_battery_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct ds2780_device_info *di = platform_get_drvdata(pdev);

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

	return 0;
}

static int ds2780_battery_resume(struct platform_device *pdev)
{
	struct ds2780_device_info *di = platform_get_drvdata(pdev);

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	power_supply_changed(&di->bat);

	cancel_delayed_work(&di->monitor_work);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ);

	return 0;
}

#else

#define ds2780_battery_suspend NULL
#define ds2780_battery_resume NULL

#endif /* CONFIG_PM */

MODULE_ALIAS("platform:ds2780-battery");

static struct platform_driver ds2780_battery_driver = {
	.driver = {
		.name = "ds2780-battery",
	},
	.probe	  = ds2780_battery_probe,
	.remove   = ds2780_battery_remove,
	.suspend  = ds2780_battery_suspend,
	.resume	  = ds2780_battery_resume,
};

static int __init ds2780_battery_init(void)
{
	return platform_driver_register(&ds2780_battery_driver);
}

static void __exit ds2780_battery_exit(void)
{
	platform_driver_unregister(&ds2780_battery_driver);
}

module_init(ds2780_battery_init);
module_exit(ds2780_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vladimir Ananiev <vovan888@gmail.com>");
MODULE_DESCRIPTION("ds2780 battery driver");
