/*
 * Driver for adcins on GPIO lines capable of generating interrupts.
 *
 * Copyright (C) 2015, Fuzhou Rockchip Electronics Co., Ltd
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>


#define EMPTY_DEFAULT_ADVALUE		1024
//#define DRIFT_DEFAULT_ADVALUE		70
#define INVALID_ADVALUE			-1

#if 0
#define adc_dbg(bdata, format, arg...)		\
	dev_info(&bdata->input->dev, format, ##arg)
#else
#define adc_dbg(bdata, format, arg...)
#endif

//#define DEBOUNCE_JIFFIES	(10 / (MSEC_PER_SEC / HZ))	/* 10ms */
#define ADC_SAMPLE_JIFFIES	(100 / (MSEC_PER_SEC / HZ))	/* 100ms */
//#define WAKE_LOCK_JIFFIES	(1 * HZ)			/* 1s */

struct zd_adcins_drvdata {
	//int nbuttons;
	/* flag to indicate if we're suspending/resuming */
	bool in_suspend;
	int result;
	int rep;
	int drift_advalue;
	//struct wake_lock wake_lock;
	struct input_dev *input;
	struct delayed_work adc_poll_work;
	struct iio_channel *chan;
	//struct zd_adcins_button button[0];
};

static struct input_dev *sinput_dev;


static ssize_t adc_inx_value_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct zd_adcins_drvdata *ddata = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ddata->result);
}
static ssize_t adc_inx_value_store(struct device* dev, struct device_attribute *attr, \
					const char* buf, size_t count)
{
	int enable;
	sscanf(buf, "%d", &enable);

	return count;
}
static DEVICE_ATTR(get_adc_in0_value, 0664, adc_inx_value_show, adc_inx_value_store);

static const struct of_device_id zd_adcins_match[] = {
	{ .compatible = "zd,adc_in0", .data = NULL},
	{},
};
MODULE_DEVICE_TABLE(of, zd_adcins_match);

static int rk_adc_iio_read(struct zd_adcins_drvdata *data)
{
	struct iio_channel *channel = data->chan;
	int val, ret;

	if (!channel)
		return INVALID_ADVALUE;
	ret = iio_read_channel_raw(channel, &val);
	if (ret < 0) {
		pr_err("read channel() error: %d\n", ret);
		return ret;
	}
	return val;
}

static void adc_iio_poll(struct work_struct *work)
{
	struct zd_adcins_drvdata *ddata;
	int result = -1;

	ddata = container_of(work, struct zd_adcins_drvdata, adc_poll_work.work);
	if (!ddata->in_suspend) {
		result = rk_adc_iio_read(ddata);
		if (result > INVALID_ADVALUE &&
		    result < (EMPTY_DEFAULT_ADVALUE - ddata->drift_advalue))
			ddata->result = result;
	}

	schedule_delayed_work(&ddata->adc_poll_work, ADC_SAMPLE_JIFFIES);
}

static int zd_adcins_parse_dt(struct zd_adcins_drvdata *pdata,
			    struct platform_device *pdev)
{
	struct iio_channel *chan;

	chan = iio_channel_get(&pdev->dev, NULL);
	if (IS_ERR(chan)) {
		dev_info(&pdev->dev, "no io-channels defined\n");
		chan = NULL;
	}
	pdata->chan = chan;

	return 0;
}

static int adcins_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	//struct device_node *np = pdev->dev.of_node;
	struct zd_adcins_drvdata *ddata = NULL;
	struct input_dev *input = NULL;
	int error = 0;
	//int wakeup, key_num = 0;

	ddata = devm_kzalloc(dev, sizeof(struct zd_adcins_drvdata), GFP_KERNEL);

	input = devm_input_allocate_device(dev);
	if (!ddata || !input) {
		error = -ENOMEM;
		return error;
	}
	platform_set_drvdata(pdev, ddata);
	dev_set_drvdata(&pdev->dev, ddata);

	input->name = "rk29-adcins";	/* pdev->name; */
	input->phys = "gpio-adcins/input0";
	input->dev.parent = dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0011;
	input->id.version = 0x0100;
	ddata->input = input;

	/* parse info from dt */
	error = zd_adcins_parse_dt(ddata, pdev);
	if (error)
		goto fail0;

	/* Enable auto repeat feature of Linux input subsystem */
	if (ddata->rep)
		__set_bit(EV_REP, input->evbit);

	error = input_register_device(input);
	if (error) {
		pr_err("gpio-adcins: Unable to register input device, error: %d\n",
		       error);
		goto fail0;
	}
	sinput_dev = input;

	//wake_lock_init(&ddata->wake_lock, WAKE_LOCK_SUSPEND, input->name);
	//device_init_wakeup(dev, wakeup);

	error = device_create_file(dev, &dev_attr_get_adc_in0_value);
	if(error)
	{
		pr_err("failed to create key file error: %d\n", error);
	}
	
	/* adc polling work */
	if (ddata->chan) {
		INIT_DELAYED_WORK(&ddata->adc_poll_work, adc_iio_poll);
		schedule_delayed_work(&ddata->adc_poll_work,ADC_SAMPLE_JIFFIES);
	}

	return error;

fail0:
	platform_set_drvdata(pdev, NULL);

	return error;
}

static int adcins_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zd_adcins_drvdata *ddata = dev_get_drvdata(dev);
	struct input_dev *input = ddata->input;
	//int i;

	//device_init_wakeup(dev, 0);
	
	if (ddata->chan)
		cancel_delayed_work_sync(&ddata->adc_poll_work);
	input_unregister_device(input);
	//wake_lock_destroy(&ddata->wake_lock);

	sinput_dev = NULL;

	return 0;
}



static struct platform_driver adcins_device_driver = {
	.probe		= adcins_probe,
	.remove		= adcins_remove,
	.driver		= {
		.name	= "zd_adc_in0",
		.owner	= THIS_MODULE,
		.of_match_table = zd_adcins_match,
	}
};

static int __init zd_adcins_driver_init(void)
{
	return platform_driver_register(&adcins_device_driver);
}

static void __exit zd_adcins_driver_exit(void)
{
	platform_driver_unregister(&adcins_device_driver);
}

late_initcall_sync(zd_adcins_driver_init);
module_exit(zd_adcins_driver_exit);
