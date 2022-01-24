/* drivers/i2c/chips/sfh7776.c - sfh7776.c optical sensors driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * create: 6/25/2013
 */

#include <linux/delay.h>
//#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include "sfh7776.h"
#include "sfh7776_als_ps.h"

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/irqdomain.h>
#endif

#define I(x...)		printk(KERN_INFO "[SFH7776] " x)
#define D()		IPS("%s() line:%d\n", __func__, __LINE__)
#define DEBUG_SFH7776
#ifdef DEBUG_SFH7776
#define DPS(x...)       printk(KERN_INFO "[SFH7776][PS] " x)
#define	DLS(x...)	printk(KERN_INFO "[SFH7776][LS] " x)
#else
#define DPS(x...)
#define DLS(x...)
#endif

#define IPS(x...)	printk(KERN_INFO "[SFH7776][PS] " x)
#define	ILS(x...)	printk(KERN_INFO "[SFH7776][LS] " x)
#define WPS(x...)	printk(KERN_WARNING "[SFH7776 WARNING][PS] " x)
#define WLS(x...)	printk(KERN_WARNING "[SFH7776 WARNING][LS] " x)
#define EPS(x...)	printk(KERN_ERR "[SFH7776 ERROR][PS] " x)
#define ELS(x...)	printk(KERN_ERR "[SFH7776 ERROR][LS] " x)

#define I2C_RETRY_COUNT		10
#define POLLING_PROXIMITY	1
#define POLLING_DELAY		200
enum ps_distance{
	PS_INIT_DISTANCE = -1,
	PS_NEAR = 0,
	PS_FAR = 1,
};
struct sfh7776_info {
	uint16_t slave_address;
	int irq;
	int intr_pin;
	struct class *sfh7776_class;
	struct device *ls_dev;
	struct device *ps_dev;
	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;
	struct i2c_client* i2c_client;
	struct workqueue_struct *lp_wq;
	int psensor_opened;
	int ps_enable;
	int lightsensor_opened;	
	int ls_enable;
	int current_level;
	int old_level;
	enum ps_distance old_distance;
	enum ps_distance current_distance;
	uint16_t ps_th;
	uint16_t ps_tl;
	uint16_t als_th;
	uint16_t als_tl;
	uint16_t *adc_table;
	uint8_t	mode_control;
	uint8_t	als_ps_control;
	uint8_t	int_setting;
	uint8_t	ps_th_lsb;
	uint8_t	ps_th_msb;
	uint8_t	ps_tl_lsb;
	uint8_t	ps_tl_msb;
	uint8_t	als_vis_th_lsb;
	uint8_t	als_vis_th_msb;
	uint8_t als_vis_tl_lsb;
	uint8_t als_vis_tl_msb;
};
struct sfh7776_info *lp_info;
static struct mutex als_ps_mutex, als_thd;
static void sensor_irq_do_work(struct work_struct* work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

static int lsensor_enable(void);
static int lsensor_disable(void);
#ifdef POLLING_PROXIMITY
static void polling_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);
#endif
static int ps_hal_enable;
static int ps_data_level = 0;
//static void report_do_work(struct work_struct* work);
//static DECLARE_WORK();
/*txData[0] is register address, txData[1] is data*/
static int i2c_TxData(char *txData, int length)
{
	int loop_i = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = lp_info->slave_address,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};
/*
	for (i = 0; i < length; i++) {
		DPS("%s() txData[%d]=0x%x\n", __func__, i, txData[i]);
	}
*/
	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i ++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 1) > 0)
			break;
		WPS("%s() i2c err\n", __func__);
	}

	if (loop_i > I2C_RETRY_COUNT) {
		EPS("%s() i2c retry over %d times\n", __func__, I2C_RETRY_COUNT);
		return -EIO;
	}
	return 0;

}

/*rxData[0] is register address, return rxData[0] which is data*/
static int i2c_RxData(char *rxData)
{
	//struct sfh7776_info *lpi = lp_info;
	int loop_i = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = lp_info->slave_address,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = lp_info->slave_address,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = rxData,
		},
	};
	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i ++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 2) > 0)
			break;
		WPS("%s() i2c err\n", __func__);
	}
	if (loop_i > I2C_RETRY_COUNT) {
		EPS("%s() i2c retry over %d times\n", __func__, I2C_RETRY_COUNT);
		return -EIO;
	}
	
	//DPS("%s() rxData=0x%x\n", __func__, *rxData);
	return 0;
}

static void lsensor_enable_intr(void)
{
	char data[2], int_setting;
	/*enable ls interrupt*/
	data[0] = INT_SETTING;
	i2c_RxData(data);
	int_setting = data[0];

	data[0] = INT_SETTING;
	data[1] = int_setting | ENABLE_ALS_INT;
	i2c_TxData(data, sizeof(data));
}
static void lsensor_disable_intr(void)
{
	char data[2], int_setting;
	/*disable ls interrupt*/
	data[0] = INT_SETTING;
	i2c_RxData(data);
	int_setting = data[0];

	data[0] = INT_SETTING;
	data[1] = int_setting & (~ENABLE_ALS_INT);
	i2c_TxData(data, sizeof(data));

}

static void set_lsensor_range(uint16_t low_thd, uint16_t hight_thd)
{
	char data[2];
	
	/*threshold low*/
	data[0] = ALS_VIS_TL_LSB;
	data[1] = low_thd & 0xf;
	i2c_TxData(data, sizeof(data));

	data[0] = ALS_VIS_TL_MSB;
	data[1] = low_thd >> 8;
	i2c_TxData(data, sizeof(data));

	/*threshold hight*/
	data[0] = ALS_VIS_TH_LSB;
	data[1] = hight_thd & 0xf;
	i2c_TxData(data, sizeof(data));
	
	data[0] = ALS_VIS_TH_MSB;
	data[1] = hight_thd >> 8;
	i2c_TxData(data, sizeof(data));
}

static int get_ls_adc_value(uint16_t *adc)
{
	char msb, lsb;
	/*msb*/
	msb = ALS_VIS_DATA_MSB;
	i2c_RxData(&msb);
	/*lsb*/
	lsb = ALS_VIS_DATA_LSB;
	if (i2c_RxData(&lsb)) {
		ELS("%s() get ls adc failed\n", __func__);
		return -EINVAL;
	}
	*adc = (msb << 8) + lsb;
	return 0;
}

static void get_ps_data(uint16_t *ps_data)
{
	uint8_t data;
	/*ps data*/
	data = PS_DATA_LSB;
	i2c_RxData(&data);
	*ps_data = data;
	
	data = PS_DATA_MSB;
	i2c_RxData(&data);
	*ps_data = ((data & 0x0f)<<8) + *ps_data;
}

static ssize_t rw_register_show(struct device* dev, struct device_attribute *attr, \
				char *buf)
{
	int ret, i;
	uint8_t data[REGISTER_NR] = {0};
	/*read register 0x40, 0x41, ..., 0x52*/
	for (i = 0; i < REGISTER_NR; i++) {
		data[i] = SYSTEM_CONTROL + i;
		i2c_RxData(&data[i]);
		//IPS("%s() i=%d\n", __func__, i);
	}

	ret = sprintf(buf,  \
		"0x40=0x%02x, 0x41=0x%02x, 0x42=0x%02x, 0x43=0x%02x, 0x44=0x%02x,\n" \
		"0x45=0x%02x, 0x46=0x%02x, 0x47=0x%02x, 0x48=0x%02x, 0x49=0x%02x, \n" \
		"0x4a=0x%02x, 0x4b=0x%02x, 0x4c=0x%02x, 0x4d=0x%02x, 0x4e=0x%02x, \n" \
		"0x4f=0x%02x, 0x50=0x%02x, 0x51=0x%02x, 0x52=0x%02x \n", 
		data[0], data[1], data[2], data[3], data[4], data[5], data[6], \
		data[7], data[8], data[9], data[10], data[11], data[12], \
		data[13], data[14], data[15], data[16], data[17], data[18]);
	return ret;
}

static ssize_t rw_register_store(struct device* dev, struct device_attribute *attr, \
				const char* buf, size_t count)
{
	int reg, cmd;
	char data[2] = {0, 0};
	
	ILS("%s() count=%d\n", __func__, count);
	sscanf(buf, "0x%x 0x%x", &reg, &cmd);
	if (reg < 0 || cmd < 0) {
		ELS("%s() reg=0x%x cmd=0x%x\n", __func__, reg, cmd);
		return -EINVAL;
	}

	data[0] = (char)reg;
	data[1] = (char)cmd;
	
	if(i2c_TxData(data, sizeof(data))) {
		ELS("%s() data[0]=0x%02x data[1]=0x%02x\n", __func__, data[0], data[1]);
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(rw_register, 0664, rw_register_show, rw_register_store);

static ssize_t ls_enable_show(struct device* dev, struct device_attribute *attr, \
				char *buf)
{
	int ret;
	ret = sprintf(buf, "light sensor enable=%d\n", lp_info->ls_enable);
	return ret;
}

static ssize_t ls_enable_store(struct device* dev, struct device_attribute *attr, \
				const char* buf, size_t count)
{
	int enable;
	sscanf(buf, "%d", &enable);
	DLS("%s() enable=%d\n", __func__, enable);
	if (enable) {
		lsensor_enable();
	} else {
		lsensor_disable();
	}
	return count;
}
static DEVICE_ATTR(ls_enable, 0664, ls_enable_show, ls_enable_store);

#ifdef POLLING_PROXIMITY
static void polling_do_work(struct work_struct *work)
{
	uint16_t ps_data;
	static int n = 0;
	get_ps_data(&ps_data);
	if (ps_data > lp_info->ps_th) { /*near*/
		lp_info->current_distance = PS_NEAR;
	} else if (ps_data < lp_info->ps_tl) { /*far*/
		lp_info->current_distance = PS_FAR;
	}
	if (lp_info->current_distance != lp_info->old_distance) {
		IPS("%s() proximity sensor %s\n", __func__, lp_info->current_distance ? "far":"near");
		lp_info->old_distance = lp_info->current_distance;
		input_report_abs(lp_info->ps_input_dev, ABS_DISTANCE, lp_info->current_distance);
		input_sync(lp_info->ps_input_dev);
	}
	n++;
	ps_data_level = lp_info->current_distance;
	queue_delayed_work(lp_info->lp_wq, &polling_work, msecs_to_jiffies(POLLING_DELAY));
}
#endif
static int psensor_enable(void)
{
	//uint8_t data[2] = {0}, int_setting;
	D();
	mutex_lock(&als_ps_mutex);

	if (lp_info->ps_enable) {
		IPS("%s() p-sensor already enabled, enable:%d\n", __func__, lp_info->ps_enable);
		lp_info->ps_enable++;
		mutex_unlock(&als_ps_mutex);
		return 0;
	}
#if 0
	/*enable ps interrupt*/
	data[0] = INT_SETTING;
	i2c_RxData(data);
	int_setting = data[0];
	
	DPS("%s() int_setting=0x%x\n", __func__, int_setting);
	data[0] = INT_SETTING;
	data[1] = int_setting | ENABLE_PS_INT;
	i2c_TxData(data, sizeof(data));
#endif

	lp_info->ps_enable = 1;
#ifdef POLLING_PROXIMITY
	queue_delayed_work(lp_info->lp_wq, &polling_work, msecs_to_jiffies(POLLING_DELAY));
#endif
	mutex_unlock(&als_ps_mutex);
	return 0;
}

static int psensor_disable(void)
{
	//uint8_t data[2] = {0}, int_setting;
		
	mutex_lock(&als_ps_mutex);
	if (lp_info->ps_enable != 1) {
		if (lp_info->ps_enable > 1)
			lp_info->ps_enable --;
		else {
			IPS("%s() p-sensor already disable\n", __func__);
		}
		mutex_unlock(&als_ps_mutex);
		return 0;
	}
	
	D();
#if 0
	/*disable ps interrupt*/
	data[0] = INT_SETTING;
	i2c_RxData(data);
	int_setting = data[0];

	data[0] = INT_SETTING;
	data[1] = int_setting & (~ENABLE_PS_INT);
	i2c_TxData(data, sizeof(data));
#endif	
	lp_info->ps_enable = 0;
	mutex_unlock(&als_ps_mutex);
	cancel_delayed_work(&polling_work);
	lp_info->old_distance = PS_INIT_DISTANCE;
	
	return 0;
}


static ssize_t psensor_enable_show(struct device* dev, struct device_attribute *attr, \
					char* buf)
{
	int ret;
	ret = sprintf(buf, "ps enable = %d\n", lp_info->ps_enable);
	return ret;
}

static ssize_t psensor_enable_store(struct device* dev, struct device_attribute *attr, \
					const char* buf, size_t count)
{
	int enable, ret;
	sscanf(buf, "%d", &enable);

	if (enable) {
		ret = psensor_enable();
	}else
		ret = psensor_disable();
	return count;
}
static DEVICE_ATTR(psensor_enable, 0664, psensor_enable_show, psensor_enable_store);



static int check_id(void)
{
	char cmd = 0;
	cmd = SYSTEM_CONTROL;
	if (i2c_RxData(&cmd)) {
		EPS("%s() i2c read error\n", __func__);
		return -1;
	}
	DPS("%s() chip id=0x%x\n", __func__, cmd);
	if (cmd != CHIP_ID) {
		EPS("%s() chip id is not %x\n", __func__, CHIP_ID);
		return -1;
	}
	return 0;
}

static int report_lsensor_input_event(void)
{
	uint16_t adc;
	int i;
	/*ls data*/
	if (get_ls_adc_value(&adc)) {
		ELS("%s() get ls adc failed\n", __func__);
		return 0;
	}
	//adc_table[0]={ 3, 5, 7, 275, 1142, 3404, 4465, 5739, 9399, 65535};
	lp_info->adc_table[0]=3;
	lp_info->adc_table[1]=5;
	lp_info->adc_table[2]=7;
	lp_info->adc_table[3]=275;
	lp_info->adc_table[4]=1142;
	lp_info->adc_table[5]=3404;
	lp_info->adc_table[6]=4465;
	lp_info->adc_table[7]=5739;
	lp_info->adc_table[8]=9399;
	lp_info->adc_table[9]=65535;
	for (i = 0; i < 10; i++) {
		if (adc < lp_info->adc_table[i])
			break;
	}
	if (i == 10)
		i = 9;
	lp_info->current_level = i;
	/*If adc=adc_table[9]=65536, then als interrupt is always trigger*/
	if (lp_info->current_level == lp_info->old_level) {
		//ILS("%s() level=%d not change, current adc=%d, th=%d, tl=%d\n", 
		//		__func__, lp_info->current_level, adc, lp_info->als_th, lp_info->als_tl);
		return lp_info->current_level;
	}
	lp_info->old_level = lp_info->current_level;
	/*set ls threshould*/
	mutex_lock(&als_thd);
	lp_info->als_tl = (i == 0 || adc == 0) ? 0:lp_info->adc_table[i-1];
	lp_info->als_th = lp_info->adc_table[i];
	set_lsensor_range(lp_info->als_tl, lp_info->als_th);
	
	/*report ls input event*/

	input_report_abs(lp_info->ls_input_dev, ABS_MISC, lp_info->current_level);
	input_sync(lp_info->ls_input_dev);
	//ILS("%s() lsensor level=%d, current adc = %d, low_thd=%d, hight_thd=%d\n",
	//		__func__, lp_info->current_level, adc, lp_info->als_tl, lp_info->als_th);
	mutex_unlock(&als_thd);
	return lp_info->current_level;
}


static void sensor_irq_do_work(struct work_struct *work)
{
	uint8_t data;
	//uint16_t ps_data;
	data = INT_SETTING;
	/*check interrupt flag*/

	i2c_RxData(&data);
	DPS("%s() 0x%x=0x%x\n", __func__, INT_SETTING, data);
	if (data & IF_PS_INT)
			EPS("%s() already disable proximity interrupt but occur interrupt\n", __func__);
	/*als interrupt*/
	if (data & IF_ALS_INT) {
		report_lsensor_input_event();
	} else {
		ELS("%s() unknow interrupt int_setting = 0x%x\n", __func__, data);
	}

	enable_irq(lp_info->irq);
}

static ssize_t ls_level_show(struct device* dev, struct device_attribute *attr, \
					char* buf)
{
	int ret;
	ret = sprintf(buf, "%d\n", report_lsensor_input_event());
	return ret;
}

static ssize_t ls_level_store(struct device* dev, struct device_attribute *attr, \
					const char* buf, size_t count)
{
	int enable;
	sscanf(buf, "%d", &enable);

	return count;
}
static DEVICE_ATTR(ls_level, 0664, ls_level_show, ls_level_store);

static ssize_t ps_level_show(struct device* dev, struct device_attribute *attr, \
					char* buf)
{
	int ret;
	queue_delayed_work(lp_info->lp_wq, &polling_work, msecs_to_jiffies(POLLING_DELAY));
	ret = sprintf(buf, "%d\n", ps_data_level);
	return ret;
}

static ssize_t ps_level_store(struct device* dev, struct device_attribute *attr, \
					const char* buf, size_t count)
{
	int enable;
	sscanf(buf, "%d", &enable);

	return count;
}
static DEVICE_ATTR(ps_level, 0664, ps_level_show, ps_level_store);

static irqreturn_t sfh7776_irq_handler(int irq, void *data)
{
	disable_irq_nosync(lp_info->irq);
	queue_work(lp_info->lp_wq, &sensor_irq_work);
	return IRQ_HANDLED;
}

static int setup_sfh7776(void)
{
	char data[2];
	int ret = 0;
	
	ret = gpio_request(lp_info->intr_pin, "sfh7776");
	if (ret < 0) {
		EPS("%s() request gpio %d failed\n", __func__, lp_info->intr_pin);
		return ret;
	}

	ret = gpio_direction_input(lp_info->intr_pin);
	if (ret < 0) {
		EPS("%s() failed to set gpio %d input\n", __func__, lp_info->intr_pin);
		goto failed_free_gpio;
	}

	/*set LED pulse to 200mA, ALS gain x128*/
	data[0] = MODE_CONTROL;
	data[1] = lp_info->mode_control;
	i2c_TxData(data, sizeof(data));
	
	/*active ALS/PS, repetition 100ms*/
	data[0] = ALS_PS_CONTROL;
	data[1] = lp_info->als_ps_control;
	i2c_TxData(data, sizeof(data));
	/*delay 100ms*/
	msleep(100);
	/*interrupt setting*/
	data[0] = INT_SETTING;
	data[1] = lp_info->int_setting;
	i2c_TxData(data, sizeof(data));

	ret = request_any_context_irq(lp_info->irq, 
			sfh7776_irq_handler, 
			IRQF_TRIGGER_FALLING,
			"sfh7776",
			lp_info);
	if (ret < 0) {
		EPS("%s() request irq %d failed\n", __func__, lp_info->irq);
		goto failed_free_gpio;
	}
	return ret;
failed_free_gpio:
	gpio_free(lp_info->intr_pin);
	return ret;
}
static int psensor_open(struct inode *inode, struct file *file)
{
	DPS("%s\n", __func__);

	if (lp_info->psensor_opened)
		return -EBUSY;

	lp_info->psensor_opened = 1;
	return 0;
}

static int psensor_release(struct inode* inode, struct file *file)
{
	D();
	lp_info->psensor_opened = 0;
	return ps_hal_enable ? psensor_disable():0;
}

static long psensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val, ret;
	IPS("%s() cmd=%d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
		case PROXIMITY_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user*)arg)) {
				EPS("%s() get usr failed\n", __func__);
				return -EFAULT;
			}
			if (val) {
				ret = psensor_enable();
				if (!ret)
					ps_hal_enable = 1;
				return ret;
			} else {
				ret = psensor_disable();
				if (!ret)
					ps_hal_enable = 0;
				return ret;
			}
			break;	
		case PROXIMITY_IOCTL_GET_ENABLED:
			return put_user(lp_info->ps_enable, (unsigned long __user*)arg);
			break;
		default:
			EPS("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
			return -EINVAL;	
	}
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl,
}; 

static struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sfh7776_proximity",
	.fops = &psensor_fops,

};


static int psensor_setup(void)
{
	int ret;
#ifndef POLLING_PROXIMITY
	char data[2];
	/*set ps threshold*/
	data[0] = PS_TH_MSB;
	data[1] = lp_info->ps_th_msb;
	i2c_TxData(data, sizeof(data));
	data[0] = PS_TH_LSB;
	data[1] = lp_info->ps_th_lsb;
	i2c_TxData(data, sizeof(data));
	data[0] = PS_TL_MSB;
	data[1] = lp_info->ps_tl_msb;
	i2c_TxData(data, sizeof(data));
	data[0] = PS_TL_LSB;
	data[1] = lp_info->ps_tl_lsb;
	i2c_TxData(data, sizeof(data));
#endif
	/*register input device*/
	lp_info->ps_input_dev = input_allocate_device();
	if (!lp_info->ps_input_dev) {
		EPS("%s() allocate input device failed\n", __func__);
		return -ENOMEM;
	}
	lp_info->ps_input_dev->name = "sfh7776_proximity";
	set_bit(EV_ABS, lp_info->ps_input_dev->evbit);
	input_set_abs_params(lp_info->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lp_info->ps_input_dev);
	if (ret) {
		EPS("%s() register psensor input device failed\n", __func__);
		goto err_register_ps_input_device;
	}

	/*register misc device*/
	ret = misc_register(&psensor_misc);
	if (ret) {
		EPS("%s() register psensor misc failed\n", __func__);
		goto err_ps_misc_register;
	}
	return 0;
err_ps_misc_register:
	input_unregister_device(lp_info->ps_input_dev);
err_register_ps_input_device:
	input_free_device(lp_info->ps_input_dev);
	return ret;
}

static int lsensor_enable(void)
{
	
	if (lp_info->ls_enable) {
		lp_info->ls_enable++;
		ILS("%s() lsensor already enabled: %d\n", __func__, lp_info->ls_enable);
		return -EBUSY;
	}
	D();
	input_report_abs(lp_info->ls_input_dev, ABS_MISC, -1);
	input_sync(lp_info->ls_input_dev);
	/*report level and set lsensor threshold*/
	report_lsensor_input_event();
	
	lsensor_enable_intr();
	lp_info->ls_enable = 1;
	return 0;
}

static int lsensor_disable(void)
{
	if (lp_info->ls_enable > 1) {
		lp_info->ls_enable--;
		return 0;
	} else if (lp_info->ls_enable == 0) {
		ILS("%s() lsensor already disable\n", __func__);
		return -EBUSY;
	}
	D();
	lsensor_disable_intr();
	lp_info->ls_enable = 0;
	lp_info->old_level = -1;
	return 0;
}
static int lsensor_open(struct inode *inode, struct file *file)
{
	D();
	if (lp_info->lightsensor_opened) {
		ILS("%s() lsensor already opened\n", __func__);
		return -EBUSY;
	}
	lp_info->lightsensor_opened = 1;
	return 0;
}

static int lsensor_release(struct inode *inode, struct file *file)
{
	D();
	lp_info->lightsensor_opened = 0;
	return 0;
}



static long lsensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret, val;
	ILS("%s() cmd=%d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
		case LIGHTSENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user*)arg)) {
				ret = -EFAULT;
				break;
			}
			DLS("%s() LIGHTSENSOR_IOCTL_ENABLE: %d\n", __func__, val);
			ret = val ? lsensor_enable() : lsensor_disable();
			break;
		case LIGHTSENSOR_IOCTL_GET_ENABLED:
			DLS("%s() LIGHTSENSOR_IOCTL_GET_ENABLED: %d\n", __func__, lp_info->ls_enable);
			ret = put_user(lp_info->ls_enable, (unsigned long __user*)arg);
			break;
		default:
			ELS("%s() invaild cmd %d\n", __func__, _IOC_NR(cmd));
			ret = -EINVAL;
	}

	return ret;
}

static struct file_operations lsensor_fops = {
	.owner		= THIS_MODULE,
	.open		= lsensor_open,
	.release	= lsensor_release,
	.unlocked_ioctl = lsensor_ioctl, 
};

static struct miscdevice lsensor_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "sfh7776_lightsensor",
	.fops	= &lsensor_fops,
};

static int lsensor_setup(void)
{
	int ret;

	lp_info->ls_input_dev = input_allocate_device();
	if (!lp_info->ls_input_dev) {
		ELS("%s() allocate input device failed\n", __func__);
		return -ENOMEM;
	}
	lp_info->ls_input_dev->name = "sfh7776_lightsensor";
	set_bit(EV_ABS, lp_info->ls_input_dev->evbit);
	input_set_abs_params(lp_info->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lp_info->ls_input_dev);
	if (ret) {
		ELS("%s() lsensor register input device failed\n", __func__);
		goto err_register_ls_input_device;
	}

	ret = misc_register(&lsensor_misc);
	if (ret) {
		ELS("%s() register lsensor misc \n", __func__);
		goto err_ls_misc_register;
	}
	return 0;
err_ls_misc_register:
	input_unregister_device(lp_info->ls_input_dev);
err_register_ls_input_device:
	input_free_device(lp_info->ls_input_dev);
	return ret;
}

static int sfh7776_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifndef CONFIG_OF
	struct sfh7776_platform_data *pdata;
#else
	struct device_node *np = client->dev.of_node;
	unsigned long irq_flags;
	uint16_t adc_table[10]={ 3, 5, 7, 275, 1142, 3404, 4465, 5739, 9399, 65535};
#endif
	struct sfh7776_info *lpi;
	int ret = 0;
	int irq = 0;
	char int_setting, data;
	lpi = kzalloc(sizeof(struct sfh7776_info), GFP_KERNEL);
	if (lp_info)
		EPS("%s() allocate platform data failed", __func__);
	lpi->i2c_client = client;

#ifdef CONFIG_OF

	client->irq = of_get_named_gpio_flags(np, "irq-gpio", 0,(enum of_gpio_flags *)&irq_flags);
	//client->irq = gpio_to_irq(client->irq);
	
	/*chip info*/
	lpi->slave_address	= client->addr; //pdata->slave_address;
	lpi->mode_control	= 0x06; //pdata->mode_control;
	lpi->als_ps_control	= 0x3f; //pdata->als_ps_control;
	lpi->ps_th_lsb		= 0x20; //pdata->ps_th_lsb;
	lpi->ps_th_msb		= 0x00; //pdata->ps_th_msb;
	lpi->ps_tl_lsb		= 0x10; //pdata->ps_tl_lsb;
	lpi->ps_tl_msb		= 0x00; //pdata->ps_tl_msb;
	lpi->als_vis_th_lsb	= 0x00; //pdata->als_vis_th_lsb;
	lpi->als_vis_th_msb	= 0x00; //pdata->als_vis_th_msb;
	lpi->als_vis_tl_lsb	= 0x00; //pdata->als_vis_tl_lsb;
	lpi->als_vis_tl_msb	= 0x00; //pdata->als_vis_tl_msb;
	lpi->ps_th		= ((0x00 & 0x0f) << 8) + 0x20;
	lpi->ps_tl		= ((0x00 & 0x0f) << 8) + 0x10;
	DPS("ps_th=0x%x, ps_tl=0x%x\n", lpi->ps_th, lpi->ps_tl);
	
	lpi->adc_table		= adc_table; //pdata->adc_table;
	//lpi->intr_pin		= pdata->intr;
	lpi->intr_pin		= client->irq;
	lpi->int_setting	= (3 << 4); //pdata->int_setting;
	irq = client->irq; //gpio_to_irq(pdata->intr);
#else
	pdata = client->dev.platform_data;
	if (!pdata) {
		EPS("%s() platform data invailed\n", __func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}
		/*chip info*/
	lpi->slave_address	= pdata->slave_address;
	lpi->mode_control	= pdata->mode_control;
	lpi->als_ps_control	= pdata->als_ps_control;
	lpi->ps_th_lsb		= pdata->ps_th_lsb;
	lpi->ps_th_msb		= pdata->ps_th_msb;
	lpi->ps_tl_lsb		= pdata->ps_tl_lsb;
	lpi->ps_tl_msb		= pdata->ps_tl_msb;
	lpi->als_vis_th_lsb	= pdata->als_vis_th_lsb;
	lpi->als_vis_th_msb	= pdata->als_vis_th_msb;
	lpi->als_vis_tl_lsb	= pdata->als_vis_tl_lsb;
	lpi->als_vis_tl_msb	= pdata->als_vis_tl_msb;
	lpi->ps_th		= ((pdata->ps_th_msb & 0x0f) << 8) + pdata->ps_th_lsb;
	lpi->ps_tl		= ((pdata->ps_tl_msb & 0x0f) << 8) + pdata->ps_tl_lsb;
	DPS("ps_th=0x%x, ps_tl=0x%x\n", lpi->ps_th, lpi->ps_tl);
	
	lpi->adc_table		= pdata->adc_table;
	lpi->intr_pin		= pdata->intr;
	lpi->int_setting	= pdata->int_setting;
	irq = gpio_to_irq(pdata->intr);
#endif

	lpi->irq = irq;
	
	lp_info = lpi;
	
	ps_hal_enable = 0;
	lp_info->ps_enable = 0;
	lp_info->old_distance = PS_INIT_DISTANCE;
	lp_info->old_level = -1;
	/*check chip id*/
	if (check_id()) {
		ret = -EINVAL;
		goto err_check_id_failed;
	}	
	
	lpi->lp_wq = create_singlethread_workqueue("sfh7776_wq");
	if (!lpi->lp_wq) {
		EPS("%s() create workqueue failed\n", __func__);
		goto err_create_workqueue;
	}

	/*setup mode*/
	ret = setup_sfh7776();
	if (ret < 0) {
		EPS("%s() setup failed\n", __func__);
		goto err_setup;
	}
	mutex_init(&als_ps_mutex);
	mutex_init(&als_thd);
		/*create class*/
	lpi->sfh7776_class = class_create(THIS_MODULE, "optical_sensor");
	if (IS_ERR(lpi->sfh7776_class)) {
		ret = PTR_ERR(lpi->sfh7776_class);
		lpi->sfh7776_class = NULL;
		goto err_create_class_failed;
	}
	
	/*light sensor device*/
	lpi->ls_dev = device_create(lpi->sfh7776_class, NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_lsdev_failed;
	}

	ret = device_create_file(lpi->ls_dev, &dev_attr_rw_register);
	if (ret) {
		ELS("%s() create rw_register file failed\n", __func__);
		goto err_create_rw_register_file;
	}

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_enable);
	if (ret) {
		ELS("%s() create ls_enable file failedn\n", __func__);
		goto err_create_ls_enable_file;
	}

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_level);
	if (ret) {
		ELS("%s() create ls_level file failedn\n", __func__);
		goto err_create_ls_level_file;
	}

	/*proximity sensor*/
	lpi->ps_dev = device_create(lpi->sfh7776_class, NULL, 0, "%s", "proximity");
	if (unlikely(IS_ERR(lpi->ps_dev))) {
		ret = PTR_ERR(lpi->ps_dev);
		lpi->ps_dev = NULL;
		goto err_create_psdev_failed;
	}

	ret = device_create_file(lpi->ps_dev, &dev_attr_psensor_enable);
	if (ret) {
		EPS("%s() create psensor_enable failed\n", __func__);
		goto err_create_psensor_enable_file;
	}
	
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_level);
	if (ret) {
		ELS("%s() create ps_level file failedn\n", __func__);
		goto err_create_ps_level_file;
	}
	
	ret = psensor_setup();
	if (ret) {
		EPS("%s() psensor setup failed\n", __func__);
		goto err_psensor_setup;
	}

	ret = lsensor_setup();
	if (ret) {
		ELS("%s() lsensor setup failed\n", __func__);
		goto err_lsensor_setup;
	}
	
	data = INT_SETTING;
	i2c_RxData(&data);
	int_setting = data;

	IPS("%s() intr_pin=%d, irq=%d, int_setting=0x%x\n", __func__, lpi->intr_pin, irq, int_setting);
	I("%s() ok!\n", __func__);
	return 0;

err_lsensor_setup:
	input_unregister_device(lp_info->ps_input_dev);
	input_free_device(lp_info->ps_input_dev);
err_psensor_setup:
err_create_psensor_enable_file:
	device_unregister(lpi->ps_dev);
err_create_psdev_failed:
err_create_ls_enable_file:
err_create_ls_level_file:
err_create_ps_level_file:
err_create_rw_register_file:
	device_unregister(lpi->ls_dev);
err_create_lsdev_failed:
	class_destroy(lpi->sfh7776_class);
err_create_class_failed:
	destroy_workqueue(lpi->lp_wq);
err_create_workqueue:
	mutex_destroy(&als_ps_mutex);
err_setup:
err_check_id_failed:
#ifndef CONFIG_OF
err_platform_data_null:
#endif
	kfree(lpi);
	EPS("%s() failed\n", __func__);
	return ret;
}

static struct i2c_device_id sfh7776_id[] = {
	{SFH7776_NAME, 0},
	{}
};

static struct of_device_id lsensor_sfh7776_dt_ids[] = {
    { .compatible = "lsensor,sfh7776" },
    { }
};

static struct i2c_driver sfh7776_driver = {
	.id_table = sfh7776_id,
	.probe = sfh7776_probe,
	.driver = {
		.name = SFH7776_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(lsensor_sfh7776_dt_ids),
	},
};
static int sfh7776_init(void)
{
	return i2c_add_driver(&sfh7776_driver);
}
static void sfh7776_exit(void)
{
	i2c_del_driver(&sfh7776_driver);
}
module_init(sfh7776_init);
module_exit(sfh7776_exit);
MODULE_DESCRIPTION("SFH7776 Driver");
MODULE_LICENSE("GPL");
