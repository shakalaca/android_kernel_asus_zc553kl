/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"
#include <linux/proc_fs.h>

#define FLASH_NAME "ti,sy7806"

#define CONFIG_MSMB_CAMERA_DEBUG_7806
#ifdef CONFIG_MSMB_CAMERA_DEBUG_7806
#define sy7806_DBG(fmt, args...) pr_debug(fmt, ##args)
#else
#define sy7806_DBG(fmt, args...)
#endif


static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver sy7806_i2c_driver;
struct msm_camera_power_ctrl_t *power_info_test = NULL;

static struct msm_camera_i2c_reg_array sy7806_init_array[] = {
	{0x0A, 0x00},
	{0x08, 0x07},
	{0x09, 0x19},
};

static struct msm_camera_i2c_reg_array sy7806_off_array[] = {
	{0x0A, 0x00},
};

static struct msm_camera_i2c_reg_array sy7806_release_array[] = {
	{0x0A, 0x00},
};

static struct msm_camera_i2c_reg_array sy7806_low_array[] = {
	{0x0A, 0x22},
};

static struct msm_camera_i2c_reg_array sy7806_high_array[] = {
	{0x0A, 0x23},
};


static const struct of_device_id sy7806_i2c_trigger_dt_match[] = {
	{.compatible = "ti,sy7806"},
	{}
};

MODULE_DEVICE_TABLE(of, sy7806_i2c_trigger_dt_match);
static const struct i2c_device_id sy7806_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

//static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				//enum led_brightness value)
static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value){
	if (value > LED_OFF) {
		fctrl.flash_op_current[0]=value;
		fctrl.flash_op_current[1]=value;
		if(fctrl.func_tbl->flash_led_low)
			fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
};

static struct led_classdev msm_torch_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_sy7806_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
	printk("%s %d,allenyao\n", __func__,__LINE__);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		printk("%s %d,allenyao\n", __func__,__LINE__);
		return rc;
	}

	return 0;
};

//add by allenyao
int sy7806_torch_duty=0;
static int sy7806_read(struct seq_file *buf, void *v)
{
	//int Range = 0;
      //Range = 0;
	seq_printf(buf, "%d\n", sy7806_torch_duty);
	  return 0;
}
static int sy7806_open(struct inode *inode, struct  file *file)
{
	pr_err("%s: Enter\n", __func__);
	pr_err("%s: Exit\n", __func__);
	return single_open(file, sy7806_read, NULL);
}
static ssize_t sy7806_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	//int val;
	char messages[10];
	ssize_t ret_len=0;
	pr_err("%s: Enter\n", __func__);


	ret_len = len;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sy7806_torch_duty = (int)simple_strtol(messages, NULL, 10);
	pr_err("%s: exit,val is %d\n", __func__,sy7806_torch_duty);

	if (sy7806_torch_duty > 0) {
		fctrl.flash_op_current[0]=sy7806_torch_duty;
		fctrl.flash_op_current[1]=sy7806_torch_duty;
		if(fctrl.func_tbl->flash_led_low)
			fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
	
	return ret_len;
}

static const struct file_operations sy7806_fops = {
	.owner = THIS_MODULE,
	.open = sy7806_open,//ATD_Laura_device_get_range_open,single_open
	.write = sy7806_write,//ATD_Laura_device_calibration_write,single_write
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

//struct proc_dir_entry *mytest_file = proc_create("driver/mytest", 0x0644, NULL, ATD_laser_focus_device_calibration_fops);
//end

int msm_flash_sy7806_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	//char data[0];	int reset_value=0;
	sy7806_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	/*gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);*/
		
		//add by allenyao enable flash
		gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
		msleep(100);
	#if 0
	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x07,data,1);
	reset_value =(int)data[0];
	//soft reset
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x07,(reset_value | 0x80),1);
		//end
		//end
	#endif
		#if 0//not  use by allenyao
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
		#endif
	return 0;
	return rc;
}

//add by allenyao
int msm_flash_sy7806_led_init_test(void)
{
	int rc = 0;
	sy7806_DBG("%s:%d called\n", __func__, __LINE__);
		//add by allenyao enable flash
		gpio_set_value_cansleep(
		power_info_test->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
		msleep(100);
	return 0;
	return rc;
}
//end

int msm_flash_sy7806_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;


	sy7806_DBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	/*gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);*/
		
		//add by allenyao disable flash
		gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
		msleep(100);



	#if 0//not use by allenyao
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->release_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	#endif
	return rc;
	//return 0;
}

//add by allenyao
int msm_flash_sy7806_led_release_test(void)
{
	int rc = 0;
		//add by allenyao disable flash
		gpio_set_value_cansleep(
		power_info_test->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
		msleep(100);


	return rc;
	//return 0;
}
//end

int msm_flash_sy7806_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	char data[0];	int enable_value=0;

	sy7806_DBG("%s:%d called\n", __func__, __LINE__);

	if (!fctrl || !fctrl->flashdata) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	/*
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	msleep(100);
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	 gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
	msleep(10);
	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
			fctrl->flash_i2c_client,0x0c,data,1);
	sy7806_DBG("%s:%d called,in is  %d,\n", __func__, __LINE__,data[0]);
	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
			fctrl->flash_i2c_client,0x03,data,1);
	msleep(10);*/
	//add by allenyao for test
	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x01,data,1);
	enable_value =(int)data[0];
	
	//2.set dutu
	/*rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x05,0x10,1);
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x06,0x10,1);
	//1.write mode and enable
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x08,0x1f,1);*/
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x01,(enable_value & 0xfc),1);
    //modify for bug#213155 by chensy@iopenlink.com
	//msleep(1000);
	//end
	sy7806_DBG("%s:%d called,in is  %d,\n", __func__, __LINE__,data[0]);
	/*
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}*/

	/*gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);*/

	//return rc;
	return 0;
}

int msm_flash_sy7806_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint32_t torch_op_curren_0=0,torch_op_curren_1=0;
	uint32_t curren_0_reg_val=0,curren_1_reg_val=0;
	int enable_value=0;char data[0];	int reset_value=0;
	sy7806_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	//add by allenyao
	torch_op_curren_0=fctrl->flash_op_current[0];
	torch_op_curren_1=fctrl->flash_op_current[1];
	sy7806_DBG("%s:%d torch_op_curren_0 is %d\n", __func__, __LINE__,torch_op_curren_0);
	sy7806_DBG("%s:%d torch_op_curren_1 is %d\n", __func__, __LINE__,torch_op_curren_1);

	//turn the current to duty regist
	curren_0_reg_val=torch_op_curren_0*10/14;
	curren_1_reg_val=torch_op_curren_1*10/14;
	sy7806_DBG("%s:%d curren_0_reg_val is %d\n", __func__, __LINE__,curren_0_reg_val);
	sy7806_DBG("%s:%d curren_1_reg_val is %d\n", __func__, __LINE__,curren_1_reg_val);
	if(curren_0_reg_val>0x7f)
		curren_0_reg_val=0x7f;
	if(curren_1_reg_val>0x7f)
		curren_1_reg_val=0x7f;
	#if 1
	// enable flash
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x07,data,1);
	reset_value =(int)data[0];
	#if 1
	//soft reset
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x07,((reset_value | 0x80)),1);
	//qudiao duan lu bao hu
	if((torch_op_curren_0+torch_op_curren_1)<500){
		rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
			fctrl->flash_i2c_client,0x07,data,1);
		reset_value =(int)data[0];
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl->flash_i2c_client,0x07,(reset_value&0xf7),1);
	}
	//end
	#endif

	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x01,data,1);
	enable_value =(int)data[0];

		//2.set dutu
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x03,0,1);
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x04,0,1);

	#if 1
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x05,curren_0_reg_val&0x7f,1);
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x06,curren_1_reg_val&0x7f,1);
	#else
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x05,0xa,1);//7F,0E,11,55,66
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x06,0xa,1);//7F,0E,11,55,66
	#endif

		//1.write mode and enable
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x08,0x1f,1);
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x01,(enable_value & 0x80) | (0x08 | 0x01|0x02),1);	
	//rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		//fctrl->flash_i2c_client,0x01,0x8b,1);	
	#endif
	//end

	//
	#if 1
	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x01,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x01  is 0x%x\n", __func__, __LINE__,enable_value);

	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x07,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x07  is 0x%x\n", __func__, __LINE__,enable_value);
	
	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x0a,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x0a  is 0x%x\n", __func__, __LINE__,enable_value);
	
	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x0b,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x0b  is 0x%x\n", __func__, __LINE__,enable_value);

	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x0c,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x0c  is 0x%x\n", __func__, __LINE__,enable_value);

	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x0d,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x0d  is 0x%x\n", __func__, __LINE__,enable_value);
	#endif
	
	//
	
	/*gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}*/

	//return rc;
	return 0;
}

//add by allenyao for test
struct DUTY{
	int high_duty;
	int low_duty;
};
int msm_flash_sy7806_led_low_test(struct DUTY duty)
{
	int rc = 0;
	uint32_t torch_op_curren_0=0,torch_op_curren_1=0;
	uint32_t curren_0_reg_val=0,curren_1_reg_val=0;
	int enable_value=0;char data[0];	int reset_value=0;
	sy7806_DBG("%s:%d called\n", __func__, __LINE__);

	//add by allenyao
	torch_op_curren_0=duty.high_duty;
	torch_op_curren_1=duty.low_duty;
	sy7806_DBG("%s:%d torch_op_curren_0 is %d\n", __func__, __LINE__,torch_op_curren_0);
	sy7806_DBG("%s:%d torch_op_curren_1 is %d\n", __func__, __LINE__,torch_op_curren_1);

	//turn the current to duty regist
	curren_0_reg_val=torch_op_curren_0*10/14;
	curren_1_reg_val=torch_op_curren_1*10/14;
	sy7806_DBG("%s:%d curren_0_reg_val is %d\n", __func__, __LINE__,curren_0_reg_val);
	sy7806_DBG("%s:%d curren_1_reg_val is %d\n", __func__, __LINE__,curren_1_reg_val);
	if(curren_0_reg_val>0x7f)
		curren_0_reg_val=0x7f;
	if(curren_1_reg_val>0x7f)
		curren_1_reg_val=0x7f;
	#if 1
	// enable flash
	gpio_set_value_cansleep(
		power_info_test->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl.flash_i2c_client,0x07,data,1);
	reset_value =(int)data[0];
	#if 1
	//soft reset
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x07,((reset_value | 0x80)),1);
	//qudiao duan lu bao hu
	if((torch_op_curren_0+torch_op_curren_1)<500){
		rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
			fctrl.flash_i2c_client,0x07,data,1);
		reset_value =(int)data[0];
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl.flash_i2c_client,0x07,(reset_value&0xf7),1);
	}
	//end
	#endif

	rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl.flash_i2c_client,0x01,data,1);
	enable_value =(int)data[0];

		//2.set dutu
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x03,0,1);
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x04,0,1);

	#if 1
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x05,curren_0_reg_val&0x7f,1);
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x06,curren_1_reg_val&0x7f,1);
	#else
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x05,0xa,1);//7F,0E,11,55,66
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x06,0xa,1);//7F,0E,11,55,66
	#endif

		//1.write mode and enable
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x08,0x1f,1);
	if((duty.high_duty>=0)&(duty.low_duty>=0)){	
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl.flash_i2c_client,0x01,(enable_value & 0x80) | (0x08 | 0x01|0x02),1);	
	}else if((duty.high_duty>=0)&(duty.low_duty<0)){
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl.flash_i2c_client,0x01,(enable_value & 0x80) | (0x08 | 0x01|0x00),1);		
	}else if((duty.high_duty<0)&(duty.low_duty>=0)){
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl.flash_i2c_client,0x01,(enable_value & 0x80) | (0x08 | 0x00|0x02),1);		
	}else{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl.flash_i2c_client,0x01,(enable_value & 0x80) | (0x08 | 0x00|0x00),1);		
	}
	//rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		//fctrl->flash_i2c_client,0x01,0x8b,1);	
	#endif
	//end

	//
	#if 1
	rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl.flash_i2c_client,0x01,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x01  is 0x%x\n", __func__, __LINE__,enable_value);

	rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl.flash_i2c_client,0x07,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x07  is 0x%x\n", __func__, __LINE__,enable_value);
	
	rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl.flash_i2c_client,0x0a,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x0a  is 0x%x\n", __func__, __LINE__,enable_value);
	
	rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl.flash_i2c_client,0x0b,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x0b  is 0x%x\n", __func__, __LINE__,enable_value);

	rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl.flash_i2c_client,0x0c,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x0c  is 0x%x\n", __func__, __LINE__,enable_value);

	rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl.flash_i2c_client,0x0d,data,1);
	enable_value =(int)data[0];
	sy7806_DBG("%s:%d 0x0d  is 0x%x\n", __func__, __LINE__,enable_value);
	#endif
	
	return 0;
}
//end

int msm_flash_sy7806_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint32_t flash_op_curren_0=0,flash_op_curren_1=0;
	uint32_t curren_0_reg_val=0,curren_1_reg_val=0;
	int enable_value=0;char data[0];int reset_value=0;
	sy7806_DBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;

	power_info = &flashdata->power_info;
	//add by allenyao
	flash_op_curren_0=fctrl->flash_op_current[0];
	flash_op_curren_1=fctrl->flash_op_current[1];
	sy7806_DBG("%s:%d flash_op_curren_0 is %d\n", __func__, __LINE__,flash_op_curren_0);
	sy7806_DBG("%s:%d flash_op_curren_1 is %d\n", __func__, __LINE__,flash_op_curren_1);

	if(flash_op_curren_0>=1000)
		flash_op_curren_0=1000;
	if(flash_op_curren_1>=1000)
		flash_op_curren_1=1000;
	sy7806_DBG("%s:%d flash_op_curren_0 is %d\n", __func__, __LINE__,flash_op_curren_0);
	sy7806_DBG("%s:%d flash_op_curren_1 is %d\n", __func__, __LINE__,flash_op_curren_1);

	//turn the current to duty regist
	curren_0_reg_val=flash_op_curren_0*1000/11725;
	curren_1_reg_val=flash_op_curren_1*1000/11725;
	sy7806_DBG("%s:%d curren_0_reg_val is %d\n", __func__, __LINE__,curren_0_reg_val);
	sy7806_DBG("%s:%d curren_1_reg_val is %d\n", __func__, __LINE__,curren_1_reg_val);
	if(curren_0_reg_val>0x7f)
		curren_0_reg_val=0x7f;
	if(curren_1_reg_val>0x7f)
		curren_1_reg_val=0x7f;
	#if 1
	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x07,data,1);
	reset_value =(int)data[0];
	//soft reset
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x07,(reset_value | 0x80),1);
		//end
	#endif

	rc=fctrl->flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl->flash_i2c_client,0x01,data,1);
	enable_value =(int)data[0];

		//2.set dutu
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x05,0,1);
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x06,0,1);
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x03,curren_0_reg_val&0x7f,1);
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x04,curren_1_reg_val&0x7f,1);

		//1.write mode and enable
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x08,0x1f,1);
	rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl->flash_i2c_client,0x01,(enable_value & 0x80) | (0x0c | 0x01|0x02),1);
	
	
	//end

	/*gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}*/

	//return rc;
	return 0;
}
//add by alenyao
int msm_flash_sy7806_led_high_test(struct DUTY duty)
{
	int rc = 0;
	uint32_t flash_op_curren_0=0,flash_op_curren_1=0;
	uint32_t curren_0_reg_val=0,curren_1_reg_val=0;
	int enable_value=0;char data[0];int reset_value=0;
	sy7806_DBG("%s:%d called\n", __func__, __LINE__);
	//add by allenyao
	flash_op_curren_0=duty.high_duty;
	flash_op_curren_1=duty.low_duty;
	sy7806_DBG("%s:%d flash_op_curren_0 is %d\n", __func__, __LINE__,flash_op_curren_0);
	sy7806_DBG("%s:%d flash_op_curren_1 is %d\n", __func__, __LINE__,flash_op_curren_1);


	//turn the current to duty regist
	curren_0_reg_val=flash_op_curren_0*1000/11725;
	curren_1_reg_val=flash_op_curren_1*1000/11725;
	sy7806_DBG("%s:%d curren_0_reg_val is %d\n", __func__, __LINE__,curren_0_reg_val);
	sy7806_DBG("%s:%d curren_1_reg_val is %d\n", __func__, __LINE__,curren_1_reg_val);
	if(curren_0_reg_val>0x7f)
		curren_0_reg_val=0x7f;
	if(curren_1_reg_val>0x7f)
		curren_1_reg_val=0x7f;
	#if 1
	rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl.flash_i2c_client,0x07,data,1);
	reset_value =(int)data[0];
	//soft reset
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x07,(reset_value | 0x80),1);
		//end
	#endif
		#if 1
	//soft reset
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x07,((reset_value | 0x80)),1);
	//qudiao duan lu bao hu
	if((flash_op_curren_0+flash_op_curren_1)<2000){
		rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
			fctrl.flash_i2c_client,0x07,data,1);
		reset_value =(int)data[0];
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl.flash_i2c_client,0x07,(reset_value&0xf7),1);
	}
	//end
	#endif


	rc=fctrl.flash_i2c_client->i2c_func_tbl->i2c_read_seq(
		fctrl.flash_i2c_client,0x01,data,1);
	enable_value =(int)data[0];

		//2.set dutu
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x05,0,1);
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x06,0,1);
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x03,curren_0_reg_val&0x7f,1);
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x04,curren_1_reg_val&0x7f,1);

		//1.write mode and enable
	rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
		fctrl.flash_i2c_client,0x08,0x1f,1);
	if((duty.high_duty>=0)&(duty.low_duty>=0)){	
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl.flash_i2c_client,0x01,(enable_value & 0x80) | (0x0c | 0x01|0x02),1);
		sy7806_DBG("%s:%d open tow leds\n", __func__, __LINE__);
	}else if((duty.high_duty>=0)&(duty.low_duty<0)){
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl.flash_i2c_client,0x01,(enable_value & 0x80) | (0x0c | 0x01|0x00),1);
		sy7806_DBG("%s:%d open high led\n", __func__, __LINE__);
	}else if((duty.high_duty<0)&(duty.low_duty>=0)){
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl.flash_i2c_client,0x01,(enable_value & 0x80) | (0x0c | 0x00|0x02),1);
		sy7806_DBG("%s:%d open low led\n", __func__, __LINE__);
	}else{
		rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl.flash_i2c_client,0x01,(enable_value & 0x80) | (0x0c | 0x00|0x00),1);
		sy7806_DBG("%s:%d not  open  leds\n", __func__, __LINE__);
	}
	
	
	//end
	return 0;
}
//end

//add by allenyao for control flash 
#if 1
//add for flashlight_cal by allenyao
static int flashduty_test1=-1;
static int flashduty_test2=-1;
static int open_flag4test=0;
static ssize_t flash3_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sy7806_DBG("[LED]get backlight duty value is:%d\n", flashduty_test1);
	return sprintf(buf, "%d\n", flashduty_test1);
}

static ssize_t flash3_store(struct device *dev, struct device_attribute *attr, const char *buf,
			  size_t size)
{

	unsigned char arg;
	char *pvalue=NULL;
	struct DUTY duty1;

	sy7806_DBG("Enter!\n");

	if((size != 2) || ((buf[0] != '0') && (buf[0] != '1'))){
		sy7806_DBG(" command!count: %d, buf: %c\n",(int)size,buf[0]);
		//return -EINVAL;
	}


	if (buf != NULL && size != 0) {
		flashduty_test1 = simple_strtol(buf, &pvalue, 0);
		sy7806_DBG("flashduty_test1: %d\n", flashduty_test1);

		if (*pvalue ) {
			flashduty_test2 = simple_strtol((pvalue + 1), NULL, 0);
			sy7806_DBG("flashduty_test2: %d\n", flashduty_test2);
		}
	}

	duty1.high_duty=flashduty_test1;
	duty1.low_duty=flashduty_test2;

	//arg = ((unsigned char)buf[0]) - '0';
	//flashduty2 = ((unsigned char)buf[0]) - '0';

	

	if((flashduty_test1>=0)||(flashduty_test2>=0))
		arg=1;
	else
		arg=0;
	
	sy7806_DBG("OnOff: %d\n", arg);
	msm_flash_sy7806_led_init_test();


	//open leds resource
	if(open_flag4test){
			sy7806_DBG("open fail!\n");
			if((flashduty_test1<0)&&(flashduty_test2<0))
				goto out;
			else if(open_flag4test>0)
				goto work3;
			else
				return -1;
	}else{
		open_flag4test+=1;
	}


	if(flashduty_test1>=1000)
		flashduty_test1=1000;
	if(flashduty_test2>=1000)
		flashduty_test2=1000;
work3:
	if((flashduty_test1<0)&&(flashduty_test2<0)){
		sy7806_DBG("flashduty_test1: %d,flashduty_test2:%d,%d\n", flashduty_test1,flashduty_test2,__LINE__);
	}else if(((flashduty_test1>=0)&&(flashduty_test1<=1))&&(flashduty_test2<0)){
		sy7806_DBG("flashduty_test1: %d,flashduty_test2:%d,%d\n", flashduty_test1,flashduty_test2,__LINE__);
		msm_flash_sy7806_led_high_test(duty1);
	}else if((flashduty_test1>1)&&(flashduty_test2<0)){
		sy7806_DBG("flashduty_test1: %d,flashduty_test2:%d,%d\n", flashduty_test1,flashduty_test2,__LINE__);
		msm_flash_sy7806_led_high_test(duty1);
	}else if(((flashduty_test2>=0)&&(flashduty_test2<=1))&&(flashduty_test1<0)){
		sy7806_DBG("flashduty_test1: %d,flashduty_test2:%d,%d\n", flashduty_test1,flashduty_test2,__LINE__);
		msm_flash_sy7806_led_high_test(duty1);
	}else if((flashduty_test2>1)&&(flashduty_test1<0)){
		sy7806_DBG("flashduty_test1: %d,flashduty_test2:%d,%d\n", flashduty_test1,flashduty_test2,__LINE__);
		msm_flash_sy7806_led_high_test(duty1);
	}else if(((flashduty_test1>=0)&&(flashduty_test1<=1))&&((flashduty_test2>=0)&&(flashduty_test2<=1))){
		sy7806_DBG("flashduty_test1: %d,flashduty_test2:%d,%d\n", flashduty_test1,flashduty_test2,__LINE__);
		msm_flash_sy7806_led_high_test(duty1);

	}else{
		sy7806_DBG("flashduty_test1: %d,flashduty_test2:%d,%d\n", flashduty_test1,flashduty_test2,__LINE__);
		msm_flash_sy7806_led_high_test(duty1);
	}

	mdelay(800);
	msm_flash_sy7806_led_release_test();
	open_flag4test=0;

	sy7806_DBG("Exit!\n");
	return size;

out:
	open_flag4test=0;
	return -1;

}


//static DEVICE_ATTR(flash3, 0666, show_flashduty_test, store_flashduty_test);
static DEVICE_ATTR_RW(flash3);

//end  by allenyao 
#endif


#if 1
//add for flashlight_cal by allenyao
static int flashduty_test3=-1;
static int flashduty_test4=-1;
static int open_flag4test_1=0;
static ssize_t flash4_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sy7806_DBG("[LED]get backlight duty value is:%d\n", flashduty_test3);
	return sprintf(buf, "%d\n", flashduty_test3);
}

static ssize_t flash4_store(struct device *dev, struct device_attribute *attr, const char *buf,
			  size_t size)
{

	unsigned char arg;
	char *pvalue=NULL;
	struct DUTY duty1;

	sy7806_DBG("Enter!\n");

	if((size != 2) || ((buf[0] != '0') && (buf[0] != '1'))){
		sy7806_DBG(" command!count: %d, buf: %c\n",(int)size,buf[0]);
		//return -EINVAL;
	}


	if (buf != NULL && size != 0) {
		flashduty_test3= simple_strtol(buf, &pvalue, 0);
		sy7806_DBG("flashduty_test1: %d\n", flashduty_test3);

		if (*pvalue ) {
			flashduty_test4 = simple_strtol((pvalue + 1), NULL, 0);
			sy7806_DBG("flashduty_test2: %d\n", flashduty_test4);
		}
	}

	duty1.high_duty=flashduty_test3;
	duty1.low_duty=flashduty_test4;

	//arg = ((unsigned char)buf[0]) - '0';
	//flashduty2 = ((unsigned char)buf[0]) - '0';

	

	if((flashduty_test3>=0)||(flashduty_test4>=0))
		arg=1;
	else
		arg=0;
	
	sy7806_DBG("OnOff: %d\n", arg);
	msm_flash_sy7806_led_init_test();


	//open leds resource
	if(open_flag4test_1){
			sy7806_DBG("open fail!\n");
			if((flashduty_test3<0)&&(flashduty_test4<0))
				goto out;
			else if(open_flag4test_1>0)
				goto work3;
			else
				return -1;
	}else{
		open_flag4test_1+=1;
	}


	if(flashduty_test3>=200)
		flashduty_test3=200;
	if(flashduty_test4>=200)
		flashduty_test4=200;
work3:
	if((flashduty_test3<0)&&(flashduty_test4<0)){
		msm_flash_sy7806_led_release_test();
		sy7806_DBG("flashduty_test3: %d,flashduty_test4:%d,%d\n", flashduty_test3,flashduty_test4,__LINE__);
	}else if(((flashduty_test3>=0)&&(flashduty_test4<=1))&&(flashduty_test2<0)){
		sy7806_DBG("flashduty_test3: %d,flashduty_test4:%d,%d\n", flashduty_test3,flashduty_test4,__LINE__);
		msm_flash_sy7806_led_low_test(duty1);
	}else if((flashduty_test3>1)&&(flashduty_test4<0)){
		sy7806_DBG("flashduty_test3: %d,flashduty_test4:%d,%d\n", flashduty_test3,flashduty_test4,__LINE__);
		msm_flash_sy7806_led_low_test(duty1);
	}else if(((flashduty_test3>=0)&&(flashduty_test4<=1))&&(flashduty_test3<0)){
		sy7806_DBG("flashduty_test3: %d,flashduty_test4:%d,%d\n", flashduty_test3,flashduty_test4,__LINE__);
		msm_flash_sy7806_led_low_test(duty1);
	}else if((flashduty_test4>1)&&(flashduty_test3<0)){
		sy7806_DBG("flashduty_test3: %d,flashduty_test4:%d,%d\n", flashduty_test3,flashduty_test4,__LINE__);
		msm_flash_sy7806_led_low_test(duty1);
	}else if(((flashduty_test3>=0)&&(flashduty_test3<=1))&&((flashduty_test4>=0)&&(flashduty_test4<=1))){
		sy7806_DBG("flashduty_test3: %d,flashduty_test4:%d,%d\n", flashduty_test3,flashduty_test4,__LINE__);
		msm_flash_sy7806_led_low_test(duty1);

	}else{
		sy7806_DBG("flashduty_test3: %d,flashduty_test4:%d,%d\n", flashduty_test3,flashduty_test4,__LINE__);
		msm_flash_sy7806_led_low_test(duty1);
	}

	//mdelay(800);
	//msm_flash_sy7806_led_release_test();
	open_flag4test_1=0;

	sy7806_DBG("Exit!\n");
	return size;

out:
	open_flag4test_1=0;
	return -1;

}


//static DEVICE_ATTR(flash3, 0666, show_flashduty_test, store_flashduty_test);
static DEVICE_ATTR_RW(flash4);

//end  by allenyao 
#endif

//end
static struct class *flashlight_class;
static struct device *flashlight_device;
static dev_t flashlight_devno;
#define ALLOC_DEVNO
#define FLASHLIGHT_DEVNAME            "kd_camera_flashlight"
static struct cdev flashlight_cdev;
static const struct file_operations flashlight_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = NULL,
	.open = NULL,
	.release = NULL,
#ifdef CONFIG_COMPAT
	.compat_ioctl = NULL,
#endif
};
static int msm_flash_sy7806_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	sy7806_DBG("%s entry\n", __func__);
	if (!id) {
		pr_err("msm_flash_sy7806_i2c_probe: id is NULL");
		id = sy7806_i2c_id;
	}
	rc = msm_flash_i2c_probe(client, id);
	sy7806_DBG("%s %d,allenyao\n", __func__,__LINE__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;
	power_info_test= &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		printk("%s %d,allenyao\n", __func__,__LINE__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		pr_err("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
	}

#ifdef ALLOC_DEVNO
	rc = alloc_chrdev_region(&flashlight_devno, 0, 1, FLASHLIGHT_DEVNAME);
	if (rc) {
		sy7806_DBG("[flashlight_probe] alloc_chrdev_region fail: %d ~", rc);
		//goto flashlight_probe_error;
	} else {
		sy7806_DBG("[flashlight_probe] major: %d, minor: %d ~", MAJOR(flashlight_devno),
		     MINOR(flashlight_devno));
	}
	cdev_init(&flashlight_cdev, &flashlight_fops);
	flashlight_cdev.owner = THIS_MODULE;
	rc = cdev_add(&flashlight_cdev, flashlight_devno, 1);
	if (rc) {
		sy7806_DBG("[flashlight_probe] cdev_add fail: %d ~", rc);
		//goto flashlight_probe_error;
	}
#else
#define FLASHLIGHT_MAJOR 242
	rc = register_chrdev(FLASHLIGHT_MAJOR, FLASHLIGHT_DEVNAME, &flashlight_fops);
	if (rc != 0) {
		logI("[flashlight_probe] Unable to register chardev on major=%d (%d) ~",
		     FLASHLIGHT_MAJOR, rc);
		return rc;
	}
	flashlight_devno = MKDEV(FLASHLIGHT_MAJOR, 0);
#endif

	flashlight_class = class_create(THIS_MODULE, "flashlightdrv");
	if (IS_ERR(flashlight_class)) {
		pr_err("[flashlight_probe] Unable to create class, err = %d ~",
		     (int)PTR_ERR(flashlight_class));
		return  -1 ;
	}

	flashlight_device =
	    device_create(flashlight_class, NULL, flashlight_devno, NULL, FLASHLIGHT_DEVNAME);
	if (NULL == flashlight_device) {
		sy7806_DBG("[flashlight_probe] device_create fail ~");
		//goto flashlight_probe_error;
    }
	rc = device_create_file(flashlight_device, &dev_attr_flash3);
	if (rc) {
		sy7806_DBG("[flashlight_probe]device_create_file flash3 fail!\n");
	}
	rc = device_create_file(flashlight_device, &dev_attr_flash4);
	if (rc) {
		sy7806_DBG("[flashlight_probe]device_create_file flash4 fail!\n");
	}

	if (!rc)
		msm_sy7806_torch_create_classdev(&(client->dev),NULL);
	return rc;
}

static int msm_flash_sy7806_i2c_remove(struct i2c_client *client)
{
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0 ;
	sy7806_DBG("%s entry\n", __func__);
	flashdata = fctrl.flashdata;
	power_info = &flashdata->power_info;

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl.pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl.pinctrl_info.pinctrl,
				fctrl.pinctrl_info.gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
	}
	return rc;
}


static struct i2c_driver sy7806_i2c_driver = {
	.id_table = sy7806_i2c_id,
	.probe  = msm_flash_sy7806_i2c_probe,
	.remove = msm_flash_sy7806_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sy7806_i2c_trigger_dt_match,
	},
};

static int __init msm_flash_sy7806_init(void)
{
	struct proc_dir_entry *mytest_file;
	mytest_file=proc_create("driver/asus_flash_brightness", 0x0644, NULL, &sy7806_fops);
	sy7806_DBG("%s entry\n", __func__);
	return i2c_add_driver(&sy7806_i2c_driver);
}

static void __exit msm_flash_sy7806_exit(void)
{
	sy7806_DBG("%s entry\n", __func__);
	i2c_del_driver(&sy7806_i2c_driver);
	return;
}


static struct msm_camera_i2c_client sy7806_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting sy7806_init_setting = {
	.reg_setting = sy7806_init_array,
	.size = ARRAY_SIZE(sy7806_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7806_off_setting = {
	.reg_setting = sy7806_off_array,
	.size = ARRAY_SIZE(sy7806_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7806_release_setting = {
	.reg_setting = sy7806_release_array,
	.size = ARRAY_SIZE(sy7806_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7806_low_setting = {
	.reg_setting = sy7806_low_array,
	.size = ARRAY_SIZE(sy7806_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7806_high_setting = {
	.reg_setting = sy7806_high_array,
	.size = ARRAY_SIZE(sy7806_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t sy7806_regs = {
	.init_setting = &sy7806_init_setting,
	.off_setting = &sy7806_off_setting,
	.low_setting = &sy7806_low_setting,
	.high_setting = &sy7806_high_setting,
	.release_setting = &sy7806_release_setting,
};

static struct msm_flash_fn_t sy7806_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_sy7806_led_init,
	.flash_led_release = msm_flash_sy7806_led_release,
	.flash_led_off = msm_flash_sy7806_led_off,
	.flash_led_low = msm_flash_sy7806_led_low,
	.flash_led_high = msm_flash_sy7806_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &sy7806_i2c_client,
	.reg_setting = &sy7806_regs,
	.func_tbl = &sy7806_func_tbl,
};

module_init(msm_flash_sy7806_init);
module_exit(msm_flash_sy7806_exit);
MODULE_DESCRIPTION("sy7806 FLASH");
MODULE_LICENSE("GPL v2");
