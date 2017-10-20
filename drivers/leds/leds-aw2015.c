/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/leds-aw2013.h>
#include <linux/err.h>


/* register address */
#define AW_REG_RESET			0x00
#define AW_REG_GLOBAL_CONTROL		0x01
#define AW_REG_LED_STATUS		0x02
#define AW_REG_LED_ENABLE		0x30
#define AW_REG_LED_CONFIG_BASE		0x31
#define AW_REG_LED_BRIGHTNESS_BASE	0x34
#define AW_REG_TIMESET0_BASE		0x37
#define AW_REG_TIMESET1_BASE		0x38

/* register bits */
#define AW2013_CHIPID			0x31
#define AW_LED_MOUDLE_ENABLE_MASK	0x01
#define AW_LED_FADE_OFF_MASK		0x40
#define AW_LED_FADE_ON_MASK		0x20
#define AW_LED_BREATHE_MODE_MASK	0x10
#define AW_LED_RESET_MASK		0x55

#define AW_LED_RESET_DELAY		8
#define AW2013_VDD_MIN_UV		2600000
#define AW2013_VDD_MAX_UV		3300000
#define AW2013_VI2C_MIN_UV		1800000
#define AW2013_VI2C_MAX_UV		1800000

#define MAX_RISE_TIME_MS		7
#define MAX_HOLD_TIME_MS		5
#define MAX_FALL_TIME_MS		7
#define MAX_OFF_TIME_MS			5
//#define LED_DEBUG_AW2015		1
struct aw2013_led {
    struct i2c_client *client;
    struct led_classdev cdev;
    struct aw2013_platform_data *pdata;
    struct work_struct brightness_work;
    struct mutex lock;
    struct regulator *vdd;
    struct regulator *vcc;
    int num_leds;
    int id;
    bool poweron;
};
static void dumpreg(struct aw2013_led *led);

static int aw2013_write(struct aw2013_led *led, u8 reg, u8 val)
{
    return i2c_smbus_write_byte_data(led->client, reg, val);
}

static int aw2013_read(struct aw2013_led *led, u8 reg, u8 *val)
{
    s32 ret;

    ret = i2c_smbus_read_byte_data(led->client, reg);
    if (ret < 0)
        return ret;

    *val = ret;
    return 0;
}
#if 0
static int aw2013_power_on(struct aw2013_led *led, bool on)
{
    int rc;

    if (on) {
#if 0
        rc = regulator_enable(led->vdd);
        if (rc) {
            dev_err(&led->client->dev,
                    "Regulator vdd enable failed rc=%d\n", rc);
            return rc;
        }
#endif
        rc = regulator_enable(led->vcc);
        if (rc) {
            dev_err(&led->client->dev,
                    "Regulator vcc enable failed rc=%d\n", rc);
            goto fail_enable_reg;
        }
        led->poweron = true;
    } else {
#if 0

        rc = regulator_disable(led->vdd);
        if (rc) {
            dev_err(&led->client->dev,
                    "Regulator vdd disable failed rc=%d\n", rc);
            return rc;
        }
#endif
        rc = regulator_disable(led->vcc);
        if (rc) {
            dev_err(&led->client->dev,
                    "Regulator vcc disable failed rc=%d\n", rc);
            goto fail_disable_reg;
        }
        led->poweron = false;
    }
    dev_err(&led->client->dev,"aw2015 vcc enable =%d\n", led->poweron);
    return rc;

fail_enable_reg:
    rc = regulator_disable(led->vdd);
    if (rc)
        dev_err(&led->client->dev,
                "Regulator vdd disable failed rc=%d\n", rc);

    return rc;

fail_disable_reg:
    rc = regulator_enable(led->vdd);
    if (rc)
        dev_err(&led->client->dev,
                "Regulator vdd enable failed rc=%d\n", rc);

    return rc;
}
#endif
#if 0
static int aw2013_power_init(struct aw2013_led *led, bool on)
{
    int rc;

    if (on) {
        led->vdd = regulator_get(&led->client->dev, "vdd");
        if (IS_ERR(led->vdd)) {
            rc = PTR_ERR(led->vdd);
            dev_err(&led->client->dev,
                    "Regulator get failed vdd rc=%d\n", rc);
            return rc;
        }

        if (regulator_count_voltages(led->vdd) > 0) {
            rc = regulator_set_voltage(led->vdd, AW2013_VDD_MIN_UV,
                    AW2013_VDD_MAX_UV);
            if (rc) {
                dev_err(&led->client->dev,
                        "Regulator set_vtg failed vdd rc=%d\n",
                        rc);
                goto reg_vdd_put;
            }
        }

        led->vcc = regulator_get(&led->client->dev, "vcc");
        if (IS_ERR(led->vcc)) {
            rc = PTR_ERR(led->vcc);
            dev_err(&led->client->dev,
                    "Regulator get failed vcc rc=%d\n", rc);
            goto reg_vdd_set_vtg;
        }

        if (regulator_count_voltages(led->vcc) > 0) {
            rc = regulator_set_voltage(led->vcc, AW2013_VI2C_MIN_UV,
                    AW2013_VI2C_MAX_UV);
            if (rc) {
                dev_err(&led->client->dev,
                        "Regulator set_vtg failed vcc rc=%d\n", rc);
                goto reg_vcc_put;
            }
        }
    } else {
        if (regulator_count_voltages(led->vdd) > 0)
            regulator_set_voltage(led->vdd, 0, AW2013_VDD_MAX_UV);

        regulator_put(led->vdd);

        if (regulator_count_voltages(led->vcc) > 0)
            regulator_set_voltage(led->vcc, 0, AW2013_VI2C_MAX_UV);

        regulator_put(led->vcc);
    }
    return 0;

reg_vcc_put:
    regulator_put(led->vcc);
reg_vdd_set_vtg:
    if (regulator_count_voltages(led->vdd) > 0)
        regulator_set_voltage(led->vdd, 0, AW2013_VDD_MAX_UV);
reg_vdd_put:
    regulator_put(led->vdd);
    return rc;
}
#endif
static void aw2013_brightness_work(struct work_struct *work)
{
    struct aw2013_led *led = container_of(work, struct aw2013_led,
            brightness_work);
    u8 val=0;

    mutex_lock(&led->pdata->led->lock);
#ifdef LED_DEBUG_AW2015
    dev_err(&led->pdata->led->client->dev, "brightness_work aw2015 power on=%d\n",led->pdata->led->poweron);
#endif
#if 0
    /* enable regulators if they are disabled */
    if (!led->pdata->led->poweron) {
        if (aw2013_power_on(led->pdata->led, true)) {
            dev_err(&led->pdata->led->client->dev, "aw2015 power on failed\n");
            mutex_unlock(&led->pdata->led->lock);
            return;
        }
    }
#endif
    dev_err(&led->pdata->led->client->dev, " aw2015 power on set brightnessn=%d",led->cdev.brightness);
#if 0
    if (led->cdev.brightness > 0) {
        if (led->cdev.brightness > led->cdev.max_brightness)
            led->cdev.brightness = led->cdev.max_brightness;
        aw2013_write(led, AW_REG_GLOBAL_CONTROL,
                AW_LED_MOUDLE_ENABLE_MASK);
        aw2013_write(led, AW_REG_LED_CONFIG_BASE + led->id,
                led->pdata->max_current);
        aw2013_write(led, AW_REG_LED_BRIGHTNESS_BASE + led->id,
                led->cdev.brightness);
        aw2013_read(led, AW_REG_LED_ENABLE, &val);
        aw2013_write(led, AW_REG_LED_ENABLE, val | (1 << led->id));
    } else {
        aw2013_read(led, AW_REG_LED_ENABLE, &val);
        aw2013_write(led, AW_REG_LED_ENABLE, val & (~(1 << led->id)));
    }
    aw2013_read(led, AW_REG_LED_ENABLE, &val);
#else
    if (led->cdev.brightness ==255) {
        if(strcmp(led->cdev.name, "red") == 0)
        {
            //aw2013_write(led, 0x00, 0x55);
            aw2013_write(led, 0x01, 0x03);
            aw2013_write(led, 0x03, 0x01);
            aw2013_write(led, 0x04, 0x00);
            //aw2013_write(led, 0x05, 0x00);
            //aw2013_write(led, 0x06, 0x00);
            aw2013_write(led, 0x08, 0x00);
            aw2013_write(led, 0x10, 0xff);//red
            aw2013_write(led, 0x1c, 0xff);
            aw2013_write(led, 0x1d, 0xff);
            aw2013_write(led, 0x1e, 0xff);
            aw2013_read(led, 0x07, &val);	
            aw2013_write(led, 0x07, (val|0x01));	
            printk("line=%d,func=%s, brightness=%d\n",__LINE__,__func__,led->cdev.brightness);
        }else if(strcmp(led->cdev.name, "green") == 0)
        {
            // aw2013_write(led, 0x00, 0x55);
            aw2013_write(led, 0x01, 0x03);
            aw2013_write(led, 0x03, 0x03);
            //aw2013_write(led, 0x04, 0x00);
            aw2013_write(led, 0x05, 0x00);
            //aw2013_write(led, 0x06, 0x00);
            aw2013_write(led, 0x08, 0x00);
            aw2013_write(led, 0x11, 0x17);//green
            aw2013_write(led, 0x1c, 0xff);
            aw2013_write(led, 0x1d, 0xff);
            aw2013_write(led, 0x1e, 0xff);
            aw2013_read(led, 0x07, &val);		
            aw2013_write(led, 0x07, (val|0x02));

            printk("line=%d,func=%s, brightness=%d\n",__LINE__,__func__,led->cdev.brightness);
        }
#ifdef LED_DEBUG_AW2015
        dev_err(&led->pdata->led->client->dev, " aw2015 brightness=%d,led en=%d\n",led->cdev.brightness,val);
#endif
    }
    else  
    {
        if(led->cdev.brightness>0){
            if(strcmp(led->cdev.name, "red") == 0)
            {
                //aw2013_write(led, 0x00, 0x55);
                aw2013_write(led, 0x01, 0x03);
                aw2013_write(led, 0x03, 0x01);
                aw2013_write(led, 0x04, 0x01);
                //aw2013_write(led, 0x05, 0x00);
                //aw2013_write(led, 0x06, 0x00);
                aw2013_write(led, 0x08, 0x00);
                aw2013_read(led, 0x07, &val);	
                aw2013_write(led, 0x07, (val|0x01));
                aw2013_write(led, 0x10, 0xFF);//red
                aw2013_write(led, 0x1c, 0xff);
                //aw2013_write(led, 0x1d, 0xff);
                //aw2013_write(led, 0x1e, 0xff);

                aw2013_write(led, 0x30, 0x02);
                aw2013_write(led, 0x31, 0x0B);
                aw2013_write(led, 0x32, 0x00);
                //aw2013_write(led, 0x35, 0x64);
                //aw2013_write(led, 0x36, 0x64);
                //aw2013_write(led, 0x37, 0x08);
                //aw2013_write(led, 0x3A, 0x64);
                //aw2013_write(led, 0x3B, 0x64);
                //aw2013_write(led, 0x3C, 0x08);
                aw2013_read(led, 0x09, &val);	
                aw2013_write(led, 0x09, (val|0x01));
#ifdef LED_DEBUG_AW2015
                printk("line=%d,func=%s, brightness=%d\n",__LINE__,__func__,led->cdev.brightness);
#endif
            }else if(strcmp(led->cdev.name, "green") == 0)

            {
                //  aw2013_write(led, 0x00, 0x55);
                aw2013_write(led, 0x01, 0x03);
                aw2013_write(led, 0x03, 0x01);
                //aw2013_write(led, 0x04, 0x00);
                aw2013_write(led, 0x05, 0x01);
                //aw2013_write(led, 0x06, 0x00);
                aw2013_write(led, 0x08, 0x00);
                aw2013_read(led, 0x07, &val);		
                aw2013_write(led, 0x07, (val|0x02));
                aw2013_write(led, 0x11, 0xff);//green
                //aw2013_write(led, 0x1c, 0xff);
                aw2013_write(led, 0x1d, 0xff);
                //aw2013_write(led, 0x1e, 0xff);
                //     aw2013_write(led, 0x30, 0x02);
                //aw2013_write(led, 0x31, 0x08);
                //aw2013_write(led, 0x32, 0x00);
                aw2013_write(led, 0x35, 0x02);
                aw2013_write(led, 0x36, 0x08);
                aw2013_write(led, 0x37, 0x08);
                //aw2013_write(led, 0x3A, 0x64);
                //aw2013_write(led, 0x3B, 0x64);
                //aw2013_write(led, 0x3C, 0x08);
                aw2013_read(led, 0x09, &val);	
                aw2013_write(led, 0x09, (val|0x02));
#ifdef LED_DEBUG_AW2015
                printk("line=%d,func=%s, brightness=%d\n",__LINE__,__func__,led->cdev.brightness);
#endif
            }
#ifdef LED_DEBUG_AW2015
            dev_err(&led->pdata->led->client->dev, " aw2015 brightness=%d,led en=%d\n",led->cdev.brightness,val);
#endif
        }
        else 
        {
            if(strcmp(led->cdev.name, "red") == 0)
            {
#ifdef LED_DEBUG_AW2015
                printk("line=%d,func=%s, brightness=%d\n",__LINE__,__func__,led->cdev.brightness);
#endif
                aw2013_write(led, 0x08, 0x00);
                aw2013_read(led, 0x07, &val);	
                aw2013_write(led, 0x07, (val&0xfe));
            }else if(strcmp(led->cdev.name, "green") == 0)

            {
                aw2013_write(led, 0x08, 0x00);
                aw2013_read(led, 0x07, &val);
                aw2013_write(led, 0x07, (val&0xfd));
#ifdef LED_DEBUG_AW2015		 
                printk("line=%d,func=%s, brightness=%d\n",__LINE__,__func__,led->cdev.brightness);
#endif
            }
#ifdef LED_DEBUG_AW2015
            pr_err("line=%d, aw2015 brightness=%d,led en=%d\n",__LINE__,led->cdev.brightness,val);
#endif
        }
    }

#if 0
    if (led->cdev.brightness > 0) {
        if (led->cdev.brightness > led->cdev.max_brightness)
            led->cdev.brightness = led->cdev.max_brightness;
        aw2013_write(led, 0x33, 1);
        aw2013_write(led, 0x4, 0);//LED Operating Mode Select. 0:manual mode.
        aw2013_write(led, AW_REG_GLOBAL_CONTROL,AW_LED_MOUDLE_ENABLE_MASK);
        aw2013_write(led, 0x10 + led->id,led->cdev.brightness);//The LED1 output current value is IMAX * ILED1_y / 255.
        aw2013_read(led, 0x07, &val);
#ifdef LED_DEBUG_AW2015
        dev_err(&led->pdata->led->client->dev, " aw2015 brightness=%d,led en=%d\n",led->cdev.brightness,val);
#endif
        aw2013_write(led, 0x07, val | (1 << led->id));
    }else{
        aw2013_read(led, 0x07, &val);
#ifdef LED_DEBUG_AW2015
        dev_err(&led->pdata->led->client->dev, " aw2015 brightness=%d,led en=%d\n",led->cdev.brightness,val);
#endif
        aw2013_write(led, 0x07, val & (~(1 << led->id)));
    }

#endif
#ifdef LED_DEBUG_AW2015
    dev_err(&led->pdata->led->client->dev, " aw2015 led en2=%d\n",val);
    dumpreg(led);
#endif
#endif
    /*
     * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
     * all off. So we need to power it off.
     */
#if 0
    if (val == 0) {
        if (aw2013_power_on(led->pdata->led, false)) {
            dev_err(&led->pdata->led->client->dev,
                    "power off failed");
            mutex_unlock(&led->pdata->led->lock);
            return;
        }
    }
#endif

    mutex_unlock(&led->pdata->led->lock);
}


#if 0
static void aw2013_led_blink_set(struct aw2013_led *led, unsigned long blinking)
{


    u8 val=0;

    /* enable regulators if they are disabled */
    if (!led->pdata->led->poweron) {
        if (aw2013_power_on(led->pdata->led, true)) {
            dev_err(&led->pdata->led->client->dev, "aw2013_led_blink_set power on failed\n");
            return;
        }
    }

    led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;

    if (blinking > 0) {
        aw2013_write(led, AW_REG_GLOBAL_CONTROL,
                AW_LED_MOUDLE_ENABLE_MASK);
        aw2013_write(led, AW_REG_LED_CONFIG_BASE + led->id,
                AW_LED_FADE_OFF_MASK | AW_LED_FADE_ON_MASK |
                AW_LED_BREATHE_MODE_MASK | led->pdata->max_current);
        aw2013_write(led, AW_REG_LED_BRIGHTNESS_BASE + led->id,
                led->cdev.brightness);
        aw2013_write(led, AW_REG_TIMESET0_BASE + led->id * 3,
                led->pdata->rise_time_ms << 4 |
                led->pdata->hold_time_ms);
        aw2013_write(led, AW_REG_TIMESET1_BASE + led->id * 3,
                led->pdata->fall_time_ms << 4 |
                led->pdata->off_time_ms);
        aw2013_read(led, AW_REG_LED_ENABLE, &val);
        aw2013_write(led, AW_REG_LED_ENABLE, val | (1 << led->id));
    } else {
        aw2013_read(led, AW_REG_LED_ENABLE, &val);
        aw2013_write(led, AW_REG_LED_ENABLE, (val & (~(1 << led->id))));
    }

    aw2013_read(led, AW_REG_LED_ENABLE, &val);
    /*
     * If value in AW_REG_LED_ENABLE is 0, it means the RGB leds are
     * all off. So we need to power it off.
     */

    if (val == 0) {
        if (aw2013_power_on(led->pdata->led, false)) {
            dev_err(&led->pdata->led->client->dev,
                    "power off failed");
            return;
        }
    }

    u8 val=0;
    led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;

    //aw2013_write(led, 0x00, 0x55);		// software reset

    //aw2013_write(led, 0x01, 0x03);		// GCR
    aw2013_write(led, 0x03, 0x01);		// IMAX
    aw2013_write(led, 0x04, 0x01);		// LCFG1
    aw2013_write(led, 0x05, 0x01);		// LCFG2
    aw2013_write(led, 0x06, 0x01);		// LCFG3
    //aw2013_write(led, 0x07, 0x07);		// LEDEN
    aw2013_write(led, 0x08, 0x00);		// LEDCTR
    aw2013_write(led, 0x1C, 0xFF);		// PWM1
    aw2013_write(led, 0x1D, 0xFF);		// PWM2
    aw2013_write(led, 0x1E, 0xFF);		// PWM3	


    aw2013_write(led,0x30, 0x64); //risetime:1.04s on time 0.51s
    aw2013_write(led,0x31, 0x64); // fall time:1.04s off time 0.51s
    aw2013_write(led,0x32, 0x00); // delay time 0S
    aw2013_write(led,0x35, 0x64); //risetime:1.04s on time 0.51s
    aw2013_write(led,0x36, 0x64); // fall time:1.04s off time 0.51s
    aw2013_write(led,0x37, 0x00); // delay time 2.1s
    aw2013_write(led,0x3A, 0x64); //risetime:1.04s on time 0.51s
    aw2013_write(led,0x3B, 0x64); // fall time:1.04s off time 0.51s
    aw2013_write(led,0x3C, 0x00); // delay time 2.1s

#ifdef LED_DEBUG_AW2015
    dev_err(&led->pdata->led->client->dev, " aw2015 blink set led id=%d\n",led->id);
#endif
    if (blinking > 0) {
        //aw2013_write(led, AW_REG_GLOBAL_CONTROL,AW_LED_MOUDLE_ENABLE_MASK);
        aw2013_write(led, 0x10 + led->id,led->cdev.brightness);
        aw2013_read(led, 0x07, &val);
#ifdef LED_DEBUG_AW2015
        dev_err(&led->pdata->led->client->dev, " aw2015 blink set brightness=%d,led en=%d\n",led->cdev.brightness,val);
#endif
        aw2013_write(led, 0x07, val | (1 << led->id));
        aw2013_read(led, 0x09, &val);
#ifdef LED_DEBUG_AW2015
        dev_err(&led->pdata->led->client->dev, " aw2015 blink set led run=%d\n",val | (1 << led->id));
#endif
        aw2013_write(led, 0x09, val | (1 << led->id));// PAT_RIN
    }else{
        aw2013_read(led, 0x07, &val);
#ifdef LED_DEBUG_AW2015
        dev_err(&led->pdata->led->client->dev, " aw2015 blink set brightness=%d,led en=%d\n",led->cdev.brightness,val);
#endif
        aw2013_write(led, 0x07, val & (~(1 << led->id)));
        aw2013_read(led, 0x09, &val);
#ifdef LED_DEBUG_AW2015
        dev_err(&led->pdata->led->client->dev, " aw2015 blink set led run=%d\n",val & (~(1 << led->id)));
#endif
        aw2013_write(led, 0x09, val & (~(1 << led->id)));// PAT_RIN
    }
    /*
       aw2013_write(led, 0x32, 0x00);		// PAT_T3				Tdelay
       aw2013_write(led, 0x33, 0x00);		// PAT_T4 	  PAT_CTR & Color
       aw2013_write(led, 0x34, 0x00);		// PAT_T5		    Timer
       */
#ifdef LED_DEBUG_AW2015
    dumpreg(led);
#endif


}
#endif

static void aw2013_set_brightness(struct led_classdev *cdev,
        enum led_brightness brightness)
{
    struct aw2013_led *led = container_of(cdev, struct aw2013_led, cdev);
    led->cdev.brightness = brightness;
    printk("func==%s,line=%d,brightness=%d\n",__func__,__LINE__,brightness);
#ifdef LED_DEBUG_AW2015
    printk("aw2015 set_brightness\n");
#endif
    schedule_work(&led->brightness_work);
}

/*static ssize_t aw2013_store_blink(struct device *dev,
  struct device_attribute *attr,
  const char *buf, size_t len)
  {
  unsigned long blinking;
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct aw2013_led *led =
  container_of(led_cdev, struct aw2013_led, cdev);
  ssize_t ret = -EINVAL;

  ret = kstrtoul(buf, 10, &blinking);
  if (ret)
  return ret;
  mutex_lock(&led->pdata->led->lock);
  aw2013_led_blink_set(led, blinking);
  mutex_unlock(&led->pdata->led->lock);

  return len;
  }

  static ssize_t aw2013_led_time_show(struct device *dev,
  struct device_attribute *attr, char *buf)
  {
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct aw2013_led *led =
  container_of(led_cdev, struct aw2013_led, cdev);

  return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
  led->pdata->rise_time_ms, led->pdata->hold_time_ms,
  led->pdata->fall_time_ms, led->pdata->off_time_ms);
  }

  static ssize_t aw2013_led_time_store(struct device *dev,
  struct device_attribute *attr,
  const char *buf, size_t len)
  {
  struct led_classdev *led_cdev = dev_get_drvdata(dev);
  struct aw2013_led *led =
  container_of(led_cdev, struct aw2013_led, cdev);
  int rc, rise_time_ms, hold_time_ms, fall_time_ms, off_time_ms;

  rc = sscanf(buf, "%d %d %d %d",
  &rise_time_ms, &hold_time_ms,
  &fall_time_ms, &off_time_ms);

  mutex_lock(&led->pdata->led->lock);
  led->pdata->rise_time_ms = (rise_time_ms > MAX_RISE_TIME_MS) ?
MAX_RISE_TIME_MS : rise_time_ms;
led->pdata->hold_time_ms = (hold_time_ms > MAX_HOLD_TIME_MS) ?
MAX_HOLD_TIME_MS : hold_time_ms;
led->pdata->fall_time_ms = (fall_time_ms > MAX_FALL_TIME_MS) ?
MAX_FALL_TIME_MS : fall_time_ms;
led->pdata->off_time_ms = (off_time_ms > MAX_OFF_TIME_MS) ?
MAX_OFF_TIME_MS : off_time_ms;
aw2013_led_blink_set(led, 1);
mutex_unlock(&led->pdata->led->lock);
return len;
}

static DEVICE_ATTR(blink, 0664, NULL, aw2013_store_blink);
static DEVICE_ATTR(led_time, 0664, aw2013_led_time_show, aw2013_led_time_store);

static struct attribute *aw2013_led_attributes[] = {
&dev_attr_blink.attr,
&dev_attr_led_time.attr,
NULL,
};

static struct attribute_group aw2013_led_attr_group = {
.attrs = aw2013_led_attributes
};*/
static void dumpreg(struct aw2013_led *led)
{
    u8 i = 0,j = 0,val = 0;

    for(i = 0;i <10;i++){
        aw2013_read(led, i, &val);
        printk("aw2015 reg:0x%x = 0x%x\n",i,val);
    }
    for(i = 0x10;i <0x1f;i++){
        aw2013_read(led, i, &val);
        printk("aw2015 reg:0x%x = 0x%x\n",i,val);
    }
    for(j = 0x30;j <0x3f;j++){
        aw2013_read(led, j, &val);
        printk("aw2015 reg:0x%x = 0x%x\n",j,val);
    }
}
static void aw2015_init(struct aw2013_led *led)
{
    aw2013_write(led, 0x4, 0);
    aw2013_write(led, 0x7, 0);
    aw2013_write(led, 0x8, 8);
    aw2013_write(led, 0x33, 1);
    aw2013_write(led, 0x10, 0);
    aw2013_write(led, 0x1, 1);
}
static int aw_2013_check_chipid(struct aw2013_led *led)
{
    u8 val=0;

    aw2013_write(led, AW_REG_RESET, AW_LED_RESET_MASK);
    //usleep(AW_LED_RESET_DELAY);
    usleep_range(AW_LED_RESET_DELAY, AW_LED_RESET_DELAY);
    aw2013_read(led, AW_REG_RESET, &val);
    printk("aw2015 check_chipid=0x %x\n",val);
    if (val == AW2013_CHIPID)
        return 0;
    else
        return -EINVAL;
}

static int aw2013_led_err_handle(struct aw2013_led *led_array,
        int parsed_leds)
{
    int i;
    /*
     * If probe fails, cannot free resource of all LEDs, only free
     * resources of LEDs which have allocated these resource really.
     */
    for (i = 0; i < parsed_leds; i++) {
        //sysfs_remove_group(&led_array[i].cdev.dev->kobj,
        //	&aw2013_led_attr_group);
        led_classdev_unregister(&led_array[i].cdev);
        cancel_work_sync(&led_array[i].brightness_work);
        devm_kfree(&led_array->client->dev, led_array[i].pdata);
        led_array[i].pdata = NULL;
    }
    return i;
}

static int aw2013_led_parse_child_node(struct aw2013_led *led_array,
        struct device_node *node)
{
    struct aw2013_led *led;
    struct device_node *temp;
    struct aw2013_platform_data *pdata;
    int rc = 0, parsed_leds = 0;

    for_each_child_of_node(node, temp) {
        led = &led_array[parsed_leds];
        led->client = led_array->client;

        pdata = devm_kzalloc(&led->client->dev,
                sizeof(struct aw2013_platform_data),
                GFP_KERNEL);
        if (!pdata) {
            dev_err(&led->client->dev,
                    "Failed to allocate memory\n");
            goto free_err;
        }
        pdata->led = led_array;
        led->pdata = pdata;

        rc = of_property_read_string(temp, "aw2013,name",
                &led->cdev.name);
        if (rc < 0) {
            dev_err(&led->client->dev,
                    "Failure reading led name, rc = %d\n", rc);
            goto free_pdata;
        }

        rc = of_property_read_u32(temp, "aw2013,id",
                &led->id);
        if (rc < 0) {
            dev_err(&led->client->dev,
                    "Failure reading id, rc = %d\n", rc);
            goto free_pdata;
        }

        rc = of_property_read_u32(temp, "aw2013,max-brightness",
                &led->cdev.max_brightness);
        if (rc < 0) {
            dev_err(&led->client->dev,
                    "Failure reading max-brightness, rc = %d\n",
                    rc);
            goto free_pdata;
        }

        rc = of_property_read_u32(temp, "aw2013,max-current",
                &led->pdata->max_current);
        if (rc < 0) {
            dev_err(&led->client->dev,
                    "Failure reading max-current, rc = %d\n", rc);
            goto free_pdata;
        }

        rc = of_property_read_u32(temp, "aw2013,rise-time-ms",
                &led->pdata->rise_time_ms);
        if (rc < 0) {
            dev_err(&led->client->dev,
                    "Failure reading rise-time-ms, rc = %d\n", rc);
            goto free_pdata;
        }

        rc = of_property_read_u32(temp, "aw2013,hold-time-ms",
                &led->pdata->hold_time_ms);
        if (rc < 0) {
            dev_err(&led->client->dev,
                    "Failure reading hold-time-ms, rc = %d\n", rc);
            goto free_pdata;
        }

        rc = of_property_read_u32(temp, "aw2013,fall-time-ms",
                &led->pdata->fall_time_ms);
        if (rc < 0) {
            dev_err(&led->client->dev,
                    "Failure reading fall-time-ms, rc = %d\n", rc);
            goto free_pdata;
        }

        rc = of_property_read_u32(temp, "aw2013,off-time-ms",
                &led->pdata->off_time_ms);
        if (rc < 0) {
            dev_err(&led->client->dev,
                    "Failure reading off-time-ms, rc = %d\n", rc);
            goto free_pdata;
        }

        INIT_WORK(&led->brightness_work, aw2013_brightness_work);

        led->cdev.brightness_set = aw2013_set_brightness;
        rc = led_classdev_register(&led->client->dev, &led->cdev);
        if (rc) {
            dev_err(&led->client->dev,
                    "unable to register led %d,rc=%d\n",
                    led->id, rc);
            goto free_pdata;
        }

        /*rc = sysfs_create_group(&led->cdev.dev->kobj,
          &aw2013_led_attr_group);
          if (rc) {
          dev_err(&led->client->dev, "led sysfs rc: %d\n", rc);
          goto free_class;
          }*/
        parsed_leds++;
    }

    return 0;

    /*free_class:
      aw2013_led_err_handle(led_array, parsed_leds);
      led_classdev_unregister(&led_array[parsed_leds].cdev);
      cancel_work_sync(&led_array[parsed_leds].brightness_work);
      devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
      led_array[parsed_leds].pdata = NULL;
      return rc;*/

free_pdata:
    aw2013_led_err_handle(led_array, parsed_leds);
    devm_kfree(&led->client->dev, led_array[parsed_leds].pdata);
    return rc;

free_err:
    aw2013_led_err_handle(led_array, parsed_leds);
    return rc;
}

static int aw2013_led_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{       
    struct aw2013_led *led_array;
    struct device_node *node;
    int ret, num_leds = 0;
    //dev_err(&client->dev, "aw2015 enter probe\n");

    node = client->dev.of_node;
    if (node == NULL){
        dev_err(&client->dev, "aw2015 node is null \n");
        return -EINVAL;
    }

    num_leds = of_get_child_count(node);

    if (!num_leds){
        dev_err(&client->dev, "aw2015 get_child_count err num_leds=%d\n",num_leds);
        return -EINVAL;

    }
    //dev_err(&client->dev, "aw2015 get_child_count num_leds=%d\n",num_leds);

    led_array = devm_kzalloc(&client->dev,
            (sizeof(struct aw2013_led) * num_leds), GFP_KERNEL);
    if (!led_array) {
        dev_err(&client->dev, "Unable to allocate memory\n");
        return -ENOMEM;
    }
    //dev_err(&client->dev, "aw2015 allocate memory\n");
    led_array->client = client;
    led_array->num_leds = num_leds;

    mutex_init(&led_array->lock);

    ret = aw_2013_check_chipid(led_array);
    ///===================


    if (ret) {
        dev_err(&client->dev, "Check chip id error\n");
        goto free_led_arry;
    }
    //dev_err(&client->dev, "aw2015 check_chipid\n");

    ret = aw2013_led_parse_child_node(led_array, node);
    if (ret) {
        dev_err(&client->dev, "parsed node error\n");
        goto free_led_arry;
    }
    //dev_err(&client->dev, "aw2015 parsed node\n");

    i2c_set_clientdata(client, led_array);

    //ret = aw2013_power_init(led_array, true);
    if (ret) {
        dev_err(&client->dev, "power init failed");
        goto fail_parsed_node;
    }
    //dev_err(&client->dev, "aw2015 power init \n");
    aw2015_init(led_array);
    dumpreg(led_array);

    return 0;

fail_parsed_node:
    aw2013_led_err_handle(led_array, num_leds);
free_led_arry:
    mutex_destroy(&led_array->lock);
    devm_kfree(&client->dev, led_array);
    led_array = NULL;
    return ret;
}

static int aw2013_led_remove(struct i2c_client *client)
{
    struct aw2013_led *led_array = i2c_get_clientdata(client);
    int i, parsed_leds = led_array->num_leds;

    for (i = 0; i < parsed_leds; i++) {
        //sysfs_remove_group(&led_array[i].cdev.dev->kobj,
        //	&aw2013_led_attr_group);
        led_classdev_unregister(&led_array[i].cdev);
        cancel_work_sync(&led_array[i].brightness_work);
        devm_kfree(&client->dev, led_array[i].pdata);
        led_array[i].pdata = NULL;
    }
    mutex_destroy(&led_array->lock);
    devm_kfree(&client->dev, led_array);
    led_array = NULL;
    return 0;
}

static const struct i2c_device_id aw2013_led_id[] = {
    {"aw2013_led", 0},
    {},
};

MODULE_DEVICE_TABLE(i2c, aw2013_led_id);

static struct of_device_id aw2013_match_table[] = {
    { .compatible = "awinic,aw2013",},
    { },
};

static struct i2c_driver aw2013_led_driver = {
    .probe = aw2013_led_probe,
    .remove = aw2013_led_remove,
    .driver = {
        .name = "aw2013_led",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(aw2013_match_table),
    },
    .id_table = aw2013_led_id,
};

static int __init aw2013_led_init(void)
{
    return i2c_add_driver(&aw2013_led_driver);
}
module_init(aw2013_led_init);

static void __exit aw2013_led_exit(void)
{
    i2c_del_driver(&aw2013_led_driver);
}
module_exit(aw2013_led_exit);

MODULE_DESCRIPTION("AWINIC aw2013 LED driver");
MODULE_LICENSE("GPL v2");
