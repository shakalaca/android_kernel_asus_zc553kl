/*
 * Gas_Gauge driver for CW2015/2013
 * Copyright (C) 2012, CellWise
 *
 * Authors: ChenGang <ben.chen@cellwise-semi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.And this driver depends on 
 * I2C and uses IIC bus for communication with the host.
 *
 */
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/power/cw2015_battery.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
//#include <mach/board.h>
#include <linux/wakelock.h>
#include <linux/switch.h>
#include <linux/rtc.h>
#if 0 //defined(CONFIG_FB)  //Other_platform modify 20160120 huangfusheng.wt update capacity when resume
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2017/03/20, add asus battery master function */
#define ASUS_BATTERY_MASTER_FUNCTION
//#define CAPACITY_DEBUG_SIMULATE
int capacity_limit_flag = false;
/* [PLATFORM]-Mod-END by pingao.yang */

#define REG_VERSION             	0x0
#define REG_VCELL               	0x2
#define REG_SOC                 	0x4
#define REG_RRT_ALERT           	0x6
#define REG_CONFIG              	0x8
#define REG_MODE                	0xA
#define REG_BATINFO             	0x10

#define MODE_SLEEP_MASK         	(0x3<<6)
#define MODE_SLEEP              	(0x3<<6)
#define MODE_NORMAL             	(0x0<<6)
#define MODE_QUICK_START        	(0x3<<4)
#define MODE_RESTART            	(0xf<<0)

#define CONFIG_UPDATE_FLG       	(0x1<<1)
#define ATHD                    	(0x0<<3)        //ATHD = 0%

//#define CW_I2C_SPEED            100000          // default i2c speed set 100khz
#define BATTERY_UP_MAX_CHANGE   	420             // the max time allow battery change quantity
#define BATTERY_DOWN_CHANGE   	60                // the max time allow battery change quantity
#define BATTERY_DOWN_MIN_CHANGE_RUN 	30          // the min time allow battery change quantity when run
#define BATTERY_DOWN_MIN_CHANGE_SLEEP 	1800      // the min time allow battery change quantity when run 30min
#define BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE 	1800

//#define NO_STANDARD_AC_BIG_CHARGE_MODE 1
// #define SYSTEM_SHUTDOWN_VOLTAGE  3400000     //set system shutdown voltage related in battery info.
//close this define by liyizeng 20160721
//#define BAT_LOW_INTERRUPT    	0 //gpio for low interrupt , not use now @hfs mask
#define INVALID_GPIO        	(-1)
#define GPIO_LOW             	0   
#define GPIO_HIGH            	1 
#define USB_CHARGER_MODE        	1      
#define AC_CHARGER_MODE         	2       
int cw_capacity;

struct cw_battery {
    struct i2c_client *client;
    struct workqueue_struct *battery_workqueue;
    struct delayed_work battery_delay_work;
    //struct delayed_work dc_wakeup_work;
    struct delayed_work bat_low_wakeup_work;
    const struct cw_bat_platform_data *plat_data;
#if 0 //defined(CONFIG_FB) //Other_platform modify 20160120 huangfusheng.wt update capacity when resume
    struct notifier_block fb_notif;
#endif
    struct power_supply rk_bat;
    struct power_supply	*batt_psy;
    //struct power_supply rk_ac;
    //struct power_supply rk_usb;

    long sleep_time_capacity_change;      // the sleep time from capacity change to present, it will set 0 when capacity change 
    long run_time_capacity_change;

    long sleep_time_charge_start;      // the sleep time from insert ac to present, it will set 0 when insert ac
    long run_time_charge_start;

    int charger_mode;
    int charger_init_mode;
    int capacity;
    int voltage;
    int status;
    int time_to_empty;
    int alt;

    int bat_change;
    struct regulator *vcc_i2c;
    struct pinctrl *ts_pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    struct pinctrl_state *pinctrl_state_suspend;
    struct pinctrl_state *pinctrl_state_release;
/*[PLATFORM]-Add-BEGIN by pingao.yang, 2017/03/20, add asus battery master function */
#if (defined ASUS_BATTERY_MASTER_FUNCTION)
	struct device *battery_master_root_dev;
	struct switch_dev sdev;
	struct delayed_work bat_master_workqueue;
	int user_limit_read;
	int user_capacity_limit;
	int last_user_capacity_limit;
	int capacity_limit;
	int last_capacity_limit;
	int last_capacity;
	int last_soc;
	int capacity_to_soc;
	int capacity_map_multiple;
	int capacity_map_divide;
	int battery_master_init;
	unsigned long last_soc_change_time;
	int battery_capacity;
	int debug_capacity;
	int debug_flag;
#endif
/* [PLATFORM]-Mod-END by pingao.yang */
};

//+project_modify 20151123 huangfusheng.wt add battery id for A9/11
static u8 config_info[SIZE_BATINFO] = {
#include "profile_WT702_88509_NT_DeSai.h"
};

static u8 config_info_desai[SIZE_BATINFO] = {
#include "profile_WT702_88509_NT_DeSai.h"
};

static u8 config_info_xinwangda[SIZE_BATINFO] = {
#include "profile_WT801_88509_NT_XinWangDa.h"
};

static u8 config_info_guanyu[SIZE_BATINFO] = {
#include "profile_WT902_88509_NT_GuangYu.h"
};

static u8 config_info_feimaotui[SIZE_BATINFO] = {
#include "profile_WT1001_88509_NT_Feimaotui.h"
};

static u8 config_info_cellwise[SIZE_BATINFO] = {
#include "profile_WT1101_K89200_NT_cellwise.h"
};

static u8 config_info_cellwise_K89218[SIZE_BATINFO] = {
#include "profile_WT1101_K89218_NT_cellwise.h"
};

//-project_modify 20151123 huangfusheng.wt add battery id for A9/11


/* write data to address */
static int cw_i2c_write(
        struct i2c_client *client,
        u8 addr,
        u8 *pdata,
        unsigned int datalen)
{
    int ret = 0;
    u8 tmp_buf[128];
    unsigned int bytelen = 0;

    if (datalen > 125)
    {
        pr_debug("%s too big datalen = %d!\n", __func__, datalen);
        return -1;
    }

    tmp_buf[0] = addr;
    bytelen++;

    if (datalen != 0 && pdata != NULL)
    {
        memcpy(&tmp_buf[bytelen], pdata, datalen);
        bytelen += datalen;
    }
    ret = i2c_master_send(client, tmp_buf, bytelen);
    return ret;
}

/* read data from addr */
static int cw_i2c_read(
        struct i2c_client *client,
        u8 addr,
        u8 *pdata,
        unsigned int datalen)
{
    int ret = 0;
    if (datalen > 126)
    {
        pr_debug("%s too big datalen = %d!\n", __func__, datalen);
        return -1;
    }

    /* set data address */
    ret = cw_i2c_write(client, addr, NULL, 0);
    if (ret < 0)
    {
        pr_debug("%s set data address fail!,ret is %d\n", __func__,ret);
        return ret;
    }
    /* read data */
    return i2c_master_recv(client, pdata, datalen);
}

static int cw_update_config_info(struct cw_battery *cw_bat)
{
    int ret;
    u8 reg_val;
    int i;
    u8 reset_val;

    dev_dbg(&cw_bat->client->dev, "func: %s-------\n", __func__);

    /* make sure no in sleep mode */     
    ret = cw_i2c_read(cw_bat->client, REG_MODE, &reg_val, 1);
    if (ret < 0)
        return ret;

    reset_val = reg_val;
    if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        dev_err(&cw_bat->client->dev, "Error, device in sleep mode, cannot update battery info\n");
        return -1;
    }

    /* update new battery info */     
    for (i = 0; i < SIZE_BATINFO; i++) {
        dev_dbg(&cw_bat->client->dev, "cw_bat->plat_data->cw_bat_config_info[%d] = 0x%x\n", i, \
                cw_bat->plat_data->cw_bat_config_info[i]);
        ret = cw_i2c_write(cw_bat->client, REG_BATINFO + i, &cw_bat->plat_data->cw_bat_config_info[i], 1);

        if (ret < 0) 
            return ret;
    }

    /* readback & check */           
    for (i = 0; i < SIZE_BATINFO; i++) {
        ret = cw_i2c_read(cw_bat->client, REG_BATINFO + i, &reg_val, 1);
        if (reg_val != cw_bat->plat_data->cw_bat_config_info[i])
            return -1;
    }

    /* set cw2015/cw2013 to use new battery info */
    ret = cw_i2c_read(cw_bat->client, REG_CONFIG, &reg_val, 1);
    if (ret < 0)
        return ret;

    reg_val |= CONFIG_UPDATE_FLG;   /* set UPDATE_FLAG */  
    reg_val &= 0x07;                /* clear ATHD */
    reg_val |= ATHD;                /* set ATHD */           
    ret = cw_i2c_write(cw_bat->client, REG_CONFIG, &reg_val, 1);
    if (ret < 0)
        return ret;

    /* check 2015/cw2013 for ATHD & update_flag */     
    ret = cw_i2c_read(cw_bat->client, REG_CONFIG, &reg_val, 1);
    if (ret < 0)
        return ret;

    if (!(reg_val & CONFIG_UPDATE_FLG)) {

        dev_dbg(&cw_bat->client->dev, "update flag for new battery info have not set..\n");
        reg_val = MODE_SLEEP;
        ret = cw_i2c_write(cw_bat->client, REG_MODE, &reg_val, 1);
        dev_dbg(&cw_bat->client->dev, "report battery capacity error");
        return -1;

    }

    if ((reg_val & 0xf8) != ATHD) {
        dev_dbg(&cw_bat->client->dev, "the new ATHD have not set..\n");
    }

    /* reset */
    reset_val &= ~(MODE_RESTART);
    reg_val = reset_val | MODE_RESTART;
    ret = cw_i2c_write(cw_bat->client, REG_MODE, &reg_val, 1);
    if (ret < 0)
        return ret;

    msleep(10);
    ret = cw_i2c_write(cw_bat->client, REG_MODE, &reset_val, 1);
    if (ret < 0)
        return ret;

    return 0;
}

static int cw_check_ic(struct cw_battery *cw_bat)
{
    int ret = 1 ;
    u8 reg_val = 0;

    ret = cw_i2c_read(cw_bat->client, REG_MODE/*REG_VERSION*/, &reg_val, 1); //platform modify 20151123 huangfusheng.wt must use REG_MODE 

    if(ret < 0) // i2c error
    {		
        if(ret == -107) //no ic exsist
        {
            return -107;
        }
        else
        {
            return ret;
        }
    }   
    if((reg_val == 0xC0 )||(reg_val == 0x00 ))
    {
        ret = 0;
    }

    return ret;	
}

static int cw_init(struct cw_battery *cw_bat)             
{
    int ret;
    int i;
    u8 reg_val = MODE_SLEEP;
#if 0
    ret = cw_i2c_read(cw_bat->client, REG_MODE, &reg_val, 1);
    if (ret < 0)
        return ret;
#endif
    if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        reg_val = MODE_NORMAL;
        ret = cw_i2c_write(cw_bat->client, REG_MODE, &reg_val, 1);
        if (ret < 0) 
            return ret;
    }

    ret = cw_i2c_read(cw_bat->client, REG_CONFIG, &reg_val, 1);
    if (ret < 0)
        return ret;

    if ((reg_val & 0xf8) != ATHD) {
        dev_dbg(&cw_bat->client->dev, "the new ATHD have not set\n");
        reg_val &= 0x07;    /* clear ATHD */
        reg_val |= ATHD;    /* set ATHD */
        ret = cw_i2c_write(cw_bat->client, REG_CONFIG, &reg_val, 1);
        if (ret < 0)
            return ret;
    }

    ret = cw_i2c_read(cw_bat->client, REG_CONFIG, &reg_val, 1);
    if (ret < 0) 
        return ret;

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
        dev_dbg(&cw_bat->client->dev, "update flag for new battery info have not set\n");
        ret = cw_update_config_info(cw_bat);
        if (ret < 0)
            return ret;
    } else {
        for(i = 0; i < SIZE_BATINFO; i++) { 
            ret = cw_i2c_read(cw_bat->client, (REG_BATINFO + i), &reg_val, 1);
            if (ret < 0)
                return ret;

            if (cw_bat->plat_data->cw_bat_config_info[i] != reg_val)
                break;
        }

        if (i != SIZE_BATINFO) {
            dev_dbg(&cw_bat->client->dev, "update flag for new battery info have not set\n"); 
            ret = cw_update_config_info(cw_bat);
            if (ret < 0)
                return ret;
        }
    }

    for (i = 0; i < 30; i++) {
        msleep(100);
        ret = cw_i2c_read(cw_bat->client, REG_SOC, &reg_val, 1);
        if (ret < 0)
            return ret;
        else if (reg_val <= 0x64) 
            break;

        if (i > 25)
            dev_err(&cw_bat->client->dev, "cw2015/cw2013 input unvalid power error\n");

    }
    if (i >=30){
        reg_val = MODE_SLEEP;
        ret = cw_i2c_write(cw_bat->client, REG_MODE, &reg_val, 1);
        dev_dbg(&cw_bat->client->dev, "report battery capacity error");
        return -1;
    } 
    return 0;
}

static void cw_update_time_member_charge_start(struct cw_battery *cw_bat)
{
    struct timespec ts;
    int new_run_time;
    int new_sleep_time;

    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;

    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;

    cw_bat->run_time_charge_start = new_run_time;
    cw_bat->sleep_time_charge_start = new_sleep_time; 
}

static void cw_update_time_member_capacity_change(struct cw_battery *cw_bat)          
{
    struct timespec ts;
    int new_run_time;
    int new_sleep_time;

    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;

    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;

    cw_bat->run_time_capacity_change = new_run_time;
    cw_bat->sleep_time_capacity_change = new_sleep_time; 
}

static int cw_quickstart(struct cw_battery *cw_bat)      
{
    int ret = 0;
    u8 reg_val = MODE_QUICK_START;

    ret = cw_i2c_write(cw_bat->client, REG_MODE, &reg_val, 1);     //(MODE_QUICK_START | MODE_NORMAL));  // 0x30
    if(ret < 0) {
        dev_err(&cw_bat->client->dev, "Error quick start1\n");
        return ret;
    }

    reg_val = MODE_NORMAL;
    ret = cw_i2c_write(cw_bat->client, REG_MODE, &reg_val, 1);
    if(ret < 0) {
        dev_err(&cw_bat->client->dev, "Error quick start2\n");
        return ret;
    }
    return 1;
}

static int cw_get_capacity(struct cw_battery *cw_bat)          
{
    //int cw_capacity;
    int ret;
    u8 reg_val[2];

    struct timespec ts;
    long new_run_time;
    long new_sleep_time;
    //long capacity_or_aconline_time;
    int allow_change;                 
    int allow_capacity;
    static int if_quickstart = 0;
    //static int jump_flag =0;
    //static int jump_flag2 =0; //Other_platform_modify 20151202 huangfusheng.wt from cw solve 100 full unplug jump issue
    static int reset_loop =0;
    int charge_time;
    u8 reset_val;



    // ret = cw_i2c_read(cw_bat->client, REG_SOC, &reg_val, 1);
    ret = cw_i2c_read(cw_bat->client, REG_SOC, reg_val, 2);      
    if (ret < 0)
        return ret;

    cw_capacity = reg_val[0];
    if ((cw_capacity < 0) || (cw_capacity > 100)) {              
        dev_err(&cw_bat->client->dev, "get cw_capacity error; cw_capacity = %d\n", cw_capacity);
        reset_loop++;

        if (reset_loop >5){ 

            reset_val = MODE_SLEEP;               
            ret = cw_i2c_write(cw_bat->client, REG_MODE, &reset_val, 1);
            if (ret < 0)
                return ret;
            reset_val = MODE_NORMAL;
            msleep(10);
            ret = cw_i2c_write(cw_bat->client, REG_MODE, &reset_val, 1);
            if (ret < 0)
                return ret;
            dev_dbg(&cw_bat->client->dev, "report battery capacity error");                              
            ret = cw_update_config_info(cw_bat);
            if (ret) 
                return ret;
            reset_loop =0;  

        }

        return cw_capacity;
    }else {
        reset_loop =0;
    }

    if (cw_capacity == 0) 
    {
        if(reg_val[1] > 70){
            cw_capacity = 1;
        } else {
            dev_dbg(&cw_bat->client->dev, "the cw201x capacity is 0 !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
        }
    }
    else 
        dev_dbg(&cw_bat->client->dev, "the cw201x capacity is %d, funciton: %s\n", cw_capacity, __func__);

    // ret = cw_i2c_read(cw_bat->client, REG_SOC + 1, &reg_val, 1);

    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;

    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;

    if (((cw_bat->charger_mode > 0) && (cw_capacity <= (cw_bat->capacity - 1)) &&
                (cw_capacity > (cw_bat->capacity - 3 /*9*/)) && cw_capacity <= 98) //@platform modify 20151123 huangfusheng.wt modify 9-->30
            || ((cw_bat->charger_mode == 0) && (cw_capacity == (cw_bat->capacity + 1)))) {             // modify battery level swing

        if (!(cw_capacity == 0 && cw_bat->capacity <= 2)) {			
            cw_capacity = cw_bat->capacity;
        }
    }        
    /*
       if ((cw_bat->charger_mode > 0) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity)) {     // avoid no charge full

       capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
       capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
       allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_UP_MAX_CHANGE;
       if (allow_change > 0) {
       allow_capacity = cw_bat->capacity + allow_change; 
       cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
       jump_flag =1;
       } else if (cw_capacity <= cw_bat->capacity) {
       cw_capacity = cw_bat->capacity; 
       }

       }
       else if ((cw_bat->charger_mode == 0) && cw_bat->capacity == 100 && cw_capacity < cw_bat->capacity && jump_flag2 == 0){
       cw_capacity = cw_bat->capacity;   //Other_platform_modify 20160309 huangfusheng.wt modify 100 unplug jump 99 issue
       jump_flag2 = 1;
       } //Other_platform_modify 20151202 huangfusheng.wt from cw solve 100 full unplug jump issue
       else if ((cw_bat->charger_mode == 0) && (cw_capacity <= cw_bat->capacity ) && (cw_capacity >= 90) && ((jump_flag == 1) || (jump_flag2 == 1))) {     // avoid battery level jump to CW_BAT
       capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
       capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
       allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_DOWN_CHANGE;
       if (allow_change > 0) {
       allow_capacity = cw_bat->capacity - allow_change; 
       if (cw_capacity >= allow_capacity){
       jump_flag =0;
       jump_flag2 = 0;
       }
       else{
       cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
       }
       } else if (cw_capacity <= cw_bat->capacity) {
       cw_capacity = cw_bat->capacity;
       }
       }
       */

    if ((cw_capacity == 0) && (cw_bat->capacity > 1)) {              // avoid battery level jump to 0% at a moment from more than 2%
        allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
        allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);

        allow_capacity = cw_bat->capacity - allow_change;
        cw_capacity = (allow_capacity >= cw_capacity) ? allow_capacity: cw_capacity;
        dev_dbg(&cw_bat->client->dev, "report GGIC POR happened");
        reg_val[0] = MODE_NORMAL;
        ret = cw_i2c_write(cw_bat->client, REG_MODE, reg_val, 1);
        if (ret < 0)
            return ret;   
        dev_dbg(&cw_bat->client->dev, "report battery capacity jump 0 ");                                                                    
    }

#if 1	
    if((cw_bat->charger_mode > 0) &&(cw_capacity == 0))
    {		  
        charge_time = new_sleep_time + new_run_time - cw_bat->sleep_time_charge_start - cw_bat->run_time_charge_start;
        if ((charge_time > BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE) && (if_quickstart == 0)) {
            cw_quickstart(cw_bat);		// if the cw_capacity = 0 the cw2015 will qstrt/
            reset_val = MODE_SLEEP;               
            ret = cw_i2c_write(cw_bat->client, REG_MODE, &reset_val, 1);
            if (ret < 0)
                return ret;
            reset_val = MODE_NORMAL;
            msleep(10);
            ret = cw_i2c_write(cw_bat->client, REG_MODE, &reset_val, 1);
            if (ret < 0)
                return ret;
            dev_dbg(&cw_bat->client->dev, "report battery capacity error");                              
            ret = cw_update_config_info(cw_bat);
            if (ret) 
                return ret;
            dev_dbg(&cw_bat->client->dev, "report battery capacity still 0 if in changing");
            if_quickstart = 1;
        }
    } else if ((if_quickstart == 1)&&(cw_bat->charger_mode == 0)) {
        if_quickstart = 0;
    }

#endif

#if 0
    if (cw_bat->plat_data->chg_ok_pin != NO_USE_GPIO) {
        if(gpio_get_value(cw_bat->plat_data->chg_ok_pin) != cw_bat->plat_data->chg_ok_level) {
            if (cw_capacity == 100) {
                cw_capacity = 99;
            }
        } else {
            if (cw_bat->charger_mode > 0) {
                cw_capacity = 100;
            }
        }
    }
#endif

#ifdef SYSTEM_SHUTDOWN_VOLTAGE
    if ((cw_bat->charger_mode == 0) && (cw_capacity <= 20) && (cw_bat->voltage <= SYSTEM_SHUTDOWN_VOLTAGE)){      	     
        if (if_quickstart == 10){  

            allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
            allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);

            allow_capacity = cw_bat->capacity - allow_change;
            cw_capacity = (allow_capacity >= 0) ? allow_capacity: 0;

            if (cw_capacity < 1){	     	      	
                cw_quickstart(cw_bat);
                if_quickstart = 12;
                cw_capacity = 0;
            }
        } else if (if_quickstart <= 10)
            if_quickstart =if_quickstart+2;
        dev_dbg(&cw_bat->client->dev, "the cw201x voltage is less than SYSTEM_SHUTDOWN_VOLTAGE !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
    } else if ((cw_bat->charger_mode > 0)&& (if_quickstart <= 12)) {
        if_quickstart = 0;
    }
#endif
    return cw_capacity;
}

static int cw_get_vol(struct cw_battery *cw_bat)
{
    int ret;
    u8 reg_val[2];
    u16 value16, value16_1, value16_2, value16_3;
    int voltage;

    ret = cw_i2c_read(cw_bat->client, REG_VCELL, reg_val, 2);   
    if (ret < 0)
        return ret;
    value16 = (reg_val[0] *256) + reg_val[1];

    ret = cw_i2c_read(cw_bat->client, REG_VCELL, reg_val, 2);
    if (ret < 0)
        return ret;
    value16_1 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_i2c_read(cw_bat->client, REG_VCELL, reg_val, 2);
    if (ret < 0)
        return ret;
    value16_2 = (reg_val[0] << 8) + reg_val[1];


    if(value16 > value16_1)
    {	 
        value16_3 = value16;
        value16 = value16_1;
        value16_1 = value16_3;
    }

    if(value16_1 > value16_2)
    {
        value16_3 =value16_1;
        value16_1 =value16_2;
        value16_2 =value16_3;
    }

    if(value16 >value16_1)
    {	 
        value16_3 =value16;
        value16 =value16_1;
        value16_1 =value16_3;
    }			

    voltage = value16_1 * 312 / 1024;
    voltage = voltage * 1000;    

    dev_dbg(&cw_bat->client->dev, "get cw_voltage : cw_voltage = %d\n", voltage);

    return voltage;
}

#ifdef BAT_LOW_INTERRUPT
static int cw_get_alt(struct cw_battery *cw_bat)
{
    int ret = 0;
    u8 reg_val;
    u8 value8 = 0;
    int alrt;

    ret = cw_i2c_read(cw_bat->client, REG_RRT_ALERT, &reg_val, 1);
    if (ret < 0)
        return ret;
    value8 = reg_val;
    alrt = value8 >>7;

    //dev_info(&cw_bat->client->dev, "read RRT %d%%. value16 0x%x\n", alrt, value16);
    value8 = value8&0x7f;
    reg_val = value8;
    ret = cw_i2c_write(cw_bat->client, REG_RRT_ALERT, &reg_val, 1);
    if(ret < 0) {
        dev_err(&cw_bat->client->dev, "Error clear ALRT\n");
        return ret;
    }
    return alrt;
}
#endif

static int cw_get_time_to_empty(struct cw_battery *cw_bat)
{
    int ret;
    u8 reg_val;
    u16 value16;

    ret = cw_i2c_read(cw_bat->client, REG_RRT_ALERT, &reg_val, 1);
    if (ret < 0)
        return ret;

    value16 = reg_val;

    ret = cw_i2c_read(cw_bat->client, REG_RRT_ALERT + 1, &reg_val, 1);
    if (ret < 0)
        return ret;

    value16 = ((value16 << 8) + reg_val) & 0x1fff;
    return value16;
}

static void rk_bat_update_capacity(struct cw_battery *cw_bat)
{
    //int cw_capacity;

    cw_capacity = cw_get_capacity(cw_bat);
    if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {
        cw_bat->capacity = cw_capacity;
        cw_bat->bat_change = 1;
        cw_update_time_member_capacity_change(cw_bat);

        if (cw_bat->capacity == 0)
            dev_dbg(&cw_bat->client->dev, "report battery capacity 0 and will shutdown if no changing");

    }
}


static void rk_bat_update_vol(struct cw_battery *cw_bat)
{
    int ret;
    ret = cw_get_vol(cw_bat);
    if ((ret >= 0) && (cw_bat->voltage != ret)) {
        cw_bat->voltage = ret;
        cw_bat->bat_change = 1;
    }
}

//extern int get_msm_otg_chg_type(void);
//extern int get_charger_state(void);
extern int power_supply_get_battery_charge_state(struct power_supply *psy);
static struct power_supply *charge_psy = NULL;
static u8 is_charger_plug = 0;
//static u8 pre_charger_status = 0;

static void rk_bat_update_status(struct cw_battery *cw_bat)
{
    int status;
    union power_supply_propval ret = {0,};
    //int mode = get_msm_otg_chg_type();
    //int mode = get_charger_state();
    //dev_dbg(&cw_bat->client->dev,"%s:mode = %d\n",__func__,mode);

    if(!charge_psy){
        charge_psy = power_supply_get_by_name("usb");
    }else{
        is_charger_plug = (u8)power_supply_get_battery_charge_state(charge_psy);
    }

    pr_debug("Chaman for test is_charger_plug %d\n", is_charger_plug);
    /*
       if ((POWER_SUPPLY_STATUS_UNKNOWN == mode)||(POWER_SUPPLY_STATUS_DISCHARGING == mode))
       cw_bat->charger_mode =  POWER_SUPPLY_TYPE_UNKNOWN;
       else if(POWER_SUPPLY_STATUS_CHARGING== mode)
       cw_bat->charger_mode = USB_CHARGER_MODE; 
       else
       */
    if(is_charger_plug == 0)
        cw_bat->charger_mode =  POWER_SUPPLY_TYPE_UNKNOWN;
    else
        cw_bat->charger_mode = USB_CHARGER_MODE; 

    if (cw_bat->batt_psy == NULL)
        cw_bat->batt_psy = power_supply_get_by_name("battery");
    if (cw_bat->batt_psy) {
        if (is_charger_plug == 0) {
            ret.intval = POWER_SUPPLY_STATUS_UNKNOWN;
            cw_bat->batt_psy->set_property(cw_bat->batt_psy, POWER_SUPPLY_PROP_STATUS, &ret);
        }
        /* if battery has been registered, use the status property */
        cw_bat->batt_psy->get_property(cw_bat->batt_psy,
                POWER_SUPPLY_PROP_STATUS, &ret);
        status = ret.intval;
    } else {
        /* Default to false if the battery power supply is not registered. */
        pr_debug("battery power supply is not registered\n");
        status = POWER_SUPPLY_STATUS_UNKNOWN;
    }

    if (cw_bat->status != status) {
        cw_bat->status = status;
        cw_bat->bat_change = 1;
    } 
}

static void rk_bat_update_time_to_empty(struct cw_battery *cw_bat)
{
    int ret;
    ret = cw_get_time_to_empty(cw_bat);
    if ((ret >= 0) && (cw_bat->time_to_empty != ret)) {
        cw_bat->time_to_empty = ret;
        cw_bat->bat_change = 1;
    }

}

static void cw_bat_work(struct work_struct *work)
{
    struct delayed_work *delay_work;
    struct cw_battery *cw_bat;
    //int ret;

    delay_work = container_of(work, struct delayed_work, work);
    cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

    rk_bat_update_status(cw_bat);
    rk_bat_update_capacity(cw_bat);
    rk_bat_update_vol(cw_bat);
    rk_bat_update_time_to_empty(cw_bat);

    if (cw_bat->bat_change) {
        power_supply_changed(&cw_bat->rk_bat);
        cw_bat->bat_change = 0;
    }

    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(500));//2s 20160630//Other_platform_modify 20151126 huangfusheng.wt modify 10s rework

    dev_dbg(&cw_bat->client->dev, "cw_bat->bat_change = %d, cw_bat->time_to_empty = %d, cw_bat->capacity = %d, cw_bat->voltage = %d\n",\
            cw_bat->bat_change, cw_bat->time_to_empty, cw_bat->capacity, cw_bat->voltage);
}

static int rk_battery_get_property(struct power_supply *psy,
        enum power_supply_property psp,
        union power_supply_propval *val)
{
    int ret = 0;
    struct cw_battery *cw_bat;

    cw_bat = container_of(psy, struct cw_battery, rk_bat); 
    switch (psp) {
        case POWER_SUPPLY_PROP_CAPACITY:
	/*[PLATFORM]-Add-BEGIN by pingao.yang, 2017/03/20, add asus battery master function */
	#if (defined ASUS_BATTERY_MASTER_FUNCTION)
	     val->intval = cw_bat->last_soc;
       #else
	     val->intval = cw_bat->capacity;
	#endif
	/* [PLATFORM]-Mod-END by pingao.yang */
            break;

        case POWER_SUPPLY_PROP_STATUS:
            val->intval = cw_bat->status;
            break;
#if 0        
        case POWER_SUPPLY_PROP_HEALTH:
            val->intval= POWER_SUPPLY_HEALTH_GOOD;
            break;
#endif
        case POWER_SUPPLY_PROP_PRESENT:
            val->intval = cw_bat->voltage <= 0 ? 0 : 1;
            break;

        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            val->intval = cw_bat->voltage;
            break;

        case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
            val->intval = cw_bat->time_to_empty;			
            break;

        case POWER_SUPPLY_PROP_TECHNOLOGY:
            val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
            break;

        default:
            break;
    }
    return ret;
}

static enum power_supply_property rk_battery_properties[] = {
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_STATUS,
    //POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
    POWER_SUPPLY_PROP_TECHNOLOGY,
};

#ifdef BAT_LOW_INTERRUPT

#define WAKE_LOCK_TIMEOUT       (10 * HZ)
static struct wake_lock bat_low_wakelock;

static void bat_low_detect_do_wakeup(struct work_struct *work)
{
    struct delayed_work *delay_work;
    struct cw_battery *cw_bat;

    delay_work = container_of(work, struct delayed_work, work);
    cw_bat = container_of(delay_work, struct cw_battery, bat_low_wakeup_work);
    dev_dbg(&cw_bat->client->dev, "func: %s-------\n", __func__);
    cw_get_alt(cw_bat);
    //enable_irq(irq);
}

static irqreturn_t bat_low_detect_irq_handler(int irq, void *dev_id)
{
    struct cw_battery *cw_bat = dev_id;
    // disable_irq_nosync(irq); // for irq debounce
    wake_lock_timeout(&bat_low_wakelock, WAKE_LOCK_TIMEOUT);
    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->bat_low_wakeup_work, msecs_to_jiffies(20));
    return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cw_battery *cw_bat = i2c_get_clientdata(client);
    dev_dbg(&cw_bat->client->dev, "%s\n", __func__);
    cancel_delayed_work(&cw_bat->battery_delay_work); //Other_plarform_modify 20151126 huangfusheng.wt modify capacity not update issue
    return 0;
}

static int cw_bat_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cw_battery *cw_bat = i2c_get_clientdata(client);
    dev_dbg(&cw_bat->client->dev, "%s\n", __func__);
    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(10)); //Other_plarform_modify 20151126 huangfusheng.wt modify capacity not update issue
    return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
    .suspend = cw_bat_suspend,
    .resume = cw_bat_resume,

};
#endif
#if 0 //defined(CONFIG_FB) //Other_platform modify 20160120 huangfusheng.wt update capacity when resume

static int fb_notifier_callback(struct notifier_block *self,
        unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct cw_battery *cw_battery =
        container_of(self, struct cw_battery, fb_notif);

    if (evdata && evdata->data && event == FB_EVENT_BLANK &&
            cw_battery && cw_battery->client) {
        blank = evdata->data;
        if (*blank == FB_BLANK_UNBLANK)
            cw_bat_resume(&cw_battery->client->dev);
        else if (*blank == FB_BLANK_POWERDOWN)
            cw_bat_suspend(&cw_battery->client->dev);
    }


    return 0;
}
#endif

#ifdef CONFIG_OF
static int cw_bat_parse_dt(struct device *dev, struct cw_bat_platform_data *pdata)
{
    struct device_node *np = dev->of_node;
    pdata->bat_low_pin = of_get_named_gpio_flags(np,
            "cw2015,irq-gpio", 0, &pdata->irq_flags);

    return 0;
}
#else
static int cw_bat_parse_dt(struct device *dev, struct cw_bat_platform_data *pdata)
{
    return 0;
}
#endif
#ifdef BAT_LOW_INTERRUPT
#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"
#define CW_I2C_VTG_MIN_UV	1800000
#define CW_I2C_VTG_MAX_UV	1800000
#define CW_VIO_LOAD_MAX_UA	10000
static int cw_bat_regulator_configure(struct cw_battery *cw_bat, bool on)
{
    int retval;

    if (on == false)
        goto hw_shutdown;

    cw_bat->vcc_i2c = regulator_get(&cw_bat->client->dev,
            "vcc_i2c");
    if (IS_ERR(cw_bat->vcc_i2c)) {
        dev_err(&cw_bat->client->dev,
                "%s: Failed to get i2c regulator\n",
                __func__);
        retval = PTR_ERR(cw_bat->vcc_i2c);
        goto hw_shutdown;
    }

    if (regulator_count_voltages(cw_bat->vcc_i2c) > 0) {
        retval = regulator_set_voltage(cw_bat->vcc_i2c,
                CW_I2C_VTG_MIN_UV, CW_I2C_VTG_MAX_UV);
        if (retval) {
            dev_err(&cw_bat->client->dev,
                    "%s reg set i2c vtg failed retval =%d\n",__func__,
                    retval);
            goto err_set_vtg_i2c;
        }
    }
    return 0;

err_set_vtg_i2c:
    regulator_put(cw_bat->vcc_i2c);

hw_shutdown:    
    if (regulator_count_voltages(cw_bat->vcc_i2c) > 0)
        regulator_set_voltage(cw_bat->vcc_i2c, 0,
                CW_I2C_VTG_MAX_UV);
    regulator_put(cw_bat->vcc_i2c);

    return 0;
};

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
    return (regulator_count_voltages(reg) > 0) ?
        regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int cw_bat_power_on(struct cw_battery *cw_bat,
        bool on) {
    int retval;

    if (on == false)
        goto power_off;

    retval = reg_set_optimum_mode_check(cw_bat->vcc_i2c,CW_VIO_LOAD_MAX_UA);
    if (retval < 0) {
        dev_err(&cw_bat->client->dev,
                "%s Regulator vcc_i2c set_opt failed rc=%d\n",__func__,
                retval);
        goto power_off;
    }

    retval = regulator_enable(cw_bat->vcc_i2c);
    if (retval) {
        dev_err(&cw_bat->client->dev,
                "%s Regulator vcc_i2c enable failed rc=%d\n",__func__,
                retval);
        goto error_reg_en_vcc_i2c;
    }

    msleep(200);
    return 0;

error_reg_en_vcc_i2c:
    reg_set_optimum_mode_check(cw_bat->vcc_i2c, 0);
    return retval;

power_off:
    reg_set_optimum_mode_check(cw_bat->vcc_i2c, 0);
    regulator_disable(cw_bat->vcc_i2c);

    msleep(100);
    return 0;
}

static int cw_bat_pinctrl_init(struct cw_battery *cw_bat)
{
    int retval;

    /* Get pinctrl if target uses pinctrl */
    cw_bat->ts_pinctrl = devm_pinctrl_get(&(cw_bat->client->dev));
    if (IS_ERR_OR_NULL(cw_bat->ts_pinctrl)) {
        retval = PTR_ERR(cw_bat->ts_pinctrl);
        dev_dbg(&cw_bat->client->dev,
                "%s Target does not use pinctrl  %d\n",__func__, retval);
        goto err_pinctrl_get;
    }

    cw_bat->pinctrl_state_active
        = pinctrl_lookup_state(cw_bat->ts_pinctrl,
                PINCTRL_STATE_ACTIVE);
    if (IS_ERR_OR_NULL(cw_bat->pinctrl_state_active)) {
        retval = PTR_ERR(cw_bat->pinctrl_state_active);
        dev_err(&cw_bat->client->dev,
                "%s Can not lookup %s pinstate %d\n",
                __func__, PINCTRL_STATE_ACTIVE, retval);
        goto err_pinctrl_lookup;
    }

    cw_bat->pinctrl_state_suspend
        = pinctrl_lookup_state(cw_bat->ts_pinctrl,
                PINCTRL_STATE_SUSPEND);
    if (IS_ERR_OR_NULL(cw_bat->pinctrl_state_suspend)) {
        retval = PTR_ERR(cw_bat->pinctrl_state_suspend);
        dev_err(&cw_bat->client->dev,
                "%s Can not lookup %s pinstate %d\n",
                __func__, PINCTRL_STATE_SUSPEND, retval);
        goto err_pinctrl_lookup;
    }

    return 0;

err_pinctrl_lookup:
    devm_pinctrl_put(cw_bat->ts_pinctrl);
err_pinctrl_get:
    cw_bat->ts_pinctrl = NULL;
    return retval;
}


static int cw_bat_pinctrl_select(struct cw_battery *cw_bat, bool on)
{
    struct pinctrl_state *pins_state;
    int ret;

    pins_state = on ? cw_bat->pinctrl_state_active
        : cw_bat->pinctrl_state_suspend;
    if (!IS_ERR_OR_NULL(pins_state)) {
        ret = pinctrl_select_state(cw_bat->ts_pinctrl, pins_state);
        if (ret) {
            dev_err(&cw_bat->client->dev,
                    "%s can not set %s pins\n",__func__,
                    on ? "pmx_ts_active" : "pmx_ts_suspend");
            return ret;
        }
    } else
        dev_err(&cw_bat->client->dev,
                "%s not a valid '%s' pinstate\n",__func__,
                on ? "pmx_ts_active" : "pmx_ts_suspend");

    return 0;
}


static int cw_bat_gpio_configure(struct cw_battery *cw_bat, bool on)
{
    int retval = 0;

    if (on) {
        if (gpio_is_valid(cw_bat->plat_data->bat_low_pin)) {
            /* configure  irq gpio */
            retval = gpio_request(cw_bat->plat_data->bat_low_pin,
                    "rmi4_irq_gpio");
            if (retval) {
                dev_err(&cw_bat->client->dev,
                        "%s unable to request gpio [%d]\n",__func__,
                        cw_bat->plat_data->bat_low_pin);
                goto err_irq_gpio_req;
            }
            retval = gpio_direction_input(cw_bat->plat_data->bat_low_pin);
            if (retval) {
                dev_err(&cw_bat->client->dev,
                        "%s unable to set direction for gpio " \
                        "[%d]\n",__func__, cw_bat->plat_data->bat_low_pin);
                goto err_irq_gpio_dir;
            }
        } else {
            dev_err(&cw_bat->client->dev,
                    "%s irq gpio not provided\n",__func__);
            goto err_irq_gpio_req;
        }

        return 0;
    } else {
        return 0;
    }

err_irq_gpio_dir:
    if (gpio_is_valid(cw_bat->plat_data->bat_low_pin))
        gpio_free(cw_bat->plat_data->bat_low_pin);
err_irq_gpio_req:
    return retval;
}
#endif

#ifdef CONFIG_K89200_FEATURES
static int battery_type_id = 4; //project_modify 20151123 huangfusheng.wt add battery id 
#else
static int battery_type_id = 5; //project_modify 20160629 dingyuchen add battery id
#endif

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2017/03/20, add asus battery master function */
#if (defined ASUS_BATTERY_MASTER_FUNCTION)
void  battery_master_set_capacity_limit_flag(struct  cw_battery *cw_bat)
{
	if (cw_bat->capacity_limit == true) {
		capacity_limit_flag = true;
	} else {
		capacity_limit_flag = false;
	}
}

int battery_master_get_capacity_limit_flag(void)
{
	return capacity_limit_flag;
}
#else
int battery_master_get_capacity_limit_flag(void)
{
	return capacity_limit_flag;
}
#endif

#if (defined ASUS_BATTERY_MASTER_FUNCTION)
#define BATTERY_MASTER_INIT_TIMES		5
#define BATTERY_MASTER_MONITOR_MES_INTERVAL	5*HZ
#define CAPACITY_LIMIT_ENABLE	80
#define CAPACITY_LIMIT_DISABLE	100
#define CAPACITY_MAPPING_THRESHOLD 40
#define CAPACITY_MAPPING_DIFF 11
#define CAPACITY_MAPPING_MULTIPLE  60
#define CAPACITY_MAPPING_DIVIDE  47
#define CAPACITY_CHARGE_FULL	CAPACITY_MAPPING_DIVIDE + 43
#define CAPACITY_DECLINE_TIME	80
#define CAPACITY_RISE_TIME	45
#define BATTERY_PROFILE_MAP1	"map1"
#define BATTERY_PROFILE_MAP2	"map2"

extern int battery_master_limit_info_write(int limit, int map);
extern int battery_master_limit_info_read(int *limit, int *map);

static int get_system_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int ret;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	ret = rtc_read_time(rtc, &tm);
	if (ret) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, ret);
		goto close_time;
	}

	ret = rtc_valid_tm(&tm);
	if (ret) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, ret);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return ret;
}


static int battery_master_recovery_capacity_limit(struct  cw_battery *cw_bat)
{
	int ret = 0;
	int save_map = 0;
	int save_limit = 0;
	static int read_count = 0;

	if (cw_bat->user_limit_read == true) {
		ret = battery_master_limit_info_read(&save_limit, &save_map);
		dev_err(&cw_bat->client->dev,
				"[allan 2]:save_limit = %d, save_map = %d\n", save_limit, save_map);
		if (save_limit == CAPACITY_LIMIT_ENABLE) {
			cw_bat->user_capacity_limit = 1;
			cw_bat->user_limit_read = false;
		} else if (save_limit == CAPACITY_LIMIT_DISABLE) {
			cw_bat->user_capacity_limit = 0;
			cw_bat->user_limit_read = false;
		} else {
			cw_bat->user_capacity_limit = 0;
			cw_bat->user_limit_read = true;
		}

		if (save_map == 1) {
			cw_bat->capacity_limit = true;
		} else {
			cw_bat->capacity_limit = false;
		}

		read_count++;
		if (read_count >= 5) {
			cw_bat->user_limit_read = false;
		}

       }

	return ret;
}

static int battery_master_store_capacity_limit(struct  cw_battery *cw_bat)
{
	int ret = 0;

	if (cw_bat->capacity_limit != cw_bat->last_capacity_limit) {
		ret = battery_master_limit_info_write(cw_bat->user_capacity_limit,
										cw_bat->capacity_limit);
		if (ret <= 0) {
			dev_err(&cw_bat->client->dev,
				"Couldn't store user limit, ret = %d\n", ret);
		}

		cw_bat->last_capacity_limit = cw_bat->capacity_limit;
	}

	if (cw_bat->user_capacity_limit != cw_bat->last_user_capacity_limit) {
		ret = battery_master_limit_info_write(cw_bat->user_capacity_limit,
										cw_bat->capacity_limit);
		if (ret <= 0) {
			dev_err(&cw_bat->client->dev,
				"Couldn't store user limit, ret = %d\n", ret);
		}

		cw_bat->last_user_capacity_limit = cw_bat->user_capacity_limit;
	}

	return ret;
}

static int battery_master_change_capacity_limit(struct  cw_battery *cw_bat, int capacity)
{
	int ret = 0;

	if (cw_bat->user_capacity_limit == 1) {
		if (capacity < CAPACITY_MAPPING_THRESHOLD) {
			cw_bat->capacity_limit = false;
		} else if (capacity > CAPACITY_MAPPING_THRESHOLD
				&& cw_bat->last_capacity <= CAPACITY_MAPPING_THRESHOLD) {
			cw_bat->capacity_limit = true;
		}
	} else if (cw_bat->user_capacity_limit == 0) {
		if (capacity < CAPACITY_MAPPING_THRESHOLD) {
			cw_bat->capacity_limit = false;
		} else if (capacity > CAPACITY_MAPPING_THRESHOLD
				&& cw_bat->last_capacity <= CAPACITY_MAPPING_THRESHOLD) {
			cw_bat->capacity_limit = false;
		}
	}

	if (cw_bat->capacity_limit != cw_bat->last_capacity_limit) {
		switch_set_state(&cw_bat->sdev, cw_bat->capacity_limit);
	}

	return ret;
}

static int battery_master_calibration_batt_capacity(struct  cw_battery *cw_bat, int capacity)
{
	int soc, soc_integer, soc_remainder;
	int soc_change, soc_change_time;
	unsigned long now_tm_sec = 0;

	cw_bat->last_capacity = capacity;
	if (cw_bat->capacity_limit != true) {
		cw_bat->last_soc = capacity;
		cw_bat->last_soc_change_time = now_tm_sec;
		return cw_bat->last_soc;
	}

	if (get_system_current_time(&now_tm_sec)) {
		pr_err("RTC read failed\n");
		return 0;
	}

	soc_change_time = now_tm_sec - cw_bat->last_soc_change_time;
	soc_integer = (capacity * cw_bat->capacity_map_multiple)
					/cw_bat->capacity_map_divide;
	soc_remainder =  (capacity * cw_bat->capacity_map_multiple)
					% cw_bat->capacity_map_divide;
	if (soc_remainder >= (cw_bat->capacity_map_divide /2)) {
		soc_remainder = 1;
	} else {
		soc_remainder = 0;
	}
	soc = soc_integer + soc_remainder - CAPACITY_MAPPING_DIFF;
	soc = max(0, soc);
	soc = min(100, soc);
	soc_change = min((int)abs(cw_bat->last_soc - soc), 1);

	if (cw_bat->capacity_to_soc == false) {
		cw_bat->last_soc = soc;
		cw_bat->capacity_to_soc = true;
	}

	dev_dbg(&cw_bat->client->dev,
				"[allan 1]:multiple = %d, divide = %d, soc_integer = %d, soc_remainder = %d\n",
				cw_bat->capacity_map_multiple, cw_bat->capacity_map_divide,
				soc_integer, soc_remainder);

	dev_dbg(&cw_bat->client->dev,
				"[allan 1]:soc = %d, last_soc = %d, soc_change = %d, soc_change_time = %d\n",
				soc, cw_bat->last_soc, soc_change, soc_change_time);

	if (soc < cw_bat->last_soc && soc >= 0
		&& soc_change_time >= CAPACITY_DECLINE_TIME) {
		cw_bat->last_soc = cw_bat->last_soc - soc_change;
		cw_bat->last_soc_change_time = now_tm_sec;
	}
	if (soc > cw_bat->last_soc && soc <= 100
		&& soc_change_time >= CAPACITY_RISE_TIME) {
		if (soc == 100 && capacity < CAPACITY_CHARGE_FULL) {
			return cw_bat->last_soc;
		}

		cw_bat->last_soc = cw_bat->last_soc + soc_change;
		cw_bat->last_soc_change_time = now_tm_sec;
	}

	cw_bat->last_soc = max(0, cw_bat->last_soc);
	cw_bat->last_soc = min(100, cw_bat->last_soc);
	dev_dbg(&cw_bat->client->dev,
				"[allan 2]:soc = %d, last_soc = %d, soc_change = %d, soc_change_time = %d\n",
				soc, cw_bat->last_soc, soc_change, soc_change_time);

	return cw_bat->last_soc;
}

static void battery_master_monitor_work_callback(struct work_struct *work)
{
	static int battery_master_init_count = 0;
	int battery_master_interval = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct cw_battery *cw_bat = container_of(dwork,
					struct cw_battery, bat_master_workqueue);

	battery_master_recovery_capacity_limit(cw_bat);
	dev_dbg(&cw_bat->client->dev,
			"[allan 1]:original_capacity = %d, last_capacity = %d, soc = %d, user_limit = %d, capacity_limit = %d\n",
			cw_bat->debug_flag ? cw_bat->debug_capacity : cw_bat->capacity,
			cw_bat->last_capacity, cw_bat->last_soc,
			cw_bat->user_capacity_limit, cw_bat->capacity_limit);

	//battery_master_recovery_capacity_limit(cw_bat);
#if (defined CAPACITY_DEBUG_SIMULATE)
	cw_bat->debug_flag = true;
	battery_master_change_capacity_limit(cw_bat, cw_bat->debug_capacity);
	battery_master_calibration_batt_capacity(cw_bat, cw_bat->debug_capacity);
#else
	cw_bat->debug_flag = false;
	battery_master_change_capacity_limit(cw_bat, cw_bat->capacity);
	battery_master_calibration_batt_capacity(cw_bat, cw_bat->capacity);
#endif
	battery_master_store_capacity_limit(cw_bat);
	battery_master_set_capacity_limit_flag(cw_bat);

	dev_err(&cw_bat->client->dev,
				"[allan 2]:original_capacity = %d, last_capacity = %d, soc = %d, user_limit = %d, capacity_limit = %d\n",
				cw_bat->debug_flag ? cw_bat->debug_capacity : cw_bat->capacity,
				cw_bat->last_capacity, cw_bat->last_soc,
				cw_bat->user_capacity_limit, cw_bat->capacity_limit);

	if (cw_bat->battery_master_init == false) {
		battery_master_init_count++;
		if (battery_master_init_count >= BATTERY_MASTER_INIT_TIMES) {
			cw_bat->battery_master_init = true;
		}
		battery_master_interval = 1*HZ;
	} else {
		battery_master_interval = BATTERY_MASTER_MONITOR_MES_INTERVAL;
	}

	schedule_delayed_work(&cw_bat->bat_master_workqueue,
							battery_master_interval);
}

static ssize_t battery_master_chg_limit_store(struct device * dev,
									struct device_attribute *attr, const char * buf, size_t count)
{
	struct cw_battery *cw_bat = dev_get_drvdata(dev);
	int limit = 0;

	printk("%s: entry\n", __func__);

	if (!strncmp(buf, "limit", 5)) {
		limit =  simple_strtol(&buf[5], NULL, 10);
	}

	if (limit == CAPACITY_LIMIT_ENABLE) {
		cw_bat->user_capacity_limit = 1;
	} else if (limit == CAPACITY_LIMIT_DISABLE) {
		cw_bat->user_capacity_limit = 0;
	}

	return count;
}

static ssize_t battery_master_chg_limit_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	struct cw_battery *cw_bat = dev_get_drvdata(dev);

	printk("%s: entry\n", __func__);
	return sprintf(buf, "%s\n", cw_bat->user_capacity_limit ? "limit80": "limit100");
}

static ssize_t battery_master_bat_map_store(struct device * dev,
									struct device_attribute *attr, const char * buf, size_t count)
{
	printk("%s: entry, do nothing !\n", __func__);

	return count;
}

static ssize_t battery_master_bat_map_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	struct cw_battery *cw_bat = dev_get_drvdata(dev);

	printk("%s: entry\n", __func__);
	return sprintf(buf, "%s\n", cw_bat->capacity_limit ? "Tab2": "Tab1");
}

static ssize_t battery_master_debug_capacity_store(struct device * dev,
									struct device_attribute *attr, const char * buf, size_t count)
{
	struct cw_battery *cw_bat = dev_get_drvdata(dev);

	printk("%s: entry\n", __func__);

	if (!strncmp(buf, "capacity", 8)) {
		cw_bat->debug_capacity =  simple_strtol(&buf[8], NULL, 10);
	} else {
		printk("use: echo [capacityxxx] > /sys/devices/battery_master/debug_capacity\n");
	}

	return count;
}

static ssize_t battery_master_debug_capacity_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	struct cw_battery *cw_bat = dev_get_drvdata(dev);

	printk("%s: entry\n", __func__);
	return sprintf(buf, "%d\n", cw_bat->debug_capacity);
}

static ssize_t battery_master_real_soc_store(struct device * dev,
									struct device_attribute *attr, const char * buf, size_t count)
{
	printk("%s: entry, do nothing !\n", __func__);

	return count;
}

static ssize_t battery_master_real_soc_show(struct device *dev,
									struct device_attribute *attr, char *buf)
{
	struct cw_battery *cw_bat = dev_get_drvdata(dev);
	int real_soc;

	printk("%s: entry\n", __func__);

	if (cw_bat->capacity_limit == true) {
		real_soc =((cw_bat->last_soc + CAPACITY_MAPPING_DIFF)
				* cw_bat->capacity_map_divide)
				/cw_bat->capacity_map_multiple;
		if (real_soc >= CAPACITY_LIMIT_ENABLE) {
			real_soc = CAPACITY_LIMIT_ENABLE;
		}
	} else {
		real_soc = cw_bat->capacity;
	}

	return sprintf(buf, "%d\n", real_soc);
}

static DEVICE_ATTR(chg_limit, S_IWUSR|S_IRUSR|S_IROTH|S_IRGRP,
						battery_master_chg_limit_show, battery_master_chg_limit_store);

static DEVICE_ATTR(bat_map, S_IWUSR|S_IRUSR|S_IROTH|S_IRGRP,
						battery_master_bat_map_show, battery_master_bat_map_store);

static DEVICE_ATTR(debug_capacity, S_IWUSR|S_IRUSR|S_IROTH|S_IRGRP,
						battery_master_debug_capacity_show, battery_master_debug_capacity_store);

static DEVICE_ATTR(real_soc, S_IWUSR|S_IRUSR|S_IROTH|S_IRGRP,
						battery_master_real_soc_show, battery_master_real_soc_store);
#endif
/* [PLATFORM]-Mod-END by pingao.yang */

static int cw_bat_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct cw_bat_platform_data *pdata = client->dev.platform_data;
    struct cw_battery *cw_bat;
    int ret;
    int loop = 0;

    printk( "\ncw2015/cw2013 driver v1.2 probe start,battery_type_id is %d\n",battery_type_id);

    cw_bat = kzalloc(sizeof(struct cw_battery), GFP_KERNEL);
    if (!cw_bat) {
        dev_err(&cw_bat->client->dev, "fail to allocate memory\n");
        return -ENOMEM;
    }
    if (client->dev.of_node) {
        pdata = devm_kzalloc(&client->dev,
                sizeof(struct cw_bat_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&client->dev,
                    "GTP Failed to allocate memory for pdata\n");
            return -ENOMEM;
        }

        ret = cw_bat_parse_dt(&client->dev, pdata);
        if (ret)
            return ret;


    } else {
        pdata = client->dev.platform_data;
    }

    if (!pdata) {
        dev_err(&client->dev, "Invalid pdata\n");
        return -EINVAL;
    }

    //+project_modify 20151123 huangfusheng.wt add battery id 
    if(battery_type_id==0)
    {
        pdata->cw_bat_config_info  = config_info_desai ;
    }
    else if (battery_type_id==1)
    {
        pdata->cw_bat_config_info  = config_info_feimaotui ;

    }
    else if (battery_type_id==2)
    {
        pdata->cw_bat_config_info  = config_info_guanyu ;

    }
    else if (battery_type_id==3)
    {
        pdata->cw_bat_config_info  = config_info_xinwangda;		

    }
    else if (battery_type_id==4)
    {
        pdata->cw_bat_config_info  = config_info_cellwise;		

    }
    else if (battery_type_id==5)
    {
        pdata->cw_bat_config_info  = config_info_cellwise_K89218;		

    }
    else
    {
        pdata->cw_bat_config_info  = config_info;	

    }

    //-project_modify 20151123 huangfusheng.wt add battery id 


    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C not supported\n");
        return -ENODEV;
    }

    cw_bat->client = client;
    i2c_set_clientdata(client, cw_bat);	

    cw_bat->plat_data = pdata;
    ret = cw_check_ic(cw_bat);

    while ((loop++ < 5) && (ret != 0)) {  
        pr_debug(" check ret is %d,loop is %d \n" ,ret,loop);
        ret = cw_check_ic(cw_bat);
    }


    if (ret != 0) 
    {   
        pr_debug(" wc_check_ic fail ,return  ENODEV \n");
        return -ENODEV;
    }

    ret = cw_init(cw_bat);
    while ((loop++ < 2000) && (ret != 0)) {  
        ret = cw_init(cw_bat);
    }

    if (ret) 
    {  		
        return ret;
    }

    cw_bat->rk_bat.name = "rk-bat";
    cw_bat->rk_bat.type = POWER_SUPPLY_TYPE_BATTERY;
    cw_bat->rk_bat.properties = rk_battery_properties;
    cw_bat->rk_bat.num_properties = ARRAY_SIZE(rk_battery_properties);
    cw_bat->rk_bat.get_property = rk_battery_get_property;
    ret = power_supply_register(&client->dev, &cw_bat->rk_bat);
    if(ret < 0) {
        dev_err(&cw_bat->client->dev, "power supply register rk_bat error\n");
        printk("rk_bat_register_fail\n");
        goto rk_bat_register_fail;
    }

    cw_bat->charger_mode = 0;
    cw_bat->capacity = 0;//2;modify for power_on capacity
    cw_bat->voltage = 0;
    cw_bat->status = 0;
    cw_bat->time_to_empty = 0;
    cw_bat->bat_change = 0;

    cw_update_time_member_capacity_change(cw_bat);
    cw_update_time_member_charge_start(cw_bat);

    cw_bat->battery_workqueue = create_singlethread_workqueue("rk_battery");
    INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
    // INIT_DELAYED_WORK(&cw_bat->dc_wakeup_work, dc_detect_do_wakeup);
    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(10));
/*[PLATFORM]-Add-BEGIN by pingao.yang, 2017/03/20, add asus battery master function */
#if (defined ASUS_BATTERY_MASTER_FUNCTION)
	cw_bat->debug_flag = false;
	cw_bat->user_limit_read = true;
	cw_bat->user_capacity_limit = 0;
	cw_bat->last_user_capacity_limit = 0;
	cw_bat->capacity_limit = false;
	cw_bat->last_capacity_limit = false;
	cw_bat->capacity_to_soc = false;
	cw_bat->battery_master_init = false;
	cw_bat->debug_capacity = CAPACITY_MAPPING_THRESHOLD - 1;
	cw_bat->last_soc = CAPACITY_MAPPING_THRESHOLD - 1;
	cw_bat->last_capacity = CAPACITY_MAPPING_THRESHOLD + 1;
	cw_bat->battery_capacity = CAPACITY_MAPPING_THRESHOLD - 1;
	cw_bat->capacity_map_multiple = CAPACITY_MAPPING_MULTIPLE;
	cw_bat->capacity_map_divide = CAPACITY_MAPPING_DIVIDE;

	INIT_DELAYED_WORK(&cw_bat->bat_master_workqueue,
							battery_master_monitor_work_callback);
	schedule_delayed_work(&cw_bat->bat_master_workqueue, 1*HZ);
	cw_bat->sdev.name = "exttable";
	ret = switch_dev_register(&cw_bat->sdev);
	if (ret  < 0) {
		pr_err("%s(): register error: sdev.name: %s\n",
				__func__, cw_bat->sdev.name);
	}

	cw_bat->battery_master_root_dev = root_device_register("battery_master");
	if (IS_ERR(cw_bat->battery_master_root_dev)) {
		ret = PTR_ERR(cw_bat->battery_master_root_dev);
	}

	ret = device_create_file(cw_bat->battery_master_root_dev, &dev_attr_chg_limit);
	if (ret) {
		pr_err("%s(): create chg_limit error !\n", __func__);
	}

	ret = device_create_file(cw_bat->battery_master_root_dev, &dev_attr_bat_map);
	if (ret) {
		pr_err("%s(): create bat_map error !\n", __func__);
	}

	ret = device_create_file(cw_bat->battery_master_root_dev, &dev_attr_debug_capacity);
	if (ret) {
		pr_err("%s(): create debug_capacity error !\n", __func__);
	}

	ret = device_create_file(cw_bat->battery_master_root_dev, &dev_attr_real_soc);
	if (ret) {
		pr_err("%s(): create real_soc error !\n", __func__);
	}

	dev_set_drvdata(cw_bat->battery_master_root_dev, cw_bat);
#endif
/* [PLATFORM]-Mod-END by pingao.yang */

#ifdef BAT_LOW_INTERRUPT
    ret = cw_bat_regulator_configure(cw_bat, true);
    if (ret < 0) {
        dev_err(&client->dev, "%s Failed to configure regulators\n",__func__);
        goto err_reg_configure;
    }

    ret = cw_bat_power_on(cw_bat, true);
    if (ret < 0) {
        dev_err(&client->dev, "%s Failed to power on\n",__func__);
        goto err_power_device;

    }

    ret = cw_bat_pinctrl_init(cw_bat);
    if (!ret && cw_bat->ts_pinctrl) {
        ret = pinctrl_select_state(cw_bat->ts_pinctrl,
                cw_bat->pinctrl_state_active);
        if (ret < 0)
            goto err_pinctrl_select;
    } 

    ret = cw_bat_gpio_configure(cw_bat, true);
    if (ret < 0) {
        dev_err(&client->dev, "%s Failed to configure gpios\n",__func__);
        goto err_gpio_config;
    }

    INIT_DELAYED_WORK(&cw_bat->bat_low_wakeup_work, bat_low_detect_do_wakeup);
    wake_lock_init(&bat_low_wakelock, WAKE_LOCK_SUSPEND, "bat_low_detect");
    cw_bat->client->irq = gpio_to_irq(pdata->bat_low_pin);
    ret = request_threaded_irq(client->irq, NULL,
            bat_low_detect_irq_handler, pdata->irq_flags,
            "bat_low_detect", cw_bat);
    if (ret) {
        dev_err(&client->dev, "request irq failed\n");
        gpio_free(cw_bat->plat_data->bat_low_pin);
    }
    /*Chaman add for charger detect*/
    charge_psy = power_supply_get_by_name("usb");

err_gpio_config:
    if (cw_bat->ts_pinctrl) {
        ret = cw_bat_pinctrl_select(cw_bat, false);
        if (ret < 0)
            pr_err("Cannot get idle pinctrl state\n");
    }
err_pinctrl_select:
    if (cw_bat->ts_pinctrl){
        pinctrl_put(cw_bat->ts_pinctrl);//free pinctrl
    }
err_power_device:
    cw_bat_power_on(cw_bat, false); 
err_reg_configure:
    cw_bat_regulator_configure(cw_bat, false);
#endif
#if 0 //defined(CONFIG_FB) //Other_platform modify 20160120 huangfusheng.wt update capacity when resume
    cw_bat->fb_notif.notifier_call = fb_notifier_callback;

    ret = fb_register_client(&cw_bat->fb_notif);

    if (ret)
        dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
                ret);
#endif

    printk( "\ncw2015/cw2013 driver v1.2 probe sucess\n");
    return 0;

rk_bat_register_fail:
    dev_dbg(&cw_bat->client->dev, "cw2015/cw2013 driver v1.2 probe error!!!!\n");
    return ret;
}

static int cw_bat_remove(struct i2c_client *client)
{
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	dev_dbg(&cw_bat->client->dev, "%s\n", __func__);

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2017/03/20, add asus battery master function */
#if (defined ASUS_BATTERY_MASTER_FUNCTION)
	switch_dev_unregister(&cw_bat->sdev);
	root_device_unregister(cw_bat->battery_master_root_dev);
	cancel_delayed_work(&cw_bat->bat_master_workqueue);
#endif
/* [PLATFORM]-Mod-END by pingao.yang */
	cancel_delayed_work(&cw_bat->battery_delay_work);

	return 0;
}

static const struct i2c_device_id cw_id[] = {
    { "cw201x", 0 },     //cw2013 & cw2015 compatible
};
MODULE_DEVICE_TABLE(i2c, cw_id);

static struct of_device_id cw2015_match_table[] = {
    { .compatible = "cellwise,cw2015", },
    { },
};
static struct i2c_driver cw_bat_driver = {
    .driver         = {
        .name   = "cw201x",    //cw2013 & cw2015 compatible   

#ifdef CONFIG_PM                    //Other_platform modify 20160120 huangfusheng.wt update capacity when resume
        .pm = &cw_bat_pm_ops,
#endif
        .of_match_table = cw2015_match_table,
    },
    .probe          = cw_bat_probe,
    .remove         = cw_bat_remove,
    .id_table	    = cw_id,
};

static int __init cw_bat_init(void)
{
    return i2c_add_driver(&cw_bat_driver);
}

static void __exit cw_bat_exit(void)
{
    i2c_del_driver(&cw_bat_driver);
}

late_initcall(cw_bat_init);
module_exit(cw_bat_exit);

MODULE_AUTHOR("ben<ben.chen@cellwise-semi.com>");
MODULE_DESCRIPTION("cw2015/cw2013 battery driver");
MODULE_LICENSE("GPL");
