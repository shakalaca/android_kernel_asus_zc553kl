#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include "../../staging/android/timed_output.h"

#define VIBRATOR_DEBUG_ON	0
#define VIBRATOR_DEBUG(fmt,arg...)	do{\
    if(VIBRATOR_DEBUG_ON)\
    printk("<VIBRATOR> [%d] "fmt"\n",__LINE__, ##arg);\
}while(0)

#ifdef CONFIG_K89200_FEATURES 
#define SABRESD_VIBRATOR_CTL                   128 
#endif 

#ifdef CONFIG_K89218_FEATURES
#define SABRESD_VIBRATOR_CTL                   97
#endif 

#define VIB_DEFAULT_TIMEOUT    15000


static struct vibrator {  
    struct hrtimer timer;     
    struct mutex lock;         
    struct work_struct work;   
} vibdata;  

static void mx6_vibrator_off(void)  
{  
    gpio_direction_output(SABRESD_VIBRATOR_CTL,0);         
}  

void mx6_motor_enable(struct timed_output_dev *sdev, int value)  
{  
    mutex_lock(&vibdata.lock);                       

    /* cancelprevious timer and set GPIO according to value */  
    hrtimer_cancel(&vibdata.timer);             
    cancel_work_sync(&vibdata.work);   

    VIBRATOR_DEBUG("dihongwei add for vibrator enable value=%d!\n", value);       
    if(value)  
    {  
        gpio_direction_output(SABRESD_VIBRATOR_CTL,1);  

        if(value > 0)  
        {  
            VIBRATOR_DEBUG("dihongwei add for vibrator enable value=%d!\n", value);
            value = (value > VIB_DEFAULT_TIMEOUT ? 
                    VIB_DEFAULT_TIMEOUT : value);

            hrtimer_start(&vibdata.timer,
                    ktime_set(value / 1000, (value % 1000) * 1000000),
                    HRTIMER_MODE_REL);  
        }  
    }  
    else  
        mx6_vibrator_off();  

    mutex_unlock(&vibdata.lock);
} 

int mx6_get_time(struct timed_output_dev *sdev)  
{  
    if(hrtimer_active(&vibdata.timer))  
    {  
        ktime_t r = hrtimer_get_remaining(&vibdata.timer);                 
        return ktime_to_ms(r);  
    }  
    return 0;  
}  

struct timed_output_dev mx6_motot_driver = {  
    .name = "vibrator",   
    .enable = mx6_motor_enable,  
    .get_time = mx6_get_time,  
};  

static enum hrtimer_restart mx6_vibrator_timer_func(struct hrtimer * timer)  
{  
    schedule_work(&vibdata.work);               
    return HRTIMER_NORESTART;  
}  
static void mx6_vibrator_work(struct work_struct *work)  
{  
    VIBRATOR_DEBUG("%s: dihongwei add vibrator off!\n", __func__);
    mx6_vibrator_off();  
}  

static int __init mx6_motor_init(void)  
{  
    int ret =0;

    ret= gpio_request(SABRESD_VIBRATOR_CTL, "vibrator_en");
    if (ret) {
        pr_err("request vibrator gpio failed, rc=%d\n", ret);
    }

    VIBRATOR_DEBUG("%s: dihongwei add vibrator mx6_motor_init!\n", __func__);
    mutex_init(&vibdata.lock);             

    INIT_WORK(&vibdata.work, mx6_vibrator_work);   

    hrtimer_init(&vibdata.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);  
    vibdata.timer.function= mx6_vibrator_timer_func;           

    ret = timed_output_dev_register(&mx6_motot_driver);
    if (ret< 0)  
        goto err_to_dev_reg;  

    return 0;  

err_to_dev_reg:           
    mutex_destroy(&vibdata.lock);  
    gpio_free(SABRESD_VIBRATOR_CTL);  
    VIBRATOR_DEBUG("%s: vibrator   err!:%d\n", __func__, ret);  
    return ret;  

}  

static void __exit mx6_motor_exit(void)  
{  
    VIBRATOR_DEBUG("%s: vibrator  exit!\n", __func__);  
    cancel_work_sync(&vibdata.work);
    hrtimer_cancel(&vibdata.timer);

    timed_output_dev_register(&mx6_motot_driver);  

    mutex_destroy(&vibdata.lock);  
    gpio_free(SABRESD_VIBRATOR_CTL);  
}  
module_init(mx6_motor_init);  
module_exit(mx6_motor_exit);  

MODULE_AUTHOR("<xiehong>");  
MODULE_DESCRIPTION("Motor Vibrator driver");  
MODULE_LICENSE("GPL");  
