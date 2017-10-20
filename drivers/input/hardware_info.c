#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>

#include <linux/hardware_info.h>

char hardwareinfo_name[HARDWARE_MAX_ITEM][HARDWARE_MAX_ITEM_LONGTH];
char Lcm_name[HARDWARE_MAX_ITEM_LONGTH]; // add for lcd
int hardwareinfo_set_prop(int cmd, const char *name)
{
	if (cmd < 0 || cmd >= HARDWARE_MAX_ITEM)
		return -1;

	strcpy(hardwareinfo_name[cmd], name);
	
	return 0;
}
EXPORT_SYMBOL_GPL(hardwareinfo_set_prop);

static long hardwareinfo_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0, hardwareinfo_num;
	void __user *data = (void __user *)arg;
	
	printk("xieh add hardwareinfo_ioctl cmd = %d", cmd);

	switch(cmd) {
	case HARDWARE_LCD_GET:
		hardwareinfo_set_prop(HARDWARE_LCD, Lcm_name);
		hardwareinfo_num = HARDWARE_LCD;
		break;
	case HARDWARE_TP_GET:
		hardwareinfo_num = HARDWARE_TP;
		break;
	case HARDWARE_FLASH_GET:
		hardwareinfo_num = HARDWARE_FLASH;
		break;
	case HARDWARE_FRONT_CAM_GET:
		hardwareinfo_num = HARDWARE_FRONT_CAM;
		break;
	case HARDWARE_BACK_CAM_GET:
		hardwareinfo_num = HARDWARE_BACK_CAM;
		break;
	case HARDWARE_ACCELEROMETER_GET:
		hardwareinfo_num = HARDWARE_ACCELEROMETER;
		break;
	case HARDWARE_ALSPS_GET:
		hardwareinfo_num = HARDWARE_ALSPS;
		break;
	case HARDWARE_GYROSCOPE_GET:
		hardwareinfo_num = HARDWARE_GYROSCOPE;
		break;
	case HARDWARE_MAGNETOMETER_GET:
		hardwareinfo_num = HARDWARE_MAGNETOMETER;
		break;
	case HARDWARE_BT_GET:
		hardwareinfo_num = HARDWARE_BT;
		break;
	case HARDWARE_WIFI_GET:
		hardwareinfo_num = HARDWARE_WIFI;
		break;
	case HARDWARE_GPS_GET:
		hardwareinfo_num = HARDWARE_GPS;
		break;
	case HARDWARE_FM_GET:
		hardwareinfo_num = HARDWARE_FM;
		break;
	case HARDWARE_BACK_CAM_MOUDULE_ID_GET:
		hardwareinfo_num = HARDWARE_BACK_CAM_MOUDULE_ID;
		break;
	case HARDWARE_FRONT_CAM_MODULE_ID_GET:
		hardwareinfo_num = HARDWARE_FRONT_CAM_MODULE_ID;
		break;
	default:
		ret = -EINVAL;
		goto err_out;
	}
	
	printk("xieh add ioctl enter cmd=%d, num=%d", cmd, hardwareinfo_num);
	memset(data, 0, HARDWARE_MAX_ITEM_LONGTH);
	if (copy_to_user(data, hardwareinfo_name[hardwareinfo_num], strlen(hardwareinfo_name[hardwareinfo_num])))
	{
		ret = -EINVAL;
	}
err_out:
	return ret;
}

#ifdef CONFIG_COMPAT
static long hardwareinfo_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return hardwareinfo_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int hardwareinfo_open(struct inode *inode, struct file *filp)
{
	nonseekable_open(inode, filp);

	return 0;
}

static struct file_operations hardwareinfo_fops = {
	.owner = THIS_MODULE,
	.open = hardwareinfo_open,
	.unlocked_ioctl = hardwareinfo_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = hardwareinfo_compat_ioctl,
#endif
};

static struct miscdevice hardwareinfo_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hardwareinfo",
	.fops = &hardwareinfo_fops,
};


static int __init hardwareinfo_init(void)
{
	int ret;
/*	
	int i;

	for(i = 0; i < HARDWARE_MAX_ITEM; i++) 
	{
		strcpy(hardwareinfo_name[i], "NULL");
	}
*/	
	hardwareinfo_set_prop(HARDWARE_BT, "Qualcomm_WCN3615");
	hardwareinfo_set_prop(HARDWARE_WIFI, "Qualcomm_WCN3615");
	hardwareinfo_set_prop(HARDWARE_GPS, "Qualcomm_WTR2965");
	hardwareinfo_set_prop(HARDWARE_FM, "Qualcomm_WCN3615");
	ret = misc_register(&hardwareinfo_device);
	if (ret < 0) 
	{
		printk("xieh add misc register failed!\n");
		return -ENODEV;		
	}
	printk("xieh add misc_register successfully!\n");
	
	return 0;
}

static void __exit hardwareinfo_exit(void)
{
	misc_deregister(&hardwareinfo_device);
}

module_init(hardwareinfo_init);
module_exit(hardwareinfo_exit);

MODULE_DESCRIPTION("User mode device interface");
MODULE_LICENSE("GPL");
