#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of_fdt.h>
#include <asm/setup.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/genhd.h> 
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>


static struct platform_device *asus_phoneinfo_pdev;
static struct device *asus_phoneinfo_root_dev;

//每个字段占用1kb长度。

//define type
typedef enum{
 WT_PHONEINFO_ssn = 30000,
 WT_PHONEINFO_isn = 30001,
 WT_PHONEINFO_color_id = 30002,
 WT_PHONEINFO_country_code = 30003,
 WT_PHONEINFO_customer_id = 30004,
 WT_PHONEINFO_packing_code = 30005,
 WT_PHONEINFO_charge_limit = 30006,
 WT_PHONEINFO_battery_map = 30007,
}wt_phoneinfo_type;

#define WT_PHONEINFO_STRING_LEN 1024

#define PHONE_INFO_PATH "/dev/block/platform/soc/7824900.sdhci/by-name/ftm"

static int wt_phoneinfo_read(wt_phoneinfo_type type, char* buf);
static int wt_phoneinfo_write(wt_phoneinfo_type type, const char* buf, int len);

//define macor ATTR
#define WT_PHONEINFO_CREATE_ATTR(name) \
static ssize_t phoneinfo_##name##_store(struct device * dev, struct device_attribute *attr, const char * buf,size_t count); \
static ssize_t phoneinfo_##name##_show(struct device *dev, struct device_attribute *attr, char *buf); \
\
\
static ssize_t phoneinfo_##name##_store(struct device * dev, struct device_attribute *attr, const char * buf,\
				  size_t count)\
{\
	int ret = -1;\
	printk("entry  %s\n",__FUNCTION__);\
\
	ret = wt_phoneinfo_write(WT_PHONEINFO_##name, buf, count);\
	\
	return count;\
}\
\
static ssize_t phoneinfo_##name##_show(struct device *dev, struct device_attribute *attr, char *buf)\
{\
	int ret = -1;\
	char pbuf[WT_PHONEINFO_STRING_LEN];\
	printk("entry  %s\n",__FUNCTION__);\
\
	ret = wt_phoneinfo_read(WT_PHONEINFO_##name,pbuf);\
\
	return sprintf(buf, #name":%s\n",pbuf);\
}\
\
static DEVICE_ATTR(name, S_IWUSR|S_IRUSR|S_IROTH|S_IRGRP, phoneinfo_##name##_show, phoneinfo_##name##_store);

//define function
WT_PHONEINFO_CREATE_ATTR(ssn)
WT_PHONEINFO_CREATE_ATTR(isn)
WT_PHONEINFO_CREATE_ATTR(color_id)
WT_PHONEINFO_CREATE_ATTR(country_code)
WT_PHONEINFO_CREATE_ATTR(customer_id)
WT_PHONEINFO_CREATE_ATTR(packing_code)
WT_PHONEINFO_CREATE_ATTR(charge_limit)
WT_PHONEINFO_CREATE_ATTR(battery_map)


static int wt_create_device_files(void)
{
	int rc = 0;

	rc = device_create_file(asus_phoneinfo_root_dev, &dev_attr_ssn);
	if (rc)
		return rc;

	rc = device_create_file(asus_phoneinfo_root_dev, &dev_attr_isn);
	if (rc)
		return rc;

	rc = device_create_file(asus_phoneinfo_root_dev, &dev_attr_country_code);
	if (rc)
		return rc;

	rc = device_create_file(asus_phoneinfo_root_dev, &dev_attr_color_id);
	if (rc)
		return rc;

	rc = device_create_file(asus_phoneinfo_root_dev, &dev_attr_customer_id);
	if (rc)
		return rc;

	rc = device_create_file(asus_phoneinfo_root_dev, &dev_attr_packing_code);
	if (rc)
		return rc;

	rc = device_create_file(asus_phoneinfo_root_dev, &dev_attr_charge_limit);
	if (rc)
		return rc;

	rc = device_create_file(asus_phoneinfo_root_dev, &dev_attr_battery_map);
	if (rc)
		return rc;

	return 0;
}

//store in buf, return len. buf least size is 1024 
static int wt_phoneinfo_read(wt_phoneinfo_type type, char* buf)
{
	struct file *fp;
	char fbuf[WT_PHONEINFO_STRING_LEN];
	mm_segment_t fs;
	int ret;
	int len;

	printk("%s\n", __func__);

	fp = filp_open(PHONE_INFO_PATH, O_RDWR | O_CREAT, 0);
	if (IS_ERR(fp)) {
	        printk("[RTX] %s: open ohone info path error\n", __func__);
	        return -1;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	fp->f_pos = fp->f_pos + WT_PHONEINFO_STRING_LEN*type;
	ret = fp->f_op->read(fp, fbuf, WT_PHONEINFO_STRING_LEN, &fp->f_pos);
	if (ret != WT_PHONEINFO_STRING_LEN) {
	        printk("%s: Read bytes from phoneinfo failed! %d\n", __func__, ret);
	        return -1;
	}

	set_fs(fs);
	filp_close(fp, NULL);

	len = strlen(fbuf);
	printk("%s,read buf len:%d\n", __func__, len);

	if (len < WT_PHONEINFO_STRING_LEN) {
		strcpy(buf, fbuf);
		printk("%s,read buf:%s\n", __func__, buf);
		return len;
	} else {
		return 0;
	}
}

//store in buf, return len. buf least size is 1024 
static int wt_phoneinfo_write(wt_phoneinfo_type type, const char* buf, int len)
{
	struct file *fp;
	mm_segment_t fs;
	int ret;
	char buf_tmp[WT_PHONEINFO_STRING_LEN] = {0};

	printk("%s\n", __func__);

	printk("%s,%s len %d\n", __func__, buf, len);

	//max len is WT_PHONEINFO_STRING_LEN
	if (len > WT_PHONEINFO_STRING_LEN) {
		return -1;
	}	
	else {
		memcpy(buf_tmp,buf,len);
	}

	if(buf_tmp[len-1] == '\n')
	buf_tmp[len-1] = 0x00;

	fp = filp_open(PHONE_INFO_PATH, O_RDWR | O_CREAT, 0);
	if (IS_ERR(fp))
	{
	    printk("[RTX] %s: open phone info path error\n", __func__);
	    return -1;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	fp->f_pos = fp->f_pos + WT_PHONEINFO_STRING_LEN*type;
	ret = fp->f_op->write(fp, buf_tmp, len, &fp->f_pos);
	if (ret != len) {
	    printk("%s: Write phoneinfo failed! %d\n", __func__, ret);
	    return -1;
	}

	set_fs(fs);
	filp_close(fp, NULL);

	printk("%s,write buf len:%d\n",__func__,ret);

	return ret;
}

#define USER_LIMIT_ENABLE	80
#define USER_LIMIT_DISABLE	100
#define BATTERY_MAP1	"map1"
#define BATTERY_MAP2	"map2"

int battery_master_limit_info_write(int limit, int map)
{
	int ret = 0;
	int user_limit = 0;
	char buf[10] = "limit";
	char buf_m[10] = "map";
	char limit_buf[4];
	char map_buf[4];

	printk("%s\n", __func__);

	if (limit == 1) {
		user_limit = USER_LIMIT_ENABLE;
	} else {
		user_limit = USER_LIMIT_DISABLE;
	}

	sprintf(limit_buf, "%d", user_limit);
	strcat(buf, limit_buf);
	ret = wt_phoneinfo_write(WT_PHONEINFO_charge_limit, buf, sizeof(buf));
	if (ret < sizeof(buf)) {
		printk("%s: Write charge_limit failed !\n", __func__);
	}

	sprintf(map_buf, "%d", map);
	strcat(buf_m, map_buf);
	ret = wt_phoneinfo_write(WT_PHONEINFO_battery_map, buf_m, sizeof(buf_m));
	if (ret < sizeof(buf_m)) {
		printk("%s: Write battery_map failed !\n", __func__);
	}

	return ret;
}

int battery_master_limit_info_read(int *limit, int *map)
{

	int ret = 0;
	char limit_buf[10] = "map";
	char map_buf[10] = "limit";

	printk("%s\n", __func__);

	ret = wt_phoneinfo_read(WT_PHONEINFO_charge_limit, limit_buf);
	if (ret <= 0) {
		printk("%s, read charge limit fail, buf:%s\n", __func__, limit_buf);
		return 0;
	}

	if (!strncmp(limit_buf, "limit", 5)) {
		*limit = simple_strtol(&limit_buf[5], NULL, 10);
		printk("%s, read buf:%s, user_limit = %d\n", __func__, limit_buf, *limit);
	}

	ret = wt_phoneinfo_read(WT_PHONEINFO_battery_map, map_buf);
	if (ret <= 0) {
		printk("%s, read battery map fail, buf:%s\n", __func__, map_buf);
		return 0;
	}

	if (!strncmp(map_buf, "map1", 4)) {
		*map = 1;
	} else {
		*map = 0;
	}

	printk("%s, read map_buf: %s, map: %d\n", __func__, map_buf, *map);

	return ret;
	
}

static struct platform_driver asus_phoneinfo_pdrv = {
	.driver = {
		.name	= "asus_phoneinfo",
		.owner	= THIS_MODULE,
	//	.pm	= &asus_phoneinfo_ops,
	},
};


static int __init
asus_phoneinfo_init(void)
{
	int rc;

	rc = platform_driver_register(&asus_phoneinfo_pdrv);
	if (rc)
		return rc;

	asus_phoneinfo_pdev = platform_device_register_simple("asus_phoneinfo", -1, NULL, 0);
	if (IS_ERR(asus_phoneinfo_pdev)) {
		rc = PTR_ERR(asus_phoneinfo_pdev);
		goto out_pdrv;
	}

	asus_phoneinfo_root_dev = root_device_register("asus_phoneinfo");
	if (IS_ERR(asus_phoneinfo_root_dev)) {
		rc = PTR_ERR(asus_phoneinfo_root_dev);
		goto out_pdev;
	}
	
	rc = wt_create_device_files();
	if (rc)
		goto out_root;

	return 0;

out_root:
	root_device_unregister(asus_phoneinfo_root_dev);
out_pdev:
	platform_device_unregister(asus_phoneinfo_pdev);
out_pdrv:
	platform_driver_unregister(&asus_phoneinfo_pdrv);
	return rc;
}

/*
 * The init/exit functions.
 */
static void __exit
asus_phoneinfo_exit(void)
{
	platform_device_unregister(asus_phoneinfo_pdev);
	platform_driver_unregister(&asus_phoneinfo_pdrv);
	root_device_unregister(asus_phoneinfo_root_dev);
}

module_init(asus_phoneinfo_init);
module_exit(asus_phoneinfo_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zkx");

