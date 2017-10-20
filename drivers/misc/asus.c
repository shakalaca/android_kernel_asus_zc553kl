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

extern int get_io_value(void);
extern int get_cardTray_State(void);//add by jiayu for cardTray check
char emmc_info[20]={0, };
static int gpiostatus_proc_show(struct seq_file *m, void *v)
{
	int temp =0;
	temp = get_io_value();
	seq_printf(m, "%d\n",temp);
	return 0;
}
static int emcp_type_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n",emmc_info);
	return 0;
}

/**********************begin-add by dingjiayu for cardtray check*************************/
static int sdtray_proc_show(struct seq_file *m, void *v)
{
	int cardTray_state = 0;
	
	
	cardTray_state = get_cardTray_State();
	if( cardTray_state < 0 )
	{
		seq_printf(m, "-1");
		seq_printf(m, "\n");
		 return 0;
	}
	else if(cardTray_state == 0)
	{		
		seq_printf(m, "0");
		seq_printf(m, "\n");
	}
	else if(cardTray_state == 1)
	{		
		seq_printf(m, "1");
		seq_printf(m, "\n");
	}
	return 0;
}
/**********************end-add by dingjiayu for cardtray check*************************/

#define PROC_FOPS_RO(name)	\
	static int name##_proc_open(struct inode *inode, struct file *file)	\
	{									\
		return single_open(file, name##_proc_show, PDE_DATA(inode));	\
	}									\
	static const struct file_operations name##_proc_fops = {		\
		.owner          = THIS_MODULE,					\
		.open           = name##_proc_open,				\
		.read           = seq_read,					\
		.llseek         = seq_lseek,					\
		.release        = single_release,				\
	}

#define PROC_ENTRY(name) {__stringify(name), &name##_proc_fops}

PROC_FOPS_RO(gpiostatus);
PROC_FOPS_RO(emcp_type);
PROC_FOPS_RO(sdtray);//add by jiayu for cardTray check

struct pentry {
	const char *name;
	const struct file_operations *fops;
};
const struct pentry auss_entries[] = {

	PROC_ENTRY(gpiostatus),
	PROC_ENTRY(emcp_type),
	PROC_ENTRY(sdtray),//add by jiayu for cardTray check
};

static int __init proc_auss_init(void)
{
	struct proc_dir_entry *dir_entry = NULL;
	int i = 0;

	dir_entry = proc_mkdir("auss", NULL);
	if (!dir_entry) {
		pr_err("AUSS: Failed to create /proc/ entry\n");
		return -ENOMEM;
	}
	
	for (i = 0; i < ARRAY_SIZE(auss_entries); i++) {
		if (! proc_create(auss_entries[i].name, S_IRUGO, dir_entry, auss_entries[i].fops))
			pr_err("AUSS: Failed to create /proc/auss entry nodes\n");
	}

    return 0;
}
module_init(proc_auss_init);
