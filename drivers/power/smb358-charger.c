/* Copyright (c) 2014 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "SMB358 %s: " fmt, __func__

#define MSM_SMB358_DEBUG_ON     0
#define SMB358_PRINT(fmt, args...) \
	do{ \
		if (MSM_SMB358_DEBUG_ON) \
			pr_err("<SMB358> "fmt, ##args); \
	} while(0)

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>
//NewFeature,zhangpu.wt,ADD,2015.8.24,add led update
#include <linux/alarmtimer.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/power_supply.h>

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add charge ntc debug simulate */
//#define QPNP_NTC_DEBUG_SIMULATE
#if (defined QPNP_NTC_DEBUG_SIMULATE)
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#endif
/* [PLATFORM]-Mod-END by pingao.yang */

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add battery temperature protection*/
#define QPNP_BATTERY_NTC_PROTECTION
#if (defined QPNP_BATTERY_NTC_PROTECTION)
#include <linux/wakelock.h>

#define NTC_PROTECTION_MES_INTERVAL	10*HZ

#define HOT_TO_WARN_DEGREE	20
#define WARM_TO_GOOD_DEGREE	20
#define COOL_TO_GOOD_DEGREE	20
#define COLD_TO_COOL_DEGREE	20
#define SYSTEM_SHUT_DOWN_HIGH	700

enum battery_health_type {
	BATTERY_HEALTH_GOOD,
	BATTERY_HEALTH_OVERHEAT,
	BATTERY_HEALTH_WARM,
	BATTERY_HEALTH_COOL,
	BATTERY_HEALTH_COLD,
};
#endif
/* [PLATFORM]-Mod-END by pingao.yang */

#define CHARGE_FULL_CAPACITY	100
#define RECHARGE_CAPACITY	98	//add by pingao.yang

#define _SMB358_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB358_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB358_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
			(RIGHT_BIT_POS))

/* Config/Control registers */
#define CHG_CURRENT_CTRL_REG		0x0
#define CHG_OTH_CURRENT_CTRL_REG	0x1
#define VARIOUS_FUNC_REG		0x2
#define VFLOAT_REG			0x3
#define CHG_CTRL_REG			0x4
#define STAT_AND_TIMER_CTRL_REG		0x5
#define CHG_PIN_EN_CTRL_REG		0x6
#define THERM_A_CTRL_REG		0x7
#define SYSOK_AND_USB3_REG		0x8
#define OTHER_CTRL_REG			0x9
#define FAULT_INT_REG			0xC
#define STATUS_INT_REG			0xD
#define OTG_CONTROL_REG  0xA  //bug_130601 modify 20160118 huangfusheng.wt for 2phone charge through otg cable
#define HARD_SOFT_LIMIT_TEMP_REG	0xB

/* Command registers */
#define CMD_A_REG			0x30
#define CMD_B_REG			0x31

/* IRQ status registers */
#define IRQ_A_REG			0x35
#define IRQ_B_REG			0x36
#define IRQ_C_REG			0x37
#define IRQ_D_REG			0x38
#define IRQ_E_REG			0x39
#define IRQ_F_REG			0x3A

/* Status registers */
#define STATUS_A_REG            0x3B
#define STATUS_A_FLOAT_VOLTAGE_MASK		0x3F
#define STATUS_B_REG            0x3C
#define STATUS_C_REG			0x3D
#define STATUS_D_REG			0x3E
#define STATUS_E_REG			0x3F

/* Config bits */
#define CHG_INHI_EN_MASK			BIT(1)
#define CHG_INHI_EN_BIT				BIT(1)
#define CMD_A_CHG_ENABLE_BIT			BIT(1)
#define CMD_A_VOLATILE_W_PERM_BIT		BIT(7)
#define CMD_A_CHG_SUSP_EN_BIT			BIT(2)
#define CMD_A_CHG_SUSP_EN_MASK			BIT(2)
#define CMD_A_OTG_ENABLE_BIT			BIT(4)
#define CMD_A_OTG_ENABLE_MASK			BIT(4)
#define CMD_B_CHG_HC_ENABLE_BIT			BIT(0)
#define USB3_ENABLE_BIT				BIT(5)
#define USB3_ENABLE_MASK			BIT(5)
#define CMD_B_CHG_USB_500_900_ENABLE_BIT	BIT(1)
#define CHG_CTRL_AUTO_RECHARGE_ENABLE_BIT	0x0
#define CHG_CTRL_CURR_TERM_END_CHG_BIT		0x0
#define CHG_CTRL_BATT_MISSING_DET_THERM_IO	SMB358_MASK(5, 4)
#define CHG_CTRL_AUTO_RECHARGE_MASK		BIT(7)
#define CHG_AUTO_RECHARGE_DIS_BIT		BIT(7)
#define CHG_CTRL_CURR_TERM_END_MASK		BIT(6)
#define CHG_CTRL_BATT_MISSING_DET_MASK		SMB358_MASK(5, 4)
#define CHG_CTRL_APSD_EN_BIT			BIT(2)
#define CHG_CTRL_APSD_EN_MASK			BIT(2)
#define CHG_ITERM_MASK				0x07
#define CHG_PIN_CTRL_USBCS_REG_BIT		0x0
#define OTG_CURRENT_CONTROL_BIT2     BIT(2)    //bug_130601 modify 20160118 huangfusheng.wt for 2phone charge through otg cable
#define OTG_CURRENT_CONTROL_BIT3     BIT(3)    //bug_130601 modify 20160118 huangfusheng.wt for 2phone charge through otg cable
#define CTRL_FREQ_SWITCH_BIT	BIT(7)

/* hw jeita config bits */
#define FLOAT_VOLTAGE_COMPENSATION_MASK		SMB358_MASK(4, 3)
#define FLOAT_VOLTAGE_COMPENSATION_BIT		SMB358_MASK(4, 3)
#define HARD_TEMPERATURE_LIMIT_MASK		BIT(2)
#define HARD_TEMPERATURE_LIMIT_BIT	0x0

#define THERM_MONITOR_SELECTION_MASK	 BIT(5)
#define THERM_MONITOR_SELECTION_BIT	 BIT(5)
#define THERM_MONITOR_ENABLE_MASK	 BIT(4)
#define THERM_MONITOR_ENABLE_BIT	 0x0
#define SOFT_COLD_TEMP_LIMIT_BEHAVIOR_MASK	 SMB358_MASK(3, 2)
#define SOFT_COLD_TEMP_LIMIT_BEHAVIOR_BIT	 0x04
#define SOFT_HOT_TEMP_LIMIT_BEHAVIOR_MASK	 SMB358_MASK(1, 0)
#define SOFT_HOT_TEMP_LIMIT_BEHAVIOR_BIT	 0x02

#define CHARGE_CURRENT_COMPENSATION_MASK		SMB358_MASK(7, 6)
#define CHARGE_CURRENT_COMPENSATION_BIT		0x80
#define DIGITAL_THERMAL_TEMP_THRESHOLD_MASK		SMB358_MASK(5, 4)
#define DIGITAL_THERMAL_TEMP_THRESHOLD_BIT		SMB358_MASK(5, 4)

#define HARD_LIMIT_COLD_TEMP_POINT_MASK		SMB358_MASK(7, 6)
#define HARD_LIMIT_COLD_TEMP_POINT_BIT		0x40
#define HARD_LIMIT_HOT_TEMP_POINT_MASK		SMB358_MASK(5, 4)
#define HARD_LIMIT_HOT_TEMP_POINT_BIT		SMB358_MASK(5, 4)

#define SOFT_LIMIT_COLD_TEMP_POINT_MASK		SMB358_MASK(3, 2)
#define SOFT_LIMIT_COLD_TEMP_POINT_BIT		0x0
#define SOFT_LIMIT_HOT_TEMP_POINT_MASK		SMB358_MASK(1, 0)
#define SOFT_LIMIT_HOT_TEMP_POINT_BIT		SMB358_MASK(1, 0)

/* This is to select if use external pin EN to control CHG */
#define CHG_PIN_CTRL_CHG_EN_LOW_PIN_BIT		SMB358_MASK(6, 5)
#define CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT		0x0
#define CHG_PIN_CTRL_CHG_EN_MASK		SMB358_MASK(6, 5)

#define CHG_LOW_BATT_THRESHOLD \
				SMB358_MASK(3, 0)
#define CHG_PIN_CTRL_USBCS_REG_MASK		BIT(4)
#define CHG_PIN_CTRL_APSD_IRQ_BIT		BIT(1)
#define CHG_PIN_CTRL_APSD_IRQ_MASK		BIT(1)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_BIT		BIT(2)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_MASK		BIT(2)
#define VARIOUS_FUNC_USB_SUSP_EN_REG_BIT	BIT(6)
#define VARIOUS_FUNC_USB_SUSP_MASK		BIT(6)
#define FAULT_INT_HOT_COLD_HARD_BIT		BIT(7)
#define FAULT_INT_HOT_COLD_SOFT_BIT		BIT(6)
#define FAULT_INT_INPUT_OV_BIT			BIT(3)
#define FAULT_INT_INPUT_UV_BIT			BIT(2)
#define FAULT_INT_AICL_COMPLETE_BIT		BIT(1)
#define STATUS_INT_CHG_TIMEOUT_BIT		BIT(7)
#define STATUS_INT_OTG_DETECT_BIT		BIT(6)
#define STATUS_INT_BATT_OV_BIT			BIT(5)
#define STATUS_INT_CHGING_BIT			BIT(4)
#define STATUS_INT_CHG_INHI_BIT			BIT(3)
#define STATUS_INT_INOK_BIT			BIT(2)
#define STATUS_INT_MISSING_BATT_BIT		BIT(1)
#define STATUS_INT_LOW_BATT_BIT			BIT(0)
#define THERM_A_THERM_MONITOR_EN_BIT		0x0
#define THERM_A_THERM_MONITOR_EN_MASK		BIT(4)
#define VFLOAT_MASK				0x3F

/* IRQ status bits */
#define IRQ_A_HOT_HARD_BIT			BIT(6)
#define IRQ_A_COLD_HARD_BIT			BIT(4)
#define IRQ_A_HOT_SOFT_BIT			BIT(2)
#define IRQ_A_COLD_SOFT_BIT			BIT(0)
#define IRQ_B_BATT_MISSING_BIT			BIT(4)
#define IRQ_B_BATT_LOW_BIT			BIT(2)
#define IRQ_B_BATT_OV_BIT			BIT(6)
#define IRQ_B_PRE_FAST_CHG_BIT			BIT(0)
#define IRQ_C_TAPER_CHG_BIT			BIT(2)
#define IRQ_C_TERM_BIT				BIT(0)
#define IRQ_C_INT_OVER_TEMP_BIT			BIT(6)
#define IRQ_D_CHG_TIMEOUT_BIT			(BIT(0) | BIT(2))
#define IRQ_D_AICL_DONE_BIT			BIT(4)
#define IRQ_D_APSD_COMPLETE			BIT(6)
#define IRQ_E_INPUT_UV_BIT			BIT(0)
#define IRQ_E_INPUT_OV_BIT			BIT(2)
#define IRQ_E_AFVC_ACTIVE                       BIT(4)
#define IRQ_F_OTG_VALID_BIT			BIT(2)
#define IRQ_F_OTG_BATT_FAIL_BIT			BIT(4)
#define IRQ_F_OTG_OC_BIT			BIT(6)
#define IRQ_F_POWER_OK				BIT(0)

/* Status  bits */
#define STATUS_C_CHARGING_MASK			SMB358_MASK(2, 1)
#define STATUS_C_FAST_CHARGING			BIT(2)
#define STATUS_C_PRE_CHARGING			BIT(1)
#define STATUS_C_TAPER_CHARGING			SMB358_MASK(2, 1)
#define STATUS_C_CHG_ERR_STATUS_BIT		BIT(6)
#define STATUS_C_CHG_ENABLE_STATUS_BIT		BIT(0)
#define STATUS_C_CHG_HOLD_OFF_BIT		BIT(3)
#define STATUS_D_CHARGING_PORT_MASK \
				SMB358_MASK(3, 0)
#define STATUS_D_PORT_ACA_DOCK			BIT(3)
#define STATUS_D_PORT_SDP			BIT(2)
#define STATUS_D_PORT_DCP			BIT(1)
#define STATUS_D_PORT_CDP			BIT(0)
#define STATUS_D_PORT_OTHER			SMB358_MASK(1, 0)
#define STATUS_D_PORT_ACA_A			(BIT(2) | BIT(0))
#define STATUS_D_PORT_ACA_B			SMB358_MASK(2, 1)
#define STATUS_D_PORT_ACA_C			SMB358_MASK(2, 0)
#define STATUS_D_CHARGER_TYPE_MASK	SMB358_MASK(3, 0)

/* constants */
#define USB2_MIN_CURRENT_MA		100
#define USB2_MAX_CURRENT_MA		500
#define USB3_MIN_CURRENT_MA		150
#define USB3_MAX_CURRENT_MA		900
#define AC_CHG_CURRENT_MASK		0x70
#define AC_CHG_CURRENT_SHIFT		4
#define SMB358_IRQ_REG_COUNT		6
#define SMB358_FAST_CHG_MIN_MA		200
#define SMB358_FAST_CHG_MAX_MA		2000
#define SMB358_FAST_CHG_SHIFT		5
#define SMB_FAST_CHG_CURRENT_MASK	0xE0
#define SMB358_DEFAULT_BATT_CAPACITY	10 //NewFeature,zhangpu.wt,ADD,2015.8.24,add led update
#define SMB358_BATT_GOOD_THRE_2P5	0x1

#define BATTERY_FCC 4000 // Other_platform modify 20151202 huangfusheng.wt add battery capacity discription

int pre_usb_current_ma = -EINVAL; //Other_platform modify 20151113 huangfusheng.wt solve current jump
bool thermal=false;
bool recovery=false;

static struct power_supply *cw2015_psy = NULL; //Other_platform_modify 20151123 huangfusheng.wt add cw2015 

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	SOC	= BIT(3),
};

enum path_type {
	USB,
	DC,
};
struct smb358_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct smb358_charger {
	struct i2c_client	*client;
	struct device		*dev;

	bool			inhibit_disabled;
	bool			recharge_disabled;
	int			recharge_mv;
	bool			iterm_disabled;
	int			iterm_ma;
	int			vfloat_mv;
	int			chg_valid_gpio;
	int			chg_valid_act_low;
	int			chg_present;
	int			fake_battery_soc;
	bool			chg_autonomous_mode;
	bool			disable_apsd;
	bool			using_pmic_therm;
	bool			battery_missing;
	const char		*bms_psy_name;
	bool			resume_completed;
	bool			irq_waiting;
	bool			bms_controlled_charging;
	struct mutex		read_write_lock;
	struct mutex		path_suspend_lock;
	struct mutex		irq_complete;
	u8			irq_cfg_mask[2];
        u8                      power_ok;
	int			irq_gpio;
	int			charging_disabled;
	int			fastchg_current_max_ma;
	//Correction
	int			psy_usb_ma;
	unsigned int		cool_bat_ma;
	unsigned int		warm_bat_ma;
	unsigned int		cool_bat_mv;
	unsigned int		warm_bat_mv;
	/* debugfs related */
#if defined(CONFIG_DEBUG_FS)
	struct dentry		*debug_root;
	u32			peek_poke_address;
#endif
	/* status tracking */
	bool			batt_full;
	bool			batt_hot;
	bool			batt_cold;
	bool			batt_warm;
	bool			batt_cool;
	bool			jeita_supported;
	int                    psy_health_sts;
	int			charging_disabled_status;
	int			usb_suspended;
	bool			chg_finish;	//add by pingyao.yang
	bool			recharge_change;
	u8			capacity;

	/* power supply */
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	*rk_bat;
	struct power_supply	batt_psy;

	/* otg 5V regulator */
	struct smb358_regulator	otg_vreg;

	/* adc_tm paramters */
	struct qpnp_vadc_chip	*vadc_dev;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;
	int			cold_bat_decidegc;
	int			hot_bat_decidegc;
	int			cool_bat_decidegc;
	int			warm_bat_decidegc;
	int			bat_present_decidegc;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	int			*thermal_mitigation;
	struct mutex			current_change_lock;
	/* i2c pull up regulator */
	struct regulator	*vcc_i2c;
	unsigned int fcc_mah;  // Other_platform modify 20151202 huangfusheng.wt add battery capacity discription

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add battery temperature protection*/
#if (defined QPNP_BATTERY_NTC_PROTECTION)
	int init_temperature;
	enum battery_health_type bat_health;
	int index;
	u8 charge_status;
	u8 charger_type;
	int ntc_temp;
	struct delayed_work monitor_workqueue;
	struct wake_lock	monitor_wake_lock;
#endif
/*[PLATFORM]-Add-END by pingao.yang */

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add charge ntc debug simulate */
#if	(defined QPNP_NTC_DEBUG_SIMULATE)
	struct dentry	*ntc_root;
	int debug_temperature;
#endif
/* [PLATFORM]-Mod-END by pingao.yang */
};

/* Fast charge current in uA */
static const unsigned int fcc_tbl[] = {
	100000,
	200000,
	450000,
	600000,
	900000,
	1300000,
	1500000,
	1800000,
};

/* Pre-charge current in uA */
static const unsigned int pcc_tbl[] = {
	100000,
	150000,
	200000,
	250000,
};

struct smb_irq_info {
	const char		*name;
	int			(*smb_irq)(struct smb358_charger *chip,
							u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};

static int chg_current[] = {
	300, 500, 700, 1000, 1200, 1500, 1800, 2000,
};

static int fast_chg_current[] = {
	200, 450, 600, 900, 1300, 1500, 1800, 2000,
};

/* add supplied to "bms" function */
static char *pm_batt_supplied_to[] = {
	"bms",
};

static int __smb358_read_reg(struct smb358_charger *chip, u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int __smb358_write_reg(struct smb358_charger *chip, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb358_read_reg(struct smb358_charger *chip, int reg,
						u8 *val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_read_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb358_write_reg(struct smb358_charger *chip, int reg,
						u8 val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_write_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb358_masked_write(struct smb358_charger *chip, int reg,
							u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_read_reg(chip, reg, &temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_read_reg Failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __smb358_write_reg(chip, reg, temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_write Failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static int smb358_enable_volatile_writes(struct smb358_charger *chip)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_VOLATILE_W_PERM_BIT,
						CMD_A_VOLATILE_W_PERM_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't write VOLATILE_W_PERM_BIT rc=%d\n",
				rc);

	return rc;
}

//for test wanggongzhen.wt test:no need
static int disable_software_temp_monitor = 0;
int dis_sof_temp_monitor_set(const char *val, const struct kernel_param *kp)
{
	if (!val) val = "1";
	return strtobool(val, kp->arg);
}

int dis_sof_temp_monitor_get(char *buffer, const struct kernel_param *kp)
{
	disable_software_temp_monitor = 1;
	return sprintf(buffer, "%c", *(bool *)kp->arg ? 'Y' : 'N');
}

static struct kernel_param_ops dis_sof_temp_monitor_ops = {
	.set = dis_sof_temp_monitor_set,
	.get = dis_sof_temp_monitor_get,
};

module_param_cb(disable_software_temp_monitor, &dis_sof_temp_monitor_ops
							, &disable_software_temp_monitor, 0644);

MODULE_PARM_DESC(debug, "1:disable software temp monitor , 0:enable,default:0");

static int smb358_fastchg_current_set(struct smb358_charger *chip,
					unsigned int fastchg_current)
{
	int i;

	if ((fastchg_current < SMB358_FAST_CHG_MIN_MA) ||
		(fastchg_current >  SMB358_FAST_CHG_MAX_MA)) {
		dev_dbg(chip->dev, "bad fastchg current mA=%d asked to set\n",
						fastchg_current);
		return -EINVAL;
	}

	for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
		if (fast_chg_current[i] <= fastchg_current)
			break;
	}

	if (i < 0) {
		dev_err(chip->dev, "Invalid current setting %dmA\n",
						fastchg_current);
		i = 0;
	}

	i = i << SMB358_FAST_CHG_SHIFT;
	dev_dbg(chip->dev, "fastchg limit=%d setting %02x\n",
					fastchg_current, i);

	return smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
				SMB_FAST_CHG_CURRENT_MASK, i);
}

#define MIN_FLOAT_MV		3500
#define MAX_FLOAT_MV		4500
#define VFLOAT_STEP_MV		20
#define VFLOAT_4350MV		4350
static int smb358_float_voltage_set(struct smb358_charger *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (VFLOAT_4350MV == vfloat_mv)
		temp = 0x2B;
	else if (vfloat_mv > VFLOAT_4350MV)
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV + 1;  // Other_platform_modify 20150928 huangfusheng.wt solve qualcom_bug from case 02171561
	else
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb358_masked_write(chip, VFLOAT_REG, VFLOAT_MASK, temp);
}

#define CHG_ITERM_30MA			0x00
#define CHG_ITERM_40MA			0x01
#define CHG_ITERM_60MA			0x02
#define CHG_ITERM_80MA			0x03
#define CHG_ITERM_100MA			0x04
#define CHG_ITERM_125MA			0x05
#define CHG_ITERM_150MA			0x06
#define CHG_ITERM_200MA			0x07
static int smb358_term_current_set(struct smb358_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled)
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");

		if (chip->iterm_ma <= 30)
			reg = CHG_ITERM_30MA;
		else if (chip->iterm_ma <= 40)
			reg = CHG_ITERM_40MA;
		else if (chip->iterm_ma <= 60)
			reg = CHG_ITERM_60MA;
		else if (chip->iterm_ma <= 80)
			reg = CHG_ITERM_80MA;
		else if (chip->iterm_ma <= 100)
			reg = CHG_ITERM_100MA;
		else if (chip->iterm_ma <= 125)
			reg = CHG_ITERM_125MA;
		else if (chip->iterm_ma <= 150)
			reg = CHG_ITERM_150MA;
		else
			reg = CHG_ITERM_200MA;

		rc = smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
							CHG_ITERM_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	if (chip->iterm_disabled) {
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK,
					CHG_CTRL_CURR_TERM_END_MASK);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n",
								rc);
			return rc;
		}
	} else {
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK, 0);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't enable iterm rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

#define VFLT_300MV			0x0C
#define VFLT_200MV			0x08
#define VFLT_100MV			0x04
#define VFLT_50MV			0x00
#define VFLT_MASK			0x0C
static int smb358_recharge_and_inhibit_set(struct smb358_charger *chip)
{
	u8 reg = 0;
	int rc;

	if (chip->recharge_disabled)
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
		CHG_CTRL_AUTO_RECHARGE_MASK, CHG_AUTO_RECHARGE_DIS_BIT);
	else
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
			CHG_CTRL_AUTO_RECHARGE_MASK, 0x0);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set auto recharge en reg rc = %d\n", rc);
	}

	if (chip->inhibit_disabled)
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, 0x0);
	else
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
					CHG_INHI_EN_MASK, CHG_INHI_EN_BIT);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set inhibit en reg rc = %d\n", rc);
	}

	if (chip->recharge_mv != -EINVAL) {
		if (chip->recharge_mv <= 50)
			reg = VFLT_50MV;
		else if (chip->recharge_mv <= 100)
			reg = VFLT_100MV;
		else if (chip->recharge_mv <= 200)
			reg = VFLT_200MV;
		else
			reg = VFLT_300MV;

		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						VFLT_MASK, reg);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set inhibit threshold rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

#if 0 //usb otg fuction no capacity limit,midifd by diganyun,2017/04/20
/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/09/01, pr-213929, add limit capacity of otg charging*/
#define OTG_CHARGING_LIMIT_CAPACITY	20
#endif

static int smb358_get_prop_batt_capacity(struct smb358_charger *chip);
/*[PLATFORM]-Add-END by pingao.yang*/

static int smb358_chg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

#if 0 //usb otg function no capacity limit,modify by diganyun,2017/04/20
/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/09/01, pr-213929, add limit capacity of otg charging*/
	if (cw2015_psy && (smb358_get_prop_batt_capacity(chip) < OTG_CHARGING_LIMIT_CAPACITY)) {
		printk("=== SMB358: Capacity Less than 20, can not use the OTG function !\n");
		return rc;
	}
/*[PLATFORM]-Add-END by pingao.yang, 2016/09/01*/
#endif

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/08/03, change to 1.5mhz switching frequency  */
	rc = smb358_masked_write(chip, THERM_A_CTRL_REG, CTRL_FREQ_SWITCH_BIT,
							CTRL_FREQ_SWITCH_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d, reg=%2x\n",
								rc, THERM_A_CTRL_REG);
	mdelay(1);
/* [PLATFORM]-Mod-END by pingao.yang */

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT,
							CMD_A_OTG_ENABLE_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d, reg=%2x\n",
								rc, CMD_A_REG);

	//+bug_130601 modify 20160118 huangfusheng.wt for 2phone charge through otg cable
	rc = smb358_masked_write(chip, OTG_CONTROL_REG, OTG_CURRENT_CONTROL_BIT2,
							OTG_CURRENT_CONTROL_BIT2);
	if (rc)
		dev_err(chip->dev, "Couldn't enable OTG current control rc=%d, reg=%2x\n",
								rc, OTG_CONTROL_REG);

	//-bug_130601 modify 20160118 huangfusheng.wt for 2phone charge through otg cable
	return rc;
}

static int smb358_chg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
	if (rc)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d, reg=%2x\n",
								rc, CMD_A_REG);
/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/08/03, change to 3mhz switching frequency  */
	rc = smb358_masked_write(chip, THERM_A_CTRL_REG, CTRL_FREQ_SWITCH_BIT, 0);
	if (rc)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d, reg=%2x\n",
							rc, THERM_A_CTRL_REG);
/* [PLATFORM]-Mod-END by pingao.yang */

	return rc;
}

static int smb358_chg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_read_reg(chip, CMD_A_REG, &reg);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't read OTG enable bit rc=%d, reg=%2x\n",
							rc, CMD_A_REG);
		return rc;
	}

	return  (reg & CMD_A_OTG_ENABLE_BIT) ? 1 : 0;
}

struct regulator_ops smb358_chg_otg_reg_ops = {
	.enable		= smb358_chg_otg_regulator_enable,
	.disable	= smb358_chg_otg_regulator_disable,
	.is_enabled	= smb358_chg_otg_regulator_is_enable,
};

static int smb358_regulator_init(struct smb358_charger *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Allocate memory failed\n");
		return -ENOMEM;
	}

	/* Give the name, then will register */
	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb358_chg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}
	return rc;
}

static int __smb358_charging_disable(struct smb358_charger *chip, bool disable)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_BIT,
			disable ? 0 : CMD_A_CHG_ENABLE_BIT);
	if (rc < 0)
		pr_err("Couldn't set CHG_ENABLE_BIT diable = %d, rc = %d\n",
				disable, rc);
	return rc;
}

static int smb358_charging_disable(struct smb358_charger *chip,
						int reason, int disable)
{
	int rc = 0;
	int disabled;

	if(disable==true) //Other_platform modify 20151113 huangfusheng.wt solve current jump
	{
		pre_usb_current_ma = -EINVAL;
	}
	disabled = chip->charging_disabled_status;

	pr_err("reason = %d requested_disable = %d disabled_status = %d\n",
						reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (!!disabled == !!chip->charging_disabled_status)
		goto skip;

	rc = __smb358_charging_disable(chip, !!disabled);
	if (rc) {
		pr_err("Failed to disable charging rc = %d\n", rc);
		return rc;
	} else {
	/* will not modify online status in this condition */
		power_supply_changed(&chip->batt_psy);
	}

skip:
	chip->charging_disabled_status = disabled;
	return rc;
}

static int smb358_hw_init(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0, mask = 0;

	/*
	 * If the charger is pre-configured for autonomous operation,
	 * do not apply additonal settings
	 */
	if (chip->chg_autonomous_mode) {
		dev_dbg(chip->dev, "Charger configured for autonomous mode\n");
		return 0;
	}

	rc = smb358_enable_volatile_writes(chip);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure volatile writes rc=%d\n",
				rc);
		return rc;
	}

	/* setup defaults for CHG_CNTRL_REG */
	reg = CHG_CTRL_BATT_MISSING_DET_THERM_IO;
	mask = CHG_CTRL_BATT_MISSING_DET_MASK;
	rc = smb358_masked_write(chip, CHG_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n", rc);
		return rc;
	}
	/* setup defaults for PIN_CTRL_REG */
	reg = CHG_PIN_CTRL_USBCS_REG_BIT | CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT |
		CHG_PIN_CTRL_APSD_IRQ_BIT | CHG_PIN_CTRL_CHG_ERR_IRQ_BIT;
	mask = CHG_PIN_CTRL_CHG_EN_MASK | CHG_PIN_CTRL_USBCS_REG_MASK |
		CHG_PIN_CTRL_APSD_IRQ_MASK | CHG_PIN_CTRL_CHG_ERR_IRQ_MASK;
	rc = smb358_masked_write(chip, CHG_PIN_EN_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	/* setup USB suspend and APSD  */
	rc = smb358_masked_write(chip, VARIOUS_FUNC_REG,
		VARIOUS_FUNC_USB_SUSP_MASK, VARIOUS_FUNC_USB_SUSP_EN_REG_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set VARIOUS_FUNC_REG rc=%d\n",
				rc);
		return rc;
	}

	if (!chip->disable_apsd)
		reg = CHG_CTRL_APSD_EN_BIT;
	else
		reg = 0;

	rc = smb358_masked_write(chip, CHG_CTRL_REG,
				CHG_CTRL_APSD_EN_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}
	/* Fault and Status IRQ configuration */
	reg = FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_HOT_COLD_SOFT_BIT
		| FAULT_INT_INPUT_UV_BIT | FAULT_INT_AICL_COMPLETE_BIT
		| FAULT_INT_INPUT_OV_BIT;
	rc = smb358_write_reg(chip, FAULT_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set FAULT_INT_REG rc=%d\n", rc);
		return rc;
	}
	reg = STATUS_INT_CHG_TIMEOUT_BIT | STATUS_INT_OTG_DETECT_BIT |
		STATUS_INT_BATT_OV_BIT | STATUS_INT_CHGING_BIT |
		STATUS_INT_CHG_INHI_BIT | STATUS_INT_INOK_BIT |
		STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT;
	rc = smb358_write_reg(chip, STATUS_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set STATUS_INT_REG rc=%d\n", rc);
		return rc;
	}

	/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/09/28, add smb358 hw jeita*/
	/* setup THERM Monitor */
	reg = CHARGE_CURRENT_COMPENSATION_BIT | DIGITAL_THERMAL_TEMP_THRESHOLD_BIT;
	mask = CHARGE_CURRENT_COMPENSATION_MASK | DIGITAL_THERMAL_TEMP_THRESHOLD_MASK;
	rc = smb358_masked_write(chip, OTG_CONTROL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set OTG_CONTROL_REG rc=%d\n", rc);
		return rc;
	}

	reg = HARD_LIMIT_COLD_TEMP_POINT_BIT | HARD_LIMIT_HOT_TEMP_POINT_BIT |
		SOFT_LIMIT_COLD_TEMP_POINT_BIT | SOFT_LIMIT_HOT_TEMP_POINT_BIT;
	mask = HARD_LIMIT_COLD_TEMP_POINT_MASK | HARD_LIMIT_HOT_TEMP_POINT_MASK |
		SOFT_LIMIT_COLD_TEMP_POINT_MASK | SOFT_LIMIT_HOT_TEMP_POINT_MASK;
	rc = smb358_masked_write(chip, HARD_SOFT_LIMIT_TEMP_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set HARD_SOFT_LIMIT_TEMP_REG rc=%d\n", rc);
		return rc;
	}

	reg = FLOAT_VOLTAGE_COMPENSATION_BIT | HARD_TEMPERATURE_LIMIT_BIT;
	mask = FLOAT_VOLTAGE_COMPENSATION_MASK | HARD_TEMPERATURE_LIMIT_MASK;
	rc = smb358_masked_write(chip, SYSOK_AND_USB3_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set SYSOK_AND_USB3_REG rc=%d\n",
				rc);
		return rc;
	}

	reg = THERM_MONITOR_SELECTION_BIT | THERM_MONITOR_ENABLE_BIT |
		SOFT_COLD_TEMP_LIMIT_BEHAVIOR_BIT | SOFT_HOT_TEMP_LIMIT_BEHAVIOR_BIT;
	mask = THERM_MONITOR_SELECTION_MASK | THERM_MONITOR_ENABLE_MASK |
		SOFT_COLD_TEMP_LIMIT_BEHAVIOR_MASK | SOFT_HOT_TEMP_LIMIT_BEHAVIOR_MASK;
	rc = smb358_masked_write(chip, THERM_A_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}
	/*[PLATFORM]-Add-END by pingao.yang*/

	/* set the fast charge current limit */
	rc = smb358_fastchg_current_set(chip, chip->fastchg_current_max_ma);
	if (rc) {
		dev_err(chip->dev, "Couldn't set fastchg current rc=%d\n", rc);
		return rc;
	}

	/* set the float voltage */
	rc = smb358_float_voltage_set(chip, chip->vfloat_mv);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't set float voltage rc = %d\n", rc);
		return rc;
	}

	/* set iterm */
	rc = smb358_term_current_set(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set term current rc=%d\n", rc);

	/* set recharge */
	rc = smb358_recharge_and_inhibit_set(chip);
	if (rc)
		dev_err(chip->dev, "Couldn't set recharge para rc=%d\n", rc);

	/* enable/disable charging */
	if (chip->charging_disabled) {
		rc = smb358_charging_disable(chip, USER, 1);
		if (rc)
			dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);
	} else {
		/*
		 * Enable charging explictly,
		 * because not sure the default behavior.
		 */
		rc = __smb358_charging_disable(chip, 0);
		if (rc)
			dev_err(chip->dev, "Couldn't enable charging\n");
	}

	/*
	* Workaround for recharge frequent issue: When battery is
	* greater than 4.2v, and charging is disabled, charger
	* stops switching. In such a case, system load is provided
	* by battery rather than input, even though input is still
	* there. Make reg09[0:3] to be a non-zero value which can
	* keep the switcher active
	*/
	rc = smb358_masked_write(chip, OTHER_CTRL_REG, CHG_LOW_BATT_THRESHOLD,
						SMB358_BATT_GOOD_THRE_2P5);
	if (rc)
		dev_err(chip->dev, "Couldn't write OTHER_CTRL_REG, rc = %d\n",
								rc);

	return rc;
}

static enum power_supply_property smb358_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_CURRENT_NOW,
        POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, // Other_platform modify 20151202 huangfusheng.wt add battery capacity discription
	POWER_SUPPLY_PROP_CHARGE_FULL,// Other_platform modify 20151209 huangfusheng.wt add battery capacity discription
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_BATTERY_TYPE	/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/09/01, pr-212436, add battery_type property */
};

extern int battery_master_get_capacity_limit_flag(void);
static int smb358_get_prop_batt_status(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/09/20, add charge full status hold mode*/
	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	dev_dbg(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	chip->capacity = smb358_get_prop_batt_capacity(chip);
	dev_dbg(chip->dev, "batt_full = %d, chip->chg_finish = %d, capacity = %d, power_ok = %d\n",
				chip->batt_full, chip->chg_finish, chip->capacity, chip->power_ok);

	if (!chip->power_ok && chip->chg_finish) {
		chip->chg_finish = false;
	}

	if (chip->power_ok && chip->chg_finish
		 && chip->capacity <= RECHARGE_CAPACITY) {
		chip->batt_full = false;
		chip->chg_finish = false;
		chip->recharge_change = true;
		__smb358_charging_disable(chip, false);
		dev_err(chip->dev, "%s: [allan]: enable recharge \n", __func__);
		return POWER_SUPPLY_STATUS_CHARGING;
	}

	if (chip->power_ok && (chip->batt_full || chip->chg_finish
		||(chip->capacity == CHARGE_FULL_CAPACITY
			&& (reg & STATUS_C_CHG_HOLD_OFF_BIT) && chip->power_ok)
		||(battery_master_get_capacity_limit_flag() == true
			&& chip->capacity == CHARGE_FULL_CAPACITY))) {
		if (chip->chg_finish != true) {
			__smb358_charging_disable(chip, true);
			dev_err(chip->dev, "%s: [allan]:disable charging \n", __func__);
		}
		chip->chg_finish = true;
		return POWER_SUPPLY_STATUS_FULL;
	}
/*[PLATFORM]-Add-END by pingao.yang */

	if ((reg & STATUS_C_CHARGING_MASK) &&
			!(reg & STATUS_C_CHG_ERR_STATUS_BIT) && chip->power_ok)
		return POWER_SUPPLY_STATUS_CHARGING;

	if (chip->power_ok) {
		return POWER_SUPPLY_STATUS_CHARGING;
	}

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int smb358_get_prop_batt_present(struct smb358_charger *chip)
{
	return !chip->battery_missing;
}

static int smb358_get_prop_batt_capacity(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if(!cw2015_psy)   //+Other_platform_modify 20151123 huangfusheng.wt add cw2015 
		cw2015_psy = power_supply_get_by_name("rk-bat");
	if(cw2015_psy){
		cw2015_psy->get_property(cw2015_psy, 
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		pr_debug("CW2015_BATTERY_CAPACITY IS:%d\n",ret.intval);
		return ret.intval;
	}
	//-Other_platform_modify 20151123 huangfusheng.wt add cw2015 
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
                pr_debug("BMS_BATTERY_CAPACITY IS:%d\n",ret.intval);
		return ret.intval;
               
	}

	pr_debug("Couldn't get bms_psy, return default capacity\n");
	return SMB358_DEFAULT_BATT_CAPACITY;
}

static int get_prop_current_now(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0,};
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
			pr_debug("xujismbcur = %d\n",ret.intval);
			return ret.intval;
		} else {
			pr_debug("No BMS supply registered return 0\n");	
		}
	return 0;
}

static int smb358_get_prop_charge_type(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	dev_dbg(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);

	reg &= STATUS_C_CHARGING_MASK;

	if (reg == STATUS_C_FAST_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (reg == STATUS_C_TAPER_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (reg == STATUS_C_PRE_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int smb358_get_prop_batt_health(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0, };
    if(!disable_software_temp_monitor){
	    if (chip->batt_hot)
		    ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	    else if (chip->batt_cold)
		    ret.intval = POWER_SUPPLY_HEALTH_COLD;
	    else if (chip->batt_warm)
		    ret.intval = POWER_SUPPLY_HEALTH_WARM;
	    else if (chip->batt_cool)
		    ret.intval = POWER_SUPPLY_HEALTH_COOL;
	    else if (chip->psy_health_sts == POWER_SUPPLY_HEALTH_OVERVOLTAGE)
		    ret.intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	    else
		    ret.intval = POWER_SUPPLY_HEALTH_GOOD;
    }   
    else
     ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret.intval;
}
//Other_platform_modify 20160520 xuji.wt add C3 charging config
#define DEFAULT_TEMP 250
static int smb358_get_prop_batt_temp(struct smb358_charger *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

#if (defined QPNP_NTC_DEBUG_SIMULATE)
	return chip->debug_temperature;
#endif

	if (!smb358_get_prop_batt_present(chip))
		return DEFAULT_TEMP;

	rc = qpnp_vadc_read(chip->vadc_dev, P_MUX2_1_1, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("get_bat_temp %d, %lld\n",
		results.adc_code, results.physical);

	return results.physical;
}
//Other_platform_modify 20160520 xuji.wt add C3 charging config

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/21,  bug-200541, adjust battery voltage when insert dcp */
#define LOW_FLOAT_VOLTAGE	150000
#define MID_FLOAT_VOLTAGE	130000
#define HIGH_FLOAT_VOLTAGE	110000
#define LOW_FLOAT_VOLTAGE_THREHOLD	3900000
#define MID_FLOAT_VOLTAGE_THREHOLD	4100000
#define HIGH_FLOAT_VOLTAGE_THREHOLD	4350000

static int smb358_adjust_battery_voltage(struct smb358_charger *chip, int voltage )
{
	int rc, adjust_vol;
	u8 reg = 0;

	adjust_vol = voltage;
	rc = smb358_read_reg(chip, STATUS_D_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STATUS_D_REG rc = %d\n", rc);
	}

	dev_dbg(chip->dev, "%s: STATUS_D_REG = %x\n", __func__, reg);

	reg &= STATUS_D_CHARGING_PORT_MASK;
	if (reg == STATUS_D_PORT_DCP) {
		dev_dbg(chip->dev, "STATUS_D_REG charge_port = %d\n", reg);
		if ( voltage <= LOW_FLOAT_VOLTAGE_THREHOLD) {
			adjust_vol = adjust_vol - LOW_FLOAT_VOLTAGE;
		}
		else if(LOW_FLOAT_VOLTAGE_THREHOLD < voltage
				&& voltage <= MID_FLOAT_VOLTAGE_THREHOLD) {
			adjust_vol = adjust_vol - MID_FLOAT_VOLTAGE;
		}
		else if (MID_FLOAT_VOLTAGE_THREHOLD < voltage
				&& voltage <= HIGH_FLOAT_VOLTAGE_THREHOLD) {
			adjust_vol = adjust_vol - HIGH_FLOAT_VOLTAGE;
		}
	}

	return adjust_vol;
}
/* [PLATFORM]-Mod-END by pingao.yang */

#define SMB358_DEFAULT_BATT_VOLTAGE 4000

static int
smb358_get_prop_battery_voltage_now(struct smb358_charger *chip)
{

	union power_supply_propval ret = {0, };

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if(!cw2015_psy)   //+Other_platform_modify 20151123 huangfusheng.wt add cw2015 
		cw2015_psy = power_supply_get_by_name("rk-bat");
	if(cw2015_psy){
		cw2015_psy->get_property(cw2015_psy, 
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
		pr_debug("POWER_SUPPLY_PROP_VOLTAGE_NOW IS:%d\n",ret.intval);

	/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/21,  bug-200541, adjust battery voltage when insert dcp */
		//return ret.intval;
		return smb358_adjust_battery_voltage(chip, ret.intval);
	/* [PLATFORM]-Mod-END by pingao.yang */
	}
	pr_debug("Couldn't get bms_psy, return default capacity\n");
	return SMB358_DEFAULT_BATT_VOLTAGE;
}

static int __smb358_path_suspend(struct smb358_charger *chip, bool suspend)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG,
			CMD_A_CHG_SUSP_EN_MASK,
				suspend ? CMD_A_CHG_SUSP_EN_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set CMD_A reg, rc = %d\n", rc);
	return rc;
}

static int smb358_path_suspend(struct smb358_charger *chip, int reason,
							bool suspend)
{
	int rc = 0;
	int suspended;

	mutex_lock(&chip->path_suspend_lock);
	suspended = chip->usb_suspended;

	if (suspend == false)
		suspended &= ~reason;
	else
		suspended |= reason;

	if (!chip->usb_suspended && suspended) {
		rc = __smb358_path_suspend(chip, true);
		chip->usb_suspended = suspended;
		power_supply_set_online(chip->usb_psy, !chip->usb_suspended);
		power_supply_changed(chip->usb_psy);
	} else if (chip->usb_suspended && !suspended) {
		rc = __smb358_path_suspend(chip, false);
		chip->usb_suspended = suspended;
		power_supply_set_online(chip->usb_psy, !chip->usb_suspended);
		power_supply_changed(chip->usb_psy);
	}

	if (rc)
		dev_err(chip->dev, "Couldn't set/unset suspend rc = %d\n", rc);

	mutex_unlock(&chip->path_suspend_lock);
	return rc;
}

static int smb358_set_usb_chg_current(struct smb358_charger *chip,
		int curr_ma)
{
	int i, rc = 0;
	u8 reg1 = 0, reg2 = 0, mask = 0;
	int current_ma;

	dev_dbg(chip->dev, "%s: USB current_ma = %d\n", __func__, curr_ma);

	if (chip->chg_autonomous_mode) {
		dev_dbg(chip->dev, "%s: Charger in autonmous mode\n", __func__);
		return 0;
	}	

	if(recovery!=true&&curr_ma == pre_usb_current_ma) //Other_platform modify 20151113 huangfusheng.wt solve current jump
	{
		return 0;
	}
        recovery=false;
	//Correction //Other_platform modify 20151113 huangfusheng.wt solve current jump
        if (thermal==false)
        {
	current_ma = curr_ma; //min(curr_ma, chip->thermal_mitigation[chip->therm_lvl_sel]);
        pre_usb_current_ma = curr_ma;
        pr_debug("current ma1 is %d\n",current_ma);
        }
        else
        {
        current_ma = min(curr_ma, chip->thermal_mitigation[chip->therm_lvl_sel]);
        pr_debug("current ma2 is %d\n",current_ma);
        }
	if (current_ma < USB3_MIN_CURRENT_MA && current_ma != 2)
		current_ma = USB2_MIN_CURRENT_MA;

	if (current_ma == USB2_MIN_CURRENT_MA) {
		/* USB 2.0 - 100mA */
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 &= ~CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB2_MAX_CURRENT_MA) {
		/* USB 2.0 - 500mA */
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB3_MAX_CURRENT_MA) {
		/* USB 3.0 - 900mA */
		reg1 |= USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma > USB2_MAX_CURRENT_MA) {
		/* HC mode  - if none of the above */
		reg2 |= CMD_B_CHG_HC_ENABLE_BIT;

		for (i = ARRAY_SIZE(chg_current) - 1; i >= 0; i--) {
			if (chg_current[i] <= current_ma)
				break;
		}
		if (i < 0) {
			dev_err(chip->dev, "Cannot find %dmA\n", current_ma);
			i = 0;
		}

		i = i << AC_CHG_CURRENT_SHIFT;
		rc = smb358_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
						AC_CHG_CURRENT_MASK, i);
		if (rc)
			dev_err(chip->dev, "Couldn't set input mA rc=%d\n", rc);
	}
        mask=0;
        rc = __smb358_write_reg(chip, CMD_B_REG, mask);
	if (rc) {
		dev_err(chip->dev,
			"smb358_write Failed: reg=%03X, rc=%d\n", CMD_B_REG, rc);
	}
	mask = CMD_B_CHG_HC_ENABLE_BIT | CMD_B_CHG_USB_500_900_ENABLE_BIT;
	rc = smb358_masked_write(chip, CMD_B_REG, mask, reg2);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set charging mode rc = %d\n", rc);

	mask = USB3_ENABLE_MASK;
	rc = smb358_masked_write(chip, SYSOK_AND_USB3_REG, mask, reg1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set USB3 mode rc = %d\n", rc);

	/* Only set suspend bit when chg present and current_ma = 2 */
	if (current_ma == 2 && chip->chg_present) {
		rc = smb358_path_suspend(chip, CURRENT, true);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't suspend rc = %d\n", rc);
	} else {
		rc = smb358_path_suspend(chip, CURRENT, false);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set susp rc = %d\n", rc);
	}

	return rc;
}

static int
smb358_batt_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		return 1;
	default:
		break;
	}

	return 0;
}

static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(soc, 100);
	return soc;
}
 //+Other_project_modify 20151112 huangfusheng.wt abondon thermal control chg current
static int smb358_set_appropriate_current(struct smb358_charger *chip,
						enum path_type path)
{
	int therm_ma;
	//int path_current = (path == USB) ? chip->usb_psy_ma : chip->dc_psy_ma;
	//int (*func)(struct smb358_charger *chip, int current_ma);
	int rc = 0;

	if (!chip->usb_psy && path == USB)
		return 0;
	/*
	 * If battery is absent do not modify the current at all, these
	 * would be some appropriate values set by the bootloader or default
	 * configuration and since it is the only source of power we should
	 * not change it
	 */
	if (chip->battery_missing) {
		pr_debug("ignoring current request since battery is absent\n");
		return 0;
	}

	#if 0
	if (path == USB) {
		path_current = chip->usb_psy_ma;
		func = smb135x_set_usb_chg_current;
	} else {
		path_current = chip->dc_psy_ma;
		func = smb135x_set_dc_chg_current;
		if (chip->dc_psy_type == -EINVAL)
			func = NULL;
	}
	#endif

	if (chip->therm_lvl_sel >= 0
			&& chip->therm_lvl_sel <= (chip->thermal_levels - 1))
	{	/*
		 * consider thermal limit only when it is active and not at
		 * the highest level
		 */
		therm_ma = chip->thermal_mitigation[chip->therm_lvl_sel];
                if (chip->therm_lvl_sel == 0)
                {
                thermal=false;
                recovery=true;
                }
                else
                {
                thermal=true;
                }
	}else
                {
		//correction
		pr_debug("Not effective thermal levels\n");
                pr_debug("therm_ma is %d\n",therm_ma);
                }

		//Correction
	therm_ma = min(therm_ma, chip->psy_usb_ma);
        pr_debug("therm_ma is %d\n",therm_ma);
        pr_debug("chip->psy_usb_ma is %d\n",chip->psy_usb_ma);
	pr_err("thermal limited charging current to %d\n", therm_ma);
	#if 0
	if (func != NULL)
		rc = func(chip, current_ma);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set %s current to min(%d, %d)rc = %d\n",
				path == USB ? "usb" : "dc",
				therm_ma, path_current,
				rc);
	#else
	//Correction
	smb358_set_usb_chg_current(chip,  therm_ma);
	#endif
	return rc;
}


static int smb358_system_temp_level_set(struct smb358_charger *chip,
								int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;

	if (!chip->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->current_change_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	pr_err("chip->therm_lvl_sel = %d\n", chip->therm_lvl_sel);
//Correction
	#if 0  //Other_platform_modify 20151123 huangfusheng.wt not disable chg wher thermal in highest level
	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		/*
		 * Disable charging if highest value selected by
		 * setting the DC and USB path in suspend
		 */
		rc = smb358_path_suspend(chip, /*DC, */THERMAL, true);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = smb358_path_suspend(chip, /*USB,*/ THERMAL, true);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
		goto out;
	}
	#endif

	smb358_set_appropriate_current(chip, USB);

	#if 0 //Other_platform_modify 20151123 huangfusheng.wt not disable chg wher thermal in highest level
	if (prev_therm_lvl == chip->thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Enable charging by taking the DC and USB path
		 * out of suspend.
		 */
		rc = smb358_path_suspend(chip,/* DC,*/ THERMAL, false);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = smb358_path_suspend(chip, /*USB,*/ THERMAL, false);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
	}
out:
	#endif
	mutex_unlock(&chip->current_change_lock);
	return rc;
}

 //-Other_project_modify 20151112 huangfusheng.wt abondon thermal control chg current

 /* Convert register value to current using lookup table */
 static int hw_to_current(const unsigned int *tbl, size_t size, unsigned int val)
 {
	 if (val >= size)
		 return -EINVAL;
	 return tbl[val];
 }

 /*
  * Returns the constant charge current programmed
  * into the charger in uA.
  */
 static int smb358_get_const_charge_current(struct smb358_charger *chip)
{
	int ret, intval;
	u8 reg = 0;

        ret = smb358_read_reg(chip, STATUS_C_REG, &reg);
        if (ret) {
                dev_err(chip->dev, "Couldn't read STAT_C ret = %d\n", ret);
                return POWER_SUPPLY_STATUS_UNKNOWN;
        }
        dev_dbg(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);
        if (! (reg & 0x01))
		return -ENODATA;

	ret = smb358_read_reg(chip, STATUS_B_REG, &reg);
	if (ret) {
		dev_err(chip->dev, "Couldn't read STATUS_B_REG ret = %d\n", ret);
		return ret;
	}
	dev_dbg(chip->dev, "%s: STATUS_B_REG=%x\n", __func__, reg);

	/*
	 * The current value is composition of FCC and PCC values
	 * and we can detect which table to use from bit 5.
	 */
	if (reg & 0x20) {
		intval = hw_to_current(fcc_tbl, ARRAY_SIZE(fcc_tbl), reg & 7);
	} else {
		reg >>= 3;
		intval = hw_to_current(pcc_tbl, ARRAY_SIZE(pcc_tbl), reg & 7);
	}

	return intval;
}

 /*
  * Returns the constant charge voltage programmed
  * into the charger in uV.
  */
 static int smb358_get_const_charge_voltage(struct smb358_charger *chip)
{
	int ret, intval;
	u8 reg = 0;

        ret = smb358_read_reg(chip, STATUS_C_REG, &reg);
        if (ret) {
                dev_err(chip->dev, "Couldn't read STAT_C ret = %d\n", ret);
                return POWER_SUPPLY_STATUS_UNKNOWN;
        }
        dev_dbg(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);
        if (! (reg & 0x01))
		return -ENODATA;

	ret = smb358_read_reg(chip, STATUS_A_REG, &reg);
	if (ret) {
		dev_err(chip->dev, "Couldn't read STATUS_A_REG ret = %d\n", ret);
		return ret;
	}
	dev_dbg(chip->dev, "%s: STATUS_A_REG=%x\n", __func__, reg);

	reg &= STATUS_A_FLOAT_VOLTAGE_MASK;
	if (reg > 0x3d)
		reg = 0x3d;

	intval = 3500000 + reg * 20000;

	return intval;
}

static int smb358_battery_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	int rc;
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!chip->bms_controlled_charging)
			return -EINVAL;
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_FULL:
			rc = smb358_charging_disable(chip, SOC, true);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't set charging disable rc = %d\n",
					rc);
			} else {
				chip->batt_full = true;
				dev_dbg(chip->dev, "status = FULL, batt_full = %d\n",
							chip->batt_full);
			}
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			chip->batt_full = false;
			power_supply_changed(&chip->batt_psy);
			dev_dbg(chip->dev, "status = DISCHARGING, batt_full = %d\n",
							chip->batt_full);
			break;
		case POWER_SUPPLY_STATUS_CHARGING:
			rc = smb358_charging_disable(chip, SOC, false);
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't set charging disable rc = %d\n",
								rc);
			} else {
				chip->batt_full = false;
				dev_dbg(chip->dev, "status = CHARGING, batt_full = %d\n",
							chip->batt_full);
			}
			break;
		default:
			return -EINVAL;
		}

		SMB358_PRINT("%s: chip->batt_full = %d\n", __func__, chip->batt_full);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		dev_dbg(chip->dev, "POWER_SUPPLY_PROP_CHARGING_ENABLED, batt_full = %d\n",
							chip->batt_full);
		smb358_charging_disable(chip, USER, !val->intval);
		/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/09/27, modify charge into suspend*/
		//smb358_path_suspend(chip, USER, !val->intval);
		rc = __smb358_path_suspend(chip, !val->intval);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set path suspend rc = %d\n", rc);
		}
		break;
		/*[PLATFORM]-Add-END by pingao.yang*/
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = bound_soc(val->intval);
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		smb358_system_temp_level_set(chip, val->intval);//+Other_project_modify 20151112 huangfusheng.wt abondon thermal control chg current
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb358_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb358_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb358_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb358_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !(chip->charging_disabled_status & USER);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb358_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb358_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SMB358";
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb358_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb358_get_prop_battery_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN: // Other_platform modify 20151202 huangfusheng.wt add battery capacity discription
		val->intval = (chip->fcc_mah * 1000);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL: // Other_platform modify 20151209 huangfusheng.wt add battery capacity discription
		val->intval = (chip->fcc_mah * 1000);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = smb358_get_const_charge_voltage(chip);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = smb358_get_const_charge_current(chip);
		break;
	/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/09/01, pr-212436, add battery_type property */
	case POWER_SUPPLY_PROP_BATTERY_TYPE:
		val->strval = "C11P1609-O-01-0006-1.1.1";
		break;
	/*[PLATFORM]-Add-END by pingao.yang*/
	default:
		return -EINVAL;
	}
	return 0;
}

static int apsd_complete(struct smb358_charger *chip, u8 status)
{
	int rc;
	u8 reg = 0;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;

	/*
	 * If apsd is disabled, charger detection is done by
	 * DCIN UV irq.
	 * status = ZERO - indicates charger removed, handled
	 * by DCIN UV irq
	 */
	if (chip->disable_apsd || status == 0) {
		dev_dbg(chip->dev, "APSD %s, status = %d\n",
			chip->disable_apsd ? "disabled" : "enabled", !!status);
		return 0;
	}

	rc = smb358_read_reg(chip, STATUS_D_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STATUS D rc = %d\n", rc);
		return rc;
	}

	dev_dbg(chip->dev, "%s: STATUS_D_REG=%x\n", __func__, reg);

	switch (reg & STATUS_D_CHARGING_PORT_MASK) {
	case STATUS_D_PORT_ACA_DOCK:
	case STATUS_D_PORT_ACA_C:
	case STATUS_D_PORT_ACA_B:
	case STATUS_D_PORT_ACA_A:
		type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case STATUS_D_PORT_CDP:
		type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case STATUS_D_PORT_DCP:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case STATUS_D_PORT_SDP:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	case STATUS_D_PORT_OTHER:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	}

	chip->chg_present = !!status;

	dev_dbg(chip->dev, "APSD complete. USB type detected=%d chg_present=%d",
						type, chip->chg_present);

	power_supply_set_supply_type(chip->usb_psy, type);

	 /* SMB is now done sampling the D+/D- lines, indicate USB driver */
	dev_dbg(chip->dev, "%s updating usb_psy present=%d", __func__,
			chip->chg_present);
	power_supply_set_present(chip->usb_psy, chip->chg_present);

	return 0;
}

static int chg_uv(struct smb358_charger *chip, u8 status)
{
	int rc;
	/* use this to detect USB insertion only if !apsd */
	if (chip->disable_apsd && status == 0) {
		chip->chg_present = true;
		pre_usb_current_ma = -EINVAL; //Other_platform modify 20151113 huangfusheng.wt solve current jump
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
				
		//power_supply_set_supply_type(chip->usb_psy,
						//POWER_SUPPLY_TYPE_USB);                 //Other_platform_modify 20151026 huangfusheng.wt del this for charger correct detect
		power_supply_set_present(chip->usb_psy, chip->chg_present);

		if (chip->bms_controlled_charging) {
			/*
			* Disable SOC based USB suspend to enable charging on
			* USB insertion.
			*/
			rc = smb358_charging_disable(chip, SOC, false);
			if (rc < 0)
				dev_err(chip->dev,
				"Couldn't disable usb suspend rc = %d\n",
								rc);
		}
	}

	__smb358_charging_disable(chip, false);

	if (status != 0) {
		chip->chg_present = false;
	    pre_usb_current_ma = -EINVAL; //Other_platform modify 20151113 huangfusheng.wt solve current jump
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
	/* we can't set usb_psy as UNKNOWN here, will lead USERSPACE issue */
		power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

	power_supply_changed(chip->usb_psy);
	dev_dbg(chip->dev, "chip->chg_present = %d\n", chip->chg_present);

	return 0;
}

static int chg_ov(struct smb358_charger *chip, u8 status)
{
	//u8 psy_health_sts;
       //+Other_add_OVP,20151030 xuji.wt add for 88509 batt OVP
	if (status)
		chip->psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	else
		chip->psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;

	power_supply_set_health_state(
				chip->usb_psy, chip->psy_health_sts);
	power_supply_changed(chip->usb_psy);

	return 0;
}

#define STATUS_FAST_CHARGING BIT(6)
static int fast_chg(struct smb358_charger *chip, u8 status)
{
	dev_dbg(chip->dev, "%s\n", __func__);

	if (status & STATUS_FAST_CHARGING) {
		chip->batt_full = false;
	}
	return 0;
}

static int chg_term(struct smb358_charger *chip, u8 status)
{
	dev_dbg(chip->dev, "%s\n", __func__);
	if (!chip->iterm_disabled)
		chip->batt_full = !!status;

	chip->recharge_change = true;	// add by pingao.yang
	return 0;
}

static int taper_chg(struct smb358_charger *chip, u8 status)
{
	dev_dbg(chip->dev, "%s\n", __func__);
	return 0;
}

static int chg_recharge(struct smb358_charger *chip, u8 status)
{
	dev_dbg(chip->dev, "%s, status = %d\n", __func__, !!status);
	/* to check the status mean */
	chip->batt_full = !status;
	chip->recharge_change = true;	// add by pingao.yang

	return 0;
}

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/09/23, add battery temperature protection*/
#define  RECHARGE_CALIBRATION_VOLTAGE	30
static void smb358_set_recharge_voltage(
				struct smb358_charger *chip)
{
	int rc;

	if (chip->recharge_change && chip->batt_full) {
		chip->recharge_change = false;
		rc = smb358_float_voltage_set(chip, chip->vfloat_mv
				- RECHARGE_CALIBRATION_VOLTAGE);
		if (rc)
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
	}
	else if (chip->recharge_change && !chip->batt_full) {
		chip->recharge_change = false;
		rc = smb358_float_voltage_set(chip, chip->vfloat_mv);
		if (rc)
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
	}
}
/*[PLATFORM]-Add-END by pingao.yang*/

static void smb358_chg_set_appropriate_battery_current(
				struct smb358_charger *chip)
{
	int rc;
	unsigned int current_max = chip->fastchg_current_max_ma;

	if (chip->batt_cool)
		current_max =
			min(current_max, chip->cool_bat_ma);
	if (chip->batt_warm)
		current_max =
			min(current_max, chip->warm_bat_ma);
	dev_dbg(chip->dev, "setting %dmA", current_max);
	rc = smb358_fastchg_current_set(chip, current_max);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set charging current rc = %d\n", rc);
}

static void smb358_chg_set_appropriate_vddmax(
				struct smb358_charger *chip)
{
	int rc;
	unsigned int vddmax = chip->vfloat_mv;

	if (chip->batt_cool)
		vddmax = min(vddmax, chip->cool_bat_mv);
	if (chip->batt_warm)
		vddmax = min(vddmax, chip->warm_bat_mv);

	dev_dbg(chip->dev, "setting %dmV\n", vddmax);
	rc = smb358_float_voltage_set(chip, vddmax);
	if (rc)
		dev_err(chip->dev,
			"Couldn't set float voltage rc = %d\n", rc);
}

#define HYSTERESIS_DECIDEGC 0 //libin.wt,MODIFY,2015.8.25

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add battery temperature protection*/
#if (defined QPNP_BATTERY_NTC_PROTECTION)
static void smb_chg_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct smb358_charger *chip = ctx;
	bool bat_hot = 0, bat_cold = 0, bat_present = 0, bat_warm = 0, bat_cool = 0;
	int temp = DEFAULT_TEMP;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invallid state parameter %d\n", state);
		return;
	}

	temp = smb358_get_prop_batt_temp(chip);
	pr_debug("smb_chg_adc_notification: temp = %d state = %s\n", temp,
				state == ADC_TM_WARM_STATE ? "hot" : "cold");

	if (state == ADC_TM_WARM_STATE) {
		if (chip->bat_health == BATTERY_HEALTH_OVERHEAT) {
			chip->bat_health = BATTERY_HEALTH_OVERHEAT;
			bat_hot = true;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->hot_bat_decidegc - HOT_TO_WARN_DEGREE;
			chip->adc_param.high_temp = SYSTEM_SHUT_DOWN_HIGH;
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (chip->bat_health == BATTERY_HEALTH_WARM) {
			chip->bat_health = BATTERY_HEALTH_OVERHEAT;
			bat_hot = true;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->hot_bat_decidegc - HOT_TO_WARN_DEGREE;
			chip->adc_param.high_temp = SYSTEM_SHUT_DOWN_HIGH;
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (chip->bat_health == BATTERY_HEALTH_GOOD) {
			chip->bat_health = BATTERY_HEALTH_WARM;
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->warm_bat_decidegc - WARM_TO_GOOD_DEGREE;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc;
		} else if (chip->bat_health == BATTERY_HEALTH_COOL) {
			chip->bat_health = BATTERY_HEALTH_GOOD;
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cool_bat_decidegc;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc;
		} else if (chip->bat_health == BATTERY_HEALTH_COLD) {
			chip->bat_health = BATTERY_HEALTH_COOL;
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cold_bat_decidegc;
			chip->adc_param.high_temp =
				chip->cool_bat_decidegc + COOL_TO_GOOD_DEGREE;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		 if (chip->bat_health == BATTERY_HEALTH_COLD) {
			chip->bat_health = BATTERY_HEALTH_COLD;
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.high_temp =
				chip->cold_bat_decidegc + COLD_TO_COOL_DEGREE;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp =
				chip->bat_present_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (chip->bat_health == BATTERY_HEALTH_COOL) {
			chip->bat_health = BATTERY_HEALTH_COLD;
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.high_temp =
				chip->cold_bat_decidegc + COLD_TO_COOL_DEGREE;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp =
				chip->bat_present_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (chip->bat_health == BATTERY_HEALTH_GOOD) {
			chip->bat_health = BATTERY_HEALTH_COOL;
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;

			chip->adc_param.high_temp =
				chip->cool_bat_decidegc + COOL_TO_GOOD_DEGREE;
			chip->adc_param.low_temp =
				chip->cold_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (chip->bat_health == BATTERY_HEALTH_WARM) {
			chip->bat_health = BATTERY_HEALTH_GOOD;
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.high_temp =
				chip->warm_bat_decidegc;
			chip->adc_param.low_temp =
				chip->cool_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (chip->bat_health == BATTERY_HEALTH_OVERHEAT) {
			chip->bat_health = BATTERY_HEALTH_WARM;
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
					chip->warm_bat_decidegc -WARM_TO_GOOD_DEGREE;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc ;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	if (bat_present)
		chip->battery_missing = false;
	else
		chip->battery_missing = true;

	pr_debug("hot %d, cold %d, warm %d, cool %d, jeita supported %d, missing %d, low = %d deciDegC, high = %d deciDegC, bat_health = %d\n",
		bat_hot, bat_cold, bat_warm,
		bat_cool, chip->jeita_supported, chip->battery_missing,
		chip->adc_param.low_temp, chip->adc_param.high_temp, chip->bat_health);

	pr_debug("hot %d, cold %d, warm %d, cool %d, jeita supported %d, missing %d, low = %d deciDegC, high = %d deciDegC, bat_health = %d\n",
		chip->batt_hot, chip->batt_cold, chip->batt_warm,
		chip->batt_cool, chip->jeita_supported, chip->battery_missing,
		chip->adc_param.low_temp, chip->adc_param.high_temp, chip->bat_health);

	if (bat_hot ^ chip->batt_hot || bat_cold ^ chip->batt_cold) {
		chip->batt_hot = bat_hot;
		chip->batt_cold = bat_cold;
		/* stop charging explicitly since we use PMIC thermal pin*/
		if ((bat_hot || bat_cold || chip->battery_missing)&&!disable_software_temp_monitor)
			smb358_charging_disable(chip, THERMAL, 1);
		else
			smb358_charging_disable(chip, THERMAL, 0);
	}

	if ((chip->batt_warm ^ bat_warm || chip->batt_cool ^ bat_cool)
						&& chip->jeita_supported) {
		chip->batt_warm = bat_warm;
		chip->batt_cool = bat_cool;
		smb358_chg_set_appropriate_battery_current(chip);
		smb358_chg_set_appropriate_vddmax(chip);
	}

	pr_debug("hot %d, cold %d, warm %d, cool %d, jeita supported %d, missing %d, low = %d deciDegC, high = %d deciDegC, bat_health = %d\n",
		chip->batt_hot, chip->batt_cold, chip->batt_warm,
		chip->batt_cool, chip->jeita_supported, chip->battery_missing,
		chip->adc_param.low_temp, chip->adc_param.high_temp, chip->bat_health);

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add charge ntc debug simulate */
#if (!defined QPNP_NTC_DEBUG_SIMULATE)
	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
#endif
/* [PLATFORM]-Mod-END by pingao.yang */
}
#else
static void smb_chg_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct smb358_charger *chip = ctx;
	bool bat_hot = 0, bat_cold = 0, bat_present = 0, bat_warm = 0, bat_cool = 0;
	int temp;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invallid state parameter %d\n", state);
		return;
	}

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add charge ntc debug simulate */
#if (!defined QPNP_NTC_DEBUG_SIMULATE)
	temp = smb358_get_prop_batt_temp(chip);
#else
	temp = chip->debug_temperature;
#endif
/* [PLATFORM]-Mod-END by pingao.yang */

	pr_debug("temp = %d state = %s\n", temp,
				state == ADC_TM_WARM_STATE ? "hot" : "cold");

	if (state == ADC_TM_WARM_STATE) {
		if (temp >= chip->hot_bat_decidegc) {
			bat_hot = true;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->hot_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (temp >=
			chip->warm_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->warm_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc;
		} else if (temp >=
			chip->cool_bat_decidegc && chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cool_bat_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc;
		} else if (temp >=
			chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;

			chip->adc_param.low_temp =
				chip->cold_bat_decidegc - HYSTERESIS_DECIDEGC;
			if (chip->jeita_supported)
				chip->adc_param.high_temp =
						chip->cool_bat_decidegc;
			else
				chip->adc_param.high_temp =
						chip->hot_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp >= chip->bat_present_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;

			chip->adc_param.high_temp = chip->cold_bat_decidegc;
			chip->adc_param.low_temp = chip->bat_present_decidegc
							- HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		if (temp <= chip->bat_present_decidegc) {
			bat_cold = true;
			bat_cool = false;
			bat_hot = false;
			bat_warm = false;
			bat_present = false;
			chip->adc_param.high_temp = chip->bat_present_decidegc
							+ HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_WARM_THR_ENABLE;
		} else if (temp <= chip->cold_bat_decidegc) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = true;
			bat_cool = false;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cold_bat_decidegc + HYSTERESIS_DECIDEGC;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp =
				chip->bat_present_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->cool_bat_decidegc &&
					chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = true;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->cool_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->cold_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->warm_bat_decidegc &&
					chip->jeita_supported) {
			bat_hot = false;
			bat_warm = false;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp =
				chip->cool_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <= chip->hot_bat_decidegc) {
			bat_hot = false;
			bat_warm = true;
			bat_cold = false;
			bat_cool = false;
			bat_present = true;
			if (chip->jeita_supported)
				chip->adc_param.low_temp =
					chip->warm_bat_decidegc;
			else
				chip->adc_param.low_temp =
					chip->cold_bat_decidegc;
			chip->adc_param.high_temp =
				chip->hot_bat_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	if (bat_present)
		chip->battery_missing = false;
	else
		chip->battery_missing = true;

	if (bat_hot ^ chip->batt_hot || bat_cold ^ chip->batt_cold) {
		chip->batt_hot = bat_hot;
		chip->batt_cold = bat_cold;
		/* stop charging explicitly since we use PMIC thermal pin*/
		if ((bat_hot || bat_cold || chip->battery_missing) && !disable_software_temp_monitor)
			smb358_charging_disable(chip, THERMAL, 1);
		else
			smb358_charging_disable(chip, THERMAL, 0);
	}

	if ((chip->batt_warm ^ bat_warm || chip->batt_cool ^ bat_cool)
						&& chip->jeita_supported) {
		chip->batt_warm = bat_warm;
		chip->batt_cool = bat_cool;
		smb358_chg_set_appropriate_battery_current(chip);
		smb358_chg_set_appropriate_vddmax(chip);
	}

	pr_debug("hot %d, cold %d, warm %d, cool %d, jeita supported %d, missing %d, low = %d deciDegC, high = %d deciDegC\n",
		chip->batt_hot, chip->batt_cold, chip->batt_warm,
		chip->batt_cool, chip->jeita_supported, chip->battery_missing,
		chip->adc_param.low_temp, chip->adc_param.high_temp);

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add charge ntc debug simulate */
#if (!defined QPNP_NTC_DEBUG_SIMULATE)
	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
#endif
/* [PLATFORM]-Mod-END by pingao.yang */
}
#endif
/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add battery temperature protection*/

/* only for SMB thermal */
static int hot_hard_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_hot = !!status;
	return 0;
}
static int cold_hard_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_cold = !!status;
	return 0;
}
static int hot_soft_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_warm = !!status;
	return 0;
}
static int cold_soft_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_cool = !!status;
	return 0;
}

static int battery_missing(struct smb358_charger *chip, u8 status)
{
	chip->battery_missing = !!status;
	return 0;
}

/**
 * power_ok_handler() - called when the switcher turns on or turns off
 * @chip: pointer to smb135x_chg chip
 * @rt_stat: the status bit indicating switcher turning on or off
 */
static int power_ok_handler(struct smb358_charger *chip, u8 rt_stat)
{
	chip->power_ok = !!rt_stat;
	msleep(30);
	power_supply_changed(&chip->batt_psy);//hoper
	return 0;
}

static struct irq_handler_info handlers[] = {
	[0] = {
		.stat_reg	= IRQ_A_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "cold_soft",
				.smb_irq	= cold_soft_handler,
			},
			{
				.name		= "hot_soft",
				.smb_irq	= hot_soft_handler,
			},
			{
				.name		= "cold_hard",
				.smb_irq	= cold_hard_handler,
			},
			{
				.name		= "hot_hard",
				.smb_irq	= hot_hard_handler,
			},
		},
	},
	[1] = {
		.stat_reg	= IRQ_B_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "chg_hot",
			},
			{
				.name		= "vbat_low",
			},
			{
				.name		= "battery_missing",
				.smb_irq	= battery_missing
			},
			{
				.name		= "battery_ov",
			},
		},
	},
	[2] = {
		.stat_reg	= IRQ_C_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "chg_term",
				.smb_irq	= chg_term,
			},
			{
				.name		= "taper",
				.smb_irq	= taper_chg,
			},
			{
				.name		= "recharge",
				.smb_irq	= chg_recharge,
			},
			{
				.name		= "fast_chg",
				.smb_irq	= fast_chg,
			},
		},
	},
	[3] = {
		.stat_reg	= IRQ_D_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "prechg_timeout",
			},
			{
				.name		= "safety_timeout",
			},
			{
				.name		= "aicl_complete",
			},
			{
				.name		= "src_detect",
				.smb_irq	= apsd_complete,
			},
		},
	},
	[4] = {
		.stat_reg	= IRQ_E_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "usbin_uv",
				.smb_irq        = chg_uv,
			},
			{
				.name		= "usbin_ov",
				.smb_irq	= chg_ov,
			},
			{
				.name		= "unknown",
			},
			{
				.name		= "unknown",
			},
		},
	},
	[5] = {
		.stat_reg	= IRQ_F_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "power_ok",
				.smb_irq	= power_ok_handler,
			},
			{
				.name		= "otg_det",
			},
			{
				.name		= "otg_batt_uv",
			},
			{
				.name		= "otg_oc",
			},
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2
static int smb358_update_power_on_state(struct smb358_charger *chip)
{
        int i,j;
        int rc;
        u8 rt_stat,prev_rt_stat,changed;
        struct smb_irq_info *info;

        for (i = 0; i < ARRAY_SIZE(handlers); i++) {
            if (handlers[i].stat_reg == IRQ_F_REG) {
                rc = smb358_read_reg(chip, handlers[i].stat_reg,
                            &handlers[i].val);
                if (rc < 0) {
                    dev_err(chip->dev, "Couldn't read %d rc = %d\n",
                            handlers[i].stat_reg, rc);
                    return -1;
                }
                info = handlers[i].irq_info;
                for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
                    if(!strcmp(info[j].name,  "power_ok")) {
                    rt_stat = handlers[i].val
                         & (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
                   prev_rt_stat = handlers[i].prev_val & (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
                   changed = prev_rt_stat ^ rt_stat;
                        if(changed) {
                            handlers[i].prev_val = handlers[i].val;
                            chip->power_ok = rt_stat;
                            pr_err("chip->power_ok  = %d\n", chip->power_ok );
                        }
                    }
                }
            }
        }

        return 0;
}
static irqreturn_t smb358_chg_stat_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int i, j;
	u8 triggered;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

	mutex_lock(&chip->irq_complete);

	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		dev_dbg(chip->dev, "IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = smb358_read_reg(chip, handlers[i].stat_reg,
						&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			triggered = handlers[i].val
			       & (IRQ_LATCHED_MASK << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if (handlers[i].stat_reg == IRQ_C_REG) {
				SMB358_PRINT("%s: name=%s, chip->batt_full=%d, rt_stat=%d\n", __func__, handlers[i].irq_info[j].name, chip->batt_full, rt_stat);
			}

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
								rt_stat);
				if (rc < 0)
					dev_err(chip->dev,
						"Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_debug("handler count = %d\n", handler_count);
	if (handler_count) {
		pr_debug("batt psy changed\n");
		power_supply_changed(&chip->batt_psy);
	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static irqreturn_t smb358_chg_valid_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int present;

	present = gpio_get_value_cansleep(chip->chg_valid_gpio);
	if (present < 0) {
		dev_err(chip->dev, "Couldn't read chg_valid gpio=%d\n",
						chip->chg_valid_gpio);
		return IRQ_HANDLED;
	}
	present ^= chip->chg_valid_act_low;

	dev_dbg(chip->dev, "%s: chg_present = %d\n", __func__, present);

	if (present != chip->chg_present) {
		chip->chg_present = present;
		pre_usb_current_ma = -EINVAL; //Other_platform modify 20151113 huangfusheng.wt solve current jump
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
		power_supply_changed(&chip->batt_psy);//hoper
	}
        pr_debug("smb358_chg_valid_handler has been started\n");
	return IRQ_HANDLED;
}

static void smb358_external_power_changed(struct power_supply *psy)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0;

	if (chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc)
		dev_err(chip->dev,
			"Couldn't read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;

	chip->psy_usb_ma = current_limit;
	smb358_enable_volatile_writes(chip);
	smb358_set_usb_chg_current(chip, current_limit);

	smb358_chg_set_appropriate_battery_current(chip); //Other_platform modify 20151113 huangfusheng.wt solve current jump
	smb358_chg_set_appropriate_vddmax(chip);//Other_platform modify 20151113 huangfusheng.wt solve current jump

	dev_dbg(chip->dev, "current_limit = %d\n", current_limit);
}

#if defined(CONFIG_DEBUG_FS)
#define LAST_CNFG_REG	0x13
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x30
#define LAST_CMD_REG	0x33
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x35
#define LAST_STATUS_REG		0x3F
static int show_status_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct smb358_charger *chip = data;
	int rc;
	u8 temp;

	rc = smb358_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct smb358_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = smb358_write_reg(chip, chip->peek_poke_address, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int force_irq_set(void *data, u64 val)
{
	struct smb358_charger *chip = data;

	smb358_chg_stat_handler(chip->client->irq, data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_irq_ops, NULL, force_irq_set, "0x%02llx\n");
#endif

#if 1	//def DEBUG
static void dump_regs(struct smb358_charger *chip)
{
	int rc;
	u8 reg;
	u8 addr;

	//for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
	for (addr = 0; addr <= 1; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_err("0x%02x = 0x%02x\n", addr, reg);
	}

	//for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
	for (addr = 0x3d; addr <= LAST_STATUS_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_err("STATUS_REG:0x%02x = 0x%02x\n", addr, reg);
	}

	//for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
	for (addr = FIRST_CMD_REG; addr <= 0x31; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			pr_err("CMD_REG:0x%02x = 0x%02x\n", addr, reg);
	}
}
#else
static void dump_regs(struct smb358_charger *chip)
{
}
#endif

static int smb_parse_dt(struct smb358_charger *chip)
{
	int rc;
	enum of_gpio_flags gpio_flags;
	struct device_node *node = chip->dev->of_node;
	int batt_present_degree_negative;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	chip->charging_disabled = of_property_read_bool(node,
					"qcom,charger-disabled");

	chip->inhibit_disabled = of_property_read_bool(node,
					"qcom,chg-inhibit-disabled");
	chip->chg_autonomous_mode = of_property_read_bool(node,
					"qcom,chg-autonomous-mode");

	chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");

	chip->using_pmic_therm = of_property_read_bool(node,
						"qcom,using-pmic-therm");
	chip->bms_controlled_charging = of_property_read_bool(node,
						"qcom,bms-controlled-charging");

	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	chip->chg_valid_gpio = of_get_named_gpio_flags(node,
				"qcom,chg-valid-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(chip->chg_valid_gpio))
		dev_dbg(chip->dev, "Invalid chg-valid-gpio");
	else
		chip->chg_valid_act_low = gpio_flags & OF_GPIO_ACTIVE_LOW;

	rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma",
						&chip->fastchg_current_max_ma);
	if (rc)
		chip->fastchg_current_max_ma = SMB358_FAST_CHG_MAX_MA;

	chip->iterm_disabled = of_property_read_bool(node,
					"qcom,iterm-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0) {
		chip->vfloat_mv = -EINVAL;
		pr_err("float-voltage-mv property missing, exit\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "qcom,recharge-mv",
						&chip->recharge_mv);
	if (rc < 0)
		chip->recharge_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node,
					"qcom,recharge-disabled");

	rc = of_property_read_u32(node, "qcom,cold-bat-decidegc",
						&chip->cold_bat_decidegc);
	if (rc < 0)
		chip->cold_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,hot-bat-decidegc",
						&chip->hot_bat_decidegc);
	if (rc < 0)
		chip->hot_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,warm-bat-decidegc",
						&chip->warm_bat_decidegc);

	rc |= of_property_read_u32(node, "qcom,cool-bat-decidegc",
						&chip->cool_bat_decidegc);

	if (!rc) {
		rc = of_property_read_u32(node, "qcom,cool-bat-mv",
						&chip->cool_bat_mv);

		rc |= of_property_read_u32(node, "qcom,warm-bat-mv",
						&chip->warm_bat_mv);

		rc |= of_property_read_u32(node, "qcom,cool-bat-ma",
						&chip->cool_bat_ma);

		rc |= of_property_read_u32(node, "qcom,warm-bat-ma",
						&chip->warm_bat_ma);
		if (rc)
			chip->jeita_supported = false;
		else
			chip->jeita_supported = true;
	}

	if (of_find_property(node, "qcom,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
			chip->thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		pr_debug("thermal_mitigations = %d, %d, %d, %d; thermal_levels = %d\n", chip->thermal_mitigation[0], chip->thermal_mitigation[1], chip->thermal_mitigation[2], chip->thermal_mitigation[3], chip->thermal_levels);
		if (rc) {
			pr_err("Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}
	pr_debug("jeita_supported = %d", chip->jeita_supported);

	rc = of_property_read_u32(node, "qcom,bat-present-decidegc",
						&batt_present_degree_negative);
	if (rc < 0)
		chip->bat_present_decidegc = -EINVAL;
	else
		chip->bat_present_decidegc = -batt_present_degree_negative;

	if (of_get_property(node, "qcom,vcc-i2c-supply", NULL)) {
		chip->vcc_i2c = devm_regulator_get(chip->dev, "vcc-i2c");
		if (IS_ERR(chip->vcc_i2c)) {
			dev_err(chip->dev,
				"%s: Failed to get vcc_i2c regulator\n",
								__func__);
			return PTR_ERR(chip->vcc_i2c);
		}
	}

  // +Other_platform modify 20151202 huangfusheng.wt add battery capacity discription
	rc = of_property_read_u32(node, "qcom,battery-fcc",
						&chip->fcc_mah);
	if (rc)
	{
		chip->fcc_mah = BATTERY_FCC;
	}
 // -Other_platform modify 20151202 huangfusheng.wt add battery capacity discription
 
	pr_debug("inhibit-disabled = %d, recharge-disabled = %d, recharge-mv = %d,",
		chip->inhibit_disabled, chip->recharge_disabled,
						chip->recharge_mv);
	pr_debug("vfloat-mv = %d, iterm-disabled = %d,",
			chip->vfloat_mv, chip->iterm_ma);
	pr_debug("fastchg-current = %d, charging-disabled = %d,",
			chip->fastchg_current_max_ma,
					chip->charging_disabled);
	pr_debug("disable-apsd = %d bms = %s cold-bat-degree = %d,",
		chip->disable_apsd, chip->bms_psy_name,
					chip->cold_bat_decidegc);
	pr_debug("hot-bat-degree = %d, bat-present-decidegc = %d\n",
		chip->hot_bat_decidegc, chip->bat_present_decidegc);
	pr_debug("fcc_mah is %d \n",chip->fcc_mah);
	return 0;
}

static int determine_initial_state(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, IRQ_B_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_B rc = %d\n", rc);
		goto fail_init_status;
	}

	rc = smb358_read_reg(chip, IRQ_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_C rc = %d\n", rc);
		goto fail_init_status;
	}
	chip->batt_full = (reg & IRQ_C_TERM_BIT) ? true : false;
	chip->chg_finish = chip->batt_full;	//add by pingyao.yang
	SMB358_PRINT("%s: chip->batt_full = %d\n", __func__, chip->batt_full);

	rc = smb358_read_reg(chip, IRQ_A_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq A rc = %d\n", rc);
		return rc;
	}

	/* For current design, can ignore this */
	if (reg & IRQ_A_HOT_HARD_BIT)
		chip->batt_hot = true;
	if (reg & IRQ_A_COLD_HARD_BIT)
		chip->batt_cold = true;
	if (reg & IRQ_A_HOT_SOFT_BIT)
		chip->batt_warm = true;
	if (reg & IRQ_A_COLD_SOFT_BIT)
		chip->batt_cool = true;

	rc = smb358_read_reg(chip, IRQ_E_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
		goto fail_init_status;
	}

	if (reg & IRQ_E_INPUT_UV_BIT) {
		chg_uv(chip, 1);
	} else {
		chg_uv(chip, 0);
		apsd_complete(chip, 1);
	}

	return 0;

fail_init_status:
	dev_err(chip->dev, "Couldn't determine initial status\n");
	return rc;
}

#if defined(CONFIG_DEBUG_FS)
static void smb358_debugfs_init(struct smb358_charger *chip)
{
	int rc;
	chip->debug_root = debugfs_create_dir("smb358", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cnfg debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &status_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create status debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cmd_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cmd debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);
		}

		ent = debugfs_create_file("force_irq",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &force_irq_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create force_irq debug file rc =%d\n",
				rc);
		}

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent || IS_ERR(ent)) {
			rc = PTR_ERR(ent);
			dev_err(chip->dev,
				"Couldn't create cnfg irq_count file rc = %d\n",
				rc);
		}
	}
}
#else
static void smb358_debugfs_init(struct smb358_charger *chip)
{
}
#endif

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add charge ntc debug simulate */
#if (defined QPNP_NTC_DEBUG_SIMULATE)
static int ntc_debugfs_show(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;

	pr_err("ntc_debug_get_bat_temp %d\n", chip->debug_temperature);
	return 0;
}

static ssize_t ntc_debugfs_simulate(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct smb358_charger *chip = s->private;

	char buf[64];

	memset(buf, 0x00, sizeof(buf));
	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "up", 2)) {
		chip->debug_temperature =  simple_strtol(&buf[2], NULL, 10);
		smb_chg_adc_notification(ADC_TM_LOW_STATE, chip);
	}
	else if (!strncmp(buf, "down", 4)) {
		chip->debug_temperature =  simple_strtol(&buf[4], NULL, 10);
		smb_chg_adc_notification(ADC_TM_HIGH_STATE, chip);
	}
	else{
		printk("Usage: echo [upxxx or downxxx] > /sys/kernel/debug/ntc_debug/simulate\n");
	}

	return count;
}

static int ntc_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, ntc_debugfs_show, chip);
}

static const struct file_operations ntc_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= ntc_debugfs_open,
	.read		= seq_read,
	.write		= ntc_debugfs_simulate,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif
/* [PLATFORM]-Mod-END by pingao.yang */

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add battery temperature protection*/
#if (defined QPNP_BATTERY_NTC_PROTECTION)
static void smb358_get_charge_status(struct smb358_charger *chip)
{
	int rc;

	chip->ntc_temp = smb358_get_prop_batt_temp(chip);

	rc = smb358_read_reg(chip, STATUS_C_REG, &chip->charge_status);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
	}

	chip->charger_type = chip->power_ok;

	dev_dbg(chip->dev, "STATUS_C_REG = %x,  STATUS_D_REG = %x, ntc_temp = %d\n",
						chip->charge_status, chip->charger_type, chip->ntc_temp);

}

static void ntc_protection_wakelock_enable(struct smb358_charger *chip)
{

	chip->charge_status &= STATUS_C_CHARGING_MASK;

	if (chip->charge_status && !wake_lock_active(&chip->monitor_wake_lock)) {
		wake_lock(&chip->monitor_wake_lock);
	} else if (!chip->charge_status && wake_lock_active(&chip->monitor_wake_lock)) {
		wake_unlock(&chip->monitor_wake_lock);
	}

	dev_dbg(chip->dev, "charger_type = %s, charge_status = %s, wake_lock = %s\n",
					chip->charger_type ? "charger" : "no charger",
					chip->charge_status ? "charging" : "not charging",
					wake_lock_active(&chip->monitor_wake_lock) ? "active" : "inactive");
}

static void ntc_protection_notification(struct smb358_charger *chip)
{
	dev_dbg(chip->dev, "[allan]:temp = %d, low_temp = %d, high_temp = %d\n", 
			chip->ntc_temp, chip->adc_param.low_temp, chip->adc_param.high_temp);
	
	if (chip->ntc_temp <= chip->adc_param.low_temp) {
		smb_chg_adc_notification(ADC_TM_HIGH_STATE, chip);
	} else if (chip->ntc_temp >= chip->adc_param.high_temp) {
		smb_chg_adc_notification(ADC_TM_LOW_STATE, chip);
	} else {
		dev_dbg(chip->dev, "ntc temp not trigger ! temp = %d\n", chip->ntc_temp);
	}
}

static void ntc_protection_monitor_work_callback(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb358_charger *chip = container_of(dwork,
					struct smb358_charger, monitor_workqueue);

	dev_err(chip->dev, "batt_full = %d, chip->chg_finish = %d, capacity = %d, power_ok = %d\n",
				chip->batt_full, chip->chg_finish, chip->capacity, chip->power_ok);
	smb358_set_recharge_voltage(chip);
	dump_regs(chip);
	smb358_get_charge_status(chip);
	ntc_protection_wakelock_enable(chip);
	ntc_protection_notification(chip);

	schedule_delayed_work(&chip->monitor_workqueue, NTC_PROTECTION_MES_INTERVAL);
}
#endif
/*[PLATFORM]-Add-END by pingao.yang*/

#define SMB_I2C_VTG_MIN_UV 1800000
#define SMB_I2C_VTG_MAX_UV 1800000
static int smb358_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc, irq;
	struct smb358_charger *chip;
	struct power_supply *usb_psy;
	u8 reg = 0;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;

	/* early for VADC get, defer probe if needed */
	chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("vadc property missing\n");
		return rc;
	}

	rc = smb_parse_dt(chip);
	if (rc) {
		dev_err(&client->dev, "Couldn't parse DT nodes rc=%d\n", rc);
		return rc;
	}
	/* i2c pull up regulator configuration */
	if (chip->vcc_i2c) {
		if (regulator_count_voltages(chip->vcc_i2c) > 0) {
			rc = regulator_set_voltage(chip->vcc_i2c,
				SMB_I2C_VTG_MIN_UV, SMB_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(&client->dev,
				"regulator vcc_i2c set failed, rc = %d\n",
								rc);
				return rc;
			}
		}

		rc = regulator_enable(chip->vcc_i2c);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_i2c enable failed rc = %d\n",
									rc);
			pr_debug("first alarm start\n");goto err_set_vtg_i2c;
		}
	}

	mutex_init(&chip->irq_complete);
	mutex_init(&chip->read_write_lock);
	mutex_init(&chip->path_suspend_lock);
	mutex_init(&chip->current_change_lock);

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add battery temperature protection*/
#if (defined QPNP_BATTERY_NTC_PROTECTION)
	INIT_DELAYED_WORK(&chip->monitor_workqueue, ntc_protection_monitor_work_callback);
#endif
/*[PLATFORM]-Add-END by pingao.yang*/

	/* probe the device to check if its actually connected */
	rc = smb358_read_reg(chip, CHG_OTH_CURRENT_CTRL_REG, &reg);
	if (rc) {
		pr_err("Failed to detect SMB358, device absent, rc = %d\n", rc);
		goto err_set_vtg_i2c;
	}

	/* using adc_tm for implementing pmic therm */
	if (chip->using_pmic_therm) {
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
		if (IS_ERR(chip->adc_tm_dev)) {
			rc = PTR_ERR(chip->adc_tm_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("adc_tm property missing\n");
			return rc;
		}
	}

	i2c_set_clientdata(client, chip);

	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smb358_battery_get_property;
	chip->batt_psy.set_property	= smb358_battery_set_property;
	chip->batt_psy.property_is_writeable =
					smb358_batt_property_is_writeable;
	chip->batt_psy.properties	= smb358_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smb358_battery_properties);
	chip->batt_psy.external_power_changed = smb358_external_power_changed;
	chip->batt_psy.supplied_to = pm_batt_supplied_to;
	chip->batt_psy.num_supplicants = ARRAY_SIZE(pm_batt_supplied_to);

	chip->resume_completed = true;
        smb358_update_power_on_state(chip);
        chip->batt_psy.bms_psy_ok=0;//hoper
	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev, "Couldn't register batt psy rc = %d\n",
				rc);
		goto err_set_vtg_i2c;
	}

	dump_regs(chip);
        
	rc = smb358_regulator_init(chip);
	if  (rc) {
		dev_err(&client->dev,
			"Couldn't initialize smb358 ragulator rc=%d\n", rc);
		goto fail_regulator_register;
	}

	rc = smb358_hw_init(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't intialize hardware rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}

	rc = determine_initial_state(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't determine initial state rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}

	/* We will not use it by default */
	if (gpio_is_valid(chip->chg_valid_gpio)) {
		rc = gpio_request(chip->chg_valid_gpio, "smb358_chg_valid");
		if (rc) {
			dev_err(&client->dev,
				"gpio_request for %d failed rc=%d\n",
				chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		irq = gpio_to_irq(chip->chg_valid_gpio);
		if (irq < 0) {
			dev_err(&client->dev,
				"Invalid chg_valid irq = %d\n", irq);
			goto fail_chg_valid_irq;
		}
		rc = devm_request_threaded_irq(&client->dev, irq,
				NULL, smb358_chg_valid_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"smb358_chg_valid_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed request_irq irq=%d, gpio=%d rc=%d\n",
				irq, chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		smb358_chg_valid_handler(irq, chip);
		enable_irq_wake(irq);
	}

	chip->irq_gpio = of_get_named_gpio_flags(chip->dev->of_node,
				"qcom,irq-gpio", 0, NULL);

	/* STAT irq configuration */
	if (gpio_is_valid(chip->irq_gpio)) {
		rc = gpio_request(chip->irq_gpio, "smb358_irq");
		if (rc) {
			dev_err(&client->dev,
					"irq gpio request failed, rc=%d", rc);
			goto fail_smb358_hw_init;
		}
		rc = gpio_direction_input(chip->irq_gpio);
		if (rc) {
			dev_err(&client->dev,
					"set_direction for irq gpio failed\n");
			goto fail_irq_gpio;
		}

		irq = gpio_to_irq(chip->irq_gpio);
		if (irq < 0) {
			dev_err(&client->dev,
				"Invalid irq_gpio irq = %d\n", irq);
			goto fail_irq_gpio;
		}
		rc = devm_request_threaded_irq(&client->dev, irq, NULL,
				smb358_chg_stat_handler,
				IRQF_TRIGGER_FALLING |  IRQF_ONESHOT,
				"smb358_chg_stat_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed STAT irq=%d request rc = %d\n",
				irq, rc);
			goto fail_irq_gpio;
		}
		enable_irq_wake(irq);
	} else {
		goto fail_irq_gpio;
	}

	if (chip->using_pmic_therm) {
		if (!chip->jeita_supported) {
			/* add hot/cold temperature monitor */
			chip->adc_param.low_temp = chip->cold_bat_decidegc;
			chip->adc_param.high_temp = chip->hot_bat_decidegc;
		} else {
			chip->adc_param.low_temp = chip->cool_bat_decidegc;
			chip->adc_param.high_temp = chip->warm_bat_decidegc;
		}
		chip->adc_param.timer_interval = ADC_MEAS2_INTERVAL_1S;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		chip->adc_param.btm_ctx = chip;
		chip->adc_param.threshold_notification =
				smb_chg_adc_notification;
		chip->adc_param.channel = P_MUX2_1_1;
		//Other_platform_modify 20160520 xuji.wt add C3 charging config

		/* update battery missing info in tm_channel_measure*/
		rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
							&chip->adc_param);
		if (rc)
			pr_err("requesting ADC error %d\n", rc);

	/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add battery temperature protection*/
	#if (defined QPNP_BATTERY_NTC_PROTECTION)
	#if (defined QPNP_NTC_DEBUG_SIMULATE)
		chip->init_temperature = DEFAULT_TEMP;
		chip->debug_temperature = chip->init_temperature;
	#else
		chip->init_temperature = smb358_get_prop_batt_temp(chip);
	#endif
		chip->ntc_temp = chip->init_temperature;
		if (chip->init_temperature <= chip->cold_bat_decidegc) {
			chip->bat_health = BATTERY_HEALTH_COLD;
		} else if (chip->init_temperature <= chip->cool_bat_decidegc) {
			chip->bat_health = BATTERY_HEALTH_COOL;
		} else if (chip->init_temperature <= chip->warm_bat_decidegc) {
			chip->bat_health = BATTERY_HEALTH_GOOD;
		} else if (chip->init_temperature <= chip->hot_bat_decidegc) {
			chip->bat_health = BATTERY_HEALTH_WARM;
		} else {
			chip->bat_health = BATTERY_HEALTH_OVERHEAT;
		}

		pr_debug("init_temperature = %d, bat_health = %d\n", chip->init_temperature, chip->bat_health);
	#endif
	/* [PLATFORM]-Mod-END by pingao.yang */

	/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add charge ntc debug simulate */
	#if (defined QPNP_NTC_DEBUG_SIMULATE)
		chip->ntc_root = debugfs_create_dir("ntc_debug", NULL);
		if (!chip->ntc_root)
			dev_err(chip->dev, "Couldn't create debug dir\n");

		if (chip->ntc_root) {
			struct dentry *ent;

			ent = debugfs_create_file("simulate", S_IFREG | S_IRUGO | S_IWUSR,
						  chip->ntc_root, chip,
						  &ntc_debugfs_ops);
			if (!ent)
				dev_err(chip->dev,
					"Couldn't create count debug file rc = %d\n",
					rc);
		}
	#endif
	/*[PLATFORM]-Add-END by pingao.yang */
	}

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add battery temperature protection*/
#if (defined QPNP_BATTERY_NTC_PROTECTION)
	schedule_delayed_work(&chip->monitor_workqueue, NTC_PROTECTION_MES_INTERVAL);
	wake_lock_init(&chip->monitor_wake_lock, WAKE_LOCK_SUSPEND, "ntc_protection");
#endif
/*[PLATFORM]-Add-END by pingao.yang*/

	smb358_debugfs_init(chip);

	dump_regs(chip);

	dev_info(chip->dev, "SMB358 successfully probed. charger=%d, batt=%d\n",
			chip->chg_present, smb358_get_prop_batt_present(chip));

	 if (chip->bms_psy_name)
         {
		chip->bms_psy =power_supply_get_by_name((char *)chip->bms_psy_name);
        	if(chip->bms_psy&&chip->bms_psy->bms_psy_ok==1&&chip->power_ok)
        	{
			chip->batt_psy.bms_psy_ok=1;
        	}   	

	}
	return 0;

fail_chg_valid_irq:
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);
fail_irq_gpio:
	if (gpio_is_valid(chip->irq_gpio))
		gpio_free(chip->irq_gpio);
fail_smb358_hw_init:
	regulator_unregister(chip->otg_vreg.rdev);
fail_regulator_register:
	power_supply_unregister(&chip->batt_psy);
err_set_vtg_i2c:
	if (chip->vcc_i2c)
		if (regulator_count_voltages(chip->vcc_i2c) > 0)
			regulator_set_voltage(chip->vcc_i2c, 0,
						SMB_I2C_VTG_MAX_UV);
	return rc;
}

static int smb358_charger_remove(struct i2c_client *client)
{
	struct smb358_charger *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->batt_psy);
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);

	if (chip->vcc_i2c)
		regulator_disable(chip->vcc_i2c);

	mutex_destroy(&chip->irq_complete);
	debugfs_remove_recursive(chip->debug_root);
      
	return 0;
}

static int smb358_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;
	int i;

	for (i = 0; i < 2; i++) {
		rc = smb358_read_reg(chip, FAULT_INT_REG + i,
					&chip->irq_cfg_mask[i]);
		if (rc)
			dev_err(chip->dev,
				"Couldn't save irq cfg regs rc = %d\n", rc);
	}

	/* enable wake up IRQs */
	rc = smb358_write_reg(chip, FAULT_INT_REG,
			FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_INPUT_UV_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set fault_irq_cfg rc = %d\n", rc);

	rc = smb358_write_reg(chip, STATUS_INT_REG,
			STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT |
			STATUS_INT_CHGING_BIT | STATUS_INT_INOK_BIT |
			STATUS_INT_OTG_DETECT_BIT | STATUS_INT_CHG_INHI_BIT);
	if (rc < 0)
		dev_err(chip->dev,
			"Couldn't set status_irq_cfg rc = %d\n", rc);

	mutex_lock(&chip->irq_complete);
	if (chip->vcc_i2c) {
		rc = regulator_disable(chip->vcc_i2c);
		if (rc) {
			dev_err(chip->dev,
				"Regulator vcc_i2c disable failed rc=%d\n", rc);
			mutex_unlock(&chip->irq_complete);
			return rc;
		}
	}

	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);
	return 0;
}

static int smb358_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int smb358_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;
	int i;

	if (chip->vcc_i2c) {
		rc = regulator_enable(chip->vcc_i2c);
		if (rc) {
			dev_err(chip->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			return rc;
		}
	}
	/* Restore IRQ config */
	for (i = 0; i < 2; i++) {
		rc = smb358_write_reg(chip, FAULT_INT_REG + i,
					chip->irq_cfg_mask[i]);
		if (rc)
			dev_err(chip->dev,
				"Couldn't restore irq cfg regs rc=%d\n", rc);
	}

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	mutex_unlock(&chip->irq_complete);
	if (chip->irq_waiting) {
		smb358_chg_stat_handler(client->irq, chip);
		enable_irq(client->irq);
	}

/*[PLATFORM]-Add-BEGIN by pingao.yang, 2016/07/11, add battery temperature protection*/
#if (defined QPNP_BATTERY_NTC_PROTECTION)
	if (chip->charger_type ) {
		smb358_get_charge_status(chip);
		ntc_protection_wakelock_enable(chip);
		ntc_protection_notification(chip);
	}
#endif
/*[PLATFORM]-Add-END by pingao.yang*/

	return 0;
}

static const struct dev_pm_ops smb358_pm_ops = {
	.suspend	= smb358_suspend,
	.suspend_noirq	= smb358_suspend_noirq,
	.resume		= smb358_resume,
};

static struct of_device_id smb358_match_table[] = {
	{ .compatible = "qcom,smb358-charger",},
	{ },
};

static const struct i2c_device_id smb358_charger_id[] = {
	{"smb358-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb358_charger_id);

static struct i2c_driver smb358_charger_driver = {
	.driver		= {
		.name		= "smb358-charger",
		.owner		= THIS_MODULE,
		.of_match_table = smb358_match_table,
		.pm		= &smb358_pm_ops,
	},
	.probe		= smb358_charger_probe,
	.remove		= smb358_charger_remove,
	.id_table	= smb358_charger_id,
};

module_i2c_driver(smb358_charger_driver);

MODULE_DESCRIPTION("SMB358 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb358-charger");
