/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2011-2013
 * Texas Instruments, <www.ti.com>
 *
 * For more details, please see the TRM at http://www.ti.com/product/tps65217a
 */

#ifndef __POWER_TPS65217_H__
#define __POWER_TPS65217_H__

/* I2C chip address */
#define TPS65217_CHIP_PM			0x24

/* Registers */
enum {
	TPS65217_CHIPID				= 0x00,
	TPS65217_POWER_PATH,
	TPS65217_INTERRUPT,
	TPS65217_CHGCONFIG0,
	TPS65217_CHGCONFIG1,
	TPS65217_CHGCONFIG2,
	TPS65217_CHGCONFIG3,
	TPS65217_WLEDCTRL1,
	TPS65217_WLEDCTRL2,
	TPS65217_MUXCTRL,
	TPS65217_STATUS,
	TPS65217_PASSWORD,
	TPS65217_PGOOD,
	TPS65217_DEFPG,
	TPS65217_DEFDCDC1,
	TPS65217_DEFDCDC2,
	TPS65217_DEFDCDC3,
	TPS65217_DEFSLEW,
	TPS65217_DEFLDO1,
	TPS65217_DEFLDO2,
	TPS65217_DEFLS1,
	TPS65217_DEFLS2,
	TPS65217_ENABLE,
	TPS65217_RESERVED0, /* no 0x17 register available */
	TPS65217_DEFUVLO,
	TPS65217_SEQ1,
	TPS65217_SEQ2,
	TPS65217_SEQ3,
	TPS65217_SEQ4,
	TPS65217_SEQ5,
	TPS65217_SEQ6,
	TPS65217_PMIC_NUM_OF_REGS,
};

#define TPS65217_PROT_LEVEL_NONE		0x00
#define TPS65217_PROT_LEVEL_1			0x01
#define TPS65217_PROT_LEVEL_2			0x02

#define TPS65217_PASSWORD_LOCK_FOR_WRITE	0x00
#define TPS65217_PASSWORD_UNLOCK		0x7D

#define TPS65217_DCDC_GO			0x80

#define TPS65217_MASK_ALL_BITS			0xFF

#define TPS65217_USB_INPUT_CUR_LIMIT_MASK	0x03
#define TPS65217_USB_INPUT_CUR_LIMIT_100MA	0x00
#define TPS65217_USB_INPUT_CUR_LIMIT_500MA	0x01
#define TPS65217_USB_INPUT_CUR_LIMIT_1300MA	0x02
#define TPS65217_USB_INPUT_CUR_LIMIT_1800MA	0x03

#define TPS65217_DCDC_VOLT_SEL_950MV		0x02
#define TPS65217_DCDC_VOLT_SEL_1100MV		0x08
#define TPS65217_DCDC_VOLT_SEL_1125MV		0x09
#define TPS65217_DCDC_VOLT_SEL_1200MV		0x0c
#define TPS65217_DCDC_VOLT_SEL_1275MV		0x0F
#define TPS65217_DCDC_VOLT_SEL_1325MV		0x11

#define TPS65217_LDO_MASK			0x1F
#define TPS65217_LDO_VOLTAGE_OUT_1_8		0x06
#define TPS65217_LDO_VOLTAGE_OUT_3_3		0x1F

#define TPS65217_PWR_OFF			0x80
#define TPS65217_PWR_SRC_USB_BITMASK		0x4
#define TPS65217_PWR_SRC_AC_BITMASK		0x8

#define TPS65217_INT_USBI			BIT(0)
#define TPS65217_INT_ACI			BIT(1)
#define TPS65217_INT_PBI			BIT(2)
#define TPS65217_INT_USBM			BIT(4)
#define TPS65217_INT_ACM			BIT(5)
#define TPS65217_INT_PBM			BIT(6)

#define TPS65217_STATUS_PB			BIT(0)
#define TPS65217_STATUS_USB			BIT(2)
#define TPS65217_STATUS_AC			BIT(3)

int power_tps65217_init(unsigned char bus);

int tps65217_reg_read(uchar src_reg, uchar *src_val);
int tps65217_reg_write(uchar prot_level, uchar dest_reg, uchar dest_val,
		       uchar mask);
int tps65217_voltage_update(uchar dc_cntrl_reg, uchar volt_sel);
#endif	/* __POWER_TPS65217_H__ */
