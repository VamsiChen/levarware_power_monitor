/*
 * Copyright (c) 2022 Reliance Foundry Co. Ltd.
 *
 */

#ifndef POWER_REG_H_
#define POWER_REG_H_

// MAX77816 slave address
#define MAX77816_ADDR       (0x18)
#define MAX77816_WRTIE_ADDR ((MAX77816_ADDR << 1) & 0x30)
#define MAX77816_READ_ADDR  (MAX77816_WRTIE_ADDR | 0x01)

// MAX77816 register address
#define MAX77816_REG_CHIP_ID (0x00)
#define MAX77816_CHIP_ID     (0x01)

#define MAX77816_REG_STATUS             (0x01)
#define MAX77816_REG_STATUS_ILIM_MASK   (0xC0)
#define MAX77816_REG_STATUS_OVPTH_MASK  (0x0C)

#define MAX77816_REG_VOUT       (0x04)
#define MAX77816_REG_VOUT_MASK  (0x7F)

#define MAX77816_VOUT_MIN_V         (2.60f)
#define MAX77816_VOUT_MAX_V         (5.14f)
#define MAX77816_VOUT_RANGE_V       (MAX77816_VOUT_MAX_V - MAX77816_VOUT_MIN_V)
#define MAX77816_VOUT_RESOLUTION    (128)

#define MAX77816_REG_CONFIG2            (0x03)
#define MAX77816_REG_CONFIG2_BB_EN_BIT  (6)

#define MAX77816_REG_CONFIG1            (0x02)
#define MAX77816_REG_CONFIG1_ILIM_MASK  (0xC0)
#define MAX77816_ILIM_1_15_A            (0x00)

// MP273 slave address
#define MP273_ADDR (0x4B)

// MP273 register
#define MP273_REG_ILIM              (0x00)
#define MP273_REG_ILIM_IINLIM_MASK  (0x3f)
#define MP273_REG_OTP               (0x18)
#define MP273_REG_OTP_ADDR_MASK     (0x04)
#define MP273_ADC_CTRL_OTG_CFG      (0x03)
#define MP273_ADC_CTRL_MSK          (0x03)

#define MP273_IINLIM_MA_PER_BIT     (50)
#define MP273_IINLIM_OFFSET_MA      (100)
#define MP273_IINLIM_MAX_MA         (3250)
#define MP273_IINLIM_MIN_MA         (100)

#define MP273_BATT_MV_PER_BIT     (20)
#define MP273_BATT_OFFSET_MV      (20)

#define MP273_REG_CTRL_VSYS (0x04)
#define MP273_CTRL_VSYS_CHG_CONF_MASK (0x30)
#define MP273_POWER_STATUS_MSK            (0xE0)
#define MP273_INT_STATUS            (0x0C)
#define MP2731_GOOD_POWER_SRC_DETECTED      (1)
#define MP2731_NO_POWER_SRC_DETECTED        (0)
#define MP2731_INPUT_OVP_MSK        (0b00100000)
#define MP2731_BATT_OVP_MSK         (0b00001000)
#define MP2731_NTC_FAULT_MSK        (0b00000111)
#define MP2731_NTC_FLOAT_MSK        (0b00000100)
#define MP2731_WDT_TIMER_RESET_MSK  (0b00001000)
#define MP2731_CHRG_STAT_MSK        (0b00011000)
#define MP2731_TIMER_FAULT_MSK      (0b10000000)
#define MP2731_TIMER_SET_MSK        (0b10111111)

#define MP2731_ADC_START            (208)
#define MP273_INPUT_CURRENT_LIMIT_DEFAULT_VAL_MA (500)
#define MP2731_NON_STD_CHRGER       (144)
#define MP2731_ICHRG_REG            (0x05)
#define MP2731_FAULT_REG            (0x0D)
#define MP2731_TIMER_REG            (0x08)
#define MP2731_SAFETY_TIMER_REG   (0x17)
#define MP273_BATT_VOLT_ADC         (0x0E)
#define MP2731_WDT_TIMER_SET        (0b10111101)

#define MP2731_WDT_RESET            (0b00001000)

#define MP2731_TRICKLE_CHARGE       (0b01)
#define MP2731_CONST_CURR_CHARGE    (0b10)
#define MP2731_CHARGE_COMPLETE      (0b11)

#define MP2731_CHARGING             (1)
#define MP2731_NOT_CHARGING         (0)
#define MP2731_SUSPEND_CHARGING     (-1)

#define MP2731_INPUT_OVP_FAULT      (0b01)
#define MP2731_BATT_OVP_FAULT       (0b01)
#define MP2731_TIMER_FAULT       (0b01)

typedef enum i2c1_bus
{
    I2C1_PINS_SENS = 0,
    I2C1_PINS_UTIL
} i2c1_bus_t;

int power_reg_external_power_on(float vout_v);
void power_reg_external_power_off(void);
int power_reg_get_power_status(struct tele_message *tele_data);

#endif /*POWER_REG_H_*/