/*
 * Copyright (c) 2022 Reliance Foundry Co. Ltd.
 *
 */

#include <zephyr.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <apps/pwr_monitor.h>

/*Logging*/
#include <logging/log.h>

#include "power_reg.h"

LOG_MODULE_REGISTER(power_reg);

#define GPIO DT_LABEL(DT_NODELABEL(gpio0))
const struct device *gpio = NULL;

#define EXT_REG_EN_NODE DT_ALIAS(ext_reg_en)

#if DT_NODE_HAS_PROP(EXT_REG_EN_NODE, gpios)
#define EXT_REG_EN_PIN DT_GPIO_PIN(EXT_REG_EN_NODE, gpios)
#define EXT_REG_EN_FLAGS (GPIO_OUTPUT_LOW | DT_GPIO_FLAGS(EXT_REG_EN_NODE, gpios))
#else
/* A build error here means your board isn't set up to drive an LED. */
#error "Unsupported board: ext-reg-en devicetree alias is not defined"
#endif

#define EXT_REG_STATE_NODE DT_ALIAS(ext_reg_status)

#if DT_NODE_HAS_PROP(EXT_REG_STATE_NODE, gpios)
#define EXT_REG_STATE_PIN DT_GPIO_PIN(EXT_REG_STATE_NODE, gpios)
#define EXT_REG_STATE_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(EXT_REG_STATE_NODE, gpios))
#else
/* A build error here means your board isn't set up to drive an LED. */
#error "Unsupported board: ext-reg-state devicetree alias is not defined"
#endif

#define I2C_NODE 			DT_NODELABEL(i2c1)
#define I2C 				DT_LABEL(I2C_NODE)
#define I2C_SENSOR_SDA_PIN 	DT_PROP(I2C_NODE, sda_pin)
#define I2C_SENSOR_SCL_PIN 	DT_PROP(I2C_NODE, scl_pin)
const struct device *i2c = NULL;

#define I2C_POWER_MGR_SDA_PIN (20)
#define I2C_POWER_MGR_SCL_PIN (19)

bool is_external_reg_init = false;
bool is_charger_init = false;

int power_reg_charger_init(uint16_t input_current_limit_ma);

static i2c1_bus_t current_i2c_bus = I2C1_PINS_SENS;
struct tele_message;

/**
 * @brief Configures the I2C1 peripheral to use on of the two available busses (SENS or UTIL). 
 *        The power regulator is on the UTIL bus, but startup default is the SENS bus
 * 
 * @param i2c_bus_select  I2C bus to use
 * 
 * @return Does not return any
 */
static void power_reg_set_i2c_bus(i2c1_bus_t i2c_bus_select)
{
	if (current_i2c_bus == i2c_bus_select) 
	{
		// nothing to do
		return;
	}

	uint8_t new_sda_pin = 0;
	uint8_t new_scl_pin = 0;

	switch(i2c_bus_select)
	{
		case I2C1_PINS_SENS: 	
			new_sda_pin = I2C_SENSOR_SDA_PIN;
			new_scl_pin = I2C_SENSOR_SCL_PIN;
			current_i2c_bus = I2C1_PINS_SENS;
			break;

		case I2C1_PINS_UTIL: 	
			new_sda_pin = I2C_POWER_MGR_SDA_PIN;
			new_scl_pin = I2C_POWER_MGR_SCL_PIN;
			current_i2c_bus = I2C1_PINS_UTIL;
			break;
			
		default:	
			// NOTHING TO DO - ERROR
			LOG_ERR("Invalid I2C1 Bus: %i", i2c_bus_select);	
			return;
	};

	// LOG_DBG("Remapping the I2C1 pins -> SDA: %d, SCL: %d", new_sda_pin, new_scl_pin);
	//disable the i2c
	NRF_TWIM1->ENABLE = 0x00;

	k_msleep(10);

	//change the pins
	NRF_TWIM1->PSEL.SDA = (1 << 31);
	NRF_TWIM1->PSEL.SCL = (1 << 31);
	NRF_TWIM1->PSEL.SDA = new_sda_pin;
	NRF_TWIM1->PSEL.SCL =  new_scl_pin;

	//re-enable the i2c
	NRF_TWIM1->ENABLE = 0x06;

	k_msleep(10);
}


/**
 * @brief Turn on/off the external regulator by controlling the enable pin
 * 
 * @param state The desired state of the regulator, TRUE -> ENABLED, FALSE -> DISABLED
 * 
 * @return DOes not return any
 */
static void power_reg_external_set_en_pin(bool state)
{
	gpio = device_get_binding(GPIO);

	int pin_val = state ? 1:0;

	gpio_pin_set(gpio, EXT_REG_EN_PIN, pin_val);

	if(state == false)
	{
		is_external_reg_init = false;
	}
}

/**
 * @brief Initializes the on-board regulators
 * 
 * @param voout_t target voltage for the external regulator
 * 
 * @return does not return any 
 */
static int power_reg_init(void)
{
	i2c = device_get_binding(I2C);

	// External Regulator's GPIO
	gpio = device_get_binding(GPIO);
	if (gpio == NULL)
	{
		LOG_ERR("Unable to get the pointer to device struct of GPIO0");
		return -EIO;
	}

	int err = gpio_pin_configure(gpio, EXT_REG_EN_PIN, EXT_REG_EN_FLAGS);
	if (err)
	{
		LOG_ERR("Unable to configure external regulator's enable pin: err = %i", err);
		return err;
	}

	// TODO: add pin interrupt to monitor and action POK state changes
    err = gpio_pin_configure(gpio, EXT_REG_STATE_PIN, EXT_REG_STATE_FLAGS);
	if (err)
	{
		LOG_ERR("Unable to configure external regulator's status pin: err = %i", err);
		return err;
	}

	is_external_reg_init = true;

	return 0;
}

/**
 * @brief Initializes the external regulator
 * 
 * @param vout_v target output voltage (2.60V to 5.14V)
 * 
 * @return does not return any
 */ 
int power_reg_external_power_on(float vout_v)
{
	int err = power_reg_init();
	if (err != 0)
	{
		LOG_ERR("Unable to initialize external power regulator -> err = %i", err);
		return err;
	}

	// External Regulator's GPIO
	gpio = device_get_binding(GPIO);
	// turn the external regulator on
	power_reg_external_set_en_pin(true);

	power_reg_set_i2c_bus(I2C1_PINS_UTIL);

	i2c= device_get_binding(I2C);
	
	if (vout_v < MAX77816_VOUT_MIN_V)
	{
		vout_v = MAX77816_VOUT_MIN_V;	
	} 
	else if (vout_v > MAX77816_VOUT_MAX_V)
	{
		vout_v = MAX77816_VOUT_MAX_V;	
	}

	// vout offset as percentage full scale of vout range
	float temp_v = (vout_v - MAX77816_VOUT_MIN_V) / MAX77816_VOUT_RANGE_V;
	uint8_t vout_byte_val = (uint8_t)(temp_v * MAX77816_VOUT_RESOLUTION);


	// device needs 800us after EN
	k_msleep(1);

	err = i2c_reg_write_byte(i2c, MAX77816_ADDR, MAX77816_REG_VOUT, vout_byte_val);
	if (err != 0)
	{
		LOG_ERR("Unable to set VOUT to %i", vout_byte_val);
		return -EXDEV;
	}

	// reset to SENS bus
	power_reg_set_i2c_bus(I2C1_PINS_SENS);	
	
	// wait for the regulator POK indicator GPIO to go high (>92.5%V)
	int timeout_ms = 2000;
	while (gpio_pin_get(gpio, EXT_REG_STATE_PIN) == 0 && --timeout_ms > 0)
	{
		k_msleep(1);
	}

	if (timeout_ms <= 0)
	{
		LOG_ERR("External power regulator failed to set POK pin!");
		return -EXDEV;
	}

	// not sure why we output this warning, let's remove it
	// LOG_WRN("External regulator is POK at %.01fV", vout_v);
	return 0;
}

void power_reg_external_power_off(void)
{
 	// turn the external regulator on
	power_reg_external_set_en_pin(false); 
}

// /**
//  * @brief Converts the value from ILIM register to corresponding milli-ampere value
//  * 
//  * @param ilim_reg_val value of the register
//  * 
//  * @return Returns the calculated current in mA unit
//  */
static uint16_t power_reg_convert_adc_batt_to_batt_mV(uint8_t adc_batt_val)
{
	uint16_t batt_mV = 0;

	batt_mV = (adc_batt_val * MP273_BATT_MV_PER_BIT) + MP273_BATT_OFFSET_MV;

	return batt_mV;
}

	int power_reg_get_power_status(struct tele_message *tele_data){

		uint8_t status_reg_buff = 0 ;
		uint8_t chrg_stat, fault_reg_buff, timer_reg_buff;
		uint8_t power_status, batt_ovp, ntc_fault, ntc_float, input_ovp, timer_fault, reset_timer, batt_adc_buff;
		int err;

	if(is_charger_init == false){

		power_reg_charger_init(MP273_INPUT_CURRENT_LIMIT_DEFAULT_VAL_MA);
	}

	power_reg_set_i2c_bus(I2C1_PINS_UTIL);

	k_msleep(10);

	i2c= device_get_binding(I2C);

	// update watchdog timer
	reset_timer = MP2731_WDT_RESET;

	err = i2c_reg_update_byte(i2c, MP273_ADDR, MP2731_TIMER_REG, MP2731_WDT_TIMER_RESET_MSK, reset_timer);
	if (err != 0)
    {
        LOG_ERR("Unable to update WDT timer");
        return err;
    };

	err = i2c_reg_read_byte(i2c, MP273_ADDR, MP2731_SAFETY_TIMER_REG, &timer_reg_buff);
	if (err != 0)
    {
        LOG_ERR("Unable to read timer register");
        return err;
    };

	err = i2c_reg_read_byte(i2c, MP273_ADDR, MP2731_FAULT_REG, &fault_reg_buff);
    if (err != 0)
    {
        LOG_ERR("Unable to fault registers");
        return err;
    };

	LOG_DBG(" fault reg : %d", fault_reg_buff);

	err = i2c_reg_read_byte(i2c, MP273_ADDR, MP273_INT_STATUS, &status_reg_buff);
    if (err != 0)
    {
        LOG_ERR("Unable to read status registers");
        return err;
    };

	LOG_DBG(" fault reg : %d", status_reg_buff);

	err = i2c_reg_read_byte(i2c, MP273_ADDR, MP273_BATT_VOLT_ADC, &batt_adc_buff);
    if (err != 0)
    {
        LOG_ERR("Unable to read adc batt registers");
        return err;
    };

	power_reg_set_i2c_bus(I2C1_PINS_SENS);

	chrg_stat = (status_reg_buff & MP2731_CHRG_STAT_MSK) >> 3;
	
	power_status = (status_reg_buff & MP273_POWER_STATUS_MSK) >> 5;

	input_ovp = (fault_reg_buff & MP2731_INPUT_OVP_MSK) >> 5;

	batt_ovp = (fault_reg_buff & MP2731_BATT_OVP_MSK) >> 3 ;

	ntc_fault = (fault_reg_buff & MP2731_NTC_FAULT_MSK);

	ntc_float = (status_reg_buff & MP2731_NTC_FLOAT_MSK) >> 2;

	timer_fault = (timer_reg_buff & MP2731_TIMER_FAULT_MSK) >> 7;
	
	if(power_status == 0){
		tele_data->power_status = MP2731_NO_POWER_SRC_DETECTED;
		tele_data->chrg_stat = MP2731_NOT_CHARGING;
	}
	else{
		tele_data->power_status  = MP2731_GOOD_POWER_SRC_DETECTED;
	}

	if (tele_data->power_status == MP2731_GOOD_POWER_SRC_DETECTED && (chrg_stat == MP2731_TRICKLE_CHARGE || chrg_stat == MP2731_CONST_CURR_CHARGE))
	{
		tele_data->chrg_stat = MP2731_CHARGING;
	}
	else if (tele_data->power_status == MP2731_GOOD_POWER_SRC_DETECTED && (chrg_stat == MP2731_CHARGE_COMPLETE || input_ovp == MP2731_INPUT_OVP_FAULT) ){

		tele_data->chrg_stat = MP2731_NOT_CHARGING;
	}
	// system ovp also causes the charging to be suspended, but cpuldnt find a register where to read this value
	else if (tele_data->power_status == MP2731_GOOD_POWER_SRC_DETECTED && (batt_ovp == MP2731_BATT_OVP_FAULT || timer_fault == MP2731_TIMER_FAULT || ntc_fault != 0 || ntc_float == 1 )){

		tele_data->chrg_stat = MP2731_SUSPEND_CHARGING;
	}
	tele_data->batt_mV = power_reg_convert_adc_batt_to_batt_mV(batt_adc_buff);

	return 0;	
}

// /**
//  * @brief Converts the value from ILIM register to corresponding milli-ampere value
//  * 
//  * @param ilim_reg_val value of the register
//  * 
//  * @return Returns the calculated current in mA unit
//  */
static uint16_t power_reg_convert_iinlim_to_ma(uint8_t ilim_reg_val)
{
	uint16_t current_ma = 0;

	ilim_reg_val &= MP273_REG_ILIM_IINLIM_MASK;

	current_ma = (ilim_reg_val * MP273_IINLIM_MA_PER_BIT) + MP273_IINLIM_OFFSET_MA;

	return current_ma;
}

// TODO: add battery charger on/off control, voltage/current read, etc. via shell
/**
 * @brief Initializes the charger
 * 
 * @param input_current_limit_ma target input current limit
 * 
 * @return returns 0 if it is a success
 */
int power_reg_charger_init(uint16_t input_current_limit_ma)
{
	LOG_INF("Initializing the external sensor regulator");

	uint8_t target_val = (input_current_limit_ma - MP273_IINLIM_OFFSET_MA) / MP273_IINLIM_MA_PER_BIT;

	// need UTIL bus to talk to charger
	power_reg_set_i2c_bus(I2C1_PINS_UTIL);	

	k_msleep(10);

	i2c= device_get_binding(I2C);
	
	int err = 0;
	uint8_t val = 0;

	err = i2c_reg_read_byte(i2c, MP273_ADDR, MP273_REG_OTP, &val);
	if (err)
	{
		LOG_ERR("Error reading the charger's register: err = %i", err);
		return err;
	}

	val &= MP273_REG_OTP_ADDR_MASK;

	if (val != 0)
	{
		LOG_ERR("Did not received correct response from the charger -> 0x%02x", val);
		return -EIO;
	}

	val = 0;
	// ILIM reg default val
	err = i2c_reg_read_byte(i2c, MP273_ADDR, MP273_REG_ILIM, &val);
	if (err != 0)
	{
		LOG_ERR("Error reading the charger's register: err = %i", err);
		return err;
	}

	uint16_t current_ma = power_reg_convert_iinlim_to_ma(val);
	LOG_DBG("Old current limit = %d", current_ma);

	// updating the ILIM reg
	err = i2c_reg_update_byte(i2c, MP273_ADDR, MP273_REG_ILIM, MP273_REG_ILIM_IINLIM_MASK, target_val);
	if (err != 0)
	{
		LOG_ERR("Error updating the charger's current limit register: target_val = %i, err = %i", target_val, err);
		return err;
	}

	val = 0;
	err = i2c_reg_read_byte(i2c, MP273_ADDR, MP273_REG_ILIM, &val);
	if (err != 0)
	{
		LOG_ERR("Error reading the charger's register: err = %i", err);
		return err;
	}

	current_ma = power_reg_convert_iinlim_to_ma(val);
	if (current_ma != input_current_limit_ma)
	{
		LOG_ERR("Unable to set the target input current limit: target_val = %i, input_current_limit_ma = %i, current_ma = %i", 
					target_val, input_current_limit_ma, current_ma);
		return -EIO;
	}
	
	LOG_DBG("New current limit = %dmA", current_ma);

	//set ADC of charging
	target_val = MP2731_ADC_START;
	err = i2c_reg_write_byte(i2c, MP273_ADDR, MP273_ADC_CTRL_OTG_CFG, target_val);
	if (err != 0)
	{
		LOG_ERR("Error writing the charger's adc control register: target_val = %i, err = %i", target_val, err);
		return err;
	}

	//set Ichrg current
	target_val = MP2731_NON_STD_CHRGER;
	err = i2c_reg_write_byte(i2c, MP273_ADDR, MP2731_ICHRG_REG, target_val);
	if (err != 0)
	{
		LOG_ERR("Error writing the charge current register: target_val = %i, err = %i", target_val, err);
		return err;
	}	

	//set wdt timer
	target_val = MP2731_WDT_TIMER_SET;
	err = i2c_reg_update_byte(i2c, MP273_ADDR, MP2731_TIMER_REG, MP2731_TIMER_SET_MSK, target_val);
	if (err != 0)
	{
		LOG_ERR("Error updating the charger's current limit register: target_val = %i, err = %i", target_val, err);
		return err;
	}

	
	// return to default SENS bus
	power_reg_set_i2c_bus(I2C1_PINS_SENS);
	
	is_charger_init = true;

	return 0;
}

