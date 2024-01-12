/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bma2_zephyr.h"
#include "bma2_defs.h"

/******************************************************************************/
/*!                Static variable definition                                 */

#define I2C     	DT_LABEL(DT_NODELABEL(i2c1))
#define GPIO        DT_LABEL(DT_NODELABEL(gpio0))

const uint16_t i2c_dev_addr = DT_PROP(DT_NODELABEL(bma253), reg);
const struct device *i2c_dev = NULL;
const struct device *gpio_dev = NULL;

const struct gpio_dt_spec gpio_spec_int1 = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = DT_GPIO_PIN(DT_NODELABEL(bma2_int1), gpios),
	.dt_flags = DT_GPIO_FLAGS(DT_NODELABEL(bma2_int1), gpios)
};

const struct gpio_dt_spec gpio_spec_int2 = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = DT_GPIO_PIN(DT_NODELABEL(bma2_int2), gpios),
	.dt_flags = DT_GPIO_FLAGS(DT_NODELABEL(bma2_int2), gpios)
};



/*! Variable that holds the I2C device address or SPI chip selection */
static struct bma2_dev bma2_dev = { 0 };

static bma2_device_data_t bma2_device_data = { 0 };
static bma2_isr_data_t bma2_isr_data_int1 = { 0 };
static bma2_isr_data_t bma2_isr_data_int2 = { 0 };



// This will create and init the high speed telemetry message 
K_MSGQ_DEFINE(bma2_int_event_data_msgq, sizeof(bma2_int_event_data_t), 10, 4);

/******************************************************************************/
/*!                User interface functions                                   */

struct bma2_dev *bma2_get_dev(void)
{
    if (bma2_dev.intf_ptr)
    {
        return &bma2_dev;
    }

    return NULL;
}

/*!
 * I2C read function map to Zephyr platform
 */
BMA2_INTF_RET_TYPE bma2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    struct device *dev = (struct device *)intf_ptr;

    return i2c_burst_read(dev, i2c_dev_addr, reg_addr, reg_data, length);
}

/*!
 * I2C write function map to Zephyr platform
 */
BMA2_INTF_RET_TYPE bma2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    struct device *dev = (struct device *)intf_ptr;

    return i2c_burst_write(dev, i2c_dev_addr, reg_addr, reg_data, length);
}

/*!
 * Delay function map to Zephyr platform
 */
void bma2_delay_us(uint32_t period, void *intf_ptr)
{
    k_usleep(period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bma2_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMA2_OK)
    {
        printk("API name %s\t", api_name);

        if (rslt == BMA2_E_NULL_PTR)
        {
            printk("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMA2_E_DEV_NOT_FOUND)
        {
            printk("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMA2_E_COM_FAIL)
        {
            printk("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMA2_E_INVALID_POWERMODE)
        {
            printk("Error [%d] : Invalid powermode configuration\r\n", rslt);
        }
        else if (rslt == BMA2_E_RW_LEN_INVALID)
        {
            printk("Error [%d] : Invalid read write length\r\n", rslt);
        }
        else if (rslt == BMA2_E_INVALID_CONFIG)
        {
            printk("Error [%d] : Invalid configuration\r\n", rslt);
        }
        else if (rslt == BMA2_W_NO_NEW_AVAILABLE)
        {
            printk("Warning [%d] : Non-availability of data\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printk("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/******************************************************************************/
/*!                     INT1/INT2 functions                                   */

void bma2_set_int_trigger_flags(bma2_int_pins_t int_pin, gpio_flags_t flags)
{
    bma2_int_pin_config_t *int_cfg;

    switch (int_pin)
    {
        case BMA2_INT1:
            int_cfg = &bma2_device_data.int1_cfg;
            break;
        case BMA2_INT2:
            int_cfg = &bma2_device_data.int2_cfg;
            break;
        default:
            return;
    }

    int_cfg->gpio_int_trigger_flags = flags;

    gpio_pin_interrupt_configure(gpio_dev, int_cfg->gpio_pin, int_cfg->gpio_int_trigger_flags);
}

static void bma2_gpio_callback_int1(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	bma2_isr_data_t *isr_data = CONTAINER_OF(cb, bma2_isr_data_t, gpio_cb);

    isr_data->evt_data.int_pin = BMA2_INT1;
    // disable interrupt
    gpio_pin_interrupt_configure(gpio_dev, bma2_device_data.int1_cfg.gpio_pin, GPIO_INT_DISABLE);
    
    isr_data->evt_data.ts = k_uptime_get();

    // we're in an ISR - do the work in main thread via worker and a msgq for now - to be improved
    // important to ensure interrupt is re-enabled
    k_work_submit(&isr_data->isr_work);
}

static void bma2_gpio_callback_int2(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	bma2_isr_data_t *isr_data = CONTAINER_OF(cb, bma2_isr_data_t, gpio_cb);

    isr_data->evt_data.int_pin = BMA2_INT2;
    // disable interrupt
    gpio_pin_interrupt_configure(gpio_dev, bma2_device_data.int2_cfg.gpio_pin, GPIO_INT_DISABLE);
    
    isr_data->evt_data.ts = k_uptime_get();

    // we're in an ISR - do the work in main thread via worker and a msgq for now - to be improved
    // important to ensure interrupt is re-enabled
    k_work_submit(&isr_data->isr_work);
}

static void bma2_get_int_status_and_send_event_to_msgq(bma2_isr_data_t *isr_data)
{
    // I2C should be safe since this is running in global worker thread
    int8_t rslt = bma2_get_int_status(&isr_data->evt_data.int_status_regs.int_status, &bma2_dev);
    bma2_error_codes_print_result("bma2_get_int_status", rslt);

    printk("%lld -> NEW int_event_data on INT%i ...\r\n", isr_data->evt_data.ts, isr_data->evt_data.int_pin+1);
    int err = k_msgq_put(&bma2_int_event_data_msgq, &isr_data->evt_data, K_NO_WAIT);
    if (err)
    {
        // queue likely full nothing much to do so print to console
        printk("bma2_int_work_cb: bma2_int_event_data_msgq FULL -> int_status_[0..3] = [ 0x%02x, 0x%02x, 0x%02x, 0x%02x ]\r\n",
                isr_data->evt_data.int_status_regs.int_status.int_status_0, isr_data->evt_data.int_status_regs.int_status.int_status_1, 
                isr_data->evt_data.int_status_regs.int_status.int_status_2, isr_data->evt_data.int_status_regs.int_status.int_status_3);
    }
}

static void bma2_int_work_cb_int1(struct k_work *work)
{
	bma2_isr_data_t *isr_data = CONTAINER_OF(work, bma2_isr_data_t, isr_work);
	
    bma2_get_int_status_and_send_event_to_msgq(isr_data);

    // work done, re-enable interrupt
    gpio_pin_interrupt_configure(gpio_dev, bma2_device_data.int1_cfg.gpio_pin, bma2_device_data.int1_cfg.gpio_int_trigger_flags);
}

// two work functions to have separate contexts between the isr executions this avoids sync and interrupt source checking 
// this avoids sync and source checking and is manageable since there will only ever be 2 int pins 
static void bma2_int_work_cb_int2(struct k_work *work)
{
	bma2_isr_data_t *isr_data = CONTAINER_OF(work, bma2_isr_data_t, isr_work);
	
    bma2_get_int_status_and_send_event_to_msgq(isr_data);

    // work done, re-enable interrupt
    gpio_pin_interrupt_configure(gpio_dev, bma2_device_data.int2_cfg.gpio_pin, bma2_device_data.int2_cfg.gpio_int_trigger_flags);
}

/*!
 *  @brief Function to initialize the I2C and GPIO interfaces
 */
struct bma2_dev *bma2_interface_init(void)
{
    bma2_dev.chip_id = i2c_dev_addr;
    bma2_dev.read = bma2_i2c_read;
    bma2_dev.write = bma2_i2c_write;
    bma2_dev.delay_us = bma2_delay_us;
    bma2_dev.intf = BMA2_I2C_INTF;

    i2c_dev = device_get_binding(I2C);
    bma2_dev.intf_ptr = (void *)i2c_dev;    

    gpio_dev = device_get_binding(GPIO);
    
    // config gpio for both int pins
    bma2_device_data.int1_cfg.gpio_pin = gpio_spec_int1.pin;
    gpio_pin_configure_dt(&gpio_spec_int1, GPIO_INPUT);
    bma2_device_data.int1_cfg.gpio_int_trigger_flags = 0;
    bma2_device_data.int1_cfg.bma_enabled_ints.int_status_u32 = 0;

    bma2_isr_data_int1.isr_work.handler = bma2_int_work_cb_int1;
    bma2_isr_data_int1.gpio_cb.handler = bma2_gpio_callback_int1;
    gpio_init_callback(&bma2_isr_data_int1.gpio_cb, bma2_gpio_callback_int1, BIT(bma2_device_data.int1_cfg.gpio_pin));
    if (gpio_add_callback(gpio_dev, &bma2_isr_data_int1.gpio_cb) < 0) 
    {
        printk("ERROR: Could not set gpio callback for INT1");
        return NULL;
    }
    
    bma2_device_data.int2_cfg.gpio_pin = gpio_spec_int2.pin;
    gpio_pin_configure_dt(&gpio_spec_int2, GPIO_INPUT);
    bma2_device_data.int2_cfg.gpio_int_trigger_flags = 0;
    bma2_device_data.int2_cfg.bma_enabled_ints.int_status_u32 = 0;
    
    // setup gpio isr callback and isr worker
    bma2_isr_data_int2.isr_work.handler = bma2_int_work_cb_int2;
    bma2_isr_data_int2.gpio_cb.handler = bma2_gpio_callback_int2;
    gpio_init_callback(&bma2_isr_data_int2.gpio_cb, bma2_gpio_callback_int2, BIT(bma2_device_data.int2_cfg.gpio_pin));
    if (gpio_add_callback(gpio_dev, &bma2_isr_data_int2.gpio_cb) < 0) 
    {
        printk("ERROR: Could not set gpio callback for INT2");
        return NULL;
    }

    return &bma2_dev;
}

void bma2_deinit(void)
{
    return;
}
