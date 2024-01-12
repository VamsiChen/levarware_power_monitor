/**\
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights rsvd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#ifndef _BMA2_ZEPHYR_H
#define _BMA2_ZEPHYR_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr.h>
#include "drivers/gpio.h"
#include <stdio.h>
#include "bma2.h"


typedef enum bma2_int_pins 
{
    BMA2_INT1,
    BMA2_INT2,
    BMA2_INTALL
} bma2_int_pins_t;

typedef union // 32 bit union with bit fields for bits-by-name
{
    // uint8_t * 4 == sizeof(uint32_t)
    struct bma2_int_status int_status;
    
    // uint32_t * 1 == sizeof(uint32_t)
    uint32_t int_status_u32;
    
    // bits * 32 == sizeof(uint32_t)
    // int_status.int_status_0
    uint32_t low_g : 1;     
    uint32_t high_g : 1;             
    uint32_t slope : 1;         
    uint32_t slow_no_motion : 1;
    uint32_t double_tap : 1;           
    uint32_t single_tap : 1;                    
    uint32_t orientation : 1;                   
    uint32_t flat : 1;
    
    // int_status.int_status_1
    uint32_t rsvd_bit_8 : 1;                          
    uint32_t rsvd_bit_9 : 1;    
    uint32_t rsvd_bit_10 : 1;    
    uint32_t rsvd_bit_11 : 1;   
    uint32_t rsvd_bit_12 : 1;    
    uint32_t fifo_full : 1;                     
    uint32_t fifo_wm : 1;
    uint32_t data_ready : 1;                           

    // int_status.int_status_2                
    uint32_t slope_x : 1;                       
    uint32_t slope_y : 1;                       
    uint32_t slope_z : 1;                       
    uint32_t slope_sign : 1;    
    uint32_t tap_x : 1;                         
    uint32_t tap_y : 1;                         
    uint32_t tap_z : 1;                         
    uint32_t tap_sign : 1;

    // int_status.int_status_3                  
    uint32_t high_x : 1;                        
    uint32_t high_y : 1;                        
    uint32_t high_z : 1;                        
    uint32_t high_sign : 1;   
    uint32_t rsvd_bit_28 : 1;    
    uint32_t rsvd_bit_29 : 1;    
    uint32_t rsvd_bit_30 : 1;    
    uint32_t flat_position : 1;                 
} bma2_interrupt_status_regs_t;

typedef struct bma2_int_event_data
{
    int64_t ts;
    bma2_interrupt_status_regs_t int_status_regs;
    bma2_int_pins_t int_pin;
} bma2_int_event_data_t;

typedef struct bma2_int_pin_config
{
    // gpio details
    gpio_pin_t gpio_pin;
    // GPIO flags to set enable/disable and trigger edges and/or levels
    gpio_flags_t gpio_int_trigger_flags;
    bma2_interrupt_status_regs_t bma_enabled_ints;
} bma2_int_pin_config_t;

typedef struct bma2_device_data 
{
    // isr_work is used to get context and int1_cfg and int2_cfg are written by isr before k_work_submit
    bma2_int_pin_config_t int1_cfg;
    bma2_int_pin_config_t int2_cfg;
} bma2_device_data_t;

typedef struct bma2_isr_data 
{
    // gpio_cb is the ISR, but is also used with CONTAINER_OF to get this context
    struct gpio_callback gpio_cb;
    struct k_work isr_work;
    bma2_int_event_data_t evt_data;
    bma2_interrupt_status_regs_t regs;
} bma2_isr_data_t;

/***************************************************************************/

/*!                 User function prototypes
 ****************************************************************************/

struct bma2_dev *bma2_get_dev(void);
void bma2_set_int_status_msgq(struct k_msgq *msgq);
void bma2_set_int_trigger_flags(bma2_int_pins_t int_pin, gpio_flags_t flags);

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data      : Pointer to data buffer where read data is stored.
 * @param[in] length         : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 *  @return Status of execution
 *
 *  @retval BMA2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMA2_INTF_RET_SUCCESS -> Fail.
 *
 */
BMA2_INTF_RET_TYPE bma2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data      : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] length        : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 *  @return Status of execution
 *
 *  @retval BMA2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMA2_INTF_RET_SUCCESS -> Failure.
 *
 */
BMA2_INTF_RET_TYPE bma2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 * @brief This function provides the delay for required time (Microsecond) as per the input provided in some of the
 * APIs.
 *
 *  @param[in] period_us      : The required wait time in microsecond.
 *  @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *  @return void.
 *
 */
void bma2_delay_us(uint32_t period_us, void *intf_ptr);

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @return pointer to internal bma2_dev struct for compatibility with higher level api calls
 */
struct bma2_dev *bma2_interface_init(void);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bma2_error_codes_print_result(const char api_name[], int8_t rslt);

/*!
 * @brief This function deinitializes the interface
 *
 *  @return void.
 *
 */
void bma2_interface_deinit(void);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _BMA2_ZEPHYR_H */
