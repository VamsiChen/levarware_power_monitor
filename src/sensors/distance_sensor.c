#include <zephyr.h>

/*UART*/
#include <device.h>
#include <drivers/uart.h>
#include <sys/crc.h>
#include <stdlib.h>

#include <math.h>

/*Logging*/
#include <logging/log.h>
LOG_MODULE_REGISTER(distance_sensor);

#include "distance_sensor.h"
#include "device_config/device_config.h"
#include "power_mgr/power_reg.h"
#include "drivers/gpio.h"
#include "subsys/modbus_serial.h"

#define UART2 DT_LABEL(DT_NODELABEL(uart2))
#define GPIO DT_LABEL(DT_NODELABEL(gpio0))

static const struct device *uart = NULL;

void distance_sensor_uart_cb(const struct device *dev, void *user_data);
int sensor_laser_get_distance(distance_sensor_uart_user_data_t *ds_data);

/**
 * @brief Initializes and starts the UART for distance sensor measurements
 *
 * @param baudrate
 *        ds_data - pointer to the uart control structure used to handle UART read process 
 *
 * @return uart config error
 */
static int distance_sensor_start_uart(uint32_t baudrate, distance_sensor_uart_user_data_t *ds_data)
{
    struct uart_config cfg;
    int err = 0;

    if (uart == NULL)
    {
        uart = device_get_binding(UART2);

        err = uart_config_get(uart, &cfg);
        if (err != 0)
        {
            LOG_ERR("Unable to get UART2 configurations");
        }

        cfg.baudrate = baudrate;
        cfg.data_bits = UART_CFG_DATA_BITS_8;
        cfg.parity = UART_CFG_PARITY_NONE;
        cfg.stop_bits = UART_CFG_STOP_BITS_1;
        cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

        err = uart_configure(uart, &cfg);
        if (err != 0)
        {
            LOG_ERR("Failed to set UART2 configurations");
        }

        uart_irq_callback_user_data_set(uart, (uart_irq_callback_user_data_t)distance_sensor_uart_cb, ds_data);
        uart_irq_rx_enable(uart);
    }

    return err;
}

/// @brief Evaluate uart chars from the Terabee Lidar sensor
/// @param ds_data pointer to data struct to hold intermediate and final results
/// @todo move to distinct sensor module (terabee_lidar.c/.h)
void uart_rx_isr_eval_terabee(distance_sensor_uart_user_data_t *ds_data, uint8_t rx_data)
{
    if (rx_data == 'T')
    {
        // 'T' is msg start delimiter for terabee
        // evo sensors
        ds_data->uart_rx_data_len = 0;
    }

    ds_data->uart_rx_data[ds_data->uart_rx_data_len++] = rx_data;

    if (ds_data->uart_rx_data_len >= UART2_BUFFER_SIZE)
    {
        ds_data->uart_rx_data_len = 0;
        // no 'T' for a while... strange why this would happen - wrong sensor config or sensor failure perhaps?
        LOG_ERR("rx_buf overflow!");
    }
    ds_data->uart_rx_data_len %= UART2_BUFFER_SIZE;

    // check for message length, crc8 of data, and not
    // samples_done
    if (ds_data->uart_rx_data_len >= TERABEE_EVO_DATA_SIZE 
        && (crc8(ds_data->uart_rx_data, TERABEE_EVO_DATA_SIZE - 1, TERABEE_CRC8_POLYNOMIAL, 0, false) ==  ds_data->uart_rx_data[TERABEE_EVO_DATA_SIZE - 1]) 
        && !ds_data->distance_samples_done)
    {
        // uart_buff[0] === 'T'
        int32_t d = (int32_t)ds_data->uart_rx_data[1] << 8 | ds_data->uart_rx_data[2];

        // check for error conditions
        // (0x01=invalid, 0x00=detection below sensors minimum range,
        // 0xffff=detection beyond sensors maximum range)
        if (d != 0x0001 && d != 0xffff && d != 0x0000)
        {
            // valid reading
            ds_data->distance_buf[ds_data->distance_buf_index++] = d;

            // buffer full, do math
            if (ds_data->distance_buf_index >= DISTANCE_NUM_SAMPLES_AVG)
            {
                ds_data->distance_buf_index = 0;
                ds_data->distance_mm = 0;

                // Average first
                for (int i = 0; i < DISTANCE_NUM_SAMPLES_AVG; i++)
                {
                    ds_data->distance_mm += ds_data->distance_buf[i];
                }

                ds_data->distance_mm /= DISTANCE_NUM_SAMPLES_AVG;
                printk("terabee avg'd distance_mm: %d\n", ds_data->distance_mm);

                ds_data->distance_samples_done = true;
            }
        }
        else
        {
            // invalid - ignore sample
            d = -1;
        }
    }
}

/** @brief Evaluate uart chars from the Maxbotix USS sensor
*  @param ds_data pointer to data struct to hold intermediate and final results
*  @todo move to distinct sensor module (maxbotix_uss.c/.h) 
* @todo - recode this whole thing. The data link framing is all blended with reading the data. The protocol is like SLIP in that the \r from the
* previous frame is the SOF (start of frame) for the next packet. if we just broke down into frames and then interpert the frames, the code would be much cleaner and the 
* buffers smaller ect... 
*/

void uart_rx_isr_eval_maxbotix(distance_sensor_uart_user_data_t *ds_data, uint8_t rx_data)
{

    int32_t reading;        // interval value of one reading
    int i;                  // for loop counter
    int32_t temp_average;

    if (rx_data == 'R')
    {
        // 'R' is msg start delimiter for maxbotix evo sensors
        ds_data->uart_rx_data_len = 0;
     //   LOG_DBG("Start of frame received");
    }

    if ((rx_data >= '0' && rx_data <= '9') || rx_data == '\r')
    {
        ds_data->uart_rx_data[ds_data->uart_rx_data_len++] = rx_data;

        if (ds_data->uart_rx_data_len >= UART2_BUFFER_SIZE)
        {
            ds_data->uart_rx_data_len = 0;
            // no 'packet start char' or no '\r' for a while... strange why this would happen - wrong sensor config or sensor failure perhaps?
          // LOG_DBG("Buffer index has overflowed -> %d", ds_data->uart_rx_data_len);
        }

        // LOG_DBG("RX_ISR - received character %x [hex], length is now: %d", rx_data,  ds_data->uart_rx_data_len);

        /* not sure what the hell we are doing here?? Seems like we are overwriting the current length for the next char if length was too long. 
        * should be delete this line as it does nothing, what am I missing?? 
        * update: I am now assuming that someone thinks they are implementing a circular buffer (which they aren't)
        */ 
        ds_data->uart_rx_data_len = ds_data->uart_rx_data_len % UART2_BUFFER_SIZE;

        // after 'R' there are 4 ascii characters from
        // 0-5000 for 5m sensor and 0-9998 for 10m
        // sensor if 5000 or 9999 then no object
        // detected in fov - no other error
        // conditions are reported

        // do we have all the characters required to get the full value? AND not done collecting samples
        if ((ds_data->uart_rx_data_len >= MAXBOTIX_DATA_SIZE) && (ds_data->uart_rx_data[4] == '\r') && !ds_data->distance_samples_done)
        {
            // get the reading by converting the ascii chars to an integer value
            reading = atoi(&ds_data->uart_rx_data[0]);

            // check for valid integer
            if (reading >= 0 && reading <= 9999)
            {

            //  LOG_DBG("Reading at index: %d, is value: %d", ds_data->distance_buf_index, reading);

                // save the new reading and point to location to store next reading
                ds_data->distance_buf[ds_data->distance_buf_index++] = reading;
      

                // have we got all the  readings required to perform the average?
                if (ds_data->distance_buf_index >= DISTANCE_NUM_SAMPLES_AVG)
                {
                    // yes we do, perform the average of all the readings in the buffer

                    ds_data->distance_buf_index = 0;    // point to the beginning of the buffer for the next set of samples 
                    temp_average = 0;                   // clear the temp value of the average

                    // Here we dont apply deviation filter, just use the raw data. ** note sure what is meant by this (Riley)
                    for (i = 0; i < DISTANCE_NUM_SAMPLES_AVG; i++)
                    {
                        temp_average = temp_average + ds_data->distance_buf[i];
                    }

                    // calculate the average and save it. 
                    ds_data->distance_mm  = temp_average / (DISTANCE_NUM_SAMPLES_AVG);
                    //LOG_DBG("maxbotix avg'd distance_mm: %d\n", ds_data->distance_mm );

                    ds_data->distance_samples_done = true;          // all done getting this set of samples for the average, the thread polls on this flag, 
                                                                    // setting to true will cause the thread to stop polling and read the averaged value. 
                }
            }
            // else - it's out of range so just ignore sample
        }
    }
}

/**
 * @brief UART2 interrupt service procedure
 *
 * @param dev pointer to the device struct of the UART2
 * @param user_data determines which sensor is being used
 *
 * @return Does not return any
 */
void distance_sensor_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t rx_data = 0;

    distance_sensor_uart_user_data_t *ds_data = (distance_sensor_uart_user_data_t *)user_data;
    uart_irq_update(uart);

    while (uart_irq_update(uart) && uart_irq_is_pending(uart)) {

		if (uart_irq_rx_ready(uart) && uart_fifo_read(uart, &rx_data, 1)) {

            switch (ds_data->distance_sensor)
            {
                case EXT_SENSOR_TERABEE:
                    uart_rx_isr_eval_terabee(ds_data, rx_data);
                    break;

                case EXT_SENSOR_MAXBOTIX:
                    uart_rx_isr_eval_maxbotix(ds_data, rx_data);
                    break;

                case EXT_SENSOR_RADAR:
                    // Radar is handled with modbus library
                    break;
                    
                default:
                    break;
            } // end: switch(distance_sensor)
		}

		if (uart_irq_tx_ready(uart)) {
			return;
		}
	}

    // if (uart_irq_rx_ready(dev) && uart_fifo_read(dev, &rx_data, 1))
    // {
    //     switch (ds_data->distance_sensor)
    //     {
    //         case EXT_SENSOR_TERABEE:
    //             uart_rx_isr_eval_terabee(ds_data, rx_data);
    //             break;

    //         case EXT_SENSOR_MAXBOTIX:
    //             uart_rx_isr_eval_maxbotix(ds_data, rx_data);
    //             break;

    //         case EXT_SENSOR_RADAR:
    //             // Radar is handled with modbus library
    //             break;

    //         default:
    //             break;
    //     } // end: switch(distance_sensor)
    // }
}

/// @brief Get distance in mm from the Terabee LiDAR sensor on UART
/// @param ds_data pointer to data struct to hold intermediate and final results
/// @todo move to distinct sensor module (terabee_lidar.c/.h)
int sensor_terabee_get_distance(distance_sensor_uart_user_data_t *ds_data)
{
    // power on external sensor
    int err = power_reg_external_power_on(TERABEE_POWER_REG_VOLTS);
    if (err)
    {
        LOG_ERR("External sensor power on failed -> err = %i", err);
        return err; 
    }
    
    /*
    * Start the UART in interrupt mode to receive characters. The interrupt handler will process the characters to get the reading 
    * and when finished, it will set the flag distance_samples_done. This crappy code then does a delay loop polling that flag. 
    * 
    *  TODO - Redesign this
    */
    err = distance_sensor_start_uart(TERABEE_UART_BAUD_RATE, ds_data);
    if (err)
    {
        LOG_ERR("Terabee sensor configure failed -> err = %i", err);
        return err;
    }
    
    int64_t expire_ms = k_uptime_get() + DISTANCE_MAX_SAMPLE_TIME_MSEC;
    while (k_uptime_get() < expire_ms && !ds_data->distance_samples_done)
    {
        k_msleep(100);
    }

    // uart_irq_rx_disable(uart);
    
    // power off external sensor
    power_reg_external_power_off();

    return err;
}

int get_charge_ic_status(distance_sensor_uart_user_data_t *ds_data){
    
    // int err = power_reg_get_power_status(&ds_data->power_status);
    // if (err)
    // {
    //     LOG_ERR("Power regulator unable to read charger IC -> err = %i", err);
    //     return err; 
    // }

    // power_reg_external_power_off();

    return 0;
}

/// @brief Get distance in mm from the Maxbotix USS sensor on UART
/// @param ds_data pointer to data struct to hold intermediate and final results
/// @todo move to distinct sensor module (maxbotix_uss.c/.h)
int sensor_maxbotox_get_distance(distance_sensor_uart_user_data_t *ds_data)
{
    // power on external sensor
    int err = power_reg_external_power_on(MAXBOTIX_POWER_REG_VOLTS);
    if (err)
    {
        LOG_ERR("External sensor power on failed -> err = %i", err);
        return err; 
    }

    /* 
    * Start the UART in interrupt mode to receive characters. The interrupt handler will process the characters to get the reading 
    * and when finished, it will set the flag distance_samples_done. This crappy code then does a delay loop polling that flag. 
    * 
    *  TODO - Redesign this
    */
    err = distance_sensor_start_uart(MAXBOTIX_UART_BAUDRATE, ds_data);
    if (err)
    {
        LOG_ERR("Maxbotiox sensor configure failed -> err = %i", err);
        return err;
    }

    int64_t expire_ms = k_uptime_get() + DISTANCE_MAX_SAMPLE_TIME_MSEC;
    while (k_uptime_get() < expire_ms && !ds_data->distance_samples_done)
    {
        k_msleep(100);


    }

        // check if we got a valid average set of values
    if (!ds_data->distance_samples_done)
        // it's not done so we didn't get a valid set of samples
        LOG_ERR("Failed to read a valid set of distance samples"); 


    // We can't disable UART because we would never get another reading. 
    //  uart_irq_rx_disable(uart);
    
    // power off external sensor
    power_reg_external_power_off();

    return err;
}
/// @brief Get distance in mm from the RD300S radar sensor on UART using Modbus
/// @param ds_data pointer to data struct to hold intermediate and final results
/// @todo move to distinct sensor module (rd300s_radar.c/.h)
static void sensor_rd300s_get_distance(distance_sensor_uart_user_data_t *ds_data)
{
    #define RD300S_POWER_SETTLE_TIME_MS          1500

    modbus_serial_init_modbus_client(RADAR_UART_BAUDRATE);
    // init external regulator
    power_reg_external_power_on(RADAR_POWER_REG_VOLTS);
    
    // let the sensor settle after power-up
    k_msleep(RD300S_POWER_SETTLE_TIME_MS);

    int32_t distance_avg = 0;
    int32_t avg_cnt = 0;
    uint16_t tmp = 0;
    for(int i = 0; i < DISTANCE_NUM_SAMPLES_AVG; i++)
    {   
        k_msleep(125);
        if (0 == modbus_serial_read_hregs(0x80, 0x02, &tmp, 1)) 
        {
            distance_avg += tmp;
            avg_cnt++;
            LOG_DBG("s%i: %i mm", avg_cnt, tmp);
        }
    }

    if (avg_cnt > 0)
    {
        ds_data->distance_mm = distance_avg / avg_cnt;
        ds_data->distance_samples_done = true;
    }
    
    if (device_config_get_int16(DEV_CONFIG_DAQ_INTERVAL_S) > MINUTE_SECONDS)
    {
        // power off external sensor
        power_reg_external_power_off();
        modbus_serial_disable_modbus_client();
    }

    return;
}

/**
 * @brief Turns on the sensor and reads the sensor data (from the  specific sensor for this hardware platform)
 *
 * @param distance_val pointer to int, gets updated if measurement is a success
 *
 * @return distance data
 * 
 * @note: No error code can be returned
 */
int32_t distance_sensor_get_mm(void)
{
    distance_sensor_uart_user_data_t ds_data = {
        .distance_sensor = device_config_get_int16(DEV_CONFIG_SENSOR_TYPE),
        .distance_mm = 0,
        .distance_buf = { 0 }, 
        .distance_buf_index = 0, 
        .distance_samples_done = false,
        .uart_rx_data = { 0 },
        .uart_rx_data_len = 0,
        .uart_tx_data = { 0 },
        .uart_tx_data_len = 0,
        .power_status = 0
    };

    switch (ds_data.distance_sensor)
    {
        case EXT_SENSOR_TERABEE:
            ds_data.distance_mm = -1; //temporary to avoid starting uart
            // sensor_terabee_get_distance(&ds_data);
            break;

        case EXT_SENSOR_MAXBOTIX:
            sensor_maxbotox_get_distance(&ds_data);
            break;

        case EXT_SENSOR_RADAR:
            sensor_rd300s_get_distance(&ds_data);
            break;
        case POWER_STATUS:
            get_charge_ic_status(&ds_data);
        default:
            break;
    } // end: switch(distance_sensor) 

    if(ds_data.distance_sensor == POWER_STATUS){

        return ds_data.power_status;

    }
    
    if (ds_data.distance_samples_done == false)
    {
        LOG_ERR("Unable to get the distance lol");
        return -1;
    }

    LOG_DBG("mm: %d", ds_data.distance_mm);

    return ds_data.distance_mm;
}
