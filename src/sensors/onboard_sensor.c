/**
 * 
*/

#include <zephyr.h>
#include <math.h>
#include <date_time.h>
#include <sys/base64.h>

/*Sensor API*/
#include <device.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>

/*Logging*/
#include <logging/log.h>
LOG_MODULE_REGISTER(onboard_sensor);

#include "apps.h"
#include "onboard_sensor.h"
#include "drivers/sensor/bme688/bme688.h"
#include "drivers/sensor/bma253/bma2.h"
#include "drivers/sensor/bma253/bma2_zephyr.h"

#include <devicetree.h>
#include <drivers/gpio.h>

#include "subsys/led.h"
#include "cloud.h"


const struct device *bme = NULL;
// const struct device *bma = NULL;

bool tphg_init = false;
bool accel_init = false;

/**
 * @brief Initializes the on-board sensors
 * 
 * @return Does not return any
 */
void onboard_sensor_init()
{
    onboard_sensor_tphg_init();
    onboard_sensor_accel_init();
}

/**
 * @brief Initializes the sensor for temperature, pressure, humidity and gas
 * 
 * @return does not return any
 */
void onboard_sensor_tphg_init(void)
{
    LOG_INF("Initializing temp, pressure, humidity, and gas sensor");
    bme = device_get_binding(DT_LABEL(DT_INST(0, bosch_bme688)));

    tphg_init = true;
}

/**
 * @brief Initializes the accelerometer API
 * 
 * @return Does not return any
 */
void onboard_sensor_accel_init()
{
    LOG_INF("Initializing accelerometer");
    
    struct bma2_dev *dev = bma2_interface_init();
    if (dev)
    {
        int8_t rslt = 0;
        rslt = bma2_init(dev);
        bma2_error_codes_print_result("bma2_init", rslt);
        printk("Chip id : 0x%x\n", dev->chip_id);

        accel_init = true;
    }
    else
    {
        LOG_ERR("ERROR initializing accelerometer!");
    }
}

/**
 * @brief Updates the temperature readout
 * 
 * @param temp value being updated
 * 
 * @return returns 0 if it is a success
 */
int onboard_sensor_get_temp_c(float *temp)
{
    if (sensor_sample_fetch(bme) != 0)
    {
        LOG_ERR("Unable to update bme's data");
        return -1;
    }

    struct sensor_value temp_c;

    if (sensor_channel_get(bme, SENSOR_CHAN_AMBIENT_TEMP, &temp_c) != 0)
    {
        LOG_ERR("Unable to get temperature");
        return -1;  
    }
    
    *temp = (float)sensor_value_to_double(&temp_c);

    return 0;
}

/**
 * @brief Updates the pressure readout
 * 
 * @param pressure value being updated
 * 
 * @return returns 0 if it is a success
 */
int onboard_sensor_get_pressure_pa(float *pressure)
{
    if (sensor_sample_fetch(bme) != 0)
    {
        LOG_ERR("Unable to update bme's data");
        return -1;
    }

    struct sensor_value press_pa;

    if (sensor_channel_get(bme, SENSOR_CHAN_PRESS, &press_pa) != 0)
    {
        LOG_ERR("Unable to get pressure");
        return -1;  
    }

    *pressure = (float)sensor_value_to_double(&press_pa);

    return 0;
}

/**
 * @brief Updates the humidity readout
 * 
 * @param humidity value being updated
 * 
 * @return returns 0 if it is a success
 */
int onboard_sensor_get_humidity_rh(float *humidity)
{
    if (sensor_sample_fetch(bme) != 0)
    {
         
        return -1;
    }

    struct sensor_value humidity_rh;

    if (sensor_channel_get(bme, SENSOR_CHAN_HUMIDITY, &humidity_rh) != 0)
    {
        LOG_ERR("Unable to get humidity");
        return -1;  
    }

    *humidity = sensor_value_to_double(&humidity_rh);

    return 0;
}

/**
 * @brief Updates the gas resistance value
 * 
 * @param gas value being updated
 * 
 * @return returns 0 if it is a success
 */
int onboard_sensor_get_gas_res_ohm(float *gas)
{
    if (sensor_sample_fetch(bme) != 0)
    {
        LOG_ERR("Unable to update bme's data");
        return -1;
    }

    struct sensor_value gas_res_ohm;

    if (sensor_channel_get(bme, SENSOR_CHAN_GAS_RES, &gas_res_ohm) != 0)
    {
        LOG_ERR("Unable to get gas");
        return -1;  
    }

    *gas = (float)sensor_value_to_double(&gas_res_ohm);

    return 0;
}

// /**
//  * @brief Gets the latest accelerometer data and calculates the angle if requested
//  * 
//  * @param angle_deg value being updated after calculating the angle - can be null
//  * @param xyz_arr array of 3 doubles to hold accel values - can be null
//  * 
//  * @return Returns 0 if it is a success, -1 if error accessing device
//  */
// int onboard_sensor_accel_get_sample(double *xyz_arr, float *angle_deg)
// {
//     struct sensor_value accel[3];
//     double accel_val[3];

//     if (sensor_sample_fetch(bma) != 0)
//     {
//         LOG_ERR("Unable to update bma's data");
//         return -1;      
//     }

//     if (sensor_channel_get(bma, SENSOR_CHAN_ACCEL_XYZ, accel) != 0)
//     {
//         LOG_ERR("Unable to get acceleration");
//         return -1;
//     }

//     for (int i = 0; i < 3; i++)
//     {
//         accel_val[i] = sensor_value_to_double(&accel[i]);
//         if (xyz_arr)
//         {
//             xyz_arr[i] = accel_val[i];
//         }
//     }

//     if (angle_deg)
//     {
//         double resultant_rad = sqrt(pow(accel_val[0], 2) + pow(accel_val[1], 2) + pow(accel_val[2], 2));
//         *angle_deg = -1;

//         if (resultant_rad != 0)
//         {
//             double radian_to_vertical = acos(accel_val[2] / resultant_rad);
//             *angle_deg = radian_to_vertical * 180 / 3.141592;
//         }
//     }

//     return 0;
// }

bool onboard_sensor_tphg_is_sleeping()
{
    bool is_sleeping = false;

    bme = device_get_binding(DT_LABEL(DT_INST(0, bosch_bme688)));

    const struct bme688_config *config = bme->config;

    uint8_t data;
    if (i2c_reg_read_byte(config->bus.bus, config->bus.addr, BME688_REG_CTRL_MEAS, &data) < 0) //0x76
    {
        LOG_ERR("Unable to read BME688 CTRL MEAS Reg");
    }
    else
    {
        data &= 0x03;

        if (data == 0)
        {
            is_sleeping = true;
        }
    }

    return is_sleeping;
}

// TODO: use the macro from the driver for the param
int onboard_sensor_tphg_set_mode(uint8_t mode)
{
    bme = device_get_binding(DT_LABEL(DT_INST(0, bosch_bme688)));

    const struct bme688_config *config = bme->config;

    int err = 0;

    if (i2c_reg_update_byte(config->bus.bus, config->bus.addr, BME688_REG_CTRL_MEAS,  0x03, mode) > 0 )
    {
        LOG_ERR("Unable to set BME688 sensor mode to %d", mode);
    }

    return err;
}

// bool onboard_sensor_accel_is_sleeping()
// {
//     bool is_sleeping = false;

//     bma = device_get_binding(DT_LABEL(DT_INST(0, bosch_bma280)));

//     const struct bme688_config *config = bme->config;

//     uint8_t data;
//     if (i2c_reg_read_byte(config->bus.bus, config->bus.addr, 0x12, &data) < 0) //0x76
//     {
//         LOG_ERR("Unable to read BME688 CTRL MEAS Reg");
//     }
//     else
//     {
//         data &= 0x40;

//         if (data == 0)
//         {
//             is_sleeping = true;
//         }
//     }

//     return is_sleeping;
// }

// int onboard_sensor_accel_set_mode()
// {
//     int err = 0;
//     bma = device_get_binding(DT_LABEL(DT_INST(0, bosch_bma280)));

//     const struct bme688_config *config = bme->config;

//     if (i2c_reg_update_byte(config->bus.bus, config->bus.addr, 0x11,  0x80, 1) > 0 )
//     {
//         LOG_ERR("Unable to set BME688 sensor mode to %d", 1);
//     }

//     // uint8_t data;
//     // if (i2c_reg_read_byte(config->bus.bus, config->bus.addr, 0x11, &data) < 0) //0x76
//     // {
//     //     LOG_ERR("Unable to read BME688 CTRL MEAS Reg");
//     // }

//     return err;
// }

// static void onboard_sensor_accel_motion_settle_timer_handler(void)
// {

// }

// function prototype for evt_timer_expire_handler
// static void onboard_sensor_accel_send_event(bool event_end);

// static void evt_timer_expire_handler(struct k_timer *dummy)
// {
//     // send event end data message
//     onboard_sensor_accel_send_event(true);
// }

// static K_TIMER_DEFINE(accel_event_end_timer, evt_timer_expire_handler, NULL);

// static void onboard_sensor_accel_send_event(bool event_end)
// {
//     // static to retain values between calls
//     static apps_evt_data_t evt_data = {
//         .start_ts = 0,
//         .end_ts = 0,
//         .source = "accel"
//     };

//     double accel_xyz[3];
//     float angle_deg = 0;

//     int err = onboard_sensor_accel_get_sample(&accel_xyz[0], &angle_deg);
//     if (err)
//     {
//         LOG_ERR("error getting accelerometer data: err = %i", err);
//         return;
//     }
    
//     apps_evt_accel_data_t evt_accel_data = {
//         .ts = 0,
//         .accel_x = accel_xyz[0],
//         .accel_y = accel_xyz[3],
//         .accel_z = accel_xyz[2],
//         .angle_deg = angle_deg
//     };

//     date_time_now(&evt_accel_data.ts);

//     // start_ts of zero indicates this is a new event
//     if (evt_data.start_ts == 0)
//     {
//        evt_data.start_ts = evt_accel_data.ts;
//        evt_data.end_ts = 0;
//     }
    
//     // event_end is set from timer expiry handler -> update end_ts to mark end of event
//     if (event_end)
//     {
//        evt_data.end_ts = evt_accel_data.ts;
//        // don't reset start_ts here - the telemetry backend needs start_ts to be set with end_ts to know which event is ending 
//        // start_ts is the telemetry key that links each event including the event_end
//     }

//     apps_evt_send_accel_event(&evt_data, &evt_accel_data);

//     // reset start/end timestamps for next event
//     if (event_end)
//     {
//        evt_data.start_ts = 0;
//        evt_data.end_ts = 0;
//     }
//     else
//     {
//         // start/restart one-shot event_end timer to send event_end message on expiry
//         // info: timer created with K_TIMER_DEFINE above
//         // TODO: add event end timeout to device config or a new config topic sub
//         k_timer_start(&accel_event_end_timer, K_MSEC(2000), K_NO_WAIT);
//     }
// }

// /**
//  * @brief Sensor callback handler. Called from sensor thread, so I2C safe
//  *
//  * @param dev Pointer to the device struct
//  * @param sensor_trigger Pointer to the sensor_trigger struct
//  * 
//  * @return Does not return anything
//  */
// static void onboard_sensor_accel_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
// {
//     onboard_sensor_accel_send_event(false);

//     return;
// }

// void onboard_sensor_accel_trigger_set(void)
// {
//     if (!accel_init)
//     {
//         LOG_ERR("Unable to set accel trigger: device not initialized");
//         return;
//     }

// 	int rc = 0;

//     struct sensor_value val;     
    
//     /** BMA253
//      * slope_th<7:0> · 3.91 mg (2-g range)
//      * slope_th<7:0> · 7.81 mg (4-g range)
//      * slope_th<7:0> · 15.63 mg (8-g range) 
//      * slope_th<7:0> · 31.25 mg (16-g range) 
//      * 
//      * 20 = ~78mg
//      */
//     sensor_value_from_double(&val, 20.0);
//     rc = sensor_attr_set(bma, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &val);
//     if (rc)
//     {
//         LOG_ERR("Error %i from sensor_attr_set for SENSOR_ATTR_SLOPE_TH", rc);
//         return;
//     }

//     /** BMA253
//      * slope_dur<1:0>: slope interrupt triggers if [slope_dur<1:0>+1] consecutive 
//      * slope data points are above the slope interrupt threshold slope_th<7:0> 
//      * 
//      * 1 = (1+1) * 15.625ms = 31.25ms
//      */
//     sensor_value_from_double(&val, 1.0);
//     rc = sensor_attr_set(bma, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &val);
//     if (rc)
//     {
//         LOG_ERR("Error %i from sensor_attr_set for SENSOR_ATTR_SLOPE_DUR", rc);
//         return;
//     }
    
//     struct sensor_trigger trig;
//     trig.type = SENSOR_TRIG_DELTA;
//     trig.chan = SENSOR_CHAN_ACCEL_XYZ;

//     rc = sensor_trigger_set(bma, &trig, onboard_sensor_accel_trigger_handler);
//     if(rc)
//     {
//         LOG_ERR("trigger set error: %i", rc);
//     }

//     trig.type = SENSOR_TRIG_DELTA;
//     trig.chan = SENSOR_CHAN_ACCEL_XYZ;

//     rc = sensor_trigger_set(bma, &trig, onboard_sensor_accel_trigger_handler);
//     if(rc)
//     {
//         LOG_ERR("trigger set error: %i", rc);
//     }
// }
