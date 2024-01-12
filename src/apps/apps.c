#include <sys/printk.h>
#include <string.h>
#include <math.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>

/*Logging*/
#include <logging/log.h>
LOG_MODULE_REGISTER(app);
#include "power_mgr/power_reg.h"

#include "apps.h"
#include "cloud.h"
#include "distance.h"
#include "audio.h"
#include "motion.h"
#include "modem.h"
#include "cs_json.h"
#include "apps/pwr_monitor.h"

#include "device_config/device_config.h"
#include "drivers/sensor/bma253/bma2.h"
#include "drivers/sensor/bma253/bma2_zephyr.h"
#include "sensors/onboard_sensor.h"

#define I2C     	DT_LABEL(DT_NODELABEL(i2c1))
#define GPIO        DT_LABEL(DT_NODELABEL(gpio0))

const uint16_t i2c_chrgr = CHARGER_ADDR;
 const struct device *i2c_dev_chrg;

const struct gpio_dt_spec pwr_chrg_int = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 9,
	.dt_flags = DT_GPIO_FLAGS(DT_NODELABEL(pwr_chrg), gpios)
};

const struct device *pwr_chrg_dev ;
struct gpio_callback pwr_chrg_cb;

K_SEM_DEFINE(apps_evt_sem, 0, 1);

void apps_hs_telem_consumer_entry_point(void *p1, int unused2, int32_t unused3);
void app_pwr_chrg_int_init();

// This will create, init and start the high speed telemetry consumer thread with no delay
K_THREAD_DEFINE(apps_hs_telem_consumer, \
                APPS_HS_TELEM_CONSUMER_STACK_SIZE, \
                apps_hs_telem_consumer_entry_point, \
                NULL, NULL, NULL, \
                APPS_HS_TELEM_CONSUMER_PRIORITY, \
                0, 0);
// This will create and init the high speed telemetry message queue
K_MSGQ_DEFINE(hs_telemetry_msgq, sizeof(apps_hs_telem_data_t), 1, 4);

/*
// TODO - implement for normal telemetry to move into direction of feature-based daq instead of app-based
K_THREAD_DEFINE(apps_telem_consumer, \
                APPS_TELEM_CONSUMER_STACK_SIZE, \
                apps_telem_consumer_entry_point, \
                NULL, NULL, NULL, \
                APPS_TELEM_CONSUMER_THREAD_PRIORITY, \
                0, 0);
*/

/**
 * @brief Start app configured with DEV_CONFIG_APP_TYPE
 * 
 * @return none
 */
void apps_start(void)
{
    LOG_INF("Starting Apps...");

    switch(device_config_get_int16(DEV_CONFIG_APP_TYPE))
    {   
        default:        
        case DISTANCE:   
            app_distance();
            break;
        case AUDIO:     
            app_audio();
            break;
        case PWR_MONITOR:
            app_pwr_monitor_init();
            break;
    }

    app_pwr_chrg_int_init();
    
    // app_motion();
}

int apps_get_onboard_telemetry(apps_onboard_telem_t *telem)
{
    telem->rsrp = modem_get_rsrp_dbm_now();
    
    // accelerometer angle
    float angle_deg = 0;

    struct bma2_sensor_data accel;
    struct bma2_dev *bma2_dev = bma2_get_dev();
    if (bma2_dev)
    {
        bma2_get_accel_data(&accel, bma2_dev);
        
        float resultant_rad = sqrt(pow(accel.x, 2) + pow(accel.y, 2) + pow(accel.z, 2));
        angle_deg = -1;

        if (resultant_rad != 0)
        {
            float radian_to_vertical = acos(accel.z / resultant_rad);
            angle_deg = radian_to_vertical * 180 / 3.141592;
        }
    }

    telem->angle_deg = angle_deg;

    //temperature
    onboard_sensor_get_temp_c(&telem->temp_c);
	
    //Battery mV
    telem->batt_mv = sensor_get_battery_mV();
 
    // use batt_mv to calc soc
    telem->batt_pct = sensor_get_battery_soc(telem->batt_mv);
	
    return 0;
}

void app_pwr_chrg_int_init(){

    LOG_WRN("Starting charger interrupt");

    pwr_chrg_dev = device_get_binding(GPIO);
    if (pwr_chrg_dev == NULL)
    {
        LOG_ERR("Unable to get a pointer to the gpio");
        return; 
    }
    gpio_pin_configure(pwr_chrg_dev, pwr_chrg_int.pin, GPIO_INPUT | pwr_chrg_int.dt_flags);
    

    gpio_init_callback(&pwr_chrg_cb, pwr_chrg_pub_cb, BIT(pwr_chrg_int.pin));

    gpio_add_callback(pwr_chrg_dev, &pwr_chrg_cb);
    gpio_pin_interrupt_configure(pwr_chrg_dev, pwr_chrg_int.pin, GPIO_INT_EDGE_TO_ACTIVE);
   

}
/// @brief assumes this is for streaming type data, so caller needs to offload telemetry sending, hence the msgq
/// @param data data to send gets put on a single element msgq which is processed by the hs consumer thread

void apps_hs_send_data(apps_hs_telem_data_t *data)
{
    k_msgq_put(&hs_telemetry_msgq, data, K_NO_WAIT);
}

void apps_hs_telem_consumer_entry_point(void *p1, int unused2, int32_t unused3)
{
    LOG_DBG("Telemetry High Speed Consumer thread started");

    int err = 0;
    apps_hs_telem_data_t hs_telem_data;

    while (1)
    {
        err = k_msgq_get(&hs_telemetry_msgq, &hs_telem_data, K_FOREVER);
        if (err == 0)
        {
            apps_hs_telemetry_send(&hs_telem_data);
        }
        else
        {
            LOG_DBG("MSGQ ERR: %i", err);
        }
    }
}

// void apps_telem_consumer_entry_point(void *p1, int unused2, int32_t unused3)
// {
//     LOG_DBG("Telemetry Consumer thread started");
 
//     int err = 0;

//     while (1)
//     {
//         err = k_msgq_get(&hs_telemetry_msgq, &hs_telem_data, K_MSEC(1000));
//         if (err == 0)
//         {
//             cloud_telemetry_update_hs(&hs_telem_data);
//         }

//         // TODO: add standard telemetry here
//     } 
// }

void apps_hs_telemetry_send(apps_hs_telem_data_t *data)
{
    int err = 0;

    // +6 for topic path len
    char topic[SERIAL_NUMBER_LEN + 6] = { '\0' };
    snprintk(topic, SERIAL_NUMBER_LEN + 6, "dt/hs/%s", device_config_get_serial_number());

    /* e.g. for audio data where data->measure_name == "laeq_db"
    {
        "hs": {
            "laeq_db": {
                "ts": 0,
                "sample_duration_ms": 0,
                "samples": [0.0, ..., 0.0]
            }
        }
    }
    */

    char *json_str = cs_json_new();
    err += cs_json_add_chars(json_str, "{");
    err +=   cs_json_add_item_start(json_str, "hs", "{");
    err += 	   cs_json_add_item_start(json_str, data->measure_name, "{");
    err += 		 cs_json_add_item_int64(json_str, "ts", data->ts, COMMA);
    err += 	     cs_json_add_item_int(json_str, "sample_duration_ms", data->sample_duration_ms, COMMA);
    err += 	     cs_json_add_item_float_array(json_str, "samples", data->samples.float_arr, data->num_samples, NO_COMMA);

    // close all objects: root, "hs" and "measure_name"
    err += cs_json_add_chars(json_str, "}}}");

    LOG_DBG("hs_telem:\r\n%s", log_strdup(json_str));

    if (err == 0) 
    {
        cloud_send_json_str(topic, json_str);
    }
    else
    {
        LOG_ERR("error sending hs telem: err = %i", err);
    }

    cs_json_free(json_str);
}

int apps_telemetry_send(void *data, app_id_t app_id)
{
    int64_t start_ts = k_uptime_get();

    int err = 0;
    int64_t message_ts = 0;

    // err = date_time_now(&message_ts);
    // if (err || message_ts <= prev_ts) {
    //     LOG_ERR("date_time_now, error: %d", err);
    //     // try to update time for next attempt - timestamp error detection and correction handled
    //     // in cloud so we can keep bad timestamps for a period of time if necessary
    //     date_time_update_async(date_time_event_handler);
    //     // return err;
    // }
    // prev_ts = message_ts;

    // TODO: add this to config level at some point to avoid doing this every time.
    char topic[SERIAL_NUMBER_LEN + 3] = { '\0' };
    snprintk(topic, SERIAL_NUMBER_LEN + 3, "dt/%s", device_config_get_serial_number());

    // avoid uninitialized variable warning
    apps_onboard_telem_t onb_telem;
    apps_onboard_telem_t *p_onboard_telem = &onb_telem;

    if (app_id == AUDIO)
    {
        // use 'ts' from audio data
        message_ts = ((acoustic_data_t *)data)->ts;
        p_onboard_telem = &((acoustic_data_t *)data)->onboard_telem;
    }
    else if (app_id == DISTANCE)
    {
        message_ts = ((struct distance_data *)data)->ts;
        p_onboard_telem = &((struct distance_data *)data)->onboard_telem;
    }
    else if (app_id == PWR_MONITOR)
    {
        message_ts = ((struct tele_message *)data)->ts;
        p_onboard_telem = &((struct tele_message *)data)->onboard_telem;
    }

    char *json_str = cs_json_new();
    err += cs_json_add_chars(json_str, "{");
    err += cs_json_add_item_int64(json_str, "ts", message_ts, COMMA);
    err += cs_json_add_item_int(json_str, "rsrp", p_onboard_telem->rsrp, COMMA);

    // BUILT-IN SENSOR TELEMETRY - ALL APPS
    // accelerometer angle
    err += cs_json_add_item_double(json_str, "angle_deg", p_onboard_telem->angle_deg, COMMA);

    // temperature
    err += cs_json_add_item_double(json_str, "temp_c", p_onboard_telem->temp_c, COMMA);

    //batt_mv
    //made an exception here for power monitor
    err += cs_json_add_item_int(json_str, "sys_mV", p_onboard_telem->batt_mv, COMMA);

    // batt_pct
    err += cs_json_add_item_double(json_str, "batt_pct", p_onboard_telem->batt_pct, NO_COMMA);

    
    // APP-SPECIFIC TELEMETRY
    if (app_id == DISTANCE)
    {
        // add comma since this is conditional and trailing commas are not added unless next sibling is known
        err += cs_json_add_chars(json_str, ",");
        err += cs_json_add_item_int(json_str, "distance_mm", ((struct distance_data *)data)->distance_mm, NO_COMMA);
    }

    // APP-SPECIFIC TELEMETRY
    if (app_id == PWR_MONITOR)
    {
        // add comma since this is conditional and trailing commas are not added unless next sibling is known
        err += cs_json_add_chars(json_str, ",");
        // err += cs_json_add_item_int(json_str, "distance_mm", ((struct distance_data *)data)->distance_mm, NO_COMMA);
        err += cs_json_add_item_int(json_str, "power_status",  ((struct tele_message *)data)->power_status, COMMA);

        err += cs_json_add_item_int(json_str, "batt_mV", ((struct tele_message *)data)->batt_mV,COMMA );

        err += cs_json_add_item_int(json_str, "chrg_stat",((struct tele_message *)data)->chrg_stat, NO_COMMA);
    }

    if (app_id == AUDIO)
    {
        /*
            {
                "ts": bigint,
                "duration_ms": int,
                "laeq_db": float,
                "max_dba": float,
                ...
            }
        */

        // add comma since this is conditional and trailing commas are not added unless next sibling is known
        err += cs_json_add_chars(json_str, ",");
        err += cs_json_add_item_int(json_str, "duration_ms", ((acoustic_data_t *)data)->duration_ms, COMMA);
        err += cs_json_add_item_double(json_str, "laeq_db", ((acoustic_data_t *)data)->laeq_db, COMMA);
        err += cs_json_add_item_double(json_str, "max_dba", ((acoustic_data_t *)data)->max_dba, NO_COMMA);
    }

    // close root
    err += cs_json_add_chars(json_str, "}");

    if (err)
    {
        LOG_DBG("cs_json, error: %d", err);
        goto cleanup;
    }

    LOG_DBG("telem:\r\n%s", log_strdup(json_str));

    cloud_send_json_str(topic, json_str);

cleanup:
    cs_json_free(json_str);

    LOG_DBG("telem exec time: %lli", k_uptime_get() - start_ts);
    return err;
}

void apps_evt_broadcast_end(apps_evt_data_t *evt)
{
    // TODO: implment support for multiple app threads - for now a single thread will be notified because sem_max is 1
        
    // allow other apps to catch events and do something like trigger a new daq sample before the next daq interval
    // but, ONLY on event END
    if (evt->end_ts > 0)
    {
        // give the event semaphore to allow any waiting app threads to send its telemetry before the normal interval
        k_sem_give(&apps_evt_sem);
    }
}

void apps_evt_send_accel_event(apps_evt_data_t *evt, apps_evt_accel_data_t *accel_data)
{
    int err = 0;

    char topic[SERIAL_NUMBER_LEN + 3] = { '\0' };
    snprintk(topic, SERIAL_NUMBER_LEN + 3, "dt/evt/%s", device_config_get_serial_number());

    char *json_str = cs_json_new();
    err += cs_json_add_chars(json_str, "{");
    err +=     cs_json_add_item_start(json_str, "evt", "{");
    err +=         cs_json_add_item_int64(json_str, "start", evt->start_ts, COMMA);
    err +=         cs_json_add_item_int64(json_str, "end", evt->end_ts, COMMA);
    err +=         cs_json_add_item_string(json_str, "source", evt->source, NO_COMMA);
    err +=     cs_json_add_chars(json_str, "},");
    err +=     cs_json_add_item_int64(json_str, "ts", accel_data->ts, COMMA);
    err +=     cs_json_add_item_double(json_str, "a_x", accel_data->accel_x, COMMA);
    err +=     cs_json_add_item_double(json_str, "a_y", accel_data->accel_y, COMMA);
    err +=     cs_json_add_item_double(json_str, "a_z", accel_data->accel_z, COMMA);
    err +=     cs_json_add_item_double(json_str, "angle_deg", accel_data->angle_deg, NO_COMMA);
    err += cs_json_add_chars(json_str, "}");

    LOG_DBG("evt_accel_telem:\r\n%s", json_str);

    if (err == 0) 
    {
        cloud_send_json_str(topic, json_str);
    }
    else
    {
        LOG_ERR("error sending evt accel telem: err = %i", err);
    }

    cs_json_free(json_str);
    
    apps_evt_broadcast_end(evt);
}
