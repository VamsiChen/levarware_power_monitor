#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <date_time.h>
#include <sys/base64.h>

/*Logging*/
#include <logging/log.h>
LOG_MODULE_REGISTER(app_motion);

#include "apps.h"
#include "cloud.h"
#include "motion.h"
#include "device_config/device_config.h"
#include "drivers/sensor/bma253/bma2.h"
#include "drivers/sensor/bma253/bma2_zephyr.h"

// static struct k_msgq telemetry_msgq;
// static struct k_work_delayable telemety_work;

struct k_thread app_motion_thread_data;
K_THREAD_STACK_DEFINE(app_motion_stack_area, APP_MOTION_STACK_SIZE);

void app_motion_entry_point(void *p1, int unused2, int32_t unused3);
void app_motion_send_event(bool event_end);
void app_motion_telemetry_work_fn(struct k_work *work);
void app_motion_init_and_start_accel_slo_no_mot(struct bma2_fifo_frame *fifo, uint8_t *fifo_buff);
void app_motion_get_accel_fifo(void);

// statically defined wtih K_MSGQ_DEFINE in bma2_driver (bma2_zephyr.c)
extern struct k_msgq bma2_int_event_data_msgq;

struct bma2_dev *bma2_dev = NULL;
static bool print_verbose = false;

// static void work_init(void)
// {
//     k_work_init_delayable(&telemety_work, app_motion_telemetry_work_fn);
// }

void app_motion(void)
{
    k_thread_create(&app_motion_thread_data, 
                    app_motion_stack_area, 
                    K_THREAD_STACK_SIZEOF(app_motion_stack_area), 
                    (k_thread_entry_t)app_motion_entry_point, 
                    NULL,
                    NULL,
                    NULL,
                    APP_MOTION_PRIORITY,
                    0,
                    K_MSEC(6000)); // give all other threads a headstart
    k_thread_name_set(&app_motion_thread_data, "app_motion");
}

void app_motion_entry_point(void *p1, int unused2, int32_t unused3)
{
    LOG_INF("Motion App thread started");

    uint32_t daq_loops = 0;
    bma2_int_event_data_t int_event_data;
    struct bma2_fifo_frame fifo;
    uint8_t fifo_buff[BMA2_FIFO_BUFFER] = { 0 };
    uint16_t acc_index;
    struct bma2_sensor_data accel_data[BMA2_FIFO_EXTRACTED_DATA_FRAME_COUNT] = { { 0 } };

    bma2_dev = NULL;

    // wait for bma2 device to come online... not sure why this would happen, 
    // but account for it anyways - allow for three attempts, 10 seconds apart
    for (int i = 0; bma2_dev == NULL && i < 3; i++)
    {
        app_motion_init_and_start_accel_slo_no_mot(&fifo, fifo_buff);
        
        if (bma2_dev == NULL)
        {
            LOG_ERR("Unable to get BMA2 device! Retrying 3 times, 10 seconds apart");
            k_sleep(K_SECONDS(10));
        }
    }

    // motion_data_t data;

    LOG_WRN("motion app waiting for slo or no motion interrupts...");

    while (1)
    {
        daq_loops++;
        
        // we wait on the sensor's mesgq for events - nothing to do until there's an event so wait forever
        k_msgq_get(&bma2_int_event_data_msgq, &int_event_data, K_FOREVER);
        
        int64_t now = k_uptime_get();
        int event_ms_ago = (int)(now - int_event_data.ts);
        
        // process each interrupt
        switch (int_event_data.int_pin)
        {
            case BMA2_INT1: 
                if (int_event_data.int_status_regs.int_status.int_status_0 & BMA2_INT_0_ASSERTED_SLOW_NO_MOTION)
                {
                    printk("%lld -> INT1 NO motion interrupt %ims ago\r\n", now, event_ms_ago);
                    // no motion is end of event
                    app_motion_send_event(true);

                    bma2_set_int_trigger_flags(BMA2_INT2, GPIO_INT_DISABLE);
                } 
                else
                {
                    printk("%lld -> INT1 SLO motion interrupt %ims ago\r\n", now, event_ms_ago);
                    // start of motion event
                    app_motion_send_event(false);

                    // fifo full interrupt only while in motion
                    bma2_set_int_trigger_flags(BMA2_INT2, GPIO_INT_EDGE_TO_ACTIVE);
                }
                break;
            
            case BMA2_INT2:
                if (int_event_data.int_status_regs.int_status.int_status_1 & BMA2_INT_1_ASSERTED_FIFO_FULL)
                {
                    printk("%lld -> INT2 FIFO FULL interrupt %ims ago\r\n", now, event_ms_ago);
                    
                    /* Read fifo data */
                    uint8_t rslt = bma2_read_fifo_data(&fifo, bma2_dev);
                    bma2_error_codes_print_result("bma2_read_fifo_data", rslt);

                    rslt = bma2_extract_accel(accel_data, &acc_index, &fifo);
                    bma2_error_codes_print_result("bma2_extract_accel", rslt);

                    if (rslt == BMA2_OK)
                    {
                        size_t b64_olen = 0;
                        // get required buffer size for base64 output by setting dlen = 0
                        base64_encode(NULL, 0, &b64_olen, (uint8_t *)&accel_data[0], fifo.length);
                        if (b64_olen > 0)
                        {
                            char b64_str[b64_olen];
                            memset(b64_str, 0, b64_olen);

                            base64_encode(b64_str, sizeof(b64_str), &b64_olen, (uint8_t *)&accel_data[0], fifo.length);
                            printk("bma253_fifo_b64 (%i samples): %s\r\n", fifo.length, b64_str);

                            // cloud_accel_telem_update(b64_str, angle_deg);
                        }
                    }
                }
                break;

            default:
                continue;
        }
        
        // k_msleep(1000);
        // LOG_DBG("%i loops of daq on the wall...", daq_loops);
    }
}


////////////////////////

void app_motion_send_event(bool event_end)
{
    // static to retain values between calls
    static apps_evt_data_t evt_data = {
        .start_ts = 0,
        .end_ts = 0,
        .source = "accel"
    };

    struct bma2_dev *bma2_dev = bma2_get_dev();
    struct bma2_sensor_data accel;
    float angle_deg = 0;

    if (bma2_dev)
    {
        int8_t rslt = bma2_get_accel_data(&accel, bma2_dev);
        bma2_error_codes_print_result("bma2_get_accel_data", rslt);
        
        float resultant_rad = sqrt(pow(accel.x, 2) + pow(accel.y, 2) + pow(accel.z, 2));
        angle_deg = -1;

        if (resultant_rad != 0)
        {
            float radian_to_vertical = acos(accel.z / resultant_rad);
            angle_deg = radian_to_vertical * 180 / 3.141592;
        }
    }

    // TODO: move scaling config to isr work handler to deliver scaled real numbers instead of bits
    float accel_lsb_scale_factor = (1.0f / 1024.0f) * 9.8067f;
    apps_evt_accel_data_t evt_accel_data = {
        .ts = 0,
        .accel_x = (float)(accel.x * accel_lsb_scale_factor),
        .accel_y = (float)(accel.y * accel_lsb_scale_factor),
        .accel_z = (float)(accel.z * accel_lsb_scale_factor),
        .angle_deg = angle_deg
    };

    date_time_now(&evt_accel_data.ts);

    // start_ts of zero indicates this is a new event
    if (evt_data.start_ts == 0)
    {
       evt_data.start_ts = evt_accel_data.ts;
       evt_data.end_ts = 0;
    }
    
    // event_end is set from timer expiry handler -> update end_ts to mark end of event
    if (event_end)
    {
       evt_data.end_ts = evt_accel_data.ts;
       // don't reset start_ts here - the telemetry backend needs start_ts to be set with end_ts to know which event is ending 
       // start_ts is the telemetry key that links each event including the event_end
    }

    apps_evt_send_accel_event(&evt_data, &evt_accel_data);
    
    // reset start/end timestamps for next event
    if (event_end)
    {
       evt_data.start_ts = 0;
       evt_data.end_ts = 0;
    }
}

void app_motion_init_and_start_accel_slo_no_mot(struct bma2_fifo_frame *fifo, uint8_t *fifo_buff)
{
    int8_t rslt = 0;
    uint32_t int_en;
    uint8_t power_mode;
    struct bma2_int_pin pin_conf;
    struct bma2_slo_no_mot_conf no_mot_setting, no_mot_cur_settings;
    struct bma2_fifo_frame get_fifo;
    

    // Get BMA2 Device and set global
    bma2_dev = bma2_get_dev();
    
    //
    // setup power mode
    //

    rslt = bma2_set_power_mode(BMA2_NORMAL_MODE, bma2_dev);
    bma2_error_codes_print_result("bma2_set_power_mode", rslt);

    rslt = bma2_get_power_mode(&power_mode, bma2_dev);
    bma2_error_codes_print_result("bma2_get_power_mode", rslt);

    //
    // setup slo-no-mot
    //

    rslt = bma2_get_slo_no_mot_conf(&no_mot_setting, bma2_dev);
    bma2_error_codes_print_result("bma2_get_slo_no_mot_conf", rslt);
    
    no_mot_setting.duration = 0x01;
    no_mot_setting.threshold = 0x64;

    rslt = bma2_set_slo_no_mot_conf(&no_mot_setting, bma2_dev);
    bma2_error_codes_print_result("bma2_set_slo_no_mot_conf", rslt);

    rslt = bma2_get_slo_no_mot_conf(&no_mot_cur_settings, bma2_dev);
    bma2_error_codes_print_result("bma2_get_slo_no_mot_conf", rslt);

    //
    // setup fifo
    //

    rslt = bma2_get_fifo_config(fifo, bma2_dev);
    bma2_error_codes_print_result("bma2_get_fifo_config", rslt);

    /* Data selection can be used to select data axis,
     * all XYZ axes - BMA2_XYZ_AXES,
     * X axis only - BMA2_X_AXIS,
     * Y axis only - BMA2_Y_AXIS,
     * Z axis only - BMA2_Z_AXIS,
     * Here, all XYZ axes data is selected(BMA2_XYZ_AXES)
     */
    fifo->fifo_data_select = BMA2_XYZ_AXES;
    fifo->fifo_mode_select = BMA2_MODE_FIFO;

    /* Fifo user buffer and length config */
    fifo->data = fifo_buff;
    fifo->length = BMA2_FIFO_BUFFER;

    rslt = bma2_set_fifo_config(fifo, bma2_dev);
    bma2_error_codes_print_result("bma2_set_fifo_config", rslt);

    rslt = bma2_get_fifo_config(&get_fifo, bma2_dev);
    bma2_error_codes_print_result("bma2_get_fifo_config", rslt);
    
    //
    // setup int pins
    //

    rslt = bma2_get_int_out_ctrl(&pin_conf, bma2_dev);
    bma2_error_codes_print_result("bma2_get_int_out_ctrl", rslt);

    pin_conf.latch_int = BMA2_NON_LATCHED;
    
    // slo-no-mot on int1
    pin_conf.int1_lvl = BMA2_ACTIVE_HIGH;
    pin_conf.int1_od = BMA2_PUSH_PULL;

    // fifo_full on int2
    pin_conf.int2_lvl = BMA2_ACTIVE_HIGH;
    pin_conf.int2_od = BMA2_PUSH_PULL;

    rslt = bma2_set_int_out_ctrl(&pin_conf, bma2_dev);
    bma2_error_codes_print_result("bma2_set_int_out_ctrl", rslt);

    //
    // setup int mapping
    //

    rslt = bma2_set_int_mapping(BMA2_INT_MAP, BMA2_INT1_MAP_SLOW_NO_MOTION | BMA2_INT2_MAP_FIFO_FULL, bma2_dev);
    bma2_error_codes_print_result("bma2_set_int_mapping for slow_no_motion and fifo_full", rslt);

    //
    // enable bma253 interrupts
    //

    int_en = BMA2_INT_EN_SLOW_NO_MOTION_X_AXIS | BMA2_INT_EN_SLOW_NO_MOTION_Y_AXIS | BMA2_INT_EN_SLOW_NO_MOTION_Z_AXIS |
             BMA2_INT_EN_SLOW_NO_MOTION_SEL | BMA2_INT_EN_FIFO_FULL;
    
    rslt = bma2_enable_interrupt(int_en, bma2_dev);
    bma2_error_codes_print_result("bma2_enable_interrupt", rslt);

    rslt = bma2_get_enabled_interrupts(&int_en, bma2_dev);
    bma2_error_codes_print_result("bma2_get_enabled_interrupts", rslt);

    //
    // enable INT1 and INT2 gpio edge interrupts
    //

    // enable interrupts on both edges to capture no_motion->motion falling edge of INT1
    bma2_set_int_trigger_flags(BMA2_INT1, GPIO_INT_EDGE_BOTH);
    // fifo full interrupt only on rising edge
    // bma2_set_int_trigger_flags(BMA2_INT2, GPIO_INT_EDGE_TO_ACTIVE);

    return;
}

void app_motion_set_verbose_print_state(bool state, bool toggle)
{
   print_verbose = toggle ? (print_verbose ^ 1) : state;
}
