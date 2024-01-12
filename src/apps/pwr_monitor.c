/**
 * @brief: 	.c - power monitor application
 *
 * @notes: 	Runs in the it's own context. Performs measurements of a power and charging status and pushes to the cloud
 *
 * 			Copyright (c) 2023 Reliance Foundry Co. Ltd.
 *
 */

#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <date_time.h>
#include "apps/pwr_monitor.h"
#include "device.h"
#include "power_mgr/power_reg.h"
#include "drivers/gpio.h"

/*Logging*/
#include <logging/log.h>
LOG_MODULE_REGISTER(app_pwr_monitor);

#include "cloud.h"

#define APP_PWR_MONITOR_STACK_SIZE (2048)
#define APP_PWR_MONITOR_PRIORITY (2)

#define PWR_MONITOR_EVENT_QUEUE_SZ (10) // size of the queue

// Events for the Light Task
enum event_code
{
    EVT_MEASUREMENT_TIMER_EXP,
    EVT_PUB_TIMER_EXP,
    EVT_PUB_TIME_CHANGE,
    EVT_TELEMETRY_INTERRUPT
};

struct pwr_monitor_control_block
{

    // struct to handle intermediate calculations;

    uint8_t curr_power_status;
    float charging_status;
    int count;

    // handle/pointer to the device structure
    // const struct device *dev_ptr;

    // to store a copy of pub interval
    int prev_pub_interval;
};

struct pwr_monitor_control_block *cblkp;

struct pwr_monitor_control_block pwr_monitor_cblk;

// the event message that contains the current event
struct event_msg
{
    // future values added to event message need to be added here
    enum event_code event;
};

// Thread declaration for light sensor thread
static struct k_thread app_pwr_monitor_thread_data;

// Timer block for Publishing
static struct k_timer pub_timer;

// Timer block for measurement timer
static struct k_timer measurement_done_timer;

// create storage for event queue
char __aligned(4) event_queue_buffer[sizeof(struct event_msg) * (PWR_MONITOR_EVENT_QUEUE_SZ)];

// storage for the event queue
struct k_msgq event_queue;

K_THREAD_STACK_DEFINE(app_pwr_monitor_stack_area, APP_PWR_MONITOR_STACK_SIZE);

/**
 * @brief    Pub timer expiry fucntions
 *
 * @param     null
 *
 * @return   nothing
 */

static void pub_timer_expiry_fn()
{
    struct event_msg event_blk;

    event_blk.event = EVT_PUB_TIMER_EXP;
    k_msgq_put(&event_queue, &event_blk, K_NO_WAIT);

}
/**
 * @brief    Measurement timer expiry function
 *
 * @param     null
 *
 * @return   nothing
 */

static void measurement_timer_expiry_fn()
{

    struct event_msg event_blk;

    event_blk.event = EVT_MEASUREMENT_TIMER_EXP;
    k_msgq_put(&event_queue, &event_blk, K_NO_WAIT);
}

void pwr_chrg_pub_cb()
{

    struct event_msg event_blk;

    LOG_WRN("power charger removed/added");

    event_blk.event = EVT_TELEMETRY_INTERRUPT;
    k_msgq_put(&event_queue, &event_blk, K_NO_WAIT);
}

/**
 * @brief    Shadow change notification function
 *
 * @param     void *userp
 *
 * @return   nothing
 *
 * @note     This function understands whats changed in the shadow, i.e. publish time or suppress flag, and puts the respective event in the queue
 */

static void shadow_change(void *userp)
{

    struct event_msg event_blk;
    int new_pub_interval;

    new_pub_interval = device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S);

    if (cblkp->prev_pub_interval != new_pub_interval)
    {
        event_blk.event = EVT_PUB_TIME_CHANGE;
        k_msgq_put(&event_queue, &event_blk, K_NO_WAIT);
        cblkp->prev_pub_interval = new_pub_interval;
    }
}
/**
 * @brief    Register reader function
 *
 * @param    struct light_data_calc      *calp
 *
 * @return   nothing
 *
 * @note     Gets lux intensity values from the sensor and calculates max and average values over the pub period.
 */
static void pwr_monitor_get_measurement(struct tele_message *pwr_data_ptr)
{
    // first get the time
    date_time_now(&pwr_data_ptr->ts);

    int err = apps_get_onboard_telemetry(&pwr_data_ptr->onboard_telem);
    if (err)
    {
        LOG_DBG("unable to get onboard telemetry");
    }

    err = power_reg_get_power_status(pwr_data_ptr);
    if (err)
    {
        LOG_DBG("unable to get power status");
    }

    pwr_monitor_cblk.curr_power_status = pwr_data_ptr->power_status;

    //TODO: this fix is because the battery ADC shuts off after the power is removed and reads 0, the battery is directly connected to the system so, batt mv = sys_mV. Need to investigate why this problem occurs

     if(pwr_monitor_cblk.curr_power_status == 0){
            pwr_data_ptr->batt_mV = pwr_data_ptr->onboard_telem.batt_mv;
        }
    
}
static void pwr_monitor_interrupt(struct tele_message *pwr_interrupt_data)
{

    int err = power_reg_get_power_status(pwr_interrupt_data);
    if (err)
    {
        LOG_DBG("unable to get power status");
    }

    if (pwr_monitor_cblk.curr_power_status != pwr_interrupt_data->power_status)
    {
        pwr_monitor_cblk.curr_power_status = pwr_interrupt_data->power_status;

        date_time_now(&pwr_interrupt_data->ts);

        err = apps_get_onboard_telemetry(&pwr_interrupt_data->onboard_telem);
        if (err)
        {
            LOG_DBG("unable to get onboard telemetry");
        }

        //TODO: this fix is because the battery ADC shuts off after the power is removed and reads 0, the battery is directly connected to the system so, batt mv = sys_mV. Need to investigate why this problem occurs

        if(pwr_monitor_cblk.curr_power_status == 0){
            pwr_interrupt_data->batt_mV = pwr_interrupt_data->onboard_telem.batt_mv;
        }



        apps_telemetry_send((void *)pwr_interrupt_data, PWR_MONITOR);
    }

    else
    {
        LOG_DBG("no change in power status");
    }
}

/**
 * @brief    Entry point of Light thread
 *
 * @param
 *           void        *p1
 *           int         unused2
 *           int32_t     unused3
 *
 * @return   never returns
 *
 * @note     This thread currently gathers up lux measurements from the light thread and then pushes
 *           them to the cloud at the interval defined in by the cloud in the shadow data.
 *
 */
static void app_pwr_monitor_entry_point(void *p1, int unused2, int32_t unused3)
{
    int err = 0;

    // event block for holding event messages
    struct event_msg event_block;

    // struct for holding all light sensor data
    struct tele_message pwr_monitor_periodic_data, pwr_monitor_interrupt_data;

    // shadow change notification block
    struct shadow_notification_blk nblk;

    // store a copy of the pub interval and suppress flag boolean
    cblkp->prev_pub_interval = device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S);

    k_timer_init(&pub_timer, pub_timer_expiry_fn, NULL);
    k_timer_init(&measurement_done_timer, measurement_timer_expiry_fn, NULL);

    // register for shadow change notifications'
    cloud_register_shadow_change(&nblk, shadow_change, NULL);

    // initial state during startup

    // delaying the publish timer by a little = 1000 ms, needs to be experimented with different values and brought down
    k_timer_start(&pub_timer,K_MSEC(1000), K_SECONDS(device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S)));
    k_timer_start(&measurement_done_timer, K_MSEC(10), K_SECONDS(device_config_get_int16(DEV_CONFIG_DAQ_INTERVAL_S)));

    LOG_INF("Power Monitor app thread started");

    while (1)
    {

        // waiting here forever for an event to occur to be processed
        err = k_msgq_get(&event_queue, &event_block, K_FOREVER);

        if (err)
        {
            // handle the error
            LOG_ERR("Error receiving message from event_queue: %d\n", err);
            break;
        }

        // now that we have an event, let's see what state we are in before processing the event
        switch (event_block.event)
        {
        case EVT_MEASUREMENT_TIMER_EXP:

            LOG_DBG("measurement timer expired, getting measurements");

            pwr_monitor_get_measurement(&pwr_monitor_periodic_data);

            break;

        case EVT_PUB_TIME_CHANGE:

            // publish time in the shadow changed, so local copies of the pub period have to be updated and timers have to be resetted
            LOG_INF("pub interval changed to %d, resetting timers", device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S));

            // TODO: Should we purge the existing data in the max and avg buffers, before we start with the new timers?

            // stop timers
            k_timer_stop(&pub_timer);
            k_timer_stop(&measurement_done_timer);

            // restart timers
            k_timer_start(&pub_timer, K_SECONDS(device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S)), K_SECONDS(device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S)));
            k_timer_start(&measurement_done_timer, K_MSEC(1000), K_SECONDS(device_config_get_int16(DEV_CONFIG_DAQ_INTERVAL_S)));

            break;
        case EVT_PUB_TIMER_EXP:

            LOG_DBG("pub timer expired, publish data");
            apps_telemetry_send((void *)&pwr_monitor_periodic_data, PWR_MONITOR);

            break;
        case EVT_TELEMETRY_INTERRUPT:

            LOG_DBG("Interrupt triggered by power supply");
            pwr_monitor_interrupt(&pwr_monitor_interrupt_data);
            break;

            // default:
            // LOG_FATAL(""); / lets do our first fatal error in the system. to be discussed.
        }
    }
}

/**
 * @brief    Thread and message queue initialization
 *
 * @param    null
 *
 * @return   nothing
 *
 */
void app_pwr_monitor_init(void)
{
    k_thread_create(&app_pwr_monitor_thread_data,
                    app_pwr_monitor_stack_area,
                    K_THREAD_STACK_SIZEOF(app_pwr_monitor_stack_area),
                    (k_thread_entry_t)app_pwr_monitor_entry_point,
                    NULL,
                    NULL,
                    NULL,
                    APP_PWR_MONITOR_PRIORITY,
                    0,
                    K_MSEC(2000)); // give all other threads a headstart
    k_thread_name_set(&app_pwr_monitor_thread_data, "app_pwr_monitor");

    // initialize the event queue
    k_msgq_init(&event_queue, event_queue_buffer, sizeof(struct event_msg), PWR_MONITOR_EVENT_QUEUE_SZ);

    // intialize the pointer to a control block
    cblkp = &pwr_monitor_cblk;
}
