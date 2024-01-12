#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <date_time.h>

/*Logging*/
#include <logging/log.h>
LOG_MODULE_REGISTER(app_distance);

#include "apps.h"
#include "cloud.h"
#include "distance.h"
#include "device_config/device_config.h"
#include "sensors/distance_sensor.h"

static struct k_msgq distance_msgq;
static struct k_work_delayable telemety_work;
extern struct k_sem apps_evt_sem;
struct k_work telemetry_interrupt_work;

int curr_power_status;

struct k_thread app_distance_thread_data;
K_THREAD_STACK_DEFINE(app_distance_stack_area, APP_DISTANCE_STACK_SIZE);

// Define storage for message queue
#define MESSAGE_QUEUE_DEPTH     4           // for testing purposes, use very shallow message queue, move back to 10 when debugging is complete

// this queue holds copies of the distance measurement data when it needs to be queued (which isn't always)
static char __aligned(4) msgq[MESSAGE_QUEUE_DEPTH * sizeof(struct distance_data)];


// storage for sensor distance data measurements and control information
static struct distance_data distance_sensor_data;

/*
* Define forward references 

*/

void app_distance_entry_point(void *p1, int unused2, int32_t unused3);
void app_distance_telemetry_work_fn(struct k_work *work);
void app_power_monitoring_interrupt_work_fn(struct k_work *work);

static bool print_verbose = false;

static void work_init(void)
{
    k_work_init_delayable(&telemety_work, app_distance_telemetry_work_fn);
    k_work_init(&telemetry_interrupt_work, app_power_monitoring_interrupt_work_fn);
}

void app_distance(void)
{
    k_thread_create(&app_distance_thread_data, 
                    app_distance_stack_area, 
                    K_THREAD_STACK_SIZEOF(app_distance_stack_area), 
                    (k_thread_entry_t)app_distance_entry_point, 
                    NULL,
                    NULL,
                    NULL,
                    APP_DISTANCE_PRIORITY,
                    0,
                    K_MSEC(5000)); // give cloud thread a headstart
    k_thread_name_set(&app_distance_thread_data, "app_distance");
}

void app_distance_entry_point(void *p1, int unused2, int32_t unused3)
{
    LOG_INF("Distance App thread started");

    // Init message queue
    k_msgq_init(&distance_msgq, msgq, sizeof(struct distance_data), MESSAGE_QUEUE_DEPTH);

    work_init();

    uint32_t daq_loops = 0;     // must be zero for first loop - used to schedule telem worker
    int64_t next_daq_ts = 0;    // must be zero for first loop - evaluated for early daq samples
    bool new_event = false; // must be false for first loop - evaluated for async telem send (events)
    
    while (1)
    {
        int64_t start_ts = k_uptime_get();
        
        // only update next_daq_ts if the daq interval has elapsed
        // - this is to prevent an early daq sample (e.g. new event) from shifting the daq interval timing out of phase
        // the aim is to keep the interval timing consistent from hour to hour as much as reasonably possible
        if (start_ts >= next_daq_ts)
        {
            // this loop is on time or late - set next daq to configured interval
            next_daq_ts = k_uptime_get() + device_config_get_int16(DEV_CONFIG_DAQ_INTERVAL_S) * MSEC_PER_SEC; 
        }
        // if start_ts < next_daq_ts, then this daq loop is early -> don't update next_daq_ms to maintain timing phase

          
        /* acquire sample data the various components of the sensor data
         *
         */

        // first get the time
        date_time_now(&distance_sensor_data.ts);

        // next get the telemetry data (ie: misc sensor values)
        apps_get_onboard_telemetry(&distance_sensor_data.onboard_telem);

        // then get the distance data, TODO - handle error case. If we fail to get a measurement
        // then this function would never know
        // distance_sensor_data.distance_mm = distance_sensor_get_mm();
        distance_sensor_data.power_status = distance_sensor_get_mm();
        curr_power_status = distance_sensor_data.power_status;
        
        // if an event triggered this daq sample, override pub interval and send immediately
        if (new_event)
        {
            new_event = false;
            LOG_WRN("Event triggered the DAQ sample and send");

            // TODO - Note: apps_telemetry_send is being called out of context here. Called from this thread OR the telemetry thread. 
            // This could be a problem, needs investigation or rewrite.  
            apps_telemetry_send((void *)&distance_sensor_data, DISTANCE);
        }
        // else add to telem data q to be sent on next pub interval
        else 
        {
            if (k_msgq_put(&distance_msgq, &distance_sensor_data, K_NO_WAIT) != 0)
            {
                LOG_WRN("distance msgq is FULL");
            }

        }
        
        if (daq_loops == 0)
        {
            // start telemetry worker here to on first daq sample so the daq and pub intervals start approx in sync
            k_work_schedule(&telemety_work, K_NO_WAIT);
        }
        
        if (print_verbose)
        {
            LOG_INF("distance_mm: %i mm", distance_sensor_data.distance_mm);
        }
        
        int32_t sleep_time_ms = (int32_t)(next_daq_ts - k_uptime_get());

        LOG_DBG("Next DAQ in %d ms", sleep_time_ms);
        // TODO: go to deep sleep here?
        
        // the apps_event semaphore enables events to trigger a new daq sample to send at the end of each event 
        // if no event, it will timout in sleep_time_ms for a new daq sample at the configured interval
        // evt_sem_taken - new daq sample from event, set flag to also override pub interval after daq 
        // sample instead of inserting into msgq
        // TODO: this needs to be abstracted better at the apps level
        new_event = k_sem_take(&apps_evt_sem, K_MSEC(sleep_time_ms)) == 0;  // returns 0 if sem is taken -> new event
        
        // daq_loops is evaluated for first loop so increment at end
        daq_loops++;
    }
}

// self scheduling work function
void app_distance_telemetry_work_fn(struct k_work *work)
{
    int64_t start_ts = k_uptime_get();
    int32_t sleep_time_ms = device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S) * MSEC_PER_SEC;

    struct distance_data queued_measurement;
    
    // process all items in queue
    while (k_msgq_get(&distance_msgq, &queued_measurement, K_NO_WAIT) == 0)
    {
        
        apps_telemetry_send((void *)&queued_measurement, DISTANCE);
    }

    int32_t delta_ms = (int32_t)(k_uptime_get() - start_ts);
    sleep_time_ms -= delta_ms;

    LOG_DBG("Next PUB in %dms", sleep_time_ms);
    k_work_schedule(&telemety_work, K_MSEC(sleep_time_ms));
}

void app_power_monitoring_interrupt_work_fn(struct k_work *work){

    struct distance_data interrupt_measurements;

        interrupt_measurements.power_status = distance_sensor_get_mm();

        if(curr_power_status != interrupt_measurements.power_status ){

            date_time_now(&interrupt_measurements.ts);

            // next get the telemetry data (ie: misc sensor values)
            apps_get_onboard_telemetry(&interrupt_measurements.onboard_telem);

            curr_power_status = interrupt_measurements.power_status;

            apps_telemetry_send((void *)&interrupt_measurements, DISTANCE);
            
            LOG_ERR("curr power status after interrupt %d", curr_power_status);
        }
        else{
            LOG_INF("No change in power status");
        }
}

void distance_set_verbose_print_state(bool state, bool toggle)
{
   print_verbose = toggle ? (print_verbose ^ 1) : state;
}
