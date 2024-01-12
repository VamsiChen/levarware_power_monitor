/*
 * Copyright (c) 2022 Reliance Foundry Co. Ltd.
 *
 */ 

/*Logging*/
#include <logging/log.h>
#include <date_time.h>

#include <device.h>
#include <pm/device.h>

#include "power_mgr.h"
#include "apps/apps.h"
#include "subsys/modem.h"
#include "subsys/led.h"
#include "sensors/onboard_sensor.h"

LOG_MODULE_REGISTER(power_mgr); 

power_state_t device_state_current = POWER_STATE_UNKNOWN;

void power_mgr_init_device_state()
{
    device_state_current = POWER_STATE_AWAKE;
}

/**
 * @brief Configures the power state of the peripheral(one of the DT's node)
 * 
 * @param dev Pointer to the device struct
 * @param action Desired action
 * 
 * @return Returns 0 if it is a success
 */
static int power_mgr_set_dt_node_state(const struct device *dev, enum pm_device_action action)
{
    if (dev == NULL) 
    {
        LOG_ERR("power_mgr_set_dt_node_state() -> *dev is NULL");
        return -1;
    }

    int err = pm_device_action_run(dev, action);
    if (err != 0)
    {       
        LOG_ERR("Unable to set the state of %s to %d", log_strdup(dev->name), (uint8_t)action);
    }

    return err;
}

void power_mgr_sleep_timer_cb(struct k_timer *t)
{
    LOG_DBG("Device has awaken");
    k_sem_give(&wakeup_sem);
}

/**
 * @brief Congfigures the peripherals depending with the desired state of the device
 * 
 * @param state the desired state that the device will enter
 * 
 * @return Returns 0 if it is a success
 */
int power_mgr_set_device_state(power_state_t state)
{
    int err = 0;

    if (device_state_current == state)
    {
        // Device current state is same as the target state, nothing will be done
        return 0;
    }

    if (state == POWER_STATE_AWAKE)
    {
        LOG_DBG("Re-initializing the peripherals");

        power_mgr_set_dt_node_state(device_get_binding(DT_LABEL(DT_NODELABEL(uart0))), PM_DEVICE_ACTION_RESUME);
        power_mgr_set_dt_node_state(device_get_binding(DT_LABEL(DT_NODELABEL(uart2))), PM_DEVICE_ACTION_RESUME);

        modem_set_state(MODEM_POWER_ON);

        power_mgr_set_dt_node_state(device_get_binding(DT_LABEL(DT_NODELABEL(adc))), PM_DEVICE_ACTION_RESUME);
        power_mgr_set_dt_node_state(device_get_binding(DT_LABEL(DT_NODELABEL(i2c1))), PM_DEVICE_ACTION_RESUME);
        power_mgr_set_dt_node_state(device_get_binding(DT_LABEL(DT_NODELABEL(spi3))), PM_DEVICE_ACTION_RESUME);

        // TODO: vout is dependent of the sensor type and application type
		power_reg_external_power_on(5.0);

        onboard_sensor_tphg_set_mode(1);
		k_msleep(10);
        if (onboard_sensor_tphg_is_sleeping())
        {
            LOG_ERR("Sensor can't be awaken");
        }

        // resume thread
        // k_thread_resume(app_tid);
        // LOG_DBG("App Thread State:%s", log_strdup(k_thread_state_str(app_tid)));
        // k_msleep(10);

        // we are now awake, let's update our wakeup timestamp
        // unix_time_set_wakeup_timestamp();
    }
    else if (state == POWER_STATE_SLEEP)
    {
        led_set_state(BLUE_LED, LED_OFF);
        led_set_state(RED_LED, LED_OFF);
        led_set_state(GREEN_LED, LED_OFF);

        // // disable thread
        // k_thread_suspend(app_tid);
        // LOG_DBG("App Thread State:%s", log_strdup(k_thread_state_str(app_tid)));
        // k_msleep(100);

        // putting the TPHG sensor to sleep
        onboard_sensor_tphg_set_mode(0);
        k_msleep(10);
        if (onboard_sensor_tphg_is_sleeping() == false)
        {
            LOG_ERR("Sensor can't be put to sleep");
        }

        LOG_DBG("Turning off the modem");
        modem_set_state(MODEM_OFFLINE);

        // putting the accelerometer to sleep mode
        // if (onboard_sensor_accel_is_sleeping())
        // {
        // 	onboard_sensor_accel_set_mode();
        // }

        LOG_DBG("Disabling the External Regulator");
        power_reg_external_power_off();

        // Other MCU peripherals
        LOG_DBG("Turning off adc, i2c and spi");
        power_mgr_set_dt_node_state(device_get_binding(DT_LABEL(DT_NODELABEL(adc))), PM_DEVICE_ACTION_SUSPEND);
        power_mgr_set_dt_node_state(device_get_binding(DT_LABEL(DT_NODELABEL(i2c1))), PM_DEVICE_ACTION_SUSPEND);
        power_mgr_set_dt_node_state(device_get_binding(DT_LABEL(DT_NODELABEL(spi3))), PM_DEVICE_ACTION_SUSPEND);
        
        // modem_set_state(MODEM_OFFLINE);
        LOG_DBG("Turning off uarts");
        power_mgr_set_dt_node_state(device_get_binding(DT_LABEL(DT_NODELABEL(uart0))), PM_DEVICE_ACTION_SUSPEND);
        power_mgr_set_dt_node_state(device_get_binding(DT_LABEL(DT_NODELABEL(uart2))), PM_DEVICE_ACTION_SUSPEND);

        // NRF_REGULATORS_NS->SYSTEMOFF = 1;

        // we are now sleeping, let's update our sleep timestamp
        // unix_time_set_sleep_timestamp();
    }
    else
    {
        // do nothing
        LOG_ERR("Invalid device power state: %d", state);
    }

    device_state_current = state;

    return err;
}
