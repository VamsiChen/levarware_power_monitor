/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */


#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <modem/modem_info.h>
#include <nrf_modem.h>
#include <net/aws_iot.h>
#include <sys/reboot.h>
#include <date_time.h>
#include <dfu/mcuboot.h>
#include <cJSON.h>
#include <cJSON_os.h>


#include <logging/log.h>
LOG_MODULE_REGISTER(main);

#include "cloud.h"
#include "apps/apps.h"
#include "apps/audio.h"
#include "device_config/device_config.h"
#include "subsys/filesystem.h"
#include "sensors/onboard_sensor.h"
#include "subsys/led.h"
#include "power_mgr/power_mgr.h"
#include "sensors/battery.h"
#include "subsys/modem.h"
#include "drivers/sensor.h"
#include "onboard_sensor.h"

void main(void)
{
	int err;
	// bool forced_sleep;

	LOG_WRN("Levaware Thunder started, version: %s", device_config_get_fw_version_str());

	onboard_sensor_init();
    led_init();
	led_set_state(BLUE_LED, LED_OFF);

	cJSON_Init();
	init_lfs(true);
	device_config_init();
	
	// this will wait until LTE connects
	err = cloud_start();
	if (err != 0) 
	{
		// what else could be done?
		LOG_ERR("cloud_start failed: error = %d. Rebooting in 10 seconds...", err);
		k_msleep(10000);
		sys_reboot(SYS_REBOOT_COLD);
	}

	// start application threads
	apps_start();

	while(1)
	{	
		
		k_msleep(100);

		// // thread_analyzer_print();
		// if (device_config_get_int16(DEV_CONFIG_APP_TYPE) == AUDIO)
		// {
		// 	forced_sleep = false;
		// }
		// else
		// {
		// 	forced_sleep = power_mgr_is_max_uptime(&awake_timer); 
		// }

		// if ((k_sem_take(&app_sem, K_MSEC(50)) == 0))
		// {

		// 	LOG_DBG("About to enter deep sleep");

		// 	power_mgr_set_device_state(POWER_STATE_SLEEP);

		// 	// TODO: Add support for other application type, currently this only works for BINs
		// 	k_timer_start(&sleep_timer, K_SECONDS(unix_time_get_next_alarm_s(device_config_get_int16(DEV_CONFIG_DAQ_INTERVAL_S))), K_NO_WAIT);

		// 	if (k_sem_take(&wakeup_sem, K_SECONDS(device_config_get_int16(DEV_CONFIG_DAQ_INTERVAL_S) + 10)) != 0)
		// 	{
		// 		LOG_ERR("Sleep timer did not generate interrupt");
				
		// 		uint8_t delay = 100;
		// 		while(delay-- > 0)
		// 		{
		// 			// toggle led from bit 0 in delay counter
		// 			led_set_state(GREEN_LED, (delay & 0x01));
		// 			k_msleep(250);
		// 		}
		// 		sys_reboot(SYS_REBOOT_WARM);
		// 	}

		// 	power_mgr_set_device_state(POWER_STATE_AWAKE);

		// 	// TODO: add support for other application type
		// 	k_sem_give(&daq_sem);
		// }

	}
	
}
