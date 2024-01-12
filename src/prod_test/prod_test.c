#ifdef CONFIG_PROD_TEST
#include <zephyr.h>
#include <string.h>

#include <device.h>
#include <drivers/sensor.h>
#include <drivers/uart.h>

/*Logging*/
#include <logging/log.h>

#include <sys/crc.h>

#include <nrf_modem_at.h>

#include <net/aws_jobs.h>
#include <net/aws_fota.h>

#include <dfu/mcuboot.h>
#include <sys/reboot.h>

#include "prod_test.h"
#include "subsys/led.h"
#include "subsys/filesystem.h"
#include "network/network.h"
#include "device_config/device_config.h"
#include "power_mgr/power_reg.h"
#include "sensors/distance_sensor.h"
#include "sensors/onboard_sensor.h"
#include "sensors/battery.h"
#include "utils/cs_json.h"
#include "power_mgr/power_mgr.h"

LOG_MODULE_REGISTER(prod_test);

/* FOTA */
K_THREAD_STACK_DEFINE(mqtt_poll_thread_stack_area, MQTT_POLL_THREAD_STACK_SIZE);
bool do_reboot = false;
k_tid_t mqtt_poll_tid;
struct k_thread mqtt_poll_thread_data;
static struct mqtt_client prod_client;
static struct pollfd fds;

static device_config_t dev_conf;

static struct aws_iot_config aws_iot;

static uint8_t current_job_id[AWS_JOBS_JOB_ID_MAX_LEN];

/* Serial */
static bool serial_crc_correct = false;
static bool serial_usb_rx_done = false;
static uint8_t serial_buf[128];
static uint32_t serial_rx_buf_index = 0;
uint16_t serial_usb_data_len = 0;
uint8_t serial_number_crc8 = 0;

/**
 * @brief Tests the on-board LEDs 
 * 
 * @return Does not return any
 */
void prod_test_led(void)
{
	led_init();

    uint8_t flash_count = 0;
    uint8_t flash_max = 3;

	while (flash_count < flash_max)
	{
		led_set_state(BLUE_LED, LED_ON);

		k_msleep(1000);

		led_set_state(BLUE_LED, LED_OFF);

		k_msleep(1000);

		led_set_state(GREEN_LED, LED_ON);

		k_msleep(1000);

		led_set_state(GREEN_LED, LED_OFF);

		k_msleep(1000);

		led_set_state(RED_LED, LED_ON);

		k_msleep(1000);

		led_set_state(RED_LED, LED_OFF);

		k_msleep(1000);

        flash_count++;
	}
}

/**
 * @brief Test verifies external flash as LFS
 * 
 * @param basic_test basic testing if TRUE otherwise will test more
 * 
 * @return Does not return any
 */
void prod_test_external_flash_lfs(bool basic_test)
{
    if (basic_test)
    {
        fs_basic_test();
    }
    else
    {
        init_lfs(true);

        char data[50] = "Hello there";

        char file_name[50] = "test_file";

        save_to_file(data, strlen(data), file_name);

        if (is_file_exists(file_name) == false)
        {
            printk("File does not exist\n");
        }

        save_store_forward_data(1649353533000, "{\"test\":123}");

        test_send_store_forward_data(255);

        test_lfs();
        
        deinit_lfs();
    }

}

/**
 * @brief Tests all the network related components
 * 
 * @return Does not return any
 */
void prod_test_network()
{
    LOG_INF("Modem Prod Test starting");
    modem_init_modem_info();

    modem_lte_init();

    char dev_imei[IMEI_LEN];
    modem_get_IMEI(dev_imei, IMEI_LEN);

    // __ASSERT(strlen(dev_imei) > 0, "Unable to get the IMEI");

    printk("IMEI:%s\n", dev_imei);

    

    uint8_t try_cnt = 0;
    uint8_t max_try = 60;

    while (try_cnt < max_try)
    {
        k_msleep(1000);
        if (lte_connected)
        {
            break;
        }
        try_cnt++;
    }

    // __ASSERT(lte_connected == true, "Unable to connect to LTE");

    int8_t current_rsrp = 0;
    try_cnt = 0;
    max_try = 10;
    // while (try_cnt < max_try)
    // {
    //     k_msleep(1000);
    //     current_rsrp = modem_get_rsrp_dbm();
    //     if (current_rsrp != 0)
    //     {
    //         break;
    //     }
    //     try_cnt++;
    // }

    

    // // __ASSERT(current_rsrp != 0, "Unable to update RSRP");

    printk("RSRP:%d\n", current_rsrp);

    printk("RSRP(basic):%d\n", modem_get_rsrp_dbm_now());

    //log_data(&prod_inst, INFORMATION, "Network Test Done!");
}

void prod_test_mqtt()
{
    LOG_INF("MQTT Prod Test starting");
    int err = 0;
    cloud_mqtt_init(&prod_client, "12233344445", &fds);

    k_msleep(5000);



    // once mqtt is connected
    // set topics to be subscribed
    // cloud_mqtt_subscribe("dt/lv/test", &prod_client);
    // publish data
    LOG_INF("Publishing data test");
    int test_val = 1100;
    char pub_data[256];
    int pub_max_item = 3;
    int pub_item_cnt = 0;
    while(pub_item_cnt < pub_max_item)
    {

        cloud_mqtt_connect_then_poll(&prod_client, &fds);

        snprintk(pub_data, sizeof(pub_data),  "{\"data1\": %d, \"text1\": \"hello\"}", test_val);

        test_val += test_val;

        cloud_mqtt_publish_to_topic_then_poll(&prod_client, &fds, MQTT_QOS_1_AT_LEAST_ONCE, pub_data, strlen(pub_data), "dt/lv/test/pub", "12233344445");

        LOG_DBG("Disconnecting MQTT");
        mqtt_disconnect(&prod_client);         

        pub_item_cnt++;
        k_msleep(60 * 1000);
    }

}

void prod_test_aws_shadow()
{
    device_config_init(&dev_conf);
    LOG_INF("after init dev config\nser_num:%s\nimei:%s\ndaq_s:%d\npub_s:%d\nconf_s:%d\nsensor_type:%d", log_strdup(dev_conf.serial_number), log_strdup(dev_conf.imei), dev_conf.dev_shadow_attrib[DEV_CONFIG_DAQ_INTERVAL_S].val.int_16, dev_conf.dev_shadow_attrib[DEV_CONFIG_PUB_INTERVAL_S].val.int_16, dev_conf.dev_shadow_attrib[DEV_CONFIG_CONF_UPDATE_INTERVAL_S].val.int_16, dev_conf.dev_shadow_attrib[DEV_CONFIG_SENSOR_TYPE].val.int_16);

    if (cloud_aws_iot_shadow_op(&dev_conf, &aws_iot) == 1)
    {
        //FW update available
        LOG_INF("FW update in progress....");
        while(1)k_msleep(100);
    }

    LOG_INF("New dev config\nser_num:%s\nimei:%s\ndaq_s:%d\npub_s:%d\nconf_s:%d\nsensor_type:%d", log_strdup(dev_conf.serial_number), log_strdup(dev_conf.imei), dev_conf.dev_shadow_attrib[DEV_CONFIG_DAQ_INTERVAL_S].val.int_16, dev_conf.dev_shadow_attrib[DEV_CONFIG_PUB_INTERVAL_S].val.int_16, dev_conf.dev_shadow_attrib[DEV_CONFIG_CONF_UPDATE_INTERVAL_S].val.int_16, dev_conf.dev_shadow_attrib[DEV_CONFIG_SENSOR_TYPE].val.int_16);

    device_config_save_all_attributes_to_file(dev_conf.dev_shadow_attrib, DEV_CONFIG_NUM);
}

void prod_test_external_reg()
{
    LOG_INF("Testing external regulator");

    power_reg_external_init(3.20, true);

    power_reg_external_set_state(true, true);

    k_msleep(500);

    power_reg_set_vout(4.0, true);

    k_msleep(500);

    power_reg_set_vout(4.5, true);

    k_msleep(500);

    power_reg_set_vout(5.0, true);

    k_msleep(2000);

    power_reg_external_set_state(false, true);

}

void prod_test_distance_sensor()
{
    LOG_INF("Testing distance sensor");


    // TODO: Scope VSYS when external regulator is set to 5V and sensor is set to terabee, seems like main PSU gets overloaded
    power_reg_init(5.0);

    distance_sensor_t sensor_type_2 = DISTANCE_SENSOR_TERABEE;
    distance_sensor_init(&sensor_type_2);

    while(1)
    {
        power_reg_external_set_state(true, true);
        // power_reg_set_vout(3.0, true);
        // power_reg_set_vout(3.5, true);
        // power_reg_set_vout(4.0, true);
        // power_reg_set_vout(4.5, true);
        // power_reg_set_vout(5.0, true);

        int32_t distance;
        distance_sensor_get_mm(&distance);

        LOG_INF("Measured distance is %dmm", distance);

       power_reg_external_set_state(false, true);

        k_msleep(5000);
    }

}

void prod_test_bme688()
{
    LOG_INF("Testing the BME688 driver");

	const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, bosch_bme688)));
	struct sensor_value temp, press, humidity, gas_res;

	printk("Device %p name is %s\n", dev, dev->name);

	while (1) {
		k_sleep(K_MSEC(3000));

		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp); //C
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press); //Pa
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);//r.H. in %
		sensor_channel_get(dev, SENSOR_CHAN_GAS_RES, &gas_res); //gas sensor resistance

		printk("T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d\n",
				temp.val1, temp.val2, press.val1, press.val2,
				humidity.val1, humidity.val2, gas_res.val1,
				gas_res.val2);
	}
}

void prod_test_onboard_sensor()
{
    LOG_INF("Testing the on board sensors");
    onboard_sensor_tphg_init();

    double val;
    onboard_sensor_get_temp_c(&val);

    printf("Temp in c is %f\n", val);

    onboard_sensor_get_pressure_pa(&val);

    printf("Pressure in Pa is %f\n", val);

    onboard_sensor_get_humidity_rh(&val);

    printf("HUmidity in rH is %f%%\n", val);

    onboard_sensor_get_gas_res_ohm(&val);

    printf("Gas resistance in ohm is %f\n", val);

    onboard_sensor_accel_init();

    onboard_sensor_accel_get_sample(&val);

    printf("The angle in degree is %f\n", val);

    int16_t batt_level = 0;

    sensor_get_battery_mV(&batt_level);

    printk("The battery level in mV is %d\n", batt_level);

    unsigned int batt_level_pct = 0;

    sensor_get_battery_pct(&batt_level_pct, (unsigned int)batt_level);

    printf("The battery level in percent is %d%%\n", batt_level_pct);


}

void prod_test_cs_json()
{
    LOG_INF("Testing the JSON builder");

    char json_buffer[100];

    cs_json_init();

    cs_json_open(json_buffer, sizeof(json_buffer));
    cs_json_add_item_string(json_buffer, sizeof(json_buffer), "key1", "Hello");
    cs_json_add_item_double(json_buffer, sizeof(json_buffer), "key2", 5.123231);
    cs_json_add_item_int(json_buffer, sizeof(json_buffer), "key3", 2000);
    cs_json_add_item_bool(json_buffer, sizeof(json_buffer), "key4", false);
    cs_json_close(json_buffer, sizeof(json_buffer));

    printk("JSON buffer value is %s\n", json_buffer);
}

/**
 * @brief Conducts production testing
 * 
 * @return Does not return any
 */
void prod_test()
{
    LOG_INF("Prod Test has started!..");

    //prod_test_led();

    // prod_test_external_flash_lfs(false);

    //prod_test_network();

    //prod_test_mqtt();

    //prod_test_aws_shadow();

    //prod_test_external_reg();

    prod_test_distance_sensor();

    //prod_test_bme688();

    //prod_test_onboard_sensor();

    //prod_test_cs_json();

    LOG_INF("Prod Test Done!..");



    while(1);
}

static void prod_test_serial_cb(struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uint8_t rx_data = 0;
    static bool crc8_ready = false;
    bool next_idx = false;

    uart_irq_update(dev);

    if (uart_irq_rx_ready(dev))
    {
        if (uart_fifo_read(dev, &rx_data, 1))
        {
            if (rx_data == 'T' && serial_crc_correct)
            {
                // '#' is msg start delimiter for get the serial nember
                printk("serial number set!\n");
                // serial_rx_buf_index = 0;
                serial_usb_rx_done = true;
            }
            else
            {
                if (rx_data == 'R')
                {
                    // 'R' is msg start delimiter for get the serial nember
                    serial_rx_buf_index = 0;
                    printk("getting serial number...");
                }
                else if (serial_rx_buf_index < serial_usb_data_len)
                {
                    serial_buf[serial_rx_buf_index] = rx_data;
                    next_idx = true;
                }
                
                if (serial_rx_buf_index == serial_usb_data_len)
                {
                    printk("serial number %s\n", serial_buf);
                    serial_number_crc8 = crc8(serial_buf, serial_usb_data_len, SERIAL_CRC8_CHECKSUM_POLY, 0, false);
                    printk("crc: 0x%x\n", crc8(serial_buf, serial_usb_data_len, SERIAL_CRC8_CHECKSUM_POLY, 0, false));
                    crc8_ready = true;
                }

                if (crc8_ready)
                {
                    printk("checking CRC...");
                    // check crc
                    if (rx_data == serial_number_crc8)
                    {
                        // crc correct
                        serial_crc_correct = true;
                        printk("valid\n");
                    }
                    else
                    {
                        printk("invalid\n");
                    }
                    crc8_ready = false;
                }

                if (next_idx)
                {
                    serial_rx_buf_index++;
                }

                
            }
        }
    }
}

/**
 * @brief Resets the necessary variables for receiving data from the serial usb
 *
 * @param expected_data_len The length of the incoming data
 *
 * @return Does not return
 */
static void prod_test_serial_usb_set_read(uint16_t expected_data_len)
{
        serial_usb_data_len = expected_data_len;
        serial_usb_rx_done = false;
        serial_crc_correct = false;
        serial_rx_buf_index = 0;
}


static void prod_test_aws_fota_cb_handler(struct aws_fota_event *fota_evt)
{
	int err;

	if (fota_evt == NULL) {
		return;
	}

	switch (fota_evt->id) {
	case AWS_FOTA_EVT_START:
		if (aws_fota_get_job_id(current_job_id,
					sizeof(current_job_id)) <= 0) {
			snprintf(current_job_id, sizeof(current_job_id), "N/A");
		}
		LOG_INF("AWS_FOTA_EVT_START, job id = %s", log_strdup(current_job_id));
		break;

	case AWS_FOTA_EVT_DL_PROGRESS:
		/* CONFIG_FOTA_DOWNLOAD_PROGRESS_EVT must be enabled */
		/* to receive progress events */
		LOG_INF("AWS_FOTA_EVT_DL_PROGRESS, %d%% downloaded",
			fota_evt->dl.progress);
		break;

	case AWS_FOTA_EVT_DONE:
		LOG_INF("AWS_FOTA_EVT_DONE, rebooting to apply update");
		do_reboot = true;
		break;

	case AWS_FOTA_EVT_ERASE_PENDING:
		LOG_INF("AWS_FOTA_EVT_ERASE_PENDING, reboot or disconnect the "
		       "LTE link");
		err = modem_set_state(MODEM_OFFLINE);
		if (err) {
			LOG_ERR("Error turning off the LTE link");
			break;
		}
		lte_connected = false;
		break;

	case AWS_FOTA_EVT_ERASE_DONE:
		LOG_INF("AWS_FOTA_EVT_ERASE_DONE");

		if (lte_connected == false) {
			LOG_INF("Reconnecting the LTE link");

			err = modem_lte_connect();
			if (err) {
				LOG_ERR("Error reconnecting the LTE link");
				break;
			}
			lte_connected = true;
		}
		break;

	case AWS_FOTA_EVT_ERROR:
		LOG_INF("AWS_FOTA_EVT_ERROR");
		break;
	}
}


/**
 * 
 * @brief For subscribing and publishing, it spawns a new thread for polling the MQTT statuses
 *
 * @param serial_munber Pointer to char
 * @param payload Pointer to char
 *
 * @return Does not return
 */
static void prod_test_fota(char *serial_munber, char * payload)
{
	int err;
	mqtt_connected = false;

    // LTE
    modem_lte_init();

    if (modem_lte_connect() != 0)
    {
        LOG_ERR("Unable to connect to LTE");
        while(1);
    }

    // MQTT
    cloud_mqtt_init(&prod_client, serial_munber, &fds);

	err = aws_fota_init(&prod_client, prod_test_aws_fota_cb_handler);
	if (err != 0) {
		LOG_ERR("ERROR: aws_fota_init %d", err);
		return;
	}


    if ((mqtt_connect(&prod_client)) != 0)
    {
        LOG_ERR("Can't connect to the broker");
        while(1);
    }

    // spawn mqtt thread here
	mqtt_poll_tid = k_thread_create(&mqtt_poll_thread_data, mqtt_poll_thread_stack_area, 
							K_THREAD_STACK_SIZEOF(mqtt_poll_thread_stack_area), 
							(k_thread_entry_t)cloud_mqtt_poll_entry_point, &prod_client, &fds,
							NULL, MQTT_POLL_THREAD_PRIORITY, 0, K_NO_WAIT);

    // poll if mqtt has successfully been established
    uint8_t try_cnt = 0;
    uint8_t try_max = 120;
    while(mqtt_connected == false)
    {
        if (mqtt_connected)
        {
            break;
        }
        k_msleep(1000);

    }

    if (try_cnt >= try_max)
    {
        
        LOG_ERR("Unable to connect to the broker");
        while(1);
    }
    

    err = cloud_mqtt_publish_to_topic(&prod_client, MQTT_QOS_1_AT_LEAST_ONCE, 
                                        payload, strlen(payload), PROD_TEST_TOPIC, 
                                        serial_munber);
    if (err != 0)
    {
        LOG_ERR("Unable to publish test result");
        while(1);
    }
    

	/* All initializations were successful mark image as working so that we
	 * will not revert upon reboot.
	 */
	//boot_write_img_confirmed();	

	while(1)
    {
        k_msleep(500); //stay here until we get the FWU
            //publish test result here
        if (do_reboot) {
            /* Teardown */
            //TODO: implement mutex between this task and the main task if ever there will be an issue
            // TODO: suspend the mqtt task
            mqtt_disconnect(&prod_client);
            sys_reboot(0);
		}
    }
    

}

#define PROD_SERIAL_NUMBER_LEN (sizeof("66e67370-b6ce-4ae0-97a1-8136b8e910e3"))
void prod_test_pre_production()
{
    int err = 0;
    bool certificates_found = false;
    char test_result_buffer[256];

    // prelim - test
    // common sensors
    onboard_sensor_tphg_init();

    onboard_sensor_accel_init();



    // Provisioning
    char response[256];
    char list_cert_cmd[25] = "AT%%CMNG=1,12345678";
    memset(response, '\0', sizeof(response));
    while (certificates_found == false)
    {
        led_set_state(GREEN_LED, LED_ON);
        led_set_state(RED_LED, LED_ON);
        // cert writing       
        err = nrf_modem_at_cmd(response, sizeof(response), "AT%%CMNG=1,12345678"); 
        if ((err != 0) && (err != -7))
        {
            LOG_ERR("Error after sending %s, err -> %d", log_strdup(list_cert_cmd), err);
            // err = -7 -> NRF_E2BIG -> argument list too long
            
            //while(1);
        }

        LOG_DBG("AT%%CMNG=1:%s", log_strdup(response));

        int match = strcmp(response, "OK\r\n");

        if (strlen(response) > 0  && strcmp(response, "OK\r\n") != 0)
        {
            certificates_found = true;
            k_msleep(1000);
            break;
        }

        k_msleep(60000);
    }

    // while(1)k_msleep(1000);
    led_set_state(GREEN_LED, LED_OFF);
    led_set_state(RED_LED, LED_OFF);

    while(1)k_msleep(10);

    // wait until LED turns Yellow
    led_set_state(GREEN_LED, LED_ON);
    led_set_state(BLUE_LED, LED_ON);

    k_msleep(100);

    if (is_file_exists(SERIAL_NUMBER_FILE_NAME) == false || is_file_exists(SENSOR_TYPE_FILE_NAME) == false)
    {
        // app type, sensor type and serial writing
        const struct device *usb = device_get_binding(DT_LABEL(DT_NODELABEL(uart0)));

        if (usb == NULL)
        {
            LOG_ERR("ERROR: cant initialized USB device");
            while(1);
        }
        else
        {
            struct uart_config cfg;
            if (uart_config_get(usb, &cfg) != 0)
            {
                LOG_ERR("ERROR: cant get UART config");
                while(1);
            }
            
            uart_irq_callback_set(usb, (uart_irq_callback_user_data_t)prod_test_serial_cb);

            LOG_INF("USB configured -> %d baud", cfg.baudrate);
        }

        uart_irq_rx_enable(usb);

    }

    if (is_file_exists(SENSOR_TYPE_FILE_NAME) == false)
    {
        //acquiring the type of sensor that needs to be tested
        prod_test_serial_usb_set_read(SERIAL_USB_SENSOR_TYPE_LEN);

        while(!serial_usb_rx_done)k_msleep(2);

        int16_t sensor_type = atoi(serial_buf);

        uint8_t data[2];
        data[0] = sensor_type & 0xff;
        data[1] = sensor_type >> 8 & 0xff;

        err = save_to_file(data, sizeof(data),SENSOR_TYPE_FILE_NAME);
    }

    if (is_file_exists(SERIAL_NUMBER_FILE_NAME) == false)
    {
        char ser_num[SERIAL_NUMBER_LEN];
        prod_test_serial_usb_set_read((uint16_t)PROD_SERIAL_NUMBER_LEN - 1);

        while(!serial_usb_rx_done)k_msleep(2);

        memset(ser_num, '\0', sizeof(ser_num));
        strcpy(ser_num, serial_buf);

        err = save_to_file(ser_num, strlen(ser_num),SERIAL_NUMBER_FILE_NAME);
    }

    led_set_state(GREEN_LED, LED_OFF);
    led_set_state(BLUE_LED, LED_OFF);

    k_msleep(10);

    // wait until LED turns Magenta
    led_set_state(RED_LED, LED_ON);
    led_set_state(BLUE_LED, LED_ON);

    device_config_t device_config;
    device_config_init(&device_config);

    // final - test

    // fota
    // fota(ser_num, test_result_buffer);
    while(1);
}

#endif //PROD_TEST