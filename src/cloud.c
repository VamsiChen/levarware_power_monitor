/*
 * Copyright (c) 2022 Reliance Foundry Co. Ltd.
 *
 * This module should really be refactored with the lte_connectiton manager.
 * The LTE and AWS handling should become stateful and the JSON logic seperated out into another module. 
 *
 */

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <modem/modem_info.h>
#include <nrf_modem.h>
#include <date_time.h>
#include <net/aws_iot.h>
#include <sys/reboot.h>
#include "sys/base64.h"


#include <dfu/mcuboot.h>
#include <cJSON.h>
#include <cJSON_os.h>

#include <string.h>
extern char *strtok_r(char *, const char *, char **);

#include <logging/log.h>
LOG_MODULE_REGISTER(cloud);

#include "cloud.h"
#include "lte_connect_mgr.h"
#include "apps/apps.h"
#include "apps/audio.h"
#include "apps/distance.h"
#include "device_config/device_config.h"
#include "sensors/onboard_sensor.h"
#include "sensors/battery.h"
#include "subsys/modem.h"
#include "cs_json.h"

static struct k_work_delayable shadow_update_work;
static struct k_work_delayable connect_work;
static struct k_work shadow_update_version_work;

//this list stores all the callback fucntions to the apps
static struct _slist shadow_update_list;

// struct to the notification block, handles all the data of the list and callback fucntions
static struct shadow_notification_blk *nblkp;

struct aws_iot_config aws_iot;

static bool cloud_connected = false;

K_SEM_DEFINE(shadow_received, 0, 1);

char json_payload[CONFIG_TELEMETRY_PAYLOAD_BUFFER_SIZE];

/* 
* @brief    Global interface function: Called from lte_connection_mgr to inform cloud module that
*			LTE modem connection has changed state (up or down)  
*
* @param    bool - true means LTE up, false means LTE down 
*
* @return   nothing
*
* @note     
*/
void lte_connection_change(bool lte_up)

{
	if (lte_up)
	{
		// kick the cloud connection worker thread to process now and bring up AWS link, don't bother waiting
		k_work_schedule(&connect_work, K_NO_WAIT);
	}

	else 
	{

		LOG_INF("LTE PDP context is down, disconnecting from AWS to clean up");

		// lost the LTE network registration, cloud is not connected
		cloud_connected = false;

		// and call the aws disconnect function to clean up after a loss of connection. See
		// asset_tracker sample application for an example/discussion 
		aws_iot_disconnect();

		// Note: the cloud connection worker thread will be kicked to run once LTE modem is back up
	}

}



static int json_add_obj(cJSON *parent, const char *str, cJSON *item)
{
    cJSON_AddItemToObject(parent, str, item);

    return 0;
}

static int json_add_str(cJSON *parent, const char *str, const char *item)
{
    cJSON *json_str;

    json_str = cJSON_CreateString(item);
    if (json_str == NULL) {
        return -ENOMEM;
    }

    return json_add_obj(parent, str, json_str);
}

static int json_add_number(cJSON *parent, const char *str, double item)
{
    cJSON *json_num;

    json_num = cJSON_CreateNumber(item);
    if (json_num == NULL) {
        return -ENOMEM;
    }

    return json_add_obj(parent, str, json_num);
}

static void date_time_event_handler(const struct date_time_evt *evt)
{
    switch (evt->type) {
    case DATE_TIME_OBTAINED_MODEM:
        LOG_INF("DATE_TIME_OBTAINED_MODEM");
        break;
    case DATE_TIME_OBTAINED_NTP:
        LOG_WRN("DATE_TIME_OBTAINED_NTP");
        break;
    case DATE_TIME_OBTAINED_EXT:
        LOG_WRN("DATE_TIME_OBTAINED_EXT");
        break;
    case DATE_TIME_NOT_OBTAINED:
        LOG_ERR("DATE_TIME_NOT_OBTAINED");
        break;
    default:
        break;
    }
}

static int shadow_get(void)
{
    struct aws_iot_data tx_data = {
        .qos = MQTT_QOS_1_AT_LEAST_ONCE,
        .topic.type = AWS_IOT_SHADOW_TOPIC_GET,
        .ptr = "",
        .len = 0
    };

    int err = aws_iot_send(&tx_data);
    if (err) {
        LOG_ERR("aws_iot_send, error: %d", err);
    }

    return err;
}

static int shadow_update(bool version_number_include)
{
    static int64_t prev_ts = 0;

    int err;
    char *message;
    int64_t message_ts = 0;

    err = date_time_now(&message_ts);
    if (err || message_ts <= prev_ts) {
        LOG_ERR("date_time_now, error: %d", err);
        // try to update time for next attempt - timestamp error detection and correction handled
        // in cloud so we can keep bad timestamps for a period of time if necessary
        date_time_update_async(date_time_event_handler);
        // return err;
    }

    prev_ts = message_ts;

    cJSON *root_obj = cJSON_CreateObject();
    cJSON *state_obj = cJSON_CreateObject();
    cJSON *reported_obj = cJSON_CreateObject();
    cJSON *attributes_obj = cJSON_CreateObject();

    if (root_obj == NULL || state_obj == NULL || reported_obj == NULL || attributes_obj == NULL) {
        cJSON_Delete(root_obj);
        cJSON_Delete(state_obj);
        cJSON_Delete(reported_obj);
        cJSON_Delete(attributes_obj);
        err = -ENOMEM;
        return err;
    }

    if (version_number_include) {
        err = json_add_str(reported_obj, "fw_version", device_config_get_fw_version_str());
    } else {
        err = 0;
    }

    err += json_add_number(reported_obj, "ts", message_ts);

    int16_t batt_mv = sensor_get_battery_mV();
    err += json_add_number(reported_obj, "batt_mv", batt_mv);
    // use batt_mv to calc soc
    err += json_add_number(reported_obj, "batt_pct", sensor_get_battery_soc(batt_mv));

    // using the shadow service version number for both version fields
    err += json_add_number(reported_obj, "version", device_config_get_int(DEV_CONFIG_CONF_VERSION));
    // err += json_add_number(attributes_obj, "config_version", device_config_get_int(DEV_CONFIG_CONF_VERSION));
    err += json_add_str(reported_obj, "iccid", device_config_get_str(DEV_CONFIG_ICCID));
    err += json_add_number(attributes_obj, "config_interval_s", device_config_get_int16(DEV_CONFIG_CONF_UPDATE_INTERVAL_S));
    err += json_add_number(attributes_obj, "daq_interval_s", device_config_get_int16(DEV_CONFIG_DAQ_INTERVAL_S));
    err += json_add_number(attributes_obj, "pub_interval_s", device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S));
    err += json_add_number(attributes_obj, "sensor_type", device_config_get_int16(DEV_CONFIG_SENSOR_TYPE));
    err += json_add_number(attributes_obj, "app_type", device_config_get_int16(DEV_CONFIG_APP_TYPE));
    err += json_add_str(attributes_obj, "topic", device_config_get_str(DEV_CONFIG_PUB_TOPIC));

    err += json_add_obj(reported_obj, "attributes", attributes_obj);
    err += json_add_obj(state_obj, "reported", reported_obj);
    err += json_add_obj(root_obj, "state", state_obj);

    if (err) {
        LOG_ERR("json_add, error: %d", err);
        goto cleanup;
    }

    message = cJSON_PrintUnformatted(root_obj);
    if (message == NULL) {
        LOG_ERR("cJSON_Print, error: returned NULL");
        err = -ENOMEM;
        goto cleanup;
    }

    struct aws_iot_data tx_data = {
        .qos = MQTT_QOS_1_AT_LEAST_ONCE,
        .topic.type = AWS_IOT_SHADOW_TOPIC_UPDATE,
        .ptr = message,
        .len = strlen(message)
    };

    LOG_DBG("Publishing: %s to AWS IoT broker", message);

    err = aws_iot_send(&tx_data);
    if (err) {
        LOG_ERR("aws_iot_send, error: %d", err);
    }

    //A node to be used for cycling through all apps after a shadow update
    struct _snode* nodep;
    

    // run thru the list of registered notification functions to signal that a change in the shadow has occurred
    nodep = sys_slist_peek_head(&shadow_update_list);
    while (nodep != NULL) {

        nblkp = (struct shadow_notification_blk *)nodep;
        LOG_DBG("Shadow updated, notifying apps to check for updates");

        nblkp->notification_callback_fn((nblkp->userp));

        nodep = sys_slist_peek_next(nodep);
    }

    cJSON_FreeString(message);

cleanup:
    cJSON_Delete(root_obj);
    return err;
}


static void connect_work_fn(struct k_work *work)
{
    int err;

    if (cloud_connected) {
        return;
    }

	/*
	 * We aren't connected to the cloud, if the LTE link is up, let's try
	 */
	if (lte_check_pdp_context())
	{
        LOG_DBG("Link up, calling aws_iot_connect to bring up session");

		// now that LTE modem is registered on the network, try to bring up AWS session
		err = aws_iot_connect(&aws_iot);
		if (err) {
			LOG_ERR("aws_iot_connect, error: %d", err);
		}

		LOG_DBG("Next connection retry in %d seconds", CONFIG_CONNECTION_RETRY_TIMEOUT_SECONDS);
		k_work_schedule(&connect_work, K_SECONDS(CONFIG_CONNECTION_RETRY_TIMEOUT_SECONDS));
	}

	// else LTE link is not up (no pdp context), when it comes up, the cloud module will
	// get called and kick this worker thread to try again 

}

static void shadow_update_work_fn(struct k_work *work)
{
    int err;

	if (!cloud_connected) {
		return;				// TODO - do we need to reschedule here?
	}

    err = shadow_get();
    if (err == 0)
    {
        // wait for shadow handler to run
        if (k_sem_take(&shadow_received, K_SECONDS(10)) == 0)
        {
            err = shadow_update(false);
            if (err)
            {
                printk("shadow_update, error: %d", err);
            }
        }
        else
        {   
            LOG_ERR("Shadow GET request timed out");
        }
    }

    int next_shadow_update_s = device_config_get_int16(DEV_CONFIG_CONF_UPDATE_INTERVAL_S);
    LOG_DBG("Next shadow update in %d seconds", next_shadow_update_s);

    k_work_schedule(&shadow_update_work, K_SECONDS(next_shadow_update_s));

    // update date_time on every config interval - library default is to do this automatically every 4 hours, but we seem to lose the timestamp sometimes
    date_time_update_async(date_time_event_handler);
}

static void shadow_update_version_work_fn(struct k_work *work)
{
    int err;

    err = shadow_update(true);
    if (err) {
        LOG_ERR("shadow_update, error: %d", err);
    }
}

// static void print_received_data(const char *buf, const char *topic,
// 				size_t topic_len)
// {
// 	char *str = NULL;
// 	cJSON *root_obj = NULL;

// 	root_obj = cJSON_Parse(buf);
// 	if (root_obj == NULL) {
// 		LOG_ERR("cJSON Parse failure");
// 		return;
// 	}

// 	str = cJSON_Print(root_obj);
// 	if (str == NULL) {
// 		LOG_ERR("Failed to print JSON object");
// 		goto clean_exit;
// 	}

// 	LOG_DBG("Data received from AWS IoT console:\nTopic: %.*s\nMessage: %s",
// 	       topic_len, topic, str);

// 	cJSON_FreeString(str);

// clean_exit:
// 	cJSON_Delete(root_obj);
// }

void aws_iot_event_handler(const struct aws_iot_evt *const evt)
{
    switch (evt->type) {
    case AWS_IOT_EVT_CONNECTING:
        LOG_INF("AWS_IOT_EVT_CONNECTING");
        break;
    case AWS_IOT_EVT_CONNECTED:
        LOG_INF("AWS_IOT_EVT_CONNECTED");

        cloud_connected = true;

        /*
        *   Given we have connected to AWS,  LTE connection manager that application connectivity is up. 
        *   This was added to help track/recover from connectivity issues issue in the field.  
        */
        lte_application_conn_up(true);


        /* This may fail if the work item is already being processed,
         * but in such case, the next time the work handler is executed,
         * it will exit after checking the above flag and the work will
         * not be scheduled again.
         */
        (void)k_work_cancel_delayable(&connect_work);

        if (evt->data.persistent_session) {
            LOG_INF("Persistent session enabled");
        }

        /** Successfully connected to AWS IoT broker, mark image as
         *  working to avoid reverting to the former image upon reboot.
         */
        boot_write_img_confirmed();

        /** Send version number to AWS IoT broker to verify that the
         *  FOTA update worked.
         */
        k_work_submit(&shadow_update_version_work);

        /// @brief Start sequential shadow data updates.
        /// @param evt
        k_work_schedule(&shadow_update_work, K_NO_WAIT);

        int err = lte_lc_psm_req(true);
        if (err) {
            LOG_ERR("Requesting PSM failed, error: %d", err);
        }
        break;
    case AWS_IOT_EVT_READY:
        LOG_INF("AWS_IOT_EVT_READY");
        break;
    case AWS_IOT_EVT_DISCONNECTED:
        LOG_INF("AWS_IOT_EVT_DISCONNECTED");
        cloud_connected = false;
        /* This may fail if the work item is already being processed,
         * but in such case, the next time the work handler is executed,
         * it will exit after checking the above flag and the work will
         * not be scheduled again.
         */
        (void)k_work_cancel_delayable(&shadow_update_work);
        k_work_schedule(&connect_work, K_SECONDS(CONFIG_CONNECTION_RETRY_TIMEOUT_SECONDS));
        break;
    case AWS_IOT_EVT_DATA_RECEIVED:
        LOG_INF("AWS_IOT_EVT_DATA_RECEIVED");

        printk("Data received from AWS IoT console:\r\nTopic: %.*s\r\nMessage[len: %d]: %s\r\n",
                       evt->data.msg.topic.len,
                    evt->data.msg.topic.str,
                    evt->data.msg.len,
                    evt->data.msg.ptr);

        // for now we are only expecting shadow data for processing, but this will have to be
        // changed to handle different types of data
        device_config_shadow_data_handler(evt->data.msg.ptr, evt->data.msg.len);
        k_sem_give(&shadow_received);

        // when this occurs async from a sub event, we need to send the updated state back to the shadow,
        // otherwise, the delta state will stay in the shadow until reported is updated on next config interval
        // and will be processed again, needlesly
        k_work_submit(&shadow_update_version_work);
        break;
    case AWS_IOT_EVT_FOTA_START:
        LOG_WRN("AWS_IOT_EVT_FOTA_START");
        break;
    case AWS_IOT_EVT_FOTA_ERASE_PENDING:
        LOG_WRN("AWS_IOT_EVT_FOTA_ERASE_PENDING");
        LOG_WRN("Disconnect LTE link or reboot");
        err = lte_lc_offline();
        if (err) {
            LOG_ERR("Error disconnecting from LTE");
        }
        break;
    case AWS_IOT_EVT_FOTA_ERASE_DONE:
        LOG_WRN("AWS_FOTA_EVT_ERASE_DONE");
        LOG_WRN("Reconnecting the LTE link");
        err = lte_lc_connect();
        if (err) {
            LOG_ERR("Error connecting to LTE");
        }
        break;
    case AWS_IOT_EVT_FOTA_DONE:
        LOG_WRN("AWS_IOT_EVT_FOTA_DONE");
        LOG_WRN("FOTA done, rebooting device");
        aws_iot_disconnect();
        sys_reboot(0);
        break;
    case AWS_IOT_EVT_FOTA_DL_PROGRESS:
        LOG_WRN("AWS_IOT_EVT_FOTA_DL_PROGRESS, (%d%%)",
               evt->data.fota_progress);
    case AWS_IOT_EVT_ERROR:
        LOG_ERR("AWS_IOT_EVT_ERROR, %d", evt->data.err);
        break;
    case AWS_IOT_EVT_FOTA_ERROR:
        LOG_ERR("AWS_IOT_EVT_FOTA_ERROR");
        break;
    default:
        LOG_ERR("Unknown AWS IoT event type: %d", evt->type);
        break;
    }
}


static void nrf_modem_lib_dfu_handler(void)
{
    int err;

    err = nrf_modem_lib_get_init_ret();

    switch (err) {
    case MODEM_DFU_RESULT_OK:
        LOG_WRN("Modem update suceeded, reboot");
        sys_reboot(SYS_REBOOT_COLD);
        break;
    case MODEM_DFU_RESULT_UUID_ERROR:
    case MODEM_DFU_RESULT_AUTH_ERROR:
        LOG_ERR("Modem update failed, error: %d", err);
        LOG_WRN("Modem will use old firmware");
        sys_reboot(SYS_REBOOT_COLD);
        break;
    case MODEM_DFU_RESULT_HARDWARE_ERROR:
    case MODEM_DFU_RESULT_INTERNAL_ERROR:
        LOG_ERR("Modem update malfunction, error: %d, reboot", err);
        sys_reboot(SYS_REBOOT_COLD);
        break;
    default:
        break;
    }
}

// static int app_topics_subscribe(char *serial_number)
// {
// 	int err;
// 	static char dt_topic[75];
// 	snprintk(dt_topic, 75,  "dt/%s", serial_number);

// 	const struct aws_iot_topic_data topics_list[APP_TOPICS_COUNT] = {
// 		[0].str = dt_topic,
// 		[0].len = strlen(dt_topic)
// 	};

// 	err = aws_iot_subscription_topics_add(topics_list,
// 					      ARRAY_SIZE(topics_list));
// 	if (err) {
// 		LOG_ERR("aws_iot_subscription_topics_add, error: %d", err);
// 	}

// 	return err;
// }

static void work_init(void)
{
    k_work_init_delayable(&shadow_update_work, shadow_update_work_fn);
    k_work_init_delayable(&connect_work, connect_work_fn);
    k_work_init(&shadow_update_version_work, shadow_update_version_work_fn);
}

int cloud_start(void)
{
    int err;

    nrf_modem_lib_dfu_handler();

    size_t ser_len = strlen(device_config_get_serial_number());
    aws_iot.client_id_len = ser_len;
    aws_iot.client_id = malloc(sizeof(char) * (ser_len + 1));
    strcpy(aws_iot.client_id, device_config_get_serial_number());

    err = aws_iot_init(&aws_iot, aws_iot_event_handler);
    if (err) {
        LOG_ERR("AWS IoT library could not be initialized, error: %d", err);
        return err;
    }

    // /** Subscribe to customizable non-shadow specific topics
    //  *  to AWS IoT backend.
    //  */
    // err = app_topics_subscribe("00000000-0000-0000-0000-000000000000");
    // if (err) {
    // 	LOG_ERR("Adding application specific topics failed, error: %d", err);
    // }

    // start app work tasks
    work_init();

	// TODO - determine if should be moved before AWS iot init
	// initialize and start the modem, it blocks block until a connection made
	lte_connect_init();

	date_time_update_async(date_time_event_handler);
	k_work_schedule(&connect_work, K_NO_WAIT);

    return 0;
}

void cloud_send_json_str(char *topic, char *json_str)
{
    struct aws_iot_data tx_data = {
        .qos = MQTT_QOS_1_AT_LEAST_ONCE,
        .topic.str = topic,
        .topic.len = strlen(topic),
        .ptr = json_str,
        .len = strlen(json_str)
    };

    // only send to cloud if it's connected
    if (cloud_connected)
    {
        int err = aws_iot_send(&tx_data);
        if (err)
            LOG_ERR("aws_iot_send, error: %d", err);
    }

    // cloud is not connected, do a health check of the modem
    else
    {
      	// TODO: handle store-forward
		LOG_WRN("telemetry update failed -> cloud not connected!");

		// notify LTE connection manager that connectivity is down and potentially in a bad state and recover
		lte_application_conn_up(false);

    }

}

/** 
* @brief   Registers the notification callback functions for applications that want to be  notified on changes to the shadow. 
*
* @param    struct shadow_notification_blk              *nblk_ptr
*           notification_callback_fn                    callback
            void                                    *userp
*
* @return  nothing
*
* @note    This function registers all the callback functions to be called whhen a shadow change occurs and feeds them into a singly linked list
*/

void cloud_register_shadow_change(struct shadow_notification_blk *nblk_ptr, void (*notification_callback_fn)(void *userp), void *userp)
{
    
    //point to the callback fucntion
    nblk_ptr->notification_callback_fn = notification_callback_fn;

    nblk_ptr->userp = userp;
    
    // add the node to the list
    sys_slist_append(&shadow_update_list, &(nblk_ptr->node)); 

}
