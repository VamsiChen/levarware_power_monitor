#include <zephyr.h>
#include <string.h>
#include <stdio.h>
#include <sys/reboot.h>

#include <cJSON.h>
#include <cJSON_os.h>
#include <modem/modem_info.h>

/*Logging*/
#include <logging/log.h>
LOG_MODULE_REGISTER(dev_config);


#include "device_config.h"
#include "subsys/filesystem.h"

#include "subsys/modem.h"

static device_config_t device_config;

static void device_config_update_from_shadow(cJSON *attributes, int32_t version, bool is_delta);



/*
*   This device configuration design needs to be rethought. In the current design, each one of the device attibutes is in it's own file. Those files are created 
*   and initialized during the provisioning process. The problem is that when new attributes are defined, those files do not exist. I am not sure what the thinking 
*   was ... to create the attibutre files later when missing? Why not just initialize all of them that are not present at boot-up. Default them in the application
*   when not present. That would be much cleaner than the current solution. Added to the list of things to refactor. 
*
*   For now, skip over new attibutes/fields that have been added. You will some of that below as we work around that issue.
*
*/


/** 
*  @brief   Get firmware version string
* 
* @param    none
*
* @return   pointer to version number string (NULL terminated)
*
* @note    
*/

char *device_config_get_fw_version_str(void)
{
    return CONFIG_FIRMWARE_VERSION;
}

  
/**
 * @brief Initializes the device configurations
 * 
 * @return Does not return any
 */
void device_config_init(void)
{
    LOG_INF("Initializing device configurations");
    
    int err = 0;

    char ser_num[SERIAL_NUMBER_LEN];
    memset(ser_num, '\0', sizeof(ser_num));
    if (false == is_file_exists(SERIAL_NUMBER_FILE_NAME))
    {
        // should not get here
        LOG_ERR("The file for serial number does not exist");      
    }
    else
    {
        err = read_file(ser_num, sizeof(ser_num), SERIAL_NUMBER_FILE_NAME);
        if (err <= 0)
        {
            // should not get here
            LOG_ERR("Unable to read the serial number file");
        }
        else
        {
            strcpy(device_config.serial_number, ser_num);
        }
    }
    
    /* 
    * We need to get information from the modem like the IMEI. Before using these functions, we need to initialize the modem information service. In earlier versions of the code, 
    * we skipped that step and that's why we failed to get IMEI. 
    * We will also get the ICCID of the SIM card as that makes it easier to manage SIM cards and service subscription like Eseye SIM.
    */
    modem_info_init();

    // Get IMEI 
    modem_get_IMEI(device_config.imei, IMEI_LEN);

    // Getting ICCID
    modem_get_ICCID(device_config.iccid, ICCID_LEN);

    device_config_dev_shadow_attrib_init();

    LOG_INF("Serial number: %s", log_strdup(device_config.serial_number));
    LOG_INF("IMEI: %s", log_strdup(device_config.imei));
    LOG_INF("fw-version: %s", CONFIG_FIRMWARE_VERSION);
    LOG_INF("ICCID: %s", log_strdup(device_config.iccid));
    LOG_INF("daq_interval_s: %d s", device_config_get_int16(DEV_CONFIG_DAQ_INTERVAL_S));
    LOG_INF("pub_interval_s: %d s", device_config_get_int16(DEV_CONFIG_PUB_INTERVAL_S));
    LOG_INF("config_interval_s: %d s", device_config_get_int16(DEV_CONFIG_CONF_UPDATE_INTERVAL_S));
    LOG_INF("sensor_type: %d", device_config_get_int16(DEV_CONFIG_SENSOR_TYPE));
    LOG_INF("app_type: %d", device_config_get_int16(DEV_CONFIG_APP_TYPE));
    LOG_INF("config_version: %d", device_config_get_int(DEV_CONFIG_CONF_VERSION));  
    LOG_INF("pub_topic: %s", log_strdup(device_config_get_str(DEV_CONFIG_PUB_TOPIC)));
    LOG_INF("sub_topic: %s", log_strdup(device_config_get_str(DEV_CONFIG_SUB_TOPIC)));

    // ensure all the config files exist by walking through them and saving attributes??
    for (dev_config_shadow_id_t attr = 0; attr < DEV_CONFIG_NUM; attr++)
    {

        /* check to ensure the attribute is not one of the newly added fields (because the file will not exist)
        *  The ICCID seems to be the first new file but there will be many more over time. 
        *  TODO - should create that file or any files that don't exist
        */

        if (attr != DEV_CONFIG_ICCID)
        {
            if (false == is_file_exists(device_config.dev_shadow_attrib[attr].filename))
            {
                device_config_save_attribute_to_file(attr);
            }
        }
    }

    // TODO: implmenet string based values in file contents - compat rewrite could happen here (imoon 20220904)
}

/**
 * @brief Initializes the device shadow configurations
 * 
 * @return Does not return any
 */
void device_config_dev_shadow_attrib_init(void)
{

int num_bytes;      // number of bytes read from file

    // dev_shadow_attrib_t *shadow = device_config.dev_shadow_attrib;

    // load defaults for each attribute
    device_config.dev_shadow_attrib[DEV_CONFIG_DAQ_INTERVAL_S].dev_conf_shadow_id = DEV_CONFIG_DAQ_INTERVAL_S;
    device_config.dev_shadow_attrib[DEV_CONFIG_DAQ_INTERVAL_S].val_type = DEV_CONFIG_VAL_TYPE_INT16;
    device_config.dev_shadow_attrib[DEV_CONFIG_DAQ_INTERVAL_S].int16_val = DAQ_INTERVAL_DEFAULT_VAL_S;
    device_config.dev_shadow_attrib[DEV_CONFIG_DAQ_INTERVAL_S].is_new_val = false;
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_DAQ_INTERVAL_S].filename, DAQ_INTERVAL_FILE_NAME);
    
    device_config.dev_shadow_attrib[DEV_CONFIG_PUB_INTERVAL_S].dev_conf_shadow_id = DEV_CONFIG_PUB_INTERVAL_S;
    device_config.dev_shadow_attrib[DEV_CONFIG_PUB_INTERVAL_S].val_type = DEV_CONFIG_VAL_TYPE_INT16;
    device_config.dev_shadow_attrib[DEV_CONFIG_PUB_INTERVAL_S].int16_val = PUB_INTERVAL_DEFAULT_VAL_S;
    device_config.dev_shadow_attrib[DEV_CONFIG_PUB_INTERVAL_S].is_new_val = false;
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_PUB_INTERVAL_S].filename, PUB_INTERVAL_FILE_NAME);
    
    device_config.dev_shadow_attrib[DEV_CONFIG_ICCID].dev_conf_shadow_id = DEV_CONFIG_ICCID;
    device_config.dev_shadow_attrib[DEV_CONFIG_ICCID].val_type = DEV_CONFIG_VAL_TYPE_STRING;
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_ICCID].str_val, ICCID_DEFAULT_VAL);
    device_config.dev_shadow_attrib[DEV_CONFIG_ICCID].is_new_val = false;
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_ICCID].filename, ICCID_INTERNAL_FILE_NAME);

    device_config.dev_shadow_attrib[DEV_CONFIG_CONF_UPDATE_INTERVAL_S].dev_conf_shadow_id = DEV_CONFIG_CONF_UPDATE_INTERVAL_S;
    device_config.dev_shadow_attrib[DEV_CONFIG_CONF_UPDATE_INTERVAL_S].val_type = DEV_CONFIG_VAL_TYPE_INT16;
    device_config.dev_shadow_attrib[DEV_CONFIG_CONF_UPDATE_INTERVAL_S].int16_val = CONFIG_INTERVAL_DEFAULT_VAL_S;
    device_config.dev_shadow_attrib[DEV_CONFIG_CONF_UPDATE_INTERVAL_S].is_new_val = false;
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_CONF_UPDATE_INTERVAL_S].filename, CONFIG_INTERVAL_FILE_NAME);
    
    device_config.dev_shadow_attrib[DEV_CONFIG_SENSOR_TYPE].dev_conf_shadow_id = DEV_CONFIG_SENSOR_TYPE;
    device_config.dev_shadow_attrib[DEV_CONFIG_SENSOR_TYPE].val_type = DEV_CONFIG_VAL_TYPE_INT16;
    device_config.dev_shadow_attrib[DEV_CONFIG_SENSOR_TYPE].int16_val = EXT_SENSOR_TERABEE;
    device_config.dev_shadow_attrib[DEV_CONFIG_SENSOR_TYPE].is_new_val = false;
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_SENSOR_TYPE].filename, SENSOR_TYPE_FILE_NAME);

    device_config.dev_shadow_attrib[DEV_CONFIG_APP_TYPE].dev_conf_shadow_id = DEV_CONFIG_APP_TYPE;
    device_config.dev_shadow_attrib[DEV_CONFIG_APP_TYPE].val_type = DEV_CONFIG_VAL_TYPE_INT16;
    device_config.dev_shadow_attrib[DEV_CONFIG_APP_TYPE].int16_val = APP_TYPE_DEFAULT_VAL;
    device_config.dev_shadow_attrib[DEV_CONFIG_APP_TYPE].is_new_val = false;
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_APP_TYPE].filename, APP_TYPE_FILE_NAME);

    device_config.dev_shadow_attrib[DEV_CONFIG_CONF_VERSION].dev_conf_shadow_id = DEV_CONFIG_CONF_VERSION;
    device_config.dev_shadow_attrib[DEV_CONFIG_CONF_VERSION].val_type = DEV_CONFIG_VAL_TYPE_INT;
    device_config.dev_shadow_attrib[DEV_CONFIG_CONF_VERSION].int_val = CONFIG_VERSION_DEFAULT_VAL;
    device_config.dev_shadow_attrib[DEV_CONFIG_CONF_VERSION].is_new_val = false;
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_CONF_VERSION].filename, CONFIG_VERSION_FILE_NAME);

    device_config.dev_shadow_attrib[DEV_CONFIG_PUB_TOPIC].dev_conf_shadow_id = DEV_CONFIG_PUB_TOPIC;
    device_config.dev_shadow_attrib[DEV_CONFIG_PUB_TOPIC].val_type = DEV_CONFIG_VAL_TYPE_STRING;
    memset(device_config.dev_shadow_attrib[DEV_CONFIG_PUB_TOPIC].str_val, '\0', sizeof(device_config.dev_shadow_attrib[DEV_CONFIG_PUB_TOPIC].str_val));
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_PUB_TOPIC].str_val, PUB_TOPIC_DEFAULT_VAL);
    device_config.dev_shadow_attrib[DEV_CONFIG_PUB_TOPIC].is_new_val = false;
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_PUB_TOPIC].filename, PUB_TOPIC_FILE_NAME);

    device_config.dev_shadow_attrib[DEV_CONFIG_SUB_TOPIC].dev_conf_shadow_id = DEV_CONFIG_SUB_TOPIC;
    device_config.dev_shadow_attrib[DEV_CONFIG_SUB_TOPIC].val_type = DEV_CONFIG_VAL_TYPE_STRING;
    memset(device_config.dev_shadow_attrib[DEV_CONFIG_SUB_TOPIC].str_val, '\0', sizeof(device_config.dev_shadow_attrib[DEV_CONFIG_SUB_TOPIC].str_val));
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_SUB_TOPIC].str_val, SUB_TOPIC_DEFAULT_VAL);
    device_config.dev_shadow_attrib[DEV_CONFIG_SUB_TOPIC].is_new_val = false;
    strcpy(device_config.dev_shadow_attrib[DEV_CONFIG_SUB_TOPIC].filename, SUB_TOPIC_FILE_NAME);

    // load from flash and overwrite defaults
    for (dev_config_shadow_id_t idx = 0; idx < DEV_CONFIG_NUM; idx++)
    {
        char file_contents[256] = {'\0'};

        /*
        *   We have a design issue with how the system was orginally designed in that these 
        *   files are created during provisioning. The problem is that as we add new files, 
        *   those files are not present. It would have been much cleaner/better to do that during bootup. 
        *   This will take some more work to fix but for now, for new fields in the shadow, skip over them 
        *   to not cause errors. 
        * 
        *   The ICCID field is new, so not it filesystem, skip over it
        */

       if (idx != DEV_CONFIG_ICCID)
       {
            num_bytes = read_file(file_contents, sizeof(file_contents), device_config.dev_shadow_attrib[idx].filename);

            /*
            * If the file read was successful, save the file contents into the shadow table
            */
            if (num_bytes >= 0)
            {
             if (device_config.dev_shadow_attrib[idx].val_type == DEV_CONFIG_VAL_TYPE_INT16)
                {
                    device_config.dev_shadow_attrib[idx].int16_val = *(int16_t *)file_contents;
                }
            else if (device_config.dev_shadow_attrib[idx].val_type == DEV_CONFIG_VAL_TYPE_INT)
                {
                    device_config.dev_shadow_attrib[idx].int_val = *(int32_t *)file_contents;
                }
            else if (device_config.dev_shadow_attrib[idx].val_type == DEV_CONFIG_VAL_TYPE_STRING && strlen(file_contents) > 0)
                {
                    memset(device_config.dev_shadow_attrib[idx].str_val, '\0', sizeof(device_config.dev_shadow_attrib[idx].str_val));
                    strcpy(device_config.dev_shadow_attrib[idx].str_val, file_contents);
                }
            else
                {
                    LOG_ERR("Unknown device shadow attribute type");
                }
            }
            else
            {
                device_config.dev_shadow_attrib[idx].is_new_val = true;
                LOG_DBG("%s file does not exist or is empty, value still set to default", log_strdup(device_config.dev_shadow_attrib[idx].filename));
            }
       }

    }
}

/**
 * @brief Saves the configuration value to a file
 * 
 * @param dev_shadow_attrib device shadow configurations handle
 * @param idx Index of the desired attribute
 * 
 * @return Returns 0 if it is a success
 */
int device_config_save_attribute_to_file(dev_config_shadow_id_t attribute)
{
    int err = 0;
    
    dev_shadow_attrib_t *dev_shadow_attrib = &device_config.dev_shadow_attrib[attribute];

    if (dev_shadow_attrib->is_new_val)
    {
        // LOG_INF("Saving %s to file", log_strdup(dev_shadow_attrib->filename));

        if (dev_shadow_attrib->val_type == DEV_CONFIG_VAL_TYPE_INT16)
        {
            int16_t data = dev_shadow_attrib->int16_val;

            err = save_to_file((char *)&data, sizeof(data), dev_shadow_attrib->filename);

        }
        else if (dev_shadow_attrib->val_type == DEV_CONFIG_VAL_TYPE_INT)
        {
            int32_t data = dev_shadow_attrib->int_val;
            err = save_to_file((char *)&data, sizeof(data), dev_shadow_attrib->filename);
        }
        else if (dev_shadow_attrib->val_type == DEV_CONFIG_VAL_TYPE_STRING)
        {
            err = save_to_file(dev_shadow_attrib->str_val, strlen(dev_shadow_attrib->str_val), dev_shadow_attrib->filename);
        }
        else
        {
            LOG_ERR("Value type is invalid when saving the file for %s", log_strdup(dev_shadow_attrib->filename));
        }

        if (!err)
            {
            LOG_ERR("Unable to save %s", log_strdup(dev_shadow_attrib->filename));
            return err;
            }

        // LOG_INF("%s's value saved to file", log_strdup(dev_shadow_attrib->filename), );
    }
    else
    {
        LOG_INF("Attribute not saved to file -> is_new_val = false");
    }

    dev_shadow_attrib->is_new_val = false;

    return 0;

}

/**
 * @brief Saves all the configurations to file that have been updated
 * 
 * @param dev_shadow_attrib device shadow configurations handle
 * @param num_elements Number of elements
 * 
 * @return Returns 0 if it is a success
 */
int device_config_save_all_attributes_to_file(void)
{
    LOG_DBG("Saving all shadow attributes to file");

    int err = 0;
    for (dev_config_shadow_id_t attr = 0; attr < DEV_CONFIG_NUM; attr++)
    {
        err = device_config_save_attribute_to_file(attr);
    }

    return err;
}

int16_t device_config_get_int16(dev_config_shadow_id_t attribute)
{
    if (attribute < DEV_CONFIG_NUM)
    {
        return device_config.dev_shadow_attrib[attribute].int16_val;
    }

    return 0;
}

void device_config_set_int16(dev_config_shadow_id_t attribute, int16_t val, bool is_new_val)
{
    if (attribute < DEV_CONFIG_NUM)
    {
        device_config.dev_shadow_attrib[attribute].int16_val = (int16_t)val;
        device_config.dev_shadow_attrib[attribute].is_new_val = is_new_val;
    }
}

int32_t device_config_get_int(dev_config_shadow_id_t attribute)
{
    if (attribute < DEV_CONFIG_NUM)
    {
        return device_config.dev_shadow_attrib[attribute].int_val;
    }

    return 0;
}

void device_config_set_int(dev_config_shadow_id_t attribute, int32_t val, bool is_new_val)
{
    if (attribute < DEV_CONFIG_NUM)
    {
        device_config.dev_shadow_attrib[attribute].int_val = (int32_t)val;
        device_config.dev_shadow_attrib[attribute].is_new_val = is_new_val;
    }
}

char * device_config_get_str(dev_config_shadow_id_t attribute)
{
    if (attribute < DEV_CONFIG_NUM && attribute != DEV_CONFIG_ICCID)
    {
        return device_config.dev_shadow_attrib[attribute].str_val;
    }
    if (attribute == DEV_CONFIG_ICCID)
    {
        return device_config.iccid;
    }
    return NULL;
}

void device_config_set_str(dev_config_shadow_id_t attribute, char *val, bool is_new_val)
{
    if (attribute < DEV_CONFIG_NUM && strlen(val) > 0)
    {
        memset(device_config.dev_shadow_attrib[attribute].str_val, '\0', sizeof(device_config.dev_shadow_attrib[attribute].str_val));
        strcpy(device_config.dev_shadow_attrib[attribute].str_val, val);
        device_config.dev_shadow_attrib[attribute].is_new_val = is_new_val;
    }
}

bool device_config_validate_flash_int16(dev_config_shadow_id_t attribute, int16_t valid_val)
{
    char file_contents[256] = {'\0'};
    read_file(file_contents, sizeof(file_contents), device_config.dev_shadow_attrib[attribute].filename);
    int16_t read_val = (int16_t)file_contents[0];     
   
    return read_val == valid_val;

}
bool device_config_validate_flash_int32(dev_config_shadow_id_t attribute, int32_t valid_val)
{
    char file_contents[256] = {'\0'};
    read_file(file_contents, sizeof(file_contents), device_config.dev_shadow_attrib[attribute].filename);

    int32_t read_val = (int32_t)file_contents[0];

    return read_val == valid_val;

}

bool device_config_validate_flash_char(dev_config_shadow_id_t attribute, char valid_val)
{
    char file_contents[256] = {'\0'};
    read_file(file_contents, sizeof(file_contents), device_config.dev_shadow_attrib[attribute].filename);

    char read_val = (char)file_contents[0];

    return read_val == valid_val;

}
char *device_config_get_serial_number(void)
{
    return device_config.serial_number;
}

/**
 * @brief Parses the data to JSON then retrieves the value for each properties. This Data is coming from the device shadow.
 * 
 * @param buf Data received from device shadow
 */
void device_config_shadow_data_handler(const char *buf, size_t len)
{
    LOG_DBG("Parsing received data from device shadow");
    
    cJSON *root_obj = NULL;
    int32_t version = 0;

	root_obj = cJSON_ParseWithLength(buf, len);
	if (root_obj == NULL) 
    {
        LOG_ERR("cJSON Parse failure at root object");
		goto clean_exit;
	}

    cJSON *state_obj = cJSON_GetObjectItemCaseSensitive(root_obj, "state");
    if (!cJSON_IsObject(state_obj))
    {
        LOG_ERR("cant get 'state' object");
		goto clean_exit;
    }
    
    cJSON *item = cJSON_GetObjectItemCaseSensitive(root_obj, "version");
    if (cJSON_IsNumber(item))
    {
        version = item->valueint;
    }
    
    // if we are processing a delta update sub event, the delta state attributes are in the state object
    cJSON *attributes = cJSON_GetObjectItemCaseSensitive(state_obj, "attributes");
    if (!cJSON_IsObject(attributes))
    {
        // if we are processing a shadow_get response, the delta state attributes are in the state.delta object
        LOG_DBG("state.attributes not found, trying state.delta.attributes...");

        cJSON *delta_obj = cJSON_GetObjectItemCaseSensitive(state_obj, "delta");
        if (!cJSON_IsObject(delta_obj))
        {
            LOG_WRN("state.delta not found -> discarding");
            goto clean_exit;
        }

        attributes = cJSON_GetObjectItemCaseSensitive(delta_obj, "attributes");
        if (!cJSON_IsObject(attributes))
        {
            LOG_ERR("state.delta.attributes not found -> discarding");
            goto clean_exit;
        }
    }

    LOG_WRN("Saving new delta.attributes: %s", cJSON_PrintUnformatted(attributes));
    device_config_update_from_shadow(attributes, version, true);

    // DON'T PROCESS OTHER STATES - if an update is required for a device, it must be in the delta object
    // if the desired state is always processed and we write everything to flash, it will wear out

clean_exit:
	cJSON_Delete(root_obj);
    return;
}

void device_config_update_from_shadow(cJSON *attributes, int32_t version, bool is_delta)
{
    if (version > 0)
    {
        // use shadow service version number in state object to track shadow version
        device_config_set_int(DEV_CONFIG_CONF_VERSION, version, true);
        device_config_save_attribute_to_file(DEV_CONFIG_CONF_VERSION);
        LOG_INF("%s updated to %d", log_strdup(DEV_SHADOW_ATTR_CONF_VERSION), version);
    }

    cJSON *item = cJSON_GetObjectItemCaseSensitive(attributes, DEV_SHADOW_ATTR_CONF_INTERVAL_S);
    if (cJSON_IsNumber(item))
    {
        int val = item->valueint;

        // protect against too short or too long intervals
        if (val < MINUTE_SECONDS) 
        {
            // don't request shadow more than once per minute
            val = MINUTE_SECONDS;
        }
        else if (val > DAY_SECONDS) 
        {
            // request shadow update at lease once per day
            val = DAY_SECONDS;
        }

        device_config_set_int16(DEV_CONFIG_CONF_UPDATE_INTERVAL_S, val, is_delta);
        device_config_save_attribute_to_file(DEV_CONFIG_CONF_UPDATE_INTERVAL_S);
        LOG_INF("%s updated to %d", log_strdup(DEV_SHADOW_ATTR_CONF_INTERVAL_S), val);
    }
    
    item = cJSON_GetObjectItemCaseSensitive(attributes, DEV_SHADOW_ATTR_DAQ_INTERVAL_S);
    if (cJSON_IsNumber(item))
    {
        int val = item->valueint;

        // protect against too short or too long intervals
        if (val < DAQ_INTERVAL_MINIMUM_S) 
        {
            // don't daq more than once per minute
            val = DAQ_INTERVAL_MINIMUM_S;
        }
        else if (val > DAY_SECONDS) 
        {
            // run daq at lease once per day
            val = DAY_SECONDS;
        }

        device_config_set_int16(DEV_CONFIG_DAQ_INTERVAL_S, val, is_delta);
        device_config_save_attribute_to_file(DEV_CONFIG_DAQ_INTERVAL_S);
        LOG_INF("%s updated to %d", log_strdup(DEV_SHADOW_ATTR_DAQ_INTERVAL_S), val);
    }
    
    item = cJSON_GetObjectItemCaseSensitive(attributes, DEV_SHADOW_ATTR_PUB_INTERVAL_S);
    if (cJSON_IsNumber(item))
    {
        int val = item->valueint;

        // protect against too short or too long intervals
        if (val < PUB_INTERVAL_MINIMUM_S) 
        {
            // don't try to publish more than once per minute
            val = PUB_INTERVAL_MINIMUM_S;
        }
        else if (val > DAY_SECONDS) 
        {
            // publish at lease once per day
            val = DAY_SECONDS;
        }

        device_config_set_int16(DEV_CONFIG_PUB_INTERVAL_S, val, is_delta);
        device_config_save_attribute_to_file(DEV_CONFIG_PUB_INTERVAL_S);
        LOG_INF("%s updated to %d", log_strdup(DEV_SHADOW_ATTR_PUB_INTERVAL_S), item->valueint);
    }

    item = cJSON_GetObjectItemCaseSensitive(attributes, DEV_SHADOW_ATTR_SENSOR_TYPE);
    if (cJSON_IsNumber(item) 
        && (external_sensor_t)item->valueint > EXT_SENSOR_UNKNOWN
        && (external_sensor_t)item->valueint < EXT_SENSOR_INVALID)
    {
        device_config_set_int16(DEV_CONFIG_SENSOR_TYPE, item->valueint, is_delta);
        device_config_save_attribute_to_file(DEV_CONFIG_SENSOR_TYPE);
        LOG_INF("%s updated to %d", log_strdup(DEV_SHADOW_ATTR_SENSOR_TYPE), item->valueint);
    }
    item = cJSON_GetObjectItemCaseSensitive(attributes, DEV_SHADOW_ATTR_PUB_TOPIC);
    if (cJSON_IsString(item) 
        && strlen(item->valuestring) > 0)
    {
        device_config_set_str(DEV_CONFIG_PUB_TOPIC, item->valuestring, is_delta);
        device_config_save_attribute_to_file(DEV_CONFIG_PUB_TOPIC);
        LOG_INF("%s updated to \"%s\"", log_strdup(DEV_SHADOW_ATTR_PUB_TOPIC), log_strdup(item->valuestring));
    }
    item = cJSON_GetObjectItemCaseSensitive(attributes, DEV_SHADOW_ATTR_APP_TYPE);
    if (cJSON_IsNumber(item) 
        && (app_id_t)item->valueint > APP_ID_UNKNOWN 
        && (app_id_t)item->valueint < APP_ID_NUM)
    {
        device_config_set_int16(DEV_CONFIG_APP_TYPE, item->valueint, is_delta);
        device_config_save_attribute_to_file(DEV_CONFIG_APP_TYPE);
        LOG_INF("%s updated to %d", log_strdup(DEV_SHADOW_ATTR_APP_TYPE), item->valueint);
        
        //check if app type has changed and reboot 
        if(device_config_validate_flash_int16(DEV_CONFIG_APP_TYPE, item->valueint))
        {
            LOG_INF("App_type changed to %d,...Rebooting!", item->valueint);
            sys_reboot(0);
        }

        
    }

}