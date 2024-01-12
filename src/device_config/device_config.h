#ifndef DEVICE_CONFIG_H_
#define DEVICE_CONFIG_H_

// #include <zephyr/types.h>
#include <stdbool.h>

#include "subsys/modem.h"

// device config files
#define SERIAL_NUMBER_FILE_NAME             "serial_number"
#define CONFIG_VERSION_FILE_NAME            "config_version"
#define CONFIG_INTERVAL_FILE_NAME           "config_interval"
#define DAQ_INTERVAL_FILE_NAME              "daq_interval"
#define PUB_INTERVAL_FILE_NAME              "pub_interval"
#define ICCID_INTERNAL_FILE_NAME            "ICCID_value"
#define SENSOR_TYPE_FILE_NAME               "sensor_type"
#define APP_TYPE_FILE_NAME                  "app_type"
#define PUB_TOPIC_FILE_NAME                 "pub_topic"
#define SUB_TOPIC_FILE_NAME                 "sub_topic"

#define SERIAL_NUMBER_LEN (50)

// device config default values
#define CONFIG_VERSION_DEFAULT_VAL          (0)
#define CONFIG_INTERVAL_DEFAULT_VAL_S       (60)
#define DAQ_INTERVAL_DEFAULT_VAL_S          (60)
#define DAQ_INTERVAL_MINIMUM_S              (1)
#define PUB_INTERVAL_DEFAULT_VAL_S          (60)
#define PUB_INTERVAL_MINIMUM_S              (1)
#define ICCID_DEFAULT_VAL                   "000"
#define SENSOR_TYPE_DEFAULT_VAL             (1)
#define APP_TYPE_DEFAULT_VAL                (1)
#define PUB_TOPIC_DEFAULT_VAL               "dt/00000000-0000-0000-0000-000000000000"
#define SUB_TOPIC_DEFAULT_VAL               "sub_topic"


// device shadow attributes
#define DEV_SHADOW_ATTR_CONF_VERSION        "config_version"
#define DEV_SHADOW_ATTR_CONF_INTERVAL_S     "config_interval_s"
#define DEV_SHADOW_ATTR_DAQ_INTERVAL_S      "daq_interval_s"
#define DEV_SHADOW_ATTR_PUB_INTERVAL_S      "pub_interval_s"
#define DEV_SHADOW_ATTR_SENSOR_TYPE         "sensor_type"
#define DEV_SHADOW_ATTR_APP_TYPE            "app_type"
#define DEV_SHADOW_ATTR_PUB_TOPIC           "topic"

// device shadow, other attributes
#define DEV_SHADOW_ATTR_FW_V                "fw_version"

#define SECONDS_PER_MINUTE                   (60)
#define MINUTES_PER_HOUR                     (60)
#define HOURS_PER_DAY                        (24)

#define MINUTE_SECONDS                      SECONDS_PER_MINUTE
#define HOUR_SECONDS                        (SECONDS_PER_MINUTE * MINUTES_PER_HOUR)
#define DAY_SECONDS                         (HOUR_SECONDS * HOURS_PER_DAY)

// enum options for updating the device shadow state
typedef enum aws_state_attributes
{
    REPORTED = 0,
    DESIRED_AND_REPORTED
} aws_state_attributes_t;

typedef struct version_struct
{
    uint8_t rev;
    uint8_t minor;   
    uint8_t major;
    uint8_t reserved;
} version_struct_t;

typedef union {
  struct
  {
    uint32_t distance : 1;
    uint32_t motion : 1;
    uint32_t audio : 1;
    uint32_t feat3 : 1;
    uint32_t feat4 : 1;
    uint32_t feat5 : 1;
    uint32_t feat6 : 1;
    uint32_t feat7 : 1;
    uint32_t feat8 : 1;
    uint32_t feat9 : 1;
    uint32_t feat10 : 1;
    uint32_t feat11 : 1;
    uint32_t feat12 : 1;
    uint32_t feat13 : 1;
    uint32_t feat14 : 1;
    uint32_t feat15 : 1;
    uint32_t feat16 : 1;
    uint32_t feat17 : 1;
    uint32_t feat18 : 1;
    uint32_t feat19 : 1;
    uint32_t feat20 : 1;
    uint32_t feat21 : 1;
    uint32_t feat22 : 1;
    uint32_t feat23 : 1;
    uint32_t feat24 : 1;
    uint32_t feat25 : 1;
    uint32_t feat26 : 1;
    uint32_t feat27 : 1;
    uint32_t feat28 : 1;
    uint32_t feat29 : 1;
    uint32_t feat30 : 1;
    uint32_t feat31 : 1;
  } features;
  uint32_t u32;
} app_feature_flags_t;

typedef enum app_id_t
{
    APP_ID_UNKNOWN = 0,
    DISTANCE,
    MOTION, // Lines below are just placeholder
    AUDIO,
    PWR_MONITOR,
    HYBRID,
    APP_ID_NUM,
    APP_ID_INVALID
} app_id_t;

typedef enum external_sensor
{
    EXT_SENSOR_UNKNOWN = 0,
    EXT_SENSOR_TERABEE,
    EXT_SENSOR_MAXBOTIX,
    EXT_SENSOR_RADAR,
    POWER_STATUS,
    EXT_SENSOR_NUM,
    EXT_SENSOR_INVALID
} external_sensor_t;

// When adding to the enumerated list, be sure to add the code in device_config.c where files are read and configured. 
typedef enum dev_config_shadow_id_t
{
    // device shadow
    DEV_CONFIG_DAQ_INTERVAL_S = 0,
    DEV_CONFIG_PUB_INTERVAL_S,
    DEV_CONFIG_ICCID,
    DEV_CONFIG_CONF_UPDATE_INTERVAL_S,
    DEV_CONFIG_SENSOR_TYPE, // current implementation
    DEV_CONFIG_APP_TYPE,
    DEV_CONFIG_CONF_VERSION,
    DEV_CONFIG_PUB_TOPIC,
    DEV_CONFIG_SUB_TOPIC,
    DEV_CONFIG_NUM,
    DEV_CONFIG_INVALID
} dev_config_shadow_id_t;

typedef enum dev_config_val_type_t
{
    DEV_CONFIG_VAL_TYPE_INT16 = 0,
    DEV_CONFIG_VAL_TYPE_INT,
    DEV_CONFIG_VAL_TYPE_STRING,
    DEV_CONFIG_VAL_TYPE_NUM,
    DEV_CONFIG_VAL_TYPE_INVALID
} dev_config_val_type_t;

typedef struct dev_shadow_attrib_t
{
    dev_config_shadow_id_t  dev_conf_shadow_id;
    dev_config_val_type_t   val_type;
    int16_t                 int16_val;
    int32_t                 int_val;
    char                    str_val[128];
    bool                    is_new_val;
    char                    filename[50];
} dev_shadow_attrib_t;

/**
 * @brief Cache memory for device configurations, i.e., non-persistent
 */
typedef struct device_config_t
{
    char                serial_number[SERIAL_NUMBER_LEN];
    char                imei[IMEI_LEN];
    char                iccid[ICCID_LEN];
    dev_shadow_attrib_t dev_shadow_attrib[DEV_CONFIG_NUM];
}device_config_t;


void device_config_init(void);
char *device_config_get_fw_version_str(void);
void device_config_dev_shadow_attrib_init(void);
int device_config_save_attribute_to_file(dev_config_shadow_id_t attribute);
int device_config_save_all_attributes_to_file(void);
int16_t device_config_get_int16(dev_config_shadow_id_t attribute);
void device_config_set_int16(dev_config_shadow_id_t attribute, int16_t val, bool is_new_val);
int32_t device_config_get_int(dev_config_shadow_id_t attribute);
void device_config_set_int(dev_config_shadow_id_t attribute, int32_t val, bool is_new_val);
char *device_config_get_str(dev_config_shadow_id_t attribute);
void device_config_set_str(dev_config_shadow_id_t attribute, char *val, bool is_new_val);
char *device_config_get_serial_number(void);
void device_config_shadow_data_handler(const char *buf, size_t len);
char *device_config_get_iccid(dev_config_shadow_id_t attribute);

#endif /*DEVICE_CONFIG_H_*/