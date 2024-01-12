#ifndef APPS_H_
#define APPS_H_

#include <zephyr.h>
#include "device_config/device_config.h"
// #include "subsys/subsys.h"
// #include "power_mgr/power_mgr.h"
// #include "sensors/sensors.h"
// #include "sensors/onboard_sensor.h"

#define APPS_HS_TELEM_CONSUMER_STACK_SIZE       (2048) 
#define APPS_HS_TELEM_CONSUMER_PRIORITY         (3)

#define APPS_HS_TELEM_MAX_SAMPLES               (128)

#define CHARGER_ADDR                            (0x4B)
#define CHARGER_STATUS                          (0x00)

#define TSL2591_ADDR    (0x29)    // 7-bit I2C address of the TSL2591 sensor
#define TSL2591_COMMAND (0xA0)    // Command byte for TSL2591 register access

#define TSL2591_REG_DEV_ID        (0x12)

typedef struct apps_telem_data
{
    app_id_t app_type;
    struct k_msgq *msgq;
} apps_telem_t;

typedef struct apps_onboard_telem
{
    int16_t batt_mv;
    int32_t rsrp;
    float angle_deg;
    float batt_pct;
    float temp_c;
    int8_t power_status;
} apps_onboard_telem_t;

typedef struct apps_hs_telem_data
{
    char *measure_name;
    int64_t ts;
    int32_t sample_duration_ms;
    int32_t num_samples;
    union
    {
        float float_arr[APPS_HS_TELEM_MAX_SAMPLES];
        int32_t int_arr[APPS_HS_TELEM_MAX_SAMPLES];
    } samples;
} apps_hs_telem_data_t;

typedef struct apps_evt_data
{
    int64_t start_ts;
    int64_t end_ts;
    char *source;
} apps_evt_data_t;

typedef struct apps_evt_accel_data
{
    int64_t ts;
    float accel_x;
    float accel_y;
    float accel_z;
    float angle_deg;
} apps_evt_accel_data_t;

void apps_start(void);
void apps_hs_send_data(apps_hs_telem_data_t *data);
void apps_hs_telem_consumer_entry_point(void *p1, int unused2, int32_t unused3);
int apps_get_onboard_telemetry(apps_onboard_telem_t *telem);
void apps_hs_telemetry_send(apps_hs_telem_data_t *data);
void apps_evt_send_accel_event(apps_evt_data_t *evt, apps_evt_accel_data_t *accel_data);
int apps_telemetry_send(void *data, app_id_t app_id);

#endif /*APPS_H_*/