#ifndef ONBOARD_SENSOR_H_
#define ONBOARD_SENSOR_H_

#include "battery.h"

typedef enum onboard_sensor
{
    ONBOARD_SENSOR_ANGLE = 0, //DOUBLE
    ONBOARD_SENSOR_TEMP, //double
    ONBOARD_SENSOR_PRESSURE, //double
    ONBOARD_SENSOR_HUMIDITY, //double
    ONBOARD_SENSOR_GAS, //double
    ONBOARD_SENSOR_BATT_MV,
    ONBOARD_SENSOR_BATT_PCT,
    ONBOARD_SENSOR_NUM,
    ONBOARD_SENSOR_INVALID
} onboard_sensor_t;

void onboard_sensor_init(void);
void onboard_sensor_tphg_init(void);
int onboard_sensor_get_temp_c(float *temp);
int onboard_sensor_get_pressure_pa(float *pressure);
int onboard_sensor_get_humidity_rh(float *humidity);
int onboard_sensor_get_gas_res_ohm(float *gas);
void onboard_sensor_accel_init(void);
int onboard_sensor_accel_get_sample(double *accel_xyz, float *angle_deg);
bool onboard_sensor_tphg_is_sleeping(void);
int onboard_sensor_tphg_set_mode(uint8_t mode);
bool onboard_sensor_accel_is_sleeping(void);
int onboard_sensor_accel_set_mode(void);
void onboard_sensor_accel_trigger_set(void);

#endif /*ONBOARD_SENSOR_H_*/