#ifndef DISTANCE_SENSOR_H_
#define DISTANCE_SENSOR_H_

#include <zephyr.h>
#include "device_config/device_config.h"

#define DEVICE_SENSOR_UART_SPEED_9600       (9600UL)
#define DEVICE_SENSOR_UART_SPEED_115200     (115200UL)

#define UART2_BUFFER_SIZE                   (128)

#define TERABEE_UART_BAUD_RATE              DEVICE_SENSOR_UART_SPEED_115200
#define TERABEE_EVO_DATA_SIZE               4
#define TERABEE_EVO_DATA_SIZE_2PX_MODE      6
#define TERABEE_EVO_DATA_SIZE_4PX_MODE      10
#define TERABEE_CRC8_POLYNOMIAL             0x07
#define TERABEE_EVO_ACCURACY                20
#define TERABEE_POWER_REG_VOLTS             (5.0)

#define DISTANCE_NUM_SAMPLES_AVG            20
#define DISTANCE_MAX_SAMPLE_TIME_MSEC       5000

#define MAXBOTIX_NO_TARGET_5M               5000
#define MAXBOTIX_NO_TARGET_10M              9999
#define MAXBOTIX_DATA_SIZE                  5
#define MAXBOTIX_UART_BAUDRATE              DEVICE_SENSOR_UART_SPEED_9600
#define MAXBOTIX_POWER_REG_VOLTS            (5.0)

#define RADAR_UART_BAUDRATE                 DEVICE_SENSOR_UART_SPEED_9600 
#define RADAR_POWER_REG_VOLTS               (5.0)
#define RADAR_STAB_TIME_MS                  5000


typedef struct distance_sensor_uart_user_data
{
    external_sensor_t distance_sensor;
    uint8_t uart_rx_data[UART2_BUFFER_SIZE];
    uint8_t uart_rx_data_len;
    uint8_t uart_tx_data[UART2_BUFFER_SIZE];
    uint8_t uart_tx_data_len;
    int32_t distance_buf[DISTANCE_NUM_SAMPLES_AVG];
    uint8_t distance_buf_index;
    bool distance_samples_done;
    int32_t distance_mm;
    int8_t power_status;
} distance_sensor_uart_user_data_t;

int32_t distance_sensor_get_mm(void);

#endif /*DISTANCE_SENSOR_H_*/