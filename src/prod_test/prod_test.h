#ifndef PROD_TEST_H_
#define PROD_TEST_H_

#define MQTT_POLL_THREAD_STACK_SIZE       (3072)
#define MQTT_POLL_THREAD_PRIORITY         (1)

#define SERIAL_CRC8_CHECKSUM_POLY 0x07
#define SERIAL_USB_SENSOR_TYPE_LEN 1

#define PROD_TEST_TOPIC "lv/prod_test/result"

void prod_test_led(void);
void prod_test(void);
void prod_test_pre_production(void);

#endif /*PROD_TEST_H_*/