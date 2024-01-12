#ifndef LED_H_
#define LED_H_

typedef enum led_t
{
    BLUE_LED = 0,
    GREEN_LED,
    RED_LED,
    LED_NUM,
    LED_INVALID
}led_t;

typedef enum led_state_t
{
    LED_OFF = 0,
    LED_ON,
    LED_STATE_NUM,
    LED_STATE_INVALID
}led_state_t;

extern bool led_init_state;

int led_init(void);
int led_set_state(led_t led, led_state_t state);

#endif /*LED_H_*/