#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>


#include "led.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(led);

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)

#define LED0_LABEL DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED1_LABEL DT_GPIO_LABEL(LED1_NODE, gpios)
#define LED2_LABEL DT_GPIO_LABEL(LED2_NODE, gpios)

#define LED0_PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define LED1_PIN DT_GPIO_PIN(LED1_NODE, gpios)
#define LED2_PIN DT_GPIO_PIN(LED2_NODE, gpios)

#define LED0_FLAG DT_GPIO_FLAGS(LED0_NODE, gpios)
#define LED1_FLAG DT_GPIO_FLAGS(LED1_NODE, gpios)
#define LED2_FLAG DT_GPIO_FLAGS(LED2_NODE, gpios)

const struct device *led_dev[LED_NUM];

uint8_t led_pins[LED_NUM] = {LED0_PIN, LED1_PIN, LED2_PIN};

bool led_init_state = false;

/**
 * @brief Initializes the on-board LEDs 
 * 
 * @return Returns ERROR_NONE if it is a success
 */
int led_init(void)
{
    int err = 0;

    if (led_init_state)
    {
        return err;
    }

    led_dev[BLUE_LED] = device_get_binding(LED0_LABEL);
    led_dev[GREEN_LED] = device_get_binding(LED1_LABEL);
    led_dev[RED_LED] = device_get_binding(LED2_LABEL);

    for (led_t led = BLUE_LED; led < LED_NUM; led++)
    {
        if ( led_dev[led] == NULL)
        {
            LOG_ERR("led[%d] is null", (uint8_t)led);
            err = -1;
            break;
        }

    }

    if (err == 0)
    {
        gpio_pin_configure(led_dev[BLUE_LED], LED0_PIN, LED0_FLAG | GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure(led_dev[GREEN_LED], LED1_PIN, LED1_FLAG | GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure(led_dev[RED_LED], LED2_PIN, LED2_FLAG | GPIO_OUTPUT_INACTIVE);

        led_init_state = true;        
    }

    return err;
}

/**
 * @brief Set an LED state
 * 
 * @param led the index of the desired LED
 * @param state the state to set the LED to
 * 
 * @return Returns ERROR_NONE if it is a success
 */
int led_set_state(led_t led, led_state_t state)
{
    int err = 0;

    // __ASSERT(led_init_state == true, "on-board LED not initialized");
    // __ASSERT(led < LED_NUM, "LED index out of range");
    // __ASSERT(state < LED_STATE_NUM, "LED status out of range");

    if (gpio_pin_set(led_dev[led], led_pins[led], state) != 0)
    {
        LOG_ERR("Unable to set the state of led[%d]", (uint8_t)led);
        err = -1;      
    }
    
    return err;
}