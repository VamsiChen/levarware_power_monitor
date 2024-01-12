#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

/*Logging*/
#include <logging/log.h>

#include "onboard_button.h"
#include "led.h"

#include "sensors/onboard_sensor.h"

LOG_MODULE_REGISTER(onboard_button);

#define MODE_BTN_NODE DT_ALIAS(sw0)
#define MODE_BTN_LABEL DT_GPIO_LABEL(MODE_BTN_NODE, gpios)
#define MODE_BTN_PIN DT_GPIO_PIN(MODE_BTN_NODE, gpios)
#define MODE_BTN_FLAG DT_GPIO_FLAGS(MODE_BTN_NODE, gpios)
const struct device *mode_btn;
struct gpio_callback button_cb_data;
uint8_t led_state = 0;

/**
 * @brief Callback function when the interrupt for the on-board button is enabled. It toggles the Blue LED
 * 
 * @return Does not return any
 */
static void mode_button_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	led_state ^= 1;
	LOG_INF("Mode button is pressed, Turning %s the LED", log_strdup(led_state ? "ON":"OFF"));
	led_set_state(BLUE_LED, led_state);
}

/**
 * @brief Initializes the onboard button
 * 
 * @return Does not return any
 */
void onboard_button_init(void)
{
    LOG_INF("Initializing the on-board button");

    mode_btn = device_get_binding(MODE_BTN_LABEL);
    if (mode_btn == NULL)
    {
        LOG_ERR("Unable to get the pointer to struct device for the mode button");
        return;
    }

    gpio_pin_configure(mode_btn, MODE_BTN_PIN, GPIO_INPUT | MODE_BTN_FLAG);
    gpio_init_callback(&button_cb_data, mode_button_cb, BIT(MODE_BTN_PIN));
    
    gpio_add_callback(mode_btn, &button_cb_data);
	gpio_pin_interrupt_configure(mode_btn, MODE_BTN_PIN, GPIO_INT_EDGE_TO_INACTIVE);

    led_set_state(BLUE_LED, LED_OFF);
}