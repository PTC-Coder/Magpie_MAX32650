
/* Private includes --------------------------------------------------------------------------------------------------*/

#include "mxc_device.h"
#include "gpio.h"
#include "pt.h"

#include "bsp_pins.h"
#include "board.h"
#include "bsp_status_led.h"

/* Public function definitions ---------------------------------------------------------------------------------------*/

void status_led_init()
{
    // Disable any Pulse Train outputs that might be using these pins
    // PT channels map to GPIO2 pins on MAX32650
    // First stop all PT channels, then fully shutdown to disable the PT clock
    MXC_PT_Stop(MXC_PTG, 0xFFFF);
    
    // Shutdown specific PT channels that correspond to LED pins:
    // Red LED = GPIO2.10 = PT10, Green LED = GPIO2.9 = PT9, Blue LED = GPIO2.5 = PT5
    MXC_PT_Shutdown(MXC_PTG, (1 << 10) | (1 << 9) | (1 << 5));
    
    // Directly clear the GPIO output register for LED pins BEFORE configuring
    // This ensures LEDs are off even if PT was driving them
    MXC_GPIO2->out_clr = (MXC_GPIO_PIN_10 | MXC_GPIO_PIN_9 | MXC_GPIO_PIN_5);
    
    // Configure pins as GPIO outputs
    MXC_GPIO_Config(&bsp_pins_red_led_cfg);
    MXC_GPIO_Config(&bsp_pins_green_led_cfg);
    MXC_GPIO_Config(&bsp_pins_blue_led_cfg);
    
    // Ensure all LEDs are off after configuration
    status_led_set(STATUS_LED_COLOR_RED, false);
    status_led_set(STATUS_LED_COLOR_GREEN, false);
    status_led_set(STATUS_LED_COLOR_BLUE, false);
}

void status_led_set(Status_LED_Color_t color, bool state)
{
    switch (color)
    {
    case STATUS_LED_COLOR_RED:
        gpio_write_pin(&bsp_pins_red_led_cfg, state);    //update to state instead of !state since we want LED to match polarity now (pull low = off)
        break;
    case STATUS_LED_COLOR_GREEN:
        gpio_write_pin(&bsp_pins_green_led_cfg, state);
        break;
    case STATUS_LED_COLOR_BLUE:
        gpio_write_pin(&bsp_pins_blue_led_cfg, state);
        break;
    default:
        break;
    }
}

void status_led_toggle(Status_LED_Color_t color)
{
    switch (color)
    {
    case STATUS_LED_COLOR_RED:
        gpio_toggle_pin(&bsp_pins_red_led_cfg);
        break;
    case STATUS_LED_COLOR_GREEN:
        gpio_toggle_pin(&bsp_pins_green_led_cfg);
        break;
    case STATUS_LED_COLOR_BLUE:
        gpio_toggle_pin(&bsp_pins_blue_led_cfg);
        break;
    default:
        break;
    }
}

void status_led_all_off()
{
    status_led_set(STATUS_LED_COLOR_RED, false);
    status_led_set(STATUS_LED_COLOR_GREEN, false);
    status_led_set(STATUS_LED_COLOR_BLUE, false);
}
