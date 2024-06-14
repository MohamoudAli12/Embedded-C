#include <stdint.h>
#include "stm32f446xx.h"
#include "gpio.h"

#define BUTTON_PRESSED                      0

void toggle_led_on_button_press(void)
{
    //configure Led pin
    gpio_peripheral_clk_cntrl(GPIOA, ENABLE);
    gpio_output_mode_config(GPIOA, GPIO_PIN_5, GPIO_SPEED_FAST, GPIO_OTYPE_PUSH_PULL);
    gpio_pull_type_config(GPIOA, GPIO_PIN_5, GPIO_NO_PULL_UP_DOWN);

    //configure button

    gpio_peripheral_clk_cntrl(GPIOC, ENABLE);
    gpio_input_mode_cofig(GPIOC,GPIO_PIN_13);
    
    

    while(1)
    {
        if(gpio_read_from_input_pin(GPIOC,GPIO_PIN_13)==BUTTON_PRESSED)
        {
            delay(300000);
            gpio_toggle_output_pin(GPIOA,GPIO_PIN_5);
        }
    }

}