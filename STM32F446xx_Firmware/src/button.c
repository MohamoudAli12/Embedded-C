#include <stdint.h>
#include "stm32f446xx.h"
#include "gpio.h"
#include "fn_prototypes.h"



_Bool debounce_active_low(void) 
{
    static uint16_t state = 0;
    state = ((state << 1) |(gpio_read_from_input_pin(GPIOC, GPIO_PIN_13))| (0xfe00));
    return (state == 0xff00 );
}


void toggle_led_on_button_press(void)
{
    //configure Led pin
    GPIOA_PCLK_EN();
    gpio_output_mode_config(GPIOA, GPIO_PIN_5, GPIO_SPEED_FAST, GPIO_OTYPE_PUSH_PULL);
    gpio_pull_type_config(GPIOA, GPIO_PIN_5, GPIO_NO_PULL_UP_DOWN);

    //configure button

    
    GPIOC_PCLK_EN();
    gpio_input_mode_cofig(GPIOC,GPIO_PIN_13);
    
    

 while(1)
    {
        if(debounce_active_low())
        {
            
            gpio_toggle_output_pin(GPIOA,GPIO_PIN_5);
        }
    }

}