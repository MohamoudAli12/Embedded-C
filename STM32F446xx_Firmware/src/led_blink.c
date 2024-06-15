#include <stdint.h>
#include "stm32f446xx.h"
#include "gpio.h"
#include "fn_prototypes.h"

void blink_led(void) 
{
gpio_peripheral_clk_cntrl(GPIOA, ENABLE);
gpio_output_mode_config(GPIOA, GPIO_PIN_5, GPIO_SPEED_LOW, GPIO_OTYPE_PUSH_PULL);
gpio_pull_type_config(GPIOA, GPIO_PIN_5, GPIO_NO_PULL_UP_DOWN);


while(1)
{
    gpio_toggle_output_pin(GPIOA, GPIO_PIN_5);
    delay(500000);
}

}
