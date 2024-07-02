#include <stdint.h>
#include "stm32f446xx.h"
#include "gpio.h"
#include "fn_prototypes.h"

#define BUTTON_PRESSED                      0





void button_interrupt_led_toggle(void)
{
    //configure Led pin
    
    GPIOA_PCLK_EN();
    gpio_output_mode_config(GPIOA, GPIO_PIN_5, GPIO_SPEED_FAST, GPIO_OTYPE_PUSH_PULL);
    gpio_pull_type_config(GPIOA, GPIO_PIN_5, GPIO_NO_PULL_UP_DOWN);

    //configure button

    GPIOC_PCLK_EN();
    gpio_intrpt_config(GPIOC_INTRPT_SRC, GPIO_PIN_13, GPIO_INTRPT_EDGE_FALL);
    gpio_irq_enable(EXTI15_10_IRQ_POS, ENABLE);
  
  
    while(1)
    {
    if(gpio_read_from_input_pin(GPIOC, GPIO_PIN_13)==BUTTON_PRESSED ) 
    {
        
        EXTI15_10_IRQHandler();
    }
    }

}

void EXTI15_10_IRQHandler(void)
{
    
    gpio_clear_irq_pending(GPIO_PIN_13);
    gpio_toggle_output_pin(GPIOA,GPIO_PIN_5);
   

}