#include "gpio.h"


void gpio_input_mode_cofig(gpio_register_def_t * p_gpio_x, gpio_pin_num_t pin_number)
{

    p_gpio_x->MODER &= ~((0x03) << (2 * pin_number)); 
}



void gpio_analog_mode_cofig(gpio_register_def_t * p_gpio_x, gpio_pin_num_t pin_number)
{
    p_gpio_x->MODER &= ~((0x03) << (2 * pin_number)); 

    p_gpio_x->MODER |= ((GPIO_MODE_ANALOG) << (2 * pin_number)); 

}


void gpio_output_mode_config(gpio_register_def_t * p_gpio_x, gpio_pin_num_t pin_number, 
                             gpio_speed_config_t output_speed, gpio_output_type_config_t output_type)
{
    
    
    // first clear the bits and then set the pin to output
    p_gpio_x->MODER &= ~((0x03) << ((2) * (pin_number))); 
    p_gpio_x->MODER |= ((GPIO_MODE_OUTPUT) << ((2) * (pin_number)));

    // first clear the speed register and then set the speed
    p_gpio_x->OSPEEDR &= ~((0x03) << (2 * pin_number));
    switch (output_speed)
    {
        case GPIO_SPEED_LOW:
            p_gpio_x->OSPEEDR |= ((GPIO_SPEED_LOW) << ((2)* (pin_number)));
            break;
        case GPIO_SPEED_MEDIUM:
            p_gpio_x->OSPEEDR |= ((GPIO_SPEED_MEDIUM) << ((2)* (pin_number)));
            break;
        case GPIO_SPEED_FAST:
            p_gpio_x->OSPEEDR |= ((GPIO_SPEED_FAST) << ((2)* (pin_number)));
            break;
        case GPIO_SPEED_HIGH:
        p_gpio_x->OSPEEDR |= ((GPIO_SPEED_HIGH) << ((2)* (pin_number)));
            break;
        default:
            break; 

    }

    // first clear the output type register and then set the output type
    
    p_gpio_x->OTYPER &= ~((0x01) << (pin_number));
    switch (output_type)
    {
        case GPIO_OTYPE_PUSH_PULL:
            p_gpio_x->OTYPER |= ((GPIO_OTYPE_PUSH_PULL) << (pin_number));
            break;
        case GPIO_OTYPE_OPEN_DRAIN:
            p_gpio_x->OTYPER |= ((GPIO_OTYPE_OPEN_DRAIN) << (pin_number));
            break;
        default:
            break;
    }
}


void gpio_alt_func_config(gpio_register_def_t * p_gpio_x, gpio_pin_num_t pin_number, gpio_alt_func_t alt_func)
{
    if ((pin_number) <= 7)
    {
        p_gpio_x->AFRL &= ~((0x0F) << ((4)*(pin_number)));
        p_gpio_x->AFRL |= ((alt_func) << ((4)*(pin_number)));
    }
    else 
    {
        p_gpio_x->AFRH &= ~((0x0F) << ((4)*(pin_number - 8)));
        p_gpio_x->AFRH |= ((alt_func) << ((4)*(pin_number-8)));
    }

}




void gpio_pull_type_config (gpio_register_def_t * p_gpio_x, gpio_pin_num_t pin_number, gpio_pull_config_t pull_type)
{
    p_gpio_x->PUPDR &= ~((0x03) << ((2) * (pin_number)));
    p_gpio_x->PUPDR |= ((pull_type) << ((2) * (pin_number)));
}



void gpio_intrpt_config(gpio_intrpt_source_config_t gpiox_intrpt_src, gpio_pin_num_t pin_number, gpio_intrpt_edge_config_t intrpt_mode)
{
    SYSCFG_PCLK_EN();

    switch (intrpt_mode)
    {
        case GPIO_INTRPT_EDGE_FALL:
            EXTI->EXTI_RTSR &= ~(1<<(pin_number));
            EXTI->EXTI_FTSR |= (1<<(pin_number));
            break;
        case GPIO_INTRPT_EDGE_RISING:
            EXTI->EXTI_FTSR &= ~(1<<(pin_number));
            EXTI->EXTI_RTSR |= (1<<(pin_number));
            break;
        case GPIO_INTRPT_EDGE_FALL_RISING:
            EXTI->EXTI_RTSR |= (1<<(pin_number));
            EXTI->EXTI_FTSR |= (1<<(pin_number));
            break;
        default:
            break;
    }

    // Calculate the register index (which EXTICR register to use)
    uint8_t reg_index = pin_number / 4;
    // Calculate the bit shift within the EXTICR register
    uint8_t bit_shift = (pin_number % 4) * 4;

    switch (reg_index)
    {
        case 0:
            SYSCFG->SYSCFG_EXTICR1 &= ~(0xF << bit_shift);
            SYSCFG->SYSCFG_EXTICR1 |= (gpiox_intrpt_src << bit_shift);
            break;
        case 1:
            SYSCFG->SYSCFG_EXTICR2 &= ~(0xF << bit_shift);
            SYSCFG->SYSCFG_EXTICR2 |= (gpiox_intrpt_src << bit_shift);
            break;
        case 2:
            SYSCFG->SYSCFG_EXTICR3 &= ~(0xF << bit_shift);
            SYSCFG->SYSCFG_EXTICR3 |= (gpiox_intrpt_src << bit_shift);
            break;
        case 3:
            SYSCFG->SYSCFG_EXTICR4 &= ~(0xF << bit_shift);
            SYSCFG->SYSCFG_EXTICR4 |= (gpiox_intrpt_src << bit_shift);
            break;
        default:
            break;

    }

    // enable the interrupt using EXTI_IMR register

    EXTI->EXTI_IMR |= (1<<pin_number);   
}







void gpio_clear_irq_pending(gpio_pin_num_t pin_number)
{
    if ((EXTI->EXTI_PR) & (1<<pin_number))
    {
        EXTI->EXTI_PR |= (1<<pin_number);
    }

}


void gpio_de_init(gpio_register_def_t * p_gpio_x)
{
    if (p_gpio_x == GPIOA)
     {
        GPIOA_RESET();
     }
    else if (p_gpio_x == GPIOB)
    {
        GPIOB_RESET();
    }
    else if (p_gpio_x == GPIOC)
    {
        GPIOC_RESET();
    }
    else if (p_gpio_x == GPIOD)
    {
        GPIOD_RESET();
    }
    else if (p_gpio_x == GPIOE)
    {
        GPIOE_RESET();
    }
    else if (p_gpio_x == GPIOF)
    {
        GPIOF_RESET();
    }
    else if (p_gpio_x == GPIOG)
    {
        GPIOG_RESET();
    }
    else if (p_gpio_x == GPIOH)
    {
        GPIOH_RESET();
    }
    
}  



uint8_t gpio_read_from_input_pin(gpio_register_def_t *p_gpio_x, gpio_pin_num_t pin_number)
{
    uint8_t pin_value_read =0;

    pin_value_read = (uint8_t) (p_gpio_x->IDR >> pin_number) & (0x01);

    return pin_value_read;

}



uint16_t gpio_read_from_input_port(gpio_register_def_t *p_gpio_x)
{

    uint16_t port_value_read = (uint16_t) (p_gpio_x->IDR );

    return port_value_read;

}



void gpio_write_to_output_pin(gpio_register_def_t *p_gpio_x, gpio_pin_num_t pin_number, uint8_t value_write)
{
   
    
    if (value_write == GPIO_PIN_SET)
    {
        p_gpio_x->ODR |= (1 << pin_number);
    }
    else
    {
        p_gpio_x->ODR &= ~(1 << pin_number);
    }

}



void gpio_write_to_output_port(gpio_register_def_t *p_gpio_x, uint16_t value_write)
{
    p_gpio_x->ODR = value_write;

}



void gpio_toggle_output_pin(gpio_register_def_t *p_gpio_x, gpio_pin_num_t pin_number)
{
    p_gpio_x->ODR ^= (1 << pin_number);

}


