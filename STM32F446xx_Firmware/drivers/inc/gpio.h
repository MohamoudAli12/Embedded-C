#ifndef GPIO_H
#define GPIO_H

#include "stm32f446xx.h"

typedef enum 
{
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALT_FUNC,
    GPIO_MODE_ANALOG
} gpio_mode_config_t;

typedef enum 
{
    GPIO_INTRPT_EDGE_FALL,
    GPIO_INTRPT_EDGE_RISING,
    GPIO_INTRPT_EDGE_FALL_RISING
}gpio_intrpt_edge_config_t;

typedef enum 
{
    GPIOA_INTRPT_SRC,
    GPIOB_INTRPT_SRC,
    GPIOC_INTRPT_SRC,
    GPIOD_INTRPT_SRC,
    GPIOE_INTRPT_SRC,
    GPIOF_INTRPT_SRC,
    GPIOG_INTRPT_SRC,
}gpio_intrpt_source_config_t;

typedef enum 
{
    GPIO_OTYPE_PUSH_PULL,
    GPIO_OTYPE_OPEN_DRAIN
} gpio_output_type_config_t;

typedef enum
{
    GPIO_SPEED_LOW,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_FAST,
    GPIO_SPEED_HIGH
} gpio_speed_config_t;

typedef enum 
{
    GPIO_NO_PULL_UP_DOWN,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN
} gpio_pull_config_t;


typedef enum 
{
    GPIO_PIN_0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15
} gpio_pin_num_t;

typedef enum 
{
    AF0,
    AF1,
    AF2,
    AF3,
    AF4,
    AF5,
    AF6,
    AF7,
    AF8,
    AF9,
    AF10,
    AF11,
    AF12,
    AF13,
    AF14,
    AF15
} gpio_alt_func_t;

typedef enum 
{
    GPIO_PIN_RESET = RESET,
    GPIO_PIN_SET = SET      
} gpio_pin_state_t;


/*********************************************************************
 * @fn      		  - gpio_peripheral_clk_cntrl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - GPIOx port where x is is the GPIO port i.e. GPIOA 
 * @param[in]         - ENABLE or DISABLE Clock for the GPIO port
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */


void gpio_peripheral_clk_cntrl(gpio_register_def_t *p_gpio_x, gpio_pin_state_t enable_or_disable);  // enable or disable pclk of given gpio pin



/*********************************************************************
 * @fn      		  - gpio_input_mode_config
 *
 * @brief             - This function configures the specified GPIO pin as an input
 *
 * @param[in]         - p_gpio_x: GPIOx port where x is the GPIO port, e.g., GPIOA 
 * @param[in]         - pin_number: GPIO pin number to be configured as input
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct GPIO port base address and pin number are passed to this function.
 */
void gpio_input_mode_cofig(gpio_register_def_t * p_gpio_x, gpio_pin_num_t pin_number);



/*********************************************************************
 * @fn      		  - gpio_analog_mode_config
 *
 * @brief             - This function configures the specified GPIO pin as an analog mode
 *
 * @param[in]         - p_gpio_x: GPIOx port where x is the GPIO port, e.g., GPIOA 
 * @param[in]         - pin_number: GPIO pin number to be configured as analog mode
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct GPIO port base address and pin number are passed to this function.
 */
void gpio_analog_mode_cofig(gpio_register_def_t * p_gpio_x, gpio_pin_num_t pin_number);


/*********************************************************************
 * @fn      		  - gpio_output_mode_config
 *
 * @brief             - This function configures the specified GPIO pin as an output with configurable speed and pull type
 *
 * @param[in]         - p_gpio_x: GPIOx port where x is the GPIO port, e.g., GPIOA 
 * @param[in]         - pin_number: GPIO pin number to be configured as output
 * @param[in]         - output_speed: Speed configuration for the output pin (LOW, MEDIUM, FAST, HIGH)
 * @param[in]         - output_type: Pull configuration for the output pin (PUSH_PULL, OPEN_DRAIN)
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct GPIO port base address, pin number, speed, and pull type are passed to this function.
 */

void gpio_output_mode_config(gpio_register_def_t * p_gpio_x, gpio_pin_num_t pin_number, 
                             gpio_speed_config_t output_speed, gpio_output_type_config_t output_type);

/*********************************************************************
 * @fn      		  - gpio_alt_func_config
 *
 * @brief             - This function configures the alternate function for the specified GPIO pin
 *
 * @param[in]         - p_gpio_x: GPIOx port where x is the GPIO port, e.g., GPIOA 
 * @param[in]         - pin_number: GPIO pin number to configure the alternate function
 * @param[in]         - alt_func: Alternate function number to be configured for the pin
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct GPIO port base address, pin number, and alternate function are passed to this function.
 */




void gpio_alt_func_config(gpio_register_def_t * p_gpio_x, gpio_pin_num_t pin_number, gpio_alt_func_t alt_func);


/*********************************************************************
 * @fn      		  - gpio_pull_type_config
 *
 * @brief             - This function configures the pull type for the specified GPIO pin
 *
 * @param[in]         - p_gpio_x: GPIOx port where x is the GPIO port, e.g., GPIOA 
 * @param[in]         - pin_number: GPIO pin number to configure the pull type
 * @param[in]         - pull_type: Pull configuration for the pin (PULL_NONE, PULL_UP, PULL_DOWN)
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct GPIO port base address, pin number, and pull type are passed to this function.
 */

void gpio_pull_type_config (gpio_register_def_t * p_gpio_x, gpio_pin_num_t pin_number, gpio_pull_config_t pull_type);

/*********************************************************************
 * @fn      		  - gpio_intrpt_config
 *
 * @brief             - This function configures interrupt for the specified GPIO pin
 *
 * @param[in]         - gpiox_intrpt_src: Interrupt source selection for the pin
 * @param[in]         - pin_number: GPIO pin number to configure interrupt
 * @param[in]         - intrpt_mode: Interrupt mode (edge detection) for the pin
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct interrupt source, pin number, and interrupt mode are passed to this function.
 */



void gpio_intrpt_config(gpio_intrpt_source_config_t gpiox_intrpt_src, gpio_pin_num_t pin_number, gpio_intrpt_edge_config_t intrpt_mode);

/*********************************************************************
 * @fn      		  - gpio_irq_enable
 *
 * @brief             - Enable or disable interrupt for the specified IRQ number
 *
 * @param[in]         - irq_number: IRQ number to enable or disable
 * @param[in]         - enable_or_disable: ENABLE or DISABLE to control the interrupt
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct IRQ number and enable/disable flag are passed to this function.
 */


void gpio_irq_enable(uint8_t irq_number, uint8_t enable_or_disable);



/*********************************************************************
 * @fn      		  - gpio_irq_priority
 *
 * @brief             - Set the priority for the specified IRQ number
 *
 * @param[in]         - irq_number: IRQ number to set the priority for
 * @param[in]         - irq_priority: Priority value to be set for the IRQ
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct IRQ number and priority value are passed to this function.
 */

void gpio_irq_priority(uint8_t irq_number, uint8_t irq_priority);

/*********************************************************************
 * @fn      		  - gpio_clear_irq_pending
 *
 * @brief             - Clear the pending interrupt for the specified pin
 *
 * @param[in]         - pin_number: Pin number to clear the pending interrupt for
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct pin number is passed to this function.
 */



void gpio_clear_irq_pending(uint8_t pin_number);

/*********************************************************************
 * @fn      		  - gpio_de_init
 *
 * @brief             - Deinitialize the GPIO port
 *
 * @param[in]         - p_gpio_x: Pointer to the GPIO port register definition structure
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct GPIO port base address is passed to this function.
 */




void gpio_de_init(gpio_register_def_t *p_gpio_x);  // you can use the rcc peripheral reset register to reset the gpio

/*********************************************************************
 * @fn      		  - gpio_read_from_input_pin
 *
 * @brief             - Read the value from the input pin
 *
 * @param[in]         - p_gpio_x: Pointer to the GPIO port register definition structure
 * @param[in]         - pin_number: Pin number to read the value from
 *
 * @return            - Value read from the input pin (0 or 1)
 *
 * @Note              - Ensure that the correct GPIO port base address and pin number are passed to this function.
 */

uint8_t gpio_read_from_input_pin(gpio_register_def_t *p_gpio_x, uint8_t pin_number);

/*********************************************************************
 * @fn      		  - gpio_read_from_input_port
 *
 * @brief             - Read the value from the input port
 *
 * @param[in]         - p_gpio_x: Pointer to the GPIO port register definition structure
 *
 * @return            - Value read from the input port (16 bits)
 *
 * @Note              - Ensure that the correct GPIO port base address is passed to this function.
 */

uint16_t gpio_read_from_input_port(gpio_register_def_t *p_gpio_x);

/*********************************************************************
 * @fn      		  - gpio_write_to_output_pin
 *
 * @brief             - Write a value to the output pin
 *
 * @param[in]         - p_gpio_x: Pointer to the GPIO port register definition structure
 * @param[in]         - pin_number: Pin number to write the value to
 * @param[in]         - value_write: Value to be written to the pin (GPIO_PIN_SET or GPIO_PIN_RESET)
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct GPIO port base address, pin number, and value are passed to this function.
 */


void gpio_write_to_output_pin(gpio_register_def_t *p_gpio_x, uint8_t pin_number, uint8_t value_write);

/*********************************************************************
 * @fn      		  - gpio_write_to_output_port
 *
 * @brief             - Write a value to the output port
 *
 * @param[in]         - p_gpio_x: Pointer to the GPIO port register definition structure
 * @param[in]         - value_write: Value to be written to the port (16 bits)
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct GPIO port base address and value are passed to this function.
 */


void gpio_write_to_output_port(gpio_register_def_t *p_gpio_x, uint16_t value_write);

/*********************************************************************
 * @fn      		  - gpio_toggle_output_pin
 *
 * @brief             - Toggle the output pin state
 *
 * @param[in]         - p_gpio_x: Pointer to the GPIO port register definition structure
 * @param[in]         - pin_number: Pin number to toggle
 *
 * @return            - None
 *
 * @Note              - Ensure that the correct GPIO port base address and pin number are passed to this function.
 */


void gpio_toggle_output_pin(gpio_register_def_t *p_gpio_x, uint8_t pin_number);



#define IS_GPIO_PIN_ACTION(ACTION)            (((ACTION)==(GPIO_PIN_SET)) || ((ACTION) ==(GPIO_PIN_RESET)))

#define IS_GPIO_PIN_MODE(MODE)                (((MODE) == (GPIO_MODE_INPUT))      || ((MODE) == (GPIO_MODE_OUTPUT))       || \
                                               ((MODE) == (GPIO_MODE_ANALOG))     || ((MODE) == (GPIO_MODE_ALT_FUNC))     || \
                                               ((MODE) == (GPIO_MODE_INTRPT_FALL))|| ((MODE) == (GPIO_MODE_INTRPT_RISING))|| \
                                               ((MODE) == (GPIO_MODE_INTRPT_FALL_RISING)))

#define IS_GPIO_PIN_NUMBER(PIN_NUMBER)        (((PIN_NUMBER) == (GPIO_PIN_0))  || ((PIN_NUMBER) == (GPIO_PIN_1))  || \
                                               ((PIN_NUMBER) == (GPIO_PIN_2))  || ((PIN_NUMBER) == (GPIO_PIN_3))  || \
                                               ((PIN_NUMBER) == (GPIO_PIN_4))  || ((PIN_NUMBER) == (GPIO_PIN_5))  || \
                                               ((PIN_NUMBER) == (GPIO_PIN_6))  || ((PIN_NUMBER) == (GPIO_PIN_7))  || \
                                               ((PIN_NUMBER) == (GPIO_PIN_8))  || ((PIN_NUMBER) == (GPIO_PIN_9))  || \
                                               ((PIN_NUMBER) == (GPIO_PIN_10)) || ((PIN_NUMBER) == (GPIO_PIN_11)) || \
                                               ((PIN_NUMBER) == (GPIO_PIN_12)) || ((PIN_NUMBER) == (GPIO_PIN_13)) || \
                                               ((PIN_NUMBER) == (GPIO_PIN_14)) || ((PIN_NUMBER) == (GPIO_PIN_15)))

#define IS_GPIO_SPEED(SPEED)                  (((SPEED) == (GPIO_SPEED_LOW))   || ((SPEED) == (GPIO_SPEED_MEDIUM)) || \
                                               ((SPEED) == (GPIO_SPEED_FAST))  || ((SPEED) == (GPIO_SPEED_HIGH)))

#define IS_GPIO_OTYPE(OTYPE)                  (((OTYPE) == (GPIO_OTYPE_PUSH_PULL))   || ((OTYPE) == (GPIO_OTYPE_OPEN_DRAIN)))

#define IS_GPIO_PULL_TYPE(PULL_TYPE)          (((PULL_TYPE) == (GPIO_PULL_UP))   || ((PULL_TYPE) == (GPIO_PULL_DOWN)) || \
                                               ((PULL_TYPE) == (GPIO_NO_PULL_UP_DOWN)))
                                               
         
#endif /* GPIO_H */
/*** end of file ***/