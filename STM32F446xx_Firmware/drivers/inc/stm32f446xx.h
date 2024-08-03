/** @file stm32f446xx.h
*
* @brief A device header file for stm32f446xx MCU
*
* @par
* COPYRIGHT NOTICE: 
*/

#ifndef STM32F446XX_H
#define STM32F446XX_H

#include <stdint.h>

// NVIC Register definitions

#define NVIC_ISER0                                        ((volatile uint32_t *) 0xE000E100)
#define NVIC_ISER1                                        ((volatile uint32_t *) 0xE000E104)
#define NVIC_ISER2                                        ((volatile uint32_t *) 0xE000E108)
#define NVIC_ISER3                                        ((volatile uint32_t *) 0xE000E10C)
#define NVIC_ISER4                                        ((volatile uint32_t *) 0xE000E110)
#define NVIC_ISER5                                        ((volatile uint32_t *) 0xE000E114)
#define NVIC_ISER6                                        ((volatile uint32_t *) 0xE000E118)
#define NVIC_ISER7                                        ((volatile uint32_t *) 0xE000E11C)

#define NVIC_ICER0                                        ((volatile uint32_t *) 0xE000E180)
#define NVIC_ICER1                                        ((volatile uint32_t *) 0xE000E184)
#define NVIC_ICER2                                        ((volatile uint32_t *) 0xE000E188)
#define NVIC_ICER3                                        ((volatile uint32_t *) 0xE000E18C)
#define NVIC_ICER4                                        ((volatile uint32_t *) 0xE000E190)
#define NVIC_ICER5                                        ((volatile uint32_t *) 0xE000E194)
#define NVIC_ICER6                                        ((volatile uint32_t *) 0xE000E198)
#define NVIC_ICER7                                        ((volatile uint32_t *) 0xE000E19C)

#define NVIC_IPR_BASE_ADDR                                ((volatile uint32_t *) 0xE000E400)

#define NVIC_IPR_BITS_IMPLEMENTED                         (0x04)// The processor implements only bits[7:4] for interrupts

/***********************************************************************************************/
/***********************************************************************************************/

/* IRQ NUMBERS*/
typedef enum
{
    EXTI0_IRQ_POS = 6,
    EXTI1_IRQ_POS = 7,
    EXTI2_IRQ_POS = 8,
    EXTI3_IRQ_POS = 9,
    EXTI4_IRQ_POS = 10,
    EXTI15_10_IRQ_POS = 40,
    SPI1_IRQ_POS = 35,
    SPI2_IRQ_POS = 36,
    SPI3_IRQ_POS = 51,
    SPI4_IRQ_POS = 84

}irq_position_t;

typedef enum 
{
    DISABLE = 0,
    ENABLE = 1,
    RESET = DISABLE,
    SET = ENABLE,
    HIGH = SET,
    LOW = RESET
}signal_state_t;








/***********************************************************************************************/
/***********************************************************************************************/




/***********************************************************************************************/
/***********************************************************************************************/


// Flash and SRAM Base address

#define FLASH_BASE_ADDR                        0x08000000U           /*!< This macro defines the base address of flash memory*/
#define SRAM1_BASE_ADDR                        0x20000000U           /*!< This macro defines the base address of SRAM1*/
#define SRAM2_BASE_ADDR                        0x2001C000U           /*!< This macro defines the base address of SRAM2*/
#define SYSMEM_BASE_ADDR                       0x1FFF0000U           /*!< This macro defines the base address of system memory*/


// Base addresses of bus domains

#define PERIPH_BASE_ADDR                       0x40000000U
#define APB1_BASE_ADDR                         PERIPH_BASE_ADDR 
#define APB2_BASE_ADDR                         0x40010000U
#define AHB1_BASE_ADDR                         0x40020000U
#define AHB2_BASE_ADDR                         0x50000000U
#define AHB3_BASE_ADDR                         0x60000000U

/***********************************************************************************************/
/***********************************************************************************************/
#define HSI                                            0x00
#define HSE                                            0x01
#define PLL                                            0x02
#define PLL_R                                          0x03
#define HSI_FREQ                                       16000000UL
#define HSE_FREQ                                       8000000UL

// RCC Base address definition

#define RCC_BASE_ADDR                         ((AHB1_BASE_ADDR) + (0x3800U))


// RCC register definitions

typedef struct
{
    volatile uint32_t RCC_CR;
    volatile uint32_t RCC_PLLCFGR;
    volatile uint32_t RCC_CFGR;
    volatile uint32_t RCC_CIR;
    volatile uint32_t RCC_AHB1RSTR;
    volatile uint32_t RCC_AHB2RSTR;
    volatile uint32_t RCC_AHB3RSTR;
    volatile uint32_t RESERVED0;
    volatile uint32_t RCC_APB1RSTR;
    volatile uint32_t RCC_APB2RSTR;
    volatile uint32_t RESERVED1[2];
    volatile uint32_t RCC_AHB1ENR;
    volatile uint32_t RCC_AHB2ENR;
    volatile uint32_t RCC_AHB3ENR;
    volatile uint32_t RESERVED3;
    volatile uint32_t RCC_APB1ENR;
    volatile uint32_t RCC_APB2ENR;
    volatile uint32_t RESERVED4[2];
    volatile uint32_t RCC_AHB1LPENR;
    volatile uint32_t RCC_AHB2LPENR;
    volatile uint32_t RCC_AHB3LPENR;
    volatile uint32_t RESERVED5;
    volatile uint32_t RCC_APB1LPENR;
    volatile uint32_t RCC_APB2LPENR;
    volatile uint32_t RESERVED6[2];
    volatile uint32_t RCC_BDCR;
    volatile uint32_t RCC_CSR;
    volatile uint32_t RESERVED7[2];
    volatile uint32_t RCC_SSCGR;
    volatile uint32_t RCC_PLLI2SCFGR;
    volatile uint32_t RCC_PLLSAICFGR;
    volatile uint32_t RCC_DCKCFGR;
    volatile uint32_t RCC_CKGATENR;
    volatile uint32_t RCC_DCKCFGR2;

}rcc_register_def_t;

#define RCC                                 ((rcc_register_def_t * )RCC_BASE_ADDR)

/***********************************************************************************************/
/***********************************************************************************************/

// GPIO Base Addresses

#define GPIOA_BASE_ADDR                       ((AHB1_BASE_ADDR) + (0x0000U))
#define GPIOB_BASE_ADDR                       ((AHB1_BASE_ADDR) + (0x0400U))
#define GPIOC_BASE_ADDR                       ((AHB1_BASE_ADDR) + (0x0800U))
#define GPIOD_BASE_ADDR                       ((AHB1_BASE_ADDR) + (0x0C00U))
#define GPIOE_BASE_ADDR                       ((AHB1_BASE_ADDR) + (0x1000U))
#define GPIOF_BASE_ADDR                       ((AHB1_BASE_ADDR) + (0x1400U))
#define GPIOG_BASE_ADDR                       ((AHB1_BASE_ADDR) + (0x1800U))
#define GPIOH_BASE_ADDR                       ((AHB1_BASE_ADDR) + (0x1C00U))

//GPIO Register definitions

typedef struct 
{
    volatile uint32_t MODER;   /*!< GPIO port mode register */
    volatile uint32_t OTYPER;  /*!< GPIO port output type register */
    volatile uint32_t OSPEEDR; /*!< GPIO port output speed register */
    volatile uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register */
    volatile uint32_t IDR;     /*!< GPIO port input data register */
    volatile uint32_t ODR;     /*!< GPIO port output data register */
    volatile uint32_t BSRR;    /*!< GPIO port bit set/reset register */
    volatile uint32_t LCKR;    /*!< GPIO port configuration lock register */
    volatile uint32_t AFRL;    /*!< GPIO alternate function low register */
    volatile uint32_t AFRH;    /*!< GPIO alternate function high register */
} gpio_register_def_t;

#define GPIOA                                 ((gpio_register_def_t * )GPIOA_BASE_ADDR)
#define GPIOB                                 ((gpio_register_def_t * )GPIOB_BASE_ADDR)
#define GPIOC                                 ((gpio_register_def_t * )GPIOC_BASE_ADDR)
#define GPIOD                                 ((gpio_register_def_t * )GPIOD_BASE_ADDR)
#define GPIOE                                 ((gpio_register_def_t * )GPIOE_BASE_ADDR)
#define GPIOF                                 ((gpio_register_def_t * )GPIOF_BASE_ADDR)
#define GPIOG                                 ((gpio_register_def_t * )GPIOG_BASE_ADDR)
#define GPIOH                                 ((gpio_register_def_t * )GPIOH_BASE_ADDR)


#define IS_GPIO_PORT(GPIO_PORT)               ((GPIO_PORT) == (GPIOA) || \
                                               (GPIO_PORT) == (GPIOB) || \
                                               (GPIO_PORT) == (GPIOC) || \
                                               (GPIO_PORT) == (GPIOD) || \
                                               (GPIO_PORT) == (GPIOE) || \
                                               (GPIO_PORT) == (GPIOF) || \
                                               (GPIO_PORT) == (GPIOG) || \
                                               (GPIO_PORT) == (GPIOH))



//GPIO clock enable macros

#define GPIOA_PCLK_EN()                        ((RCC->RCC_AHB1ENR)  |= (1<<0))
#define GPIOB_PCLK_EN()                        ((RCC->RCC_AHB1ENR)  |= (1<<1))
#define GPIOC_PCLK_EN()                        ((RCC->RCC_AHB1ENR)  |= (1<<2))
#define GPIOD_PCLK_EN()                        ((RCC->RCC_AHB1ENR)  |= (1<<3))
#define GPIOE_PCLK_EN()                        ((RCC->RCC_AHB1ENR)  |= (1<<4))
#define GPIOF_PCLK_EN()                        ((RCC->RCC_AHB1ENR)  |= (1<<5))
#define GPIOG_PCLK_EN()                        ((RCC->RCC_AHB1ENR)  |= (1<<6))
#define GPIOH_PCLK_EN()                        ((RCC->RCC_AHB1ENR)  |= (1<<7))

//GPIO clock disable macros

#define GPIOA_PCLK_DI()                        ((RCC->RCC_AHB1ENR)  &= ~(1<<0))
#define GPIOB_PCLK_DI()                        ((RCC->RCC_AHB1ENR)  &= ~(1<<1))
#define GPIOC_PCLK_DI()                        ((RCC->RCC_AHB1ENR)  &= ~(1<<2))
#define GPIOD_PCLK_DI()                        ((RCC->RCC_AHB1ENR)  &= ~(1<<3))
#define GPIOE_PCLK_DI()                        ((RCC->RCC_AHB1ENR)  &= ~(1<<4))
#define GPIOF_PCLK_DI()                        ((RCC->RCC_AHB1ENR)  &= ~(1<<5))
#define GPIOG_PCLK_DI()                        ((RCC->RCC_AHB1ENR)  &= ~(1<<6))
#define GPIOH_PCLK_DI()                        ((RCC->RCC_AHB1ENR)  &= ~(1<<7))

// GPIOx Reset 

#define GPIOA_RESET()                          do{ ((RCC->RCC_AHB1RSTR)  |= (1<<0)); ((RCC->RCC_AHB1RSTR)  &= ~(1<<0));} while (0)
#define GPIOB_RESET()                          do{ ((RCC->RCC_AHB1RSTR)  |= (1<<1)); ((RCC->RCC_AHB1RSTR)  &= ~(1<<1));} while (0)
#define GPIOC_RESET()                          do{ ((RCC->RCC_AHB1RSTR)  |= (1<<2)); ((RCC->RCC_AHB1RSTR)  &= ~(1<<2));} while (0)
#define GPIOD_RESET()                          do{ ((RCC->RCC_AHB1RSTR)  |= (1<<3)); ((RCC->RCC_AHB1RSTR)  &= ~(1<<3));} while (0)
#define GPIOE_RESET()                          do{ ((RCC->RCC_AHB1RSTR)  |= (1<<4)); ((RCC->RCC_AHB1RSTR)  &= ~(1<<4));} while (0)
#define GPIOF_RESET()                          do{ ((RCC->RCC_AHB1RSTR)  |= (1<<5)); ((RCC->RCC_AHB1RSTR)  &= ~(1<<5));} while (0)
#define GPIOG_RESET()                          do{ ((RCC->RCC_AHB1RSTR)  |= (1<<6)); ((RCC->RCC_AHB1RSTR)  &= ~(1<<6));} while (0)
#define GPIOH_RESET()                          do{ ((RCC->RCC_AHB1RSTR)  |= (1<<7)); ((RCC->RCC_AHB1RSTR)  &= ~(1<<7));} while (0)

/***********************************************************************************************/
/***********************************************************************************************/

// EXTI base address definition

#define EXTI_BASE_ADDR                        ((APB2_BASE_ADDR) + (0x3C00U))

//EXTI Register definitions

typedef struct
{
    volatile uint32_t EXTI_IMR;
    volatile uint32_t EXTI_EMR;
    volatile uint32_t EXTI_RTSR;
    volatile uint32_t EXTI_FTSR;
    volatile uint32_t EXTI_SWIER;
    volatile uint32_t EXTI_PR;

}exti_register_def_t;

#define EXTI                                  ((exti_register_def_t * )EXTI_BASE_ADDR)

/***********************************************************************************************/
/***********************************************************************************************/

// SYSCFG base address definition

#define SYSCFG_BASE_ADDR                                ((APB2_BASE_ADDR) + (0x3800U))
//SYSCFG Register definitions
typedef struct
{
    volatile uint32_t SYSCFG_MEMRMP;
    volatile uint32_t SYSCFG_PMC;
    volatile uint32_t SYSCFG_EXTICR1;
    volatile uint32_t SYSCFG_EXTICR2;
    volatile uint32_t SYSCFG_EXTICR3;
    volatile uint32_t SYSCFG_EXTICR4;
    volatile uint32_t RESERVED1[2];
    volatile uint32_t SYSCFG_CMPCR;
    volatile uint32_t RESERVED2[2];
    volatile uint32_t SYSCFG_CFGR;
}syscfg_register_def_t;

#define SYSCFG                                           ((syscfg_register_def_t *)SYSCFG_BASE_ADDR)

// clock enable for SYSCFG

#define SYSCFG_PCLK_EN()                                 ((RCC->RCC_APB2ENR)  |= (1<<14))
#define SYSCFG_PCLK_DI()                                 ((RCC->RCC_APB2ENR)  &= ~(1<<14))

/***********************************************************************************************/
/***********************************************************************************************/

// SPI Base Address definition

#define SPI1_BASE_ADDR                                    ((APB2_BASE_ADDR)+(0x3000U))
#define SPI2_BASE_ADDR                                    ((APB1_BASE_ADDR)+(0x3800U)) 
#define SPI3_BASE_ADDR                                    ((APB1_BASE_ADDR)+(0x3C00U)) 
#define SPI4_BASE_ADDR                                    ((APB2_BASE_ADDR)+(0x3400U))

// SPI register definitions
typedef struct 
{
    volatile uint32_t SPI_CR1;
    volatile uint32_t SPI_CR2;
    volatile uint32_t SPI_SR;
    volatile uint32_t SPI_DR;
    volatile uint32_t SPI_CRCPR;
    volatile uint32_t SPI_RXCRCR;
    volatile uint32_t SPI_TXCRCR;
    volatile uint32_t SPI_I2SCFGR;
    volatile uint32_t SPI_I2SPR;
}spi_register_def_t;

#define SPI1                                                ((spi_register_def_t *)SPI1_BASE_ADDR)
#define SPI2                                                ((spi_register_def_t *)SPI2_BASE_ADDR)
#define SPI3                                                ((spi_register_def_t *)SPI3_BASE_ADDR)
#define SPI4                                                ((spi_register_def_t *)SPI4_BASE_ADDR)

// spi peripheral clock enable 

#define SPI1_PCLK_EN()                                      ((RCC->RCC_APB2ENR)  |= (1<<12))
#define SPI2_PCLK_EN()                                      ((RCC->RCC_APB1ENR)  |= (1<<14))
#define SPI3_PCLK_EN()                                      ((RCC->RCC_APB1ENR)  |= (1<<15))
#define SPI4_PCLK_EN()                                      ((RCC->RCC_APB2ENR)  |= (1<<13))

//SPI peripheral clock disable

#define SPI1_PCLK_DI()                                      ((RCC->RCC_APB2ENR)  &= ~(1<<12))
#define SPI2_PCLK_DI()                                      ((RCC->RCC_APB1ENR)  &= ~(1<<14))
#define SPI3_PCLK_DI()                                      ((RCC->RCC_APB1ENR)  &= ~(1<<15))
#define SPI4_PCLK_DI()                                      ((RCC->RCC_APB2ENR)  &= ~(1<<13))


// SPI cr1 Register bits fields

#define SPI_CR1_CPHA                                        0  
#define SPI_CR1_CPOL                                        1  
#define SPI_CR1_MSTR                                        2  
#define SPI_CR1_BR                                          3   
#define SPI_CR1_SPE                                         6  
#define SPI_CR1_LSBFIRST                                    7  
#define SPI_CR1_SSI                                         8  
#define SPI_CR1_SSM                                         9  
#define SPI_CR1_RXONLY                                      10 
#define SPI_CR1_DFF                                         11 
#define SPI_CR1_CRCNEXT                                     12 
#define SPI_CR1_CRCEN                                       13 
#define SPI_CR1_BIDIOE                                      14 
#define SPI_CR1_BIDIMODE                                    15

// SPI CR2 Register bits fields

#define SPI_CR2_TXEIE                                       7
#define SPI_CR2_RXNEIE                                      6
#define SPI_CR2_ERRIE                                       5


// SPI status register bit fields

#define SPI_SR_RXNE                                         0
#define SPI_SR_TXE                                          1
#define SPI_SR_CHSIDE                                       2
#define SPI_SR_UDR                                          3
#define SPI_SR_CRCERR                                       4
#define SPI_SR_MODF                                         5
#define SPI_SR_OVR                                          6
#define SPI_SR_BSY                                          7
#define SPI_SR_FRE                                          8




/***********************************************************************************************/
/***********************************************************************************************/

//I2C Base Address definition

#define I2C1_BASE_ADDR                                    ((APB1_BASE_ADDR)+(0x5400U))
#define I2C2_BASE_ADDR                                    ((APB1_BASE_ADDR)+(0x5800U)) 
#define I2C3_BASE_ADDR                                    ((APB1_BASE_ADDR)+(0x5C00U)) 

// I2C register definitions

typedef struct 
{
    volatile uint32_t I2C_CR1;
    volatile uint32_t I2C_CR2;
    volatile uint32_t I2C_OAR1;
    volatile uint32_t I2C_OAR2;
    volatile uint32_t I2C_DR;
    volatile uint32_t I2C_SR1;
    volatile uint32_t I2C_SR2;
    volatile uint32_t I2C_CCR;
    volatile uint32_t I2C_TRISE;
    volatile uint32_t I2C_FLTR;
}i2c_register_def_t;

#define I2C1                                                ((i2c_register_def_t *)I2C1_BASE_ADDR)
#define I2C2                                                ((i2c_register_def_t *)I2C2_BASE_ADDR)
#define I2C3                                                ((i2c_register_def_t *)I2C3_BASE_ADDR)

// I2C peripheral clock enable 

#define I2C1_PCLK_EN()                                      ((RCC->RCC_APB1ENR)  |= (1<<21))
#define I2C2_PCLK_EN()                                      ((RCC->RCC_APB1ENR)  |= (1<<22))
#define I2C3_PCLK_EN()                                      ((RCC->RCC_APB1ENR)  |= (1<<23))

// I2C peripheral clock disable 

#define I2C1_PCLK_DI()                                      ((RCC->RCC_APB1ENR)  &= ~(1<<21))
#define I2C2_PCLK_DI()                                      ((RCC->RCC_APB1ENR)  &= ~(1<<22))
#define I2C3_PCLK_DI()                                      ((RCC->RCC_APB1ENR)  &= ~(1<<23))




/***********************************************************************************************/
/***********************************************************************************************/




#endif /* MODULE_H */
/*** end of file ***/