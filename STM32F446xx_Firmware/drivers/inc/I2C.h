/** @file I2C.h
*
* @brief A header file for I2C peripheral
*
* @par
* COPYRIGHT NOTICE: 
*/

#ifndef I2C_H
#define I2C_H

#include "stm32f446xx.h"

#define I2C_CR1_PE                                         0
#define I2C_CCR_FS                                         15


typedef enum 
{
    I2C_SPEED_STANDARD,
    I2C_SPEED_FAST
}i2c_speed_t;

typedef enum
{
    I2C_SLAVE_TX,
    I2C_SLAVE_RX,
    I2C_MASTER_TX,
    I2C_MASTER_RX
}i2c_mode_t;


typedef enum
{
    I2C_FREQ_2MHZ = 0x02


}i2c_freq_t;

void i2c_peripheral_enable(i2c_register_def_t *p_i2c_x);

void i2c_peripheral_disable(i2c_register_def_t *p_i2c_x);

void i2c_sm_mode_config(i2c_register_def_t *p_i2c_x);

void i2c_fm_mode_config(i2c_register_def_t *p_i2c_x);

void i2c_own_addr_config(i2c_register_def_t *p_i2c_x, uint8_t own_addr);











#endif /* I2C_H */
/*** end of file ***/