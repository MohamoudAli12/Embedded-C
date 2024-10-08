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
    I2C_SCL_FREQ_100KHZ = 100000UL,
    I2C_SCL_FREQ_400KHZ = 400000UL


}i2c_scl_freq_t;

void i2c_peripheral_enable(i2c_register_def_t *p_i2c_x);

void i2c_peripheral_disable(i2c_register_def_t *p_i2c_x);
void i2c_fm_mode_config(i2c_register_def_t *p_i2c_x);
void i2c_sm_mode_config(i2c_register_def_t *p_i2c_x);
void i2c_own_addr_config(i2c_register_def_t *p_i2c_x, uint8_t own_addr);


void i2c_scl_speed_config(i2c_register_def_t *p_i2c_x, i2c_speed_t fm_or_sm_mode, i2c_scl_freq_t scl_freq);

void i2c_ack_config();
void i2c_fm_duty_config(i2c_register_def_t *p_i2c_x, signal_state_t enable_disable);







#endif /* I2C_H */
/*** end of file ***/