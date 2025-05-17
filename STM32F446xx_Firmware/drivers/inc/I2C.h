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
#include <stddef.h>



#define I2C_FM_RISE_TIME                                   300 //nanosecond
#define I2C_SM_RISE_TIME                                   1000  //nanosecond


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

typedef enum
{
    I2C_READY,
    I2C_BUSY_TX,
    I2C_BUSY_RX

}i2c_state_t;

typedef struct
{
    volatile uint8_t *tx_buffer;
    volatile uint8_t *rx_buffer;
    volatile size_t tx_length;
    volatile size_t rx_length;
    volatile i2c_state_t tx_rx_state;
    volatile uint8_t device_addr;
    volatile uint32_t rx_size;
    volatile uint8_t repeated_start;
}i2c_handle_t;



void i2c_peripheral_enable(i2c_register_def_t *p_i2c_x);

void i2c_peripheral_disable(i2c_register_def_t *p_i2c_x);
void i2c_fm_mode_config(i2c_register_def_t *p_i2c_x);
void i2c_sm_mode_config(i2c_register_def_t *p_i2c_x);
void i2c_own_addr_config(i2c_register_def_t *p_i2c_x, uint8_t own_addr);


void i2c_scl_speed_config(i2c_register_def_t *p_i2c_x, i2c_speed_t fm_or_sm_mode, i2c_scl_freq_t scl_freq);

void i2c_ack_config(i2c_register_def_t *p_i2c_x, signal_state_t enable_disable);
void i2c_fm_duty_config(i2c_register_def_t *p_i2c_x, signal_state_t enable_disable);

void i2c_data_tx(i2c_register_def_t *p_i2c_x, uint8_t *p_tx_buffer, size_t size_of_data, uint8_t slave_addr);
void i2c_data_rx(i2c_register_def_t *p_i2c_x, uint8_t *p_rx_buffer, size_t size_of_data, uint8_t slave_addr);
uint8_t i2c_interrupt_data_tx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x, uint8_t *p_tx_buffer, size_t size_of_data, uint8_t slave_addr, uint8_t repeated_start);
uint8_t i2c_interrupt_data_rx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x, uint8_t *p_rx_buffer, size_t size_of_data, uint8_t slave_addr, uint8_t repeated_start);


#endif /* I2C_H */
/*** end of file ***/