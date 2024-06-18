#ifndef SPI_H
#define SPI_H

#include "stm32f446xx.h"

typedef enum
{
    x,
    y
} spi_config_master_slave_t;

typedef enum
{
    x,
    y,
    z
}spi_config_comms_mode_t;


typedef enum
{
    x,
    y
}spi_config_data_length_t;


typedef enum
{
    x,
    y

}spi_config_cpha_cpol_t;

typedef enum 
{
    SPI_PIN_RESET,
    SPI_PIN_SET      
} spi_pin_state_t;


void spi_peripheral_clk_cntrl(spi_register_def_t *p_spi_x, spi_pin_state_t enable_or_disable);

void config_spi_as_slave(spi_register_def_t *p_spi_x);

void config_spi_as_master(spi_register_def_t *p_spi_x, spi_config_master_slave_t master_slave);

void config_spi_comm_mode(spi_register_def_t *p_spi_x, spi_config_comms_mode_t comm_mode);

void config_spi_data_length(spi_register_def_t *p_spi_x, spi_config_data_length_t data_length);

void config_spi_cpha_cpol(spi_register_def_t *p_spi_x,spi_config_cpha_cpol_t cpha_cpol);

void config_spi_speed(spi_register_def_t *p_spi_x);











#endif /* SPI_H */
/*** end of file ***/