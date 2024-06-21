#ifndef SPI_H
#define SPI_H

#include "stm32f446xx.h"

typedef enum
{
    SPI_CLK_SPEED_8MHZ,
    SPI_CLK_SPEED_4MHZ,
    SPI_CLK_SPEED_2MHZ,
    SPI_CLK_SPEED_1MHZ,
    SPI_CLK_SPEED_500KHZ,
    SPI_CLK_SPEED_250KHZ,
    SPI_CLK_SPEED_125KHZ,
    SPI_CLK_SPEED_62K5HZ
}spi_clk_speed_t;


typedef enum
{
    SPI_MODE_SLAVE,
    SPI_MODE_MASTER
} spi_config_master_slave_t;

typedef enum
{
    SPI_FULL_DUPLEX,
    SPI_HALF_DUPLEX,
    SPI_SIMPLEX_TXONLY,
    SPI_SIMPLEX_RXONLY
}spi_config_comms_mode_t;


typedef enum
{
    SPI_8_BIT_DATA,
    SPI_16_BIT_DATA
}spi_config_data_length_t;


typedef enum
{
    SPI_CPHA_LOW,
    SPI_CPHA_HIGH

}spi_config_cpha_t;

typedef enum
{
    SPI_CPOL_LOW,
    SPI_CPOL_HIGH

}spi_config_cpol_t;

typedef enum
{
    SPI_HW_SLAVE_MANAGE,
    SPI_SW_SLAVE_MANAGE

}spi_config_slave_manage_t;

typedef enum 
{
    SPI_PIN_RESET,
    SPI_PIN_SET      
} spi_pin_state_t;

void config_spi_speed(spi_register_def_t *p_spi_x,spi_clk_speed_t clk_speed);
void spi_peripheral_clk_cntrl(spi_register_def_t *p_spi_x, spi_pin_state_t enable_or_disable);
void config_spi_baud_rate();


void config_spi_as_master_or_slave(spi_register_def_t *p_spi_x, spi_config_master_slave_t master_or_slave);

void config_spi_comm_mode(spi_register_def_t *p_spi_x, spi_config_comms_mode_t comm_mode);

void config_spi_data_length(spi_register_def_t *p_spi_x, spi_config_data_length_t data_length);

void config_spi_cpha(spi_register_def_t *p_spi_x,spi_config_cpha_t cpha);
void config_spi_cpol(spi_register_def_t *p_spi_x,spi_config_cpol_t cpol);












#endif /* SPI_H */
/*** end of file ***/