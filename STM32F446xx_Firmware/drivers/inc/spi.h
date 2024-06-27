#ifndef SPI_H
#define SPI_H

#include "stm32f446xx.h"
#include <stdbool.h>

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
    SPI_TWO_LINE_MODE,
    SPI_ONE_LINE_MODE_TX_ONLY,
    SPI_ONE_LINE_MODE_RX_ONLY,
    SPI_SIMPLEX_RXONLY
}spi_config_line_mode_t;

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

void config_spi_speed(spi_register_def_t *p_spi_x, spi_clk_speed_t clk_speed);

void config_spi_as_master_or_slave(spi_register_def_t *p_spi_x, spi_config_master_slave_t master_or_slave);

void config_spi_line_mode(spi_register_def_t *p_spi_x, spi_config_line_mode_t line_mode);

void config_spi_data_length(spi_register_def_t *p_spi_x, spi_config_data_length_t data_length);

void config_spi_cpha(spi_register_def_t *p_spi_x, spi_config_cpha_t cpha);

void config_spi_cpol(spi_register_def_t *p_spi_x, spi_config_cpol_t cpol);

void config_spi_slave_management(spi_register_def_t *p_spi_x, spi_config_slave_manage_t slave_management);

void spi_peripheral_enable(spi_register_def_t *p_spi_x);

void spi_peripheral_disable(spi_register_def_t *p_spi_x);

void spi_data_send(spi_register_def_t *p_spi_x, uint8_t *p_tx_buffer, uint32_t size_of_data);

_Bool spi_tx_buffer_is_empty(spi_register_def_t *p_spi_x);


#endif /* SPI_H */
/*** end of file ***/