#ifndef SPI_H
#define SPI_H

#include "stm32f446xx.h"
#include <stdbool.h>
#include <stddef.h>

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
    SPI_READY,
    SPI_BUSY_TX,
    SPI_BUSY_RX

}spi_state_t;

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



void spi_speed_config(spi_register_def_t *p_spi_x, spi_clk_speed_t clk_speed);

void spi_as_master_or_slave_config(spi_register_def_t *p_spi_x, spi_config_master_slave_t master_or_slave);

void spi_line_mode_config(spi_register_def_t *p_spi_x, spi_config_line_mode_t line_mode);

void spi_data_length_config(spi_register_def_t *p_spi_x, spi_config_data_length_t data_length);

void spi_cpha_config(spi_register_def_t *p_spi_x, spi_config_cpha_t cpha);

void spi_cpol_config(spi_register_def_t *p_spi_x, spi_config_cpol_t cpol);

void spi_slave_management_config(spi_register_def_t *p_spi_x, spi_config_slave_manage_t slave_management);



void spi_peripheral_enable(spi_register_def_t *p_spi_x);

void spi_peripheral_disable(spi_register_def_t *p_spi_x);

void spi_data_send(spi_register_def_t *p_spi_x, uint8_t *p_tx_buffer, size_t size_of_data);
void spi_data_receive(spi_register_def_t *p_spi_x, uint8_t *p_rx_buffer, size_t size_of_data);

bool spi_tx_buffer_is_empty(spi_register_def_t *p_spi_x);


void spi_disable_interrupts(spi_register_def_t *p_spi_x);
void spi_intrpt_data_send(spi_register_def_t *p_spi_x, uint8_t *p_tx_buffer, size_t size_of_data);
void spi_intrpt_data_receive(spi_register_def_t *p_spi_x, uint8_t *p_tx_buffer, size_t size_of_data);










#endif /* SPI_H */
/*** end of file ***/