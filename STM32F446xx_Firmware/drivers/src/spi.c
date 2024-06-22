#include "spi.h"



void config_spi_speed(spi_register_def_t *p_spi_x, spi_clk_speed_t clk_speed)
{
    const uint8_t spi_cr1_br_mask =0x07;
    p_spi_x->SPI_CR1 &= ~(spi_cr1_br_mask << SPI_CR1_BR);
    p_spi_x->SPI_CR1 |= (clk_speed << SPI_CR1_BR);

}

void config_spi_as_master_or_slave(spi_register_def_t *p_spi_x, spi_config_master_slave_t master_or_slave)
{
    p_spi_x->SPI_CR1 &= ~(0x01 << SPI_CR1_MSTR);
    p_spi_x->SPI_CR1 |= (master_or_slave << SPI_CR1_MSTR);    
}


void config_spi_data_length(spi_register_def_t *p_spi_x, spi_config_data_length_t data_length)
{
    p_spi_x->SPI_CR1 &= ~(0x01 << SPI_CR1_DFF);
    p_spi_x->SPI_CR1 |= (data_length << SPI_CR1_DFF);
}

void config_spi_cpha(spi_register_def_t *p_spi_x,spi_config_cpha_t cpha)
{
    p_spi_x->SPI_CR1 &= ~(0x01 << SPI_CR1_CPHA);
    p_spi_x->SPI_CR1 |= (cpha << SPI_CR1_CPHA);
}

void config_spi_cpol(spi_register_def_t *p_spi_x,spi_config_cpol_t cpol)
{
    p_spi_x->SPI_CR1 &= ~(0x01 << SPI_CR1_CPOL);
    p_spi_x->SPI_CR1 |= (cpol << SPI_CR1_CPOL);
}

void config_spi_line_mode(spi_register_def_t *p_spi_x, spi_config_line_mode_t line_mode)
{
    switch (line_mode)
    {
        case SPI_SIMPLEX_RXONLY:
            p_spi_x->SPI_CR1 &= ~(0x01 << SPI_CR1_BIDIMODE);
            p_spi_x->SPI_CR1 |= (0x01 << SPI_CR1_RXONLY);
            break;

        case SPI_TWO_LINE_MODE:
            p_spi_x->SPI_CR1 &= ~(0x01 << SPI_CR1_RXONLY);
            p_spi_x->SPI_CR1 &= ~(0x01 << SPI_CR1_BIDIMODE);
            break;

        case SPI_ONE_LINE_MODE_RX_ONLY:
            p_spi_x->SPI_CR1 |= (0x01 << SPI_CR1_BIDIMODE);
            p_spi_x->SPI_CR1 &= ~(0x01 << SPI_CR1_BIDIOE);
            break;

        case SPI_ONE_LINE_MODE_TX_ONLY:
            p_spi_x->SPI_CR1 |= (0x01 << SPI_CR1_BIDIMODE);
            p_spi_x->SPI_CR1 |= (0x01 << SPI_CR1_BIDIOE);
            break;

        default:
            break;
    }    

}