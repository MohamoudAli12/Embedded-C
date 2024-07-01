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

void config_spi_slave_management(spi_register_def_t *p_spi_x,spi_config_slave_manage_t slave_management)
{
    p_spi_x->SPI_CR1 &= ~(0x01 << SPI_CR1_SSM);
    if (slave_management == SPI_SW_SLAVE_MANAGE)
    {
        p_spi_x->SPI_CR1 |= (slave_management << SPI_CR1_SSM);
        p_spi_x->SPI_CR1 |= (0x01 << SPI_CR1_SSI);

    }
    
}


void spi_peripheral_enable(spi_register_def_t *p_spi_x)
{
    p_spi_x->SPI_CR1 |= (0x01 << SPI_CR1_SPE);
}

void spi_peripheral_disable(spi_register_def_t *p_spi_x)
{
    p_spi_x->SPI_CR1 &= ~(0x01 << SPI_CR1_SPE);
}

_Bool spi_tx_buffer_is_full(spi_register_def_t *p_spi_x)
{
    
    return (((p_spi_x->SPI_SR) & (0x02)) == 0);

}


void spi_data_send(spi_register_def_t *p_spi_x, uint8_t *p_tx_buffer, uint32_t size_of_data)
{
    while (size_of_data > 0)
    {
        while (spi_tx_buffer_is_full(p_spi_x));

        if((p_spi_x->SPI_CR1) & (0x01 << SPI_CR1_DFF)) // if spi dataframe is 16bit
        {
            p_spi_x->SPI_DR = *((uint16_t *) p_tx_buffer);
            size_of_data-=2;
            p_tx_buffer+=2;

        }
        else
        {
            p_spi_x->SPI_DR = *p_tx_buffer;
            size_of_data--;
            p_tx_buffer++;
        }
    
    }
     
}

_Bool spi_rx_buffer_is_empty(spi_register_def_t *p_spi_x)
{
    return (((p_spi_x->SPI_SR) & (0x01)) == 0);

}

void spi_data_receive(spi_register_def_t *p_spi_x, uint8_t *p_rx_buffer, uint32_t size_of_data)
{
    while (size_of_data > 0)
    {
        while(spi_rx_buffer_is_empty(p_spi_x));

        if((p_spi_x->SPI_CR1) & (0x01 << SPI_CR1_DFF)) // if spi dataframe is 16bit
        {
           *((uint16_t *) p_rx_buffer) = (uint16_t)p_spi_x->SPI_DR;
           size_of_data-=2;
           p_rx_buffer+=2;
        }
        else
        {
            *p_rx_buffer = (uint8_t)p_spi_x->SPI_DR;
            size_of_data--;
            p_rx_buffer++;
        }
    }
}