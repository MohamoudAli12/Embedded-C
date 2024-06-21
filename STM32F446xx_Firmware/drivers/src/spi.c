#include "spi.h"


void spi_peripheral_clk_cntrl(spi_register_def_t *p_spi_x, spi_pin_state_t enable_or_disable )
{
    if (enable_or_disable == ENABLE)
    {
        if (p_spi_x == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (p_spi_x == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (p_spi_x == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (p_spi_x == SPI4)
        {
            SPI4_PCLK_EN();
        }
    }

    else if (enable_or_disable == DISABLE)
    {
        if (p_spi_x == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (p_spi_x == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (p_spi_x == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (p_spi_x == SPI4)
        {
            SPI4_PCLK_DI();
        }
    }

}

void config_spi_speed(spi_register_def_t *p_spi_x, spi_clk_speed_t clk_speed)
{
    const uint8_t spi_cr1_br_mask =0x07;
    p_spi_x->SPI_CR1 &= ~(spi_cr1_br_mask<<BIT_3);
    p_spi_x->SPI_CR1 |= (clk_speed << BIT_3);

}

void config_spi_as_master_or_slave(spi_register_def_t *p_spi_x, spi_config_master_slave_t master_or_slave)
{
    p_spi_x->SPI_CR1 &= ~(0x01 << BIT_2);
    p_spi_x->SPI_CR1 |= (master_or_slave << BIT_2);    
}


void config_spi_data_length(spi_register_def_t *p_spi_x, spi_config_data_length_t data_length)
{
    p_spi_x->SPI_CR1 &= ~(0x01 << BIT_11);
    p_spi_x->SPI_CR1 |= (data_length << BIT_11);
}

void config_spi_cpha(spi_register_def_t *p_spi_x,spi_config_cpha_t cpha)
{
    p_spi_x->SPI_CR1 &= ~(0x01 << BIT_0);
    p_spi_x->SPI_CR1 |= (cpha << BIT_0);
}

void config_spi_cpol(spi_register_def_t *p_spi_x,spi_config_cpol_t cpol)
{
    p_spi_x->SPI_CR1 &= ~(0x01 << BIT_1);
    p_spi_x->SPI_CR1 |= (cpol << BIT_1);
}