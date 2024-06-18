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