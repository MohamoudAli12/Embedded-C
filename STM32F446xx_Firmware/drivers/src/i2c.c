#include "i2c.h"




void i2c_peripheral_enable(i2c_register_def_t *p_i2c_x)
{
    p_i2c_x->I2C_CR1 |= (1<<I2C_CR1_PE);
}

void i2c_peripheral_disable(i2c_register_def_t *p_i2c_x)
{
    p_i2c_x->I2C_CR1 &= ~(1<<I2C_CR1_PE);
}

static void i2c_sm_mode_config(i2c_register_def_t *p_i2c_x)
{
    p_i2c_x->I2C_CR2 &= ~(1<<I2C_CCR_FS);
}

static void i2c_fm_mode_config(i2c_register_def_t *p_i2c_x)
{
    p_i2c_x->I2C_CR2 |= (1<<I2C_CCR_FS);
}

static uint32_t i2c_pclk1_freq_get(void)
{
    uint32_t pclk1_value_in_mhz;
    uint32_t system_clk;
    uint8_t clksrc;
    uint8_t  ahb_prescaler_register_value;
    uint16_t ahb_prescaler;
    const uint16_t ahb_prescaler_factor[]={2,4,8,16,64,128,256,512};
    uint8_t apb1_prescaler_register_value;
    uint8_t apb1_prescaler;
    const uint8_t apb1_prescaler_factor[]={2,4,8,16};
    
    clksrc = (RCC->RCC_CFGR >> 2) & (0x03);

    switch (clksrc)
    {
        case HSI:
            system_clk = HSI_FREQ;
            break;
        case HSE:
            system_clk = HSE_FREQ;
            break;
        //TODO: PLL CLK Source
        default:
            break; 

    }

    ahb_prescaler_register_value = (RCC->RCC_CFGR >> 4) & (0x0F);
    if (ahb_prescaler_register_value < 8)
    {
        ahb_prescaler = 1;
    }
    else
    {
        ahb_prescaler = ahb_prescaler_factor[ahb_prescaler_register_value - 8];
    }

    apb1_prescaler_register_value = (RCC->RCC_CFGR >> 10) & (0x07);
    if (apb1_prescaler_register_value < 4)
    {
        apb1_prescaler = 1;
    }
    else
    {
        apb1_prescaler = apb1_prescaler_factor[apb1_prescaler_register_value - 4];
    }

    pclk1_value_in_mhz = ((system_clk / ahb_prescaler)/apb1_prescaler)/1000000UL;
    
    return pclk1_value_in_mhz;

}

static void i2c_cr2_freq_config(i2c_register_def_t *p_i2c_x)
{
    p_i2c_x->I2C_CR2 |= i2c_pclk1_freq_get() & (0x3F);
}


void i2c_scl_speed_config(i2c_register_def_t *p_i2c_x, i2c_speed_t fm_or_sm_mode, i2c_scl_freq_t scl_freq)
{
    uint16_t ccr_mask = 0xFFF;
    
    if ((fm_or_sm_mode == I2C_SPEED_STANDARD) && (scl_freq == I2C_SCL_FREQ_100KHZ))
    {
        
    }
    else if ((fm_or_sm_mode == I2C_SPEED_FAST) && (scl_freq == I2C_SCL_FREQ_400KHZ))
    {
    
    }
}