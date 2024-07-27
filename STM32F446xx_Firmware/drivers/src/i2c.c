#include "i2c.h"

i2c_peripheral_enable(i2c_register_def_t *p_i2c_x)
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

static void i2c_pclk1_freq_config(i2c_register_def_t *p_i2c_x)
{
    const uint16_t mask = 0x3F;
    p_i2c_x->I2C_CR2 &= ~mask;
    p_i2c_x->I2C_CR2 |= (I2C_PCLK1_FREQ/1000000UL) & mask;
}

static uint8_t ccr_value_calculate(i2c_scl_freq_t scl_freq)
{
    uint32_t clk_period = (1000000UL/scl_freq);
    uint32_t pclk1_time = (1000000UL/I2C_PCLK1_FREQ);
    uint8_t ccr_value = ((clk_period)/(2*pclk1_time));
    return ccr_value;
}


void i2c_scl_speed_config(i2c_register_def_t *p_i2c_x, i2c_speed_t fm_or_sm_mode, i2c_scl_freq_t scl_freq)
{
    uint16_t ccr_mask = 0xFFF;
    i2c_pclk1_freq_config(p_i2c_x);  //16Mhz

    if ((fm_or_sm_mode == I2C_SPEED_STANDARD) && (scl_freq == I2C_SCL_FREQ_100KHZ))
    {
        i2c_sm_mode_config(p_i2c_x);
        p_i2c_x->I2C_CCR = ccr_value_calculate(scl_freq) & ccr_mask;
    }
    else if ((fm_or_sm_mode == I2C_SPEED_FAST) && (scl_freq == I2C_SCL_FREQ_400KHZ))
    {
        i2c_fm_mode_config(p_i2c_x);
        p_i2c_x->I2C_CCR = ccr_value_calculate(scl_freq) & ccr_mask;
    }
}