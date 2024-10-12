#include "i2c.h"




void i2c_peripheral_enable(i2c_register_def_t *p_i2c_x)
{
    p_i2c_x->I2C_CR1 |= (1<<I2C_CR1_PE);
}

void i2c_peripheral_disable(i2c_register_def_t *p_i2c_x)
{
    p_i2c_x->I2C_CR1 &= ~(1<<I2C_CR1_PE);
}

void i2c_sm_mode_config(i2c_register_def_t *p_i2c_x)
{
    p_i2c_x->I2C_CR2 &= ~(1<<I2C_CCR_FS);
}

void i2c_fm_mode_config(i2c_register_def_t *p_i2c_x)
{
    p_i2c_x->I2C_CR2 |= (1<<I2C_CCR_FS);
}

static uint32_t i2c_pclk1_freq_get(void)
{
    uint32_t pclk1_value_in_hz;
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
            system_clk = HSE_FREQ;
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

    pclk1_value_in_hz = ((system_clk / ahb_prescaler)/apb1_prescaler);
    
    return pclk1_value_in_hz;

}

void i2c_cr2_freq_config(i2c_register_def_t *p_i2c_x)
{
    p_i2c_x->I2C_CR2 |= i2c_pclk1_freq_get()/(1000000UL) & (0x3F);
}

void i2c_fm_duty_config(i2c_register_def_t *p_i2c_x, signal_state_t enable_disable)
{
    if (enable_disable == HIGH)
    {
        p_i2c_x->I2C_CCR |= (0x01 << 14);
    
    }
    else
    {
        p_i2c_x->I2C_CCR |= ~(0x01 << 14);

    }

}

void i2c_scl_speed_config(i2c_register_def_t *p_i2c_x, i2c_speed_t fm_or_sm_mode, i2c_scl_freq_t scl_freq)
{
    
    if ((fm_or_sm_mode == I2C_SPEED_STANDARD) && (scl_freq == I2C_SCL_FREQ_100KHZ))
    {
        p_i2c_x->I2C_TRISE |= ((I2C_SM_RISE_TIME/1000000000UL) * (i2c_pclk1_freq_get())+1);

        p_i2c_x->I2C_CCR |= ((i2c_pclk1_freq_get())/(2 *(I2C_SCL_FREQ_100KHZ))) & (0xFFF);
    }
    
    else if ((fm_or_sm_mode == I2C_SPEED_FAST) && (scl_freq == I2C_SCL_FREQ_400KHZ))
    {
        p_i2c_x->I2C_TRISE |= ((I2C_FM_RISE_TIME/1000000000UL) * (i2c_pclk1_freq_get())+1);

        if ((p_i2c_x->I2C_CCR >> 14) & (0x01))
        {
            p_i2c_x->I2C_CCR |= ((i2c_pclk1_freq_get())/(3 *(I2C_SCL_FREQ_400KHZ))) & (0xFFF); // check logic
        }
        else
        {
            p_i2c_x->I2C_CCR |= ((i2c_pclk1_freq_get())/(25 *(I2C_SCL_FREQ_400KHZ))) & (0xFFF);
        }
        
    }
}


static void i2c_generate_start_condition(i2c_register_def_t *p_i2c_x)
{
   p_i2c_x->I2C_CR1 |= (1 << I2C_CR1_START); 
}

static void i2c_slave_addr_send(i2c_register_def_t *p_i2c_x, uint8_t slave_addr)
{
    slave_addr = slave_addr << 1;
    slave_addr &= ~(1);
    p_i2c_x->I2C_DR = slave_addr;

}

static void i2c_clear_addr_flag(i2c_register_def_t *p_i2c_x)
{
    uint32_t dummy_read = p_i2c_x->I2C_SR1;
    dummy_read = p_i2c_x->I2C_SR2;
    (void) dummy_read;
}

static void i2c_generate_stop_condition(i2c_register_def_t *p_i2c_x)
{
   p_i2c_x->I2C_CR1 |= (1 << I2C_CR1_STOP); 
}

void i2c_data_tx(i2c_register_def_t *p_i2c_x, uint8_t *p_tx_buffer, size_t size_of_data,  uint8_t slave_addr)
{
    i2c_generate_start_condition(p_i2c_x);

    while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_SB));
    i2c_slave_addr_send(p_i2c_x, slave_addr);
    while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_ADDR));
    i2c_clear_addr_flag(p_i2c_x);

    while (size_of_data > 0)
    {
        while(!((p_i2c_x->I2C_SR1) & I2C_FLAG_TXE));
        p_i2c_x->I2C_DR = *p_tx_buffer;
        p_tx_buffer++;
        size_of_data--;

    }
    while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_TXE));
    while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_BTF));

    i2c_generate_stop_condition(p_i2c_x);

}