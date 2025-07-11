#include "i2c.h"

static void i2c_generate_start_condition(i2c_register_def_t *p_i2c_x);
static void i2c_generate_stop_condition(i2c_register_def_t *p_i2c_x);
static void i2c_slave_addr_write(i2c_register_def_t *p_i2c_x, uint8_t slave_addr);
static void i2c_slave_addr_read(i2c_register_def_t *p_i2c_x, uint8_t slave_addr);
static void i2c_clear_addr_flag(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x);
static bool i2c_is_master(i2c_register_def_t *p_i2c_x);
static bool i2c_is_slave(i2c_register_def_t *p_i2c_x);
static uint32_t i2c_pclk1_freq_get(void);
static bool i2c_sb_interrupt_event(i2c_register_def_t *p_i2c_x);
static bool i2c_addr_interrupt_event(i2c_register_def_t *p_i2c_x);
static bool i2c_btf_interrupt_event(i2c_register_def_t *p_i2c_x);
static bool i2c_stopf_interrupt_event(i2c_register_def_t *p_i2c_x);
static bool i2c_txe_interrupt_event(i2c_register_def_t *p_i2c_x);
static bool i2c_rxne_interrupt_event(i2c_register_def_t *p_i2c_x);
static bool i2c_berr_interrupt_error(i2c_register_def_t *p_i2c_x);
static bool i2c_arlo_interrupt_error(i2c_register_def_t *p_i2c_x);
static bool i2c_af_interrupt_error(i2c_register_def_t *p_i2c_x);
static bool i2c_ovr_interrupt_error(i2c_register_def_t *p_i2c_x);
static bool i2c_timeout_interrupt_error(i2c_register_def_t *p_i2c_x);
static void i2c_close_tx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x);
static void i2c_close_rx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x);

i2c_handle_t i2c1_handle = { 0 };
i2c_handle_t i2c2_handle = { 0 };
i2c_handle_t i2c3_handle = { 0 };

__weak void i2c_application_event_callback(i2c_register_def_t *p_i2c_x, i2c_events_t event)
{
}

static void i2c_generate_start_condition(i2c_register_def_t *p_i2c_x)
{
  p_i2c_x->I2C_CR1 |= (1 << I2C_CR1_START);
}

static void i2c_generate_stop_condition(i2c_register_def_t *p_i2c_x)
{
  p_i2c_x->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

static void i2c_slave_addr_write(i2c_register_def_t *p_i2c_x, uint8_t slave_addr)
{
  slave_addr = slave_addr << 1;
  slave_addr &= ~(1);
  p_i2c_x->I2C_DR = slave_addr;
}
static void i2c_slave_addr_read(i2c_register_def_t *p_i2c_x, uint8_t slave_addr)
{
  slave_addr = slave_addr << 1;
  slave_addr |= (1);
  p_i2c_x->I2C_DR = slave_addr;
}

static void i2c_clear_addr_flag(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x)
{
  uint32_t dummy_read;

  if (i2c_is_master(p_i2c_x))
  {
    if (p_i2c_handle->tx_rx_state == I2C_BUSY_RX)
    {
      if (p_i2c_handle->rx_total_size == 1)
      {
        // 1-byte receive: disable ACK
        i2c_ack_config(p_i2c_x, DISABLE);
      }
      else if (p_i2c_handle->rx_total_size == 2)
      {
        // 2-byte receive: set POS and disable ACK
        p_i2c_x->I2C_CR1 |= (1 << I2C_CR1_POS);
        i2c_ack_config(p_i2c_x, DISABLE);
      }
      // For >2 bytes: leave ACK enabled (default)
    }
  }

  // Always clear ADDR by reading SR1 and SR2
  dummy_read = p_i2c_x->I2C_SR1;
  dummy_read = p_i2c_x->I2C_SR2;
  (void)dummy_read;
}

static bool i2c_is_master(i2c_register_def_t *p_i2c_x)
{
  return (((p_i2c_x->I2C_SR2) & (1 << I2C_SR2_MSL)) == 1);
}
static bool i2c_is_slave(i2c_register_def_t *p_i2c_x)
{
  return (((p_i2c_x->I2C_SR2) & (1 << I2C_SR2_MSL)) == 0);
}

static uint32_t i2c_pclk1_freq_get(void)
{
  uint32_t pclk1_value_in_hz;
  uint32_t system_clk;
  uint8_t clksrc;
  uint8_t ahb_prescaler_register_value;
  uint16_t ahb_prescaler;
  const uint16_t ahb_prescaler_factor[] = { 2, 4, 8, 16, 64, 128, 256, 512 };
  uint8_t apb1_prescaler_register_value;
  uint8_t apb1_prescaler;
  const uint8_t apb1_prescaler_factor[] = { 2, 4, 8, 16 };

  clksrc = (RCC->RCC_CFGR >> 2) & (0x03);

  switch (clksrc)
  {
  case HSI:
    system_clk = HSI_FREQ;
    break;
  case HSE:
    system_clk = HSE_FREQ;
    break;
  // TODO: PLL CLK Source
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

  pclk1_value_in_hz = ((system_clk / ahb_prescaler) / apb1_prescaler);

  return pclk1_value_in_hz;
}

static bool i2c_sb_interrupt_event(i2c_register_def_t *p_i2c_x)
{
  bool sb_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_SB) != 0;
  bool itevten_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITEVTEN)) != 0;
  return sb_flag_set && itevten_flag_set;
}

static bool i2c_addr_interrupt_event(i2c_register_def_t *p_i2c_x)
{
  bool addr_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_ADDR) != 0;
  bool itevten_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITEVTEN)) != 0;
  return addr_flag_set && itevten_flag_set;
}
static bool i2c_btf_interrupt_event(i2c_register_def_t *p_i2c_x)
{
  bool btf_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_BTF) != 0;
  bool itevten_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITEVTEN)) != 0;
  return btf_flag_set && itevten_flag_set;
}
static bool i2c_stopf_interrupt_event(i2c_register_def_t *p_i2c_x)
{
  bool stopf_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_STOPF) != 0;
  bool itevten_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITEVTEN)) != 0;
  return stopf_flag_set && itevten_flag_set;
}

static bool i2c_txe_interrupt_event(i2c_register_def_t *p_i2c_x)
{
  bool txe_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_TXE) != 0;
  bool itevten_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITEVTEN)) != 0;
  bool itbufen_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITBUFEN)) != 0;
  return txe_flag_set && itevten_flag_set && itbufen_flag_set;
}
static bool i2c_rxne_interrupt_event(i2c_register_def_t *p_i2c_x)
{
  bool rxne_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_RXNE) != 0;
  bool itevten_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITEVTEN)) != 0;
  bool itbufen_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITBUFEN)) != 0;
  return rxne_flag_set && itevten_flag_set && itbufen_flag_set;
}

static bool i2c_berr_interrupt_error(i2c_register_def_t *p_i2c_x)
{
  bool berr_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_BERR) != 0;
  bool iterren_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITERREN)) != 0;
  return berr_flag_set && iterren_flag_set;
}

static bool i2c_arlo_interrupt_error(i2c_register_def_t *p_i2c_x)
{
  bool arlo_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_ARLO) != 0;
  bool iterren_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITERREN)) != 0;
  return arlo_flag_set && iterren_flag_set;
}

static bool i2c_af_interrupt_error(i2c_register_def_t *p_i2c_x)
{
  bool af_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_AF) != 0;
  bool iterren_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITERREN)) != 0;
  return af_flag_set && iterren_flag_set;
}

static bool i2c_ovr_interrupt_error(i2c_register_def_t *p_i2c_x)
{
  bool ovr_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_OVR) != 0;
  bool iterren_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITERREN)) != 0;
  return ovr_flag_set && iterren_flag_set;
}
static bool i2c_timeout_interrupt_error(i2c_register_def_t *p_i2c_x)
{
  bool timeout_flag_set = ((p_i2c_x->I2C_SR1) & I2C_FLAG_TIMEOUT) != 0;
  bool iterren_flag_set = ((p_i2c_x->I2C_CR2) & (1 << I2C_CR2_ITERREN)) != 0;
  return timeout_flag_set && iterren_flag_set;
}

static void i2c_close_tx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x)
{
  // disable ITBUFEN
  p_i2c_x->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

  // disable ITEVEN
  p_i2c_x->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

  // reset / clear members of i2c handle
  p_i2c_handle->tx_rx_state = I2C_READY;
  p_i2c_handle->tx_buffer = NULL;
  p_i2c_handle->tx_length = 0;
}
static void i2c_close_rx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x)
{
  // disable ITBUFEN
  p_i2c_x->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

  // disable ITEVEN
  p_i2c_x->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

  // reset / clear members of i2c handle
  p_i2c_handle->tx_rx_state = I2C_READY;
  p_i2c_handle->rx_buffer = NULL;
  p_i2c_handle->rx_bytes_remaining = 0;
  p_i2c_handle->rx_total_size = 0;
  i2c_ack_config(p_i2c_x, ENABLE);
}

static void i2c_master_handle_1byte_rx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x)
{
  *p_i2c_handle->rx_buffer = p_i2c_x->I2C_DR;
  p_i2c_handle->rx_bytes_remaining--;

  if (p_i2c_handle->repeated_start == I2C_REPEATED_START_DISABLED)
    i2c_generate_stop_condition(p_i2c_x);

  i2c_close_rx(p_i2c_handle, p_i2c_x);
  i2c_application_event_callback(p_i2c_x, I2C_EVENT_RX_COMPLETE);
}

static void i2c_master_handle_2byte_rx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x)
{
  if (i2c_btf_interrupt_event(p_i2c_x))
  {
    if (p_i2c_handle->repeated_start == I2C_REPEATED_START_DISABLED)
      i2c_generate_stop_condition(p_i2c_x);

    *p_i2c_handle->rx_buffer++ = p_i2c_x->I2C_DR;
    *p_i2c_handle->rx_buffer++ = p_i2c_x->I2C_DR;
    p_i2c_handle->rx_bytes_remaining -= 2;

    i2c_close_rx(p_i2c_handle, p_i2c_x);
    i2c_application_event_callback(p_i2c_x, I2C_EVENT_RX_COMPLETE);
  }
}

static void i2c_master_handle_3byte_rx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x)
{
  if (i2c_btf_interrupt_event(p_i2c_x))
  {
    i2c_ack_config(p_i2c_x, DISABLE); // Prepare for final two bytes
    *p_i2c_handle->rx_buffer++ = p_i2c_x->I2C_DR;
    p_i2c_handle->rx_bytes_remaining--;
  }
}

static void i2c_master_handle_gt3byte_rx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x)
{
  if (i2c_rxne_interrupt_event(p_i2c_x))
  {
    *p_i2c_handle->rx_buffer++ = p_i2c_x->I2C_DR;
    p_i2c_handle->rx_bytes_remaining--;
  }
}

static void i2c_master_handle_rxne(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x)
{
  if (p_i2c_handle->rx_total_size == 1 && p_i2c_handle->rx_bytes_remaining == 1)
  {
    i2c_master_handle_1byte_rx(p_i2c_handle, p_i2c_x);
    return;
  }

  if (p_i2c_handle->rx_total_size > 1)
  {
    if (p_i2c_handle->rx_bytes_remaining == 2)
    {
      i2c_master_handle_2byte_rx(p_i2c_handle, p_i2c_x);
      return;
    }
    else if (p_i2c_handle->rx_bytes_remaining == 3)
    {
      i2c_master_handle_3byte_rx(p_i2c_handle, p_i2c_x);
      return;
    }
    else if (p_i2c_handle->rx_bytes_remaining > 3)
    {
      i2c_master_handle_gt3byte_rx(p_i2c_handle, p_i2c_x);
      return;
    }
  }
}

void i2c_peripheral_enable(i2c_register_def_t *p_i2c_x)
{
  p_i2c_x->I2C_CR1 |= (1 << I2C_CR1_PE);
}

void i2c_peripheral_disable(i2c_register_def_t *p_i2c_x)
{
  p_i2c_x->I2C_CR1 &= ~(1 << I2C_CR1_PE);
}

void i2c_sm_mode_config(i2c_register_def_t *p_i2c_x)
{
  p_i2c_x->I2C_CR2 &= ~(1 << I2C_CCR_FS);
}

void i2c_fm_mode_config(i2c_register_def_t *p_i2c_x)
{
  p_i2c_x->I2C_CR2 |= (1 << I2C_CCR_FS);
}

void i2c_ack_config(i2c_register_def_t *p_i2c_x, signal_state_t enable_disable)
{
  if (enable_disable == ENABLE)
  {
    p_i2c_x->I2C_CR1 |= (1 << I2C_CR1_ACK);
  }
  else if (enable_disable == DISABLE)
  {
    p_i2c_x->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
  }
}

void i2c_cr2_freq_config(i2c_register_def_t *p_i2c_x)
{
  p_i2c_x->I2C_CR2 |= (i2c_pclk1_freq_get() / (1000000UL) & (0x3F));
}

void i2c_fm_duty_config(i2c_register_def_t *p_i2c_x, signal_state_t enable_disable)
{
  if (enable_disable == ENABLE)
  {
    p_i2c_x->I2C_CCR |= (1 << I2C_CCR_DUTY);
  }
  else
  {
    p_i2c_x->I2C_CCR &= ~(1 << I2C_CCR_DUTY);
  }
}

void i2c_scl_speed_config(i2c_register_def_t *p_i2c_x, i2c_speed_t fm_or_sm_mode, i2c_scl_freq_t scl_freq)
{

  if ((fm_or_sm_mode == I2C_SPEED_STANDARD) && (scl_freq == I2C_SCL_FREQ_100KHZ))
  {
    p_i2c_x->I2C_TRISE |= ((I2C_SM_RISE_TIME / 1000000000UL) * (i2c_pclk1_freq_get()) + 1);

    p_i2c_x->I2C_CCR |= ((i2c_pclk1_freq_get()) / (2 * (I2C_SCL_FREQ_100KHZ))) & (0xFFF);
  }

  else if ((fm_or_sm_mode == I2C_SPEED_FAST) && (scl_freq == I2C_SCL_FREQ_400KHZ))
  {
    p_i2c_x->I2C_TRISE |= ((I2C_FM_RISE_TIME / 1000000000UL) * (i2c_pclk1_freq_get()) + 1);

    if ((p_i2c_x->I2C_CCR >> 14) & (0x01))
    {
      p_i2c_x->I2C_CCR |= ((i2c_pclk1_freq_get()) / (3 * (I2C_SCL_FREQ_400KHZ))) & (0xFFF); // check logic
    }
    else
    {
      p_i2c_x->I2C_CCR |= ((i2c_pclk1_freq_get()) / (25 * (I2C_SCL_FREQ_400KHZ))) & (0xFFF);
    }
  }
}

void i2c_7bit_own_addr_config(i2c_register_def_t *p_i2c_x, uint8_t _7bit_own_addr)
{
  p_i2c_x->I2C_OAR1 &= ~(0x7F << 1);
  p_i2c_x->I2C_OAR1 &= ~(1 << I2C_OAR1_ADD_MODE);
  p_i2c_x->I2C_OAR1 |= (1 << I2C_OAR1_BIT_14);
  p_i2c_x->I2C_OAR1 |= (_7bit_own_addr << 1);
}

void i2c_data_tx(i2c_register_def_t *p_i2c_x, uint8_t *p_tx_buffer, size_t size_of_data, uint8_t slave_addr)
{
  uint32_t dummy_read;
  // 1. generate start condition
  i2c_generate_start_condition(p_i2c_x);

  // 2. confirm that start condition is generated by checking SB flag in SR1 register

  while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_SB))
    ;

  // 3. send address of slave (7bits) with r/nw bit set to 0

  i2c_slave_addr_write(p_i2c_x, slave_addr);

  // 4. confirm address phase is completed by checking the ADDR flag. the ADDR flag is set when slave sends ACK

  while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_ADDR))
    ;

  // 5. Clear ADDR flag
  dummy_read = p_i2c_x->I2C_SR1;
  dummy_read = p_i2c_x->I2C_SR2;
  (void)dummy_read;

  // 6. send data until len =0

  while (size_of_data > 0)
  {
    while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_TXE))
      ;
    p_i2c_x->I2C_DR = *p_tx_buffer;
    p_tx_buffer++;
    size_of_data--;
  }
  // 7. wait for TXE flag and BTF =1

  while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_TXE))
    ;
  while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_BTF))
    ;

  // 8. generate stop condition

  i2c_generate_stop_condition(p_i2c_x);
}

void i2c_data_rx(i2c_register_def_t *p_i2c_x, uint8_t *p_rx_buffer, size_t size_of_data, uint8_t slave_addr)
{
  uint32_t dummy_read;
  i2c_generate_start_condition(p_i2c_x);
  while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_SB))
    ;
  i2c_slave_addr_read(p_i2c_x, slave_addr);
  while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_ADDR))
    ;

  if (size_of_data == 1)
  {
    i2c_ack_config(p_i2c_x, DISABLE);
    i2c_generate_stop_condition(p_i2c_x);
    // clear addr flag
    dummy_read = p_i2c_x->I2C_SR1;
    dummy_read = p_i2c_x->I2C_SR2;
    (void)dummy_read;
    while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_RXNE))
      ;
    *p_rx_buffer = (uint8_t)p_i2c_x->I2C_DR;
  }

  if (size_of_data > 1)
  {
    // clear addr flag
    dummy_read = p_i2c_x->I2C_SR1;
    dummy_read = p_i2c_x->I2C_SR2;
    (void)dummy_read;
    for (uint32_t i = size_of_data; i > 0; i--)
    {
      while (!((p_i2c_x->I2C_SR1) & I2C_FLAG_RXNE))
        ;
      if (i == 2)
      {
        i2c_ack_config(p_i2c_x, DISABLE);
        i2c_generate_stop_condition(p_i2c_x);
      }

      *p_rx_buffer = (uint8_t)p_i2c_x->I2C_DR;
      p_rx_buffer++;
    }
  }

  i2c_ack_config(p_i2c_x, ENABLE);
}

/*************************************************I2C INTERRUPT*********************************/
uint8_t i2c_interrupt_data_tx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x, uint8_t *p_tx_buffer, size_t size_of_data, uint8_t slave_addr, i2c_repeated_start_t repeated_start)
{
  uint8_t i2c_bus_state = p_i2c_handle->tx_rx_state;
  if ((i2c_bus_state != I2C_BUSY_RX) && (i2c_bus_state != I2C_BUSY_TX))
  {
    p_i2c_handle->tx_buffer = p_tx_buffer;
    p_i2c_handle->tx_length = size_of_data;
    p_i2c_handle->tx_rx_state = I2C_BUSY_TX;
    p_i2c_handle->slave_addr = slave_addr;
    p_i2c_handle->repeated_start = repeated_start;

    // generate start condition

    i2c_generate_start_condition(p_i2c_x);

    // set ITBUFEN control bit
    p_i2c_x->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

    // enable ITEVFEN
    p_i2c_x->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

    // enable ITERREN
    p_i2c_x->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
  }
  return i2c_bus_state;
}

uint8_t i2c_interrupt_data_rx(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x, uint8_t *p_rx_buffer, size_t size_of_data, uint8_t slave_addr, i2c_repeated_start_t repeated_start)
{
  uint8_t i2c_bus_state = p_i2c_handle->tx_rx_state;

  if ((i2c_bus_state != I2C_BUSY_RX) && (i2c_bus_state != I2C_BUSY_TX))
  {
    p_i2c_handle->rx_buffer = p_rx_buffer;
    p_i2c_handle->rx_bytes_remaining = size_of_data;
    p_i2c_handle->tx_rx_state = I2C_BUSY_RX;
    p_i2c_handle->slave_addr = slave_addr;
    p_i2c_handle->repeated_start = repeated_start;

    // generate start condition

    i2c_generate_start_condition(p_i2c_x);

    // set ITBUFEN control bit
    p_i2c_x->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

    // enable ITEVFEN
    p_i2c_x->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

    // enable ITERREN
    p_i2c_x->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
  }
  return i2c_bus_state;
}

void i2c_event_irq_handler(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x)
{
  // Handle SB flag interrupt

  if (i2c_sb_interrupt_event(p_i2c_x))
  {
    if (p_i2c_handle->tx_rx_state == I2C_BUSY_TX)
    {
      i2c_slave_addr_write(p_i2c_x, p_i2c_handle->slave_addr);
    }
    else if (p_i2c_handle->tx_rx_state == I2C_BUSY_RX)
    {
      i2c_slave_addr_read(p_i2c_x, p_i2c_handle->slave_addr);
    }
  }

  // handle ADDR
  if (i2c_addr_interrupt_event(p_i2c_x))
  {
    i2c_clear_addr_flag(p_i2c_handle, p_i2c_x);
  }

  // handle BTF (byte transfer finished)

  if (i2c_btf_interrupt_event(p_i2c_x))
  {
    if (p_i2c_handle->tx_rx_state == I2C_BUSY_TX)
    {
      // lets make sure the txe is also set
      if (p_i2c_x->I2C_SR1 & I2C_FLAG_TXE)
      {
        // generate stop condition if repeat start is disabled
        if (p_i2c_handle->repeated_start == I2C_REPEATED_START_DISABLED)
        {
          i2c_generate_stop_condition(p_i2c_x);
        }

        // reset all members of handle

        i2c_close_tx(p_i2c_handle, p_i2c_x);

        // notify application about transmission complete using EventCallback
        i2c_application_event_callback(p_i2c_x, I2C_EVENT_TX_COMPLETE);
      }
    }
  }

  // handle stopf
  if (i2c_stopf_interrupt_event(p_i2c_x))
  {
    // clear stopf. to clear stopf read SR1 register and write to CR1 register
    // SR1 register is read by stopf_interrupt_event, we just need to write to CR1
    p_i2c_x->I2C_CR1 |= 0x0000;
    // notify application
    i2c_application_event_callback(p_i2c_x, I2C_EVENT_STOP);
  }

  // handle txe
  if (i2c_txe_interrupt_event(p_i2c_x))
  {
    if (i2c_is_master(p_i2c_x))
    {
      if (p_i2c_handle->tx_rx_state == I2C_BUSY_TX)
      {
        if (p_i2c_handle->tx_length > 0)
        {
          p_i2c_x->I2C_DR = *(p_i2c_handle->tx_buffer);
          p_i2c_handle->tx_length--;
          p_i2c_handle->tx_buffer++;
        }
      }
    }
  }

  // handle rxne
  if (i2c_rxne_interrupt_event(p_i2c_x))
  {
    i2c_master_handle_rxne(p_i2c_handle, p_i2c_x);
  }
}

void i2c_error_irq_handler(i2c_handle_t *p_i2c_handle, i2c_register_def_t *p_i2c_x)
{
  // handle BERR bus error
  if (i2c_berr_interrupt_error(p_i2c_x))
  {
    // clear the error interrupt
    p_i2c_x->I2C_SR1 &= ~(I2C_FLAG_BERR);

    // notify the application
    i2c_application_event_callback(p_i2c_x, I2C_ERROR_BERR);
  }

  // handle ARLO. arbitration loss
  if (i2c_arlo_interrupt_error(p_i2c_x))
  {
    // clear the error interrupt
    p_i2c_x->I2C_SR1 &= ~(I2C_FLAG_ARLO);

    // notify the application
    i2c_application_event_callback(p_i2c_x, I2C_ERROR_ARLO);
  }
  // handle AF. acknowledge error
  if (i2c_af_interrupt_error(p_i2c_x))
  {
    // clear the error interrupt
    p_i2c_x->I2C_SR1 &= ~(I2C_FLAG_AF);

    // notify the application
    i2c_application_event_callback(p_i2c_x, I2C_ERROR_AF);
  }
  // handle OVR. overrun

  if (i2c_ovr_interrupt_error(p_i2c_x))
  {
    // clear the error interrupt
    p_i2c_x->I2C_SR1 &= ~(I2C_FLAG_OVR);

    // notify the application
    i2c_application_event_callback(p_i2c_x, I2C_ERROR_OVR);
  }
  if (i2c_timeout_interrupt_error(p_i2c_x))
  {
    // clear the error interrupt
    p_i2c_x->I2C_SR1 &= ~(I2C_FLAG_TIMEOUT);

    // notify the application
    i2c_application_event_callback(p_i2c_x, I2C_ERROR_TIMEOUT);
  }
}

void I2C1_EV_IRQHandler(void)
{
  i2c_event_irq_handler(&i2c1_handle, I2C1);
}

void I2C2_EV_IRQHandler(void)
{
  i2c_event_irq_handler(&i2c2_handle, I2C2);
}

void I2C3_EV_IRQHandler(void)
{
  i2c_event_irq_handler(&i2c3_handle, I2C3);
}

void I2C1_ER_IRQHandler(void)
{
  i2c_error_irq_handler(&i2c1_handle, I2C1);
}

void I2C2_ER_IRQHandler(void)
{
  i2c_error_irq_handler(&i2c2_handle, I2C2);
}

void I2C3_ER_IRQHandler(void)
{
  i2c_error_irq_handler(&i2c3_handle, I2C3);
}