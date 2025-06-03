#include "fn_prototypes.h"
#include "gpio.h"
#include "stm32f446xx.h"
#include <stdint.h>

int main()
{
  // button_interrupt_led_toggle();
  toggle_led_on_button_press();
  // blink_led();
}
