#include <stdint.h>
void toggle_led_on_button_press(void);
void delay(uint32_t delay_time);
void blink_led(void);
_Bool debounce_active_low(void);
_Bool debounce_active_high(void);
void EXTI15_10_IRQHandler(void);