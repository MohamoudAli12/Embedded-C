#include "gpio.h"
#include "stm32f446xx.h"
#include <stdint.h>

/**** active low

_Bool debounce_active_low(uint8_t read_input_from_gpiox) {
    static uint16_t state = 0;
    state = ((state << 1) |(read_input_from_gpiox)| (0xfe00));
    return (state == 0xff00 );
}


// active high
_Bool debounce_active_high(uint8_t read_input_from_gpiox) {
    static uint16_t state = 0;
    state = ((state << 1) | (read_input_from_gpiox) | 0xfe00);
    return (state == 0xfffe);
}
****/