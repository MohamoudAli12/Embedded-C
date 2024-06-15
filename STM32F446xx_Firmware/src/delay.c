#include <stdint.h>
#include "fn_prototypes.h"


void delay (uint32_t delay_time)
{
    for (uint32_t i=0; i <delay_time; i++);
}