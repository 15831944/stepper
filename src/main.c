#include "stm32f4xx_conf.h"

#include "sys.h"
#include "leds.h"
#include "stepper.h"
#include "inputs.h"
#include "usb.h"
#include "gcode.h"

#include <stdio.h>

volatile uint32_t time_var1;

void Delay(volatile uint32_t nCount);

int main(void)
{
    sys_init();
    leds_init();
    stepper_init();
    inputs_init();
    usb_init();
    gcode_init();

    while (1)
    {
        // should go idle...
        Delay(10);

        char buffer[255];
        gets(buffer);

        if (buffer[0] != '\0')
        {
            gcode_parse(buffer);
        }
    }
}

/*
 * Called from systick handler
 */
void timing_handler() {
        if (time_var1) {
                time_var1--;
        }
}

/*
 * Delay a number of systick cycles (1ms)
 */
void Delay(volatile uint32_t nCount) {
        time_var1 = nCount;
        while(time_var1){};
}

