#include "sys.h"

#include "stm32f4xx_conf.h"

void sys_init()
{
    // Enable FPU
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
}

