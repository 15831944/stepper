#include "stm32f4xx_conf.h"

#include "sys.h";
#include "leds.h";
#include "stepper.h";
#include "inputs.h";


void EXTI0_IRQHandler(void)
{
    TIM4->CCR1 = 0x001;
    TIM4->CCR2 = TIM4->CCR3 = TIM4->CCR4 = TIM4->CCR1;

    TIM2->ARR -= (TIM2->ARR / 20);
    //freq_request ++;// (freq_request /20);

    //GPIOD->ODR ^= (1 << 13);

    //EXTI->PR |= EXTI_PR_PR0; // reset the interrupt
    EXTI_ClearITPendingBit(EXTI_Line0);
}


int main(void)
{
    sys_init();
    leds_init();
    stepper_init();
    inputs_init();


    while (1)
    {
        // should go idle...
    }

}
