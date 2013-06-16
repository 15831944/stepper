#include "stepper.h"

#include "stm32f4xx_conf.h"


uint32_t freq_request = 1;
uint32_t freq_current = 1;

void TIM2_IRQHandler(void)
{
    static int direction_up = 1;

    if (TIM2->SR & TIM_SR_UIF)
    {
        GPIOE->ODR ^= (1 << 11);

        if (direction_up == 1)
        {
            freq_current++;
        }
        else
        {
            freq_current--;
        }

        if (freq_current >= freq_request)
        {
            direction_up = 0;
        }
        if (freq_current == 0)
        {
            direction_up = 1;
        }

        //TIM2->ARR = freq_current;
        TIM2->ARR = (uint32_t) ( (float)0x1000 / (float) freq_current) + 3;
    }
    TIM2->SR = 0x0; // reset the status register
}

void stepper_init()
{
    // Timer 2 : stepper
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable TIM2 clock

    NVIC->ISER[0] |= 1 << (TIM2_IRQn); // enable the TIM2 IRQ

    TIM2->PSC = 0x0008; // no prescaler, timer counts up in sync with the peripheral clock
    TIM2->DIER |= TIM_DIER_UIE; // enable update interrupt
    TIM2->ARR = 0x1000; // count to 1 (autoreload value 1)
    freq_current = 1;
    freq_request = 1;

    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; // autoreload on, counter enabled
    TIM2->EGR = 1; // trigger update event to reload timer registers

    // Output
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef   GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; // enable the clock to GPIOE
//    GPIOE->MODER |= (1 << (2*11));
}

