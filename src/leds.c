#include "leds.h"

#include "stm32f4xx_conf.h"

#include "stepper.h"

void TIM4_IRQHandler(void)
{

    static int dir_up = 1;
  // flash on update event
  if (TIM4->SR & TIM_SR_UIF)
  {
      //GPIOD->ODR ^= (1 << 13);
      if (dir_up == 1)
      {
          TIM4->CCR1 += 0x01;
      }
      else
      {
          TIM4->CCR1 -= 0x01;
      }

      if (TIM4->CCR1 >= 0x0FF)
      {
          dir_up = 0;
      }
      if (TIM4->CCR1 == 0x000)
      {
          dir_up = 1;
      }

      //TIM4->CCR2 = TIM4->CCR1;
      //TIM4->CCR3 = 0x100 - TIM4->CCR1;
      //TIM4->CCR4 = 0x100 - TIM4->CCR1;
      //TIM4->CCR4 = TIM4->CCR1;

      if (StepperIsForward() == true)
      {
          TIM4->CCR2 = 0x100;
      }
      else
      {
          TIM4->CCR2 = 0x0;
      }

      if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) == Bit_SET)
      {
          TIM4->CCR3 = 0x100;//0x100 - TIM4->CCR1;
      }
      else
      {
          TIM4->CCR3 = 0x0;//TIM4->CCR3 = TIM4->CCR1;
      }

      if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == Bit_SET)
      {
          TIM4->CCR4 = 0x100;//0x100 - TIM4->CCR1;
      }
      else
      {
          TIM4->CCR4 = 0x0;TIM4->CCR1;
      }
  }

  TIM4->SR = 0x0; // reset the status register
}

void leds_init()
{
    // Leds
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // enable the clock to GPIOD
    GPIOD->MODER = (2 << (2*12) ) + (2 << (2*13) ) + (2 << (2*14) ) + (2 << (2*15) ); // set pin 13 to be alternate function mode
    GPIOD->AFR[1] = (2 << 16) + (2 << 20) + (2 << 24) + (2 << 28); // pin 13 in alternate function AF2

    GPIOD->ODR ^= (1 << 13);


    // Timer 4
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // enable TIM4 clock

    NVIC->ISER[0] |= 1 << (TIM4_IRQn); // enable the TIM4 IRQ

    TIM4->PSC = 0x0080; // no prescaler, timer counts up in sync with the peripheral clock
    TIM4->DIER |= TIM_DIER_UIE; // enable update interrupt
    TIM4->ARR = 0x800; // count to 1 (autoreload value 1)
    TIM4->CCR1 = 0x000;// PWM value
    TIM4->CCR2 = 0x200;// PWM value
    TIM4->CCR3 = 0x400;// PWM value
    TIM4->CCR4 = 0x600;// PWM value

    TIM4->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; // autoreload on, counter enabled

    TIM4->EGR = 1; // trigger update event to reload timer registers

    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; // enable OC1 as Output
    TIM4->CCMR1 |=   TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE
                   | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
    TIM4->CCMR2 |=   TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE
                   | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;

}
