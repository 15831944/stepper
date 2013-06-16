#include "inputs.h"

#include "stm32f4xx_conf.h"

#include "stepper.h"


void EXTI9_5_IRQHandler(void)
{
//    TIM4->CCR1 = 0x001;
//    TIM4->CCR2 = TIM4->CCR3 = TIM4->CCR4 = TIM4->CCR1;

//    TIM2->ARR -= (TIM2->ARR / 20);

//    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7) == Bit_SET)
//    {
//        stepper_set_forward();
//    }

//    else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == Bit_SET)
//    {
//        stepper_set_backward();
//    }

    //EXTI->PR |= EXTI_PR_PR0; // reset the interrupt
    if (EXTI_GetFlagStatus(EXTI_Line7) == SET)
    {
        EXTI_ClearITPendingBit(EXTI_Line7);
        stepper_set_forward();
    }

    if (EXTI_GetFlagStatus(EXTI_Line9) == SET)
    {
        EXTI_ClearITPendingBit(EXTI_Line9);
        stepper_set_backward();
    }

}


void inputs_init()
{
    // External interrupt on PE9
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource9);

    GPIO_InitTypeDef   GPIO_InitStructureE9;
    EXTI_InitTypeDef   EXTI_InitStructureE9;
    NVIC_InitTypeDef   NVIC_InitStructureE9;

    GPIO_InitStructureE9.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructureE9.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructureE9.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOE, &GPIO_InitStructureE9);

    EXTI_InitStructureE9.EXTI_Line = EXTI_Line9;
    EXTI_InitStructureE9.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructureE9.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructureE9.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructureE9);

    NVIC_InitStructureE9.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructureE9.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructureE9.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructureE9.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructureE9);

    // External interrupt on PE7
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource7);

    GPIO_InitTypeDef   GPIO_InitStructureE7;
    EXTI_InitTypeDef   EXTI_InitStructureE7;
    NVIC_InitTypeDef   NVIC_InitStructureE7;

    GPIO_InitStructureE7.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructureE7.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructureE7.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOE, &GPIO_InitStructureE7);

    EXTI_InitStructureE7.EXTI_Line = EXTI_Line7;
    EXTI_InitStructureE7.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructureE7.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructureE7.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructureE7);

    NVIC_InitStructureE7.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructureE7.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructureE7.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructureE7.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructureE7);
}
