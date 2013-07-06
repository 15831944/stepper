#include "stepper.h"

#include "stm32f4xx_conf.h"

#include <stdio.h>

uint32_t freq_request = 1;
uint32_t freq_current = 1;

int32_t position_current = 0;
int32_t position_request = 0;




// consts
//const float MICROSTEPS = 16;
#define MICROSTEPS 16
const float mm2steps = MICROSTEPS * 200.0 / (8.0 * 5.0); // microstep ratio * NbSteps / (Ntooth * PitchBelt)

typedef enum
{
    MOVE_Relative = 0,
    MOVE_Absolute
} MOVE_MODE;


typedef enum
{
    STEPPER_Stopped = 0,
    STEPPER_Forward,
    STEPPER_Backward,
    STEPPER_Waiting
} STEPPER_DIR;


STEPPER_DIR direction = STEPPER_Forward;
MOVE_MODE mode = MOVE_Relative;

void TIM2_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {

        if (position_current < position_request)
        {
            // Go Forward
            direction = STEPPER_Forward;
        }
        else if (position_current > position_request)
        {
            // Go Backward
            direction = STEPPER_Backward;
        }
        else
        {
            // Stop
            if (direction != STEPPER_Stopped)
            {
                puts("ok\n"); // Only once when it happens
            }
            direction = STEPPER_Stopped;
        }


        if (direction == STEPPER_Forward)
        {
            GPIO_WriteBit(GPIOE, GPIO_Pin_13, Bit_SET);
            //GPIO_SetBits(GPIOE, GPIO_Pin_13);
            position_current++;
        }
        else if (direction == STEPPER_Backward)
        {
            GPIO_WriteBit(GPIOE, GPIO_Pin_13, Bit_RESET);
            //GPIO_ResetBits(GPIOE, GPIO_Pin_13);
            position_current--;
        }

        if (direction != STEPPER_Stopped)
        {
            GPIO_ToggleBits(GPIOE, GPIO_Pin_11);
        }

       TIM_ARRPreloadConfig(TIM2, (uint32_t) ( (float)0x1000 / (float) freq_current) + 3);
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}


void stepper_init()
{
    freq_current = 1;
    freq_request = 1;

    // Timer 2 : stepper
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN, ENABLE);

    TIM_TimeBaseInitTypeDef timeBaseInitStruct;
    timeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    timeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBaseInitStruct.TIM_Prescaler = 8;
    timeBaseInitStruct.TIM_Period = 0x1000;
    timeBaseInitStruct.TIM_RepetitionCounter = 0; // Only TIM1 & TIM8

    TIM_TimeBaseInit(TIM2, &timeBaseInitStruct);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM_Cmd(TIM2, ENABLE);

    // Output
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef   GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13;

    GPIO_Init(GPIOE, &GPIO_InitStructure);

}

void stepper_move(float delta)
{
    if (mode == MOVE_Absolute)
    {
        position_request = 2 * delta * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
    }
    else
    {
        position_request += 2 * delta * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
    }

    direction = STEPPER_Waiting;
}

void stepper_stop()
{
    direction = STEPPER_Stopped;
    position_request = position_current;
}


void stepper_set_forward()
{
    direction = STEPPER_Forward;
    printf("FW\n");
}

void stepper_set_backward()
{
    direction = STEPPER_Backward;
    printf("BW\n");
}

bool StepperIsForward()
{
    return (direction == STEPPER_Forward);
}

uint32_t getStepperPos()
{
    return position_current;
}

void stepper_set_absolute()
{
    mode = MOVE_Absolute;
}

void stepper_set_relative()
{
    mode = MOVE_Relative;
}

void stepper_reset()
{
    direction = STEPPER_Stopped;
    position_current = 0;
    position_request = 0;
}
