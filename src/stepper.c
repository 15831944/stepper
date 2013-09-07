#include "stepper.h"

#include "stm32f4xx_conf.h"

#include <stdio.h>

uint32_t freq_request = 1;
uint32_t freq_current = 1;

// Stepper A
int32_t STEPA_position_current = 0;
int32_t STEPA_position_request = 0;

// Stepper B
int32_t STEPB_position_current = 0;
int32_t STEPB_position_request = 0;


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


STEPPER_DIR STEPA_direction = STEPPER_Forward;
STEPPER_DIR STEPB_direction = STEPPER_Forward;


MOVE_MODE mode = MOVE_Relative;


void STEPA_move_finished(void);
void STEPB_move_finished(void);

/// \todo remove, debug only
extern bool g_Led2State;
extern bool g_Led3State;
extern bool g_Led4State;

void TIM2_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {

        if (STEPA_position_current < STEPA_position_request)
        {
            // Go Forward
            STEPA_direction = STEPPER_Forward;
        }
        else if (STEPA_position_current > STEPA_position_request)
        {
            // Go Backward
            STEPA_direction = STEPPER_Backward;
        }
        else
        {
            // Stop
            if (STEPA_direction != STEPPER_Stopped)
            {
                STEPA_move_finished();
            }
            STEPA_direction = STEPPER_Stopped;
        }


        if (STEPA_direction == STEPPER_Forward)
        {
            GPIO_WriteBit(GPIOE, GPIO_Pin_8, Bit_SET);
            GPIO_WriteBit(GPIOE, GPIO_Pin_12, Bit_SET); /// \todo remove : debug only
            STEPA_position_current++;
            g_Led2State = 1;
        }
        else if (STEPA_direction == STEPPER_Backward)
        {
            GPIO_WriteBit(GPIOE, GPIO_Pin_8, Bit_RESET);
            GPIO_WriteBit(GPIOE, GPIO_Pin_12, Bit_RESET); /// \todo remove : debug only
            STEPA_position_current--;
            g_Led2State = 0;
        }

        if (STEPA_direction != STEPPER_Stopped)
        {
            GPIO_ToggleBits(GPIOE, GPIO_Pin_7);
            GPIO_ToggleBits(GPIOE, GPIO_Pin_11); /// \todo remove : debug only
        }

        TIM_ARRPreloadConfig(TIM2, (uint32_t) ( (float)0x1000 / (float) freq_current) + 3);
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void TIM5_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
    {

        if (STEPB_position_current < STEPB_position_request)
        {
            // Go Forward
            STEPB_direction = STEPPER_Forward;
        }
        else if (STEPB_position_current > STEPB_position_request)
        {
            // Go Backward
            STEPB_direction = STEPPER_Backward;
        }
        else
        {
            // Stop
            if (STEPB_direction != STEPPER_Stopped)
            {
                STEPB_move_finished();
            }
            STEPB_direction = STEPPER_Stopped;
        }


        if (STEPB_direction == STEPPER_Forward)
        {
            GPIO_WriteBit(GPIOE, GPIO_Pin_10, Bit_SET);
            GPIO_WriteBit(GPIOE, GPIO_Pin_14, Bit_SET); /// \todo remove : debug only
            STEPB_position_current++;
            g_Led4State = 1;
        }
        else if (STEPB_direction == STEPPER_Backward)
        {
            GPIO_WriteBit(GPIOE, GPIO_Pin_10, Bit_RESET);
            GPIO_WriteBit(GPIOE, GPIO_Pin_14, Bit_RESET); /// \todo remove : debug only
            STEPB_position_current--;
            g_Led4State = 0;
        }

        if (STEPB_direction != STEPPER_Stopped)
        {
            GPIO_ToggleBits(GPIOE, GPIO_Pin_9);
            GPIO_ToggleBits(GPIOE, GPIO_Pin_13); /// \todo remove : debug only
        }
       TIM_ARRPreloadConfig(TIM5, (uint32_t) ( (float)0x1000 / (float) freq_current) + 3);
    }
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}

/*
/// \todo to remove, debug only, extracted from temp
void TIM3_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        g_Led3State = !g_Led3State;
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}
*/

void STEPA_move_finished()
{
    if (STEPB_direction == STEPPER_Stopped)
    {
       puts("ok\n"); // Only once when it happens
    }
}

void STEPB_move_finished()
{
    if (STEPA_direction == STEPPER_Stopped)
    {
       puts("ok\n"); // Only once when it happens
    }
}


void stepper_init()
{
    freq_current = 1;
    freq_request = 1;

    // Timer 2 : stepper A
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN, ENABLE);

    TIM_TimeBaseInitTypeDef timeBaseInitStruct2;
    timeBaseInitStruct2.TIM_CounterMode = TIM_CounterMode_Up;
    timeBaseInitStruct2.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBaseInitStruct2.TIM_Prescaler = 8;
    timeBaseInitStruct2.TIM_Period = 0x1000;
    timeBaseInitStruct2.TIM_RepetitionCounter = 0; // Only TIM1 & TIM8

    TIM_TimeBaseInit(TIM2, &timeBaseInitStruct2);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM2_IRQn);

    TIM_Cmd(TIM2, ENABLE);


    // Timer 5 : stepper B
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM5EN, ENABLE);

    TIM_TimeBaseInitTypeDef timeBaseInitStruct5;
    TIM_TimeBaseStructInit(&timeBaseInitStruct5);
    timeBaseInitStruct5.TIM_CounterMode = TIM_CounterMode_Up;
    timeBaseInitStruct5.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBaseInitStruct5.TIM_Prescaler = 0x8;
    timeBaseInitStruct5.TIM_Period = 0x1000;
    timeBaseInitStruct5.TIM_RepetitionCounter = 0; // Only TIM1 & TIM8

    TIM_TimeBaseInit(TIM5, &timeBaseInitStruct5);

    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM5_IRQn);

    TIM_Cmd(TIM5, ENABLE);
/*
    /// \todo remove, debug only
    // Timer 3 : temp measurements
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);

    TIM_TimeBaseInitTypeDef timeBaseInitStruct;
    timeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    timeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBaseInitStruct.TIM_Prescaler = 0xFFFF;
    timeBaseInitStruct.TIM_Period = 2563;
    timeBaseInitStruct.TIM_RepetitionCounter = 0; // Only TIM1 & TIM8

    TIM_TimeBaseInit(TIM3, &timeBaseInitStruct);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM_Cmd(TIM3, ENABLE);
*/

    // Output
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef   GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8; // MotA (step, dir)
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10; // MotA (step, dir) + MotB (step, dir)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14; // // MotA (step, dir) + MotB (step, dir) + MotC (step, dir) + MotE (step, dir)

    GPIO_Init(GPIOE, &GPIO_InitStructure);

}

void stepper_move(float deltaA, float deltaB)
{
    if (mode == MOVE_Absolute)
    {
        STEPA_position_request = 2 * deltaA * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
    }
    else
    {
        STEPA_position_request += 2 * deltaA * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
    }

    STEPA_direction = STEPPER_Waiting;

    if (mode == MOVE_Absolute)
    {
        STEPB_position_request = 2 * deltaB * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
    }
    else
    {
        STEPB_position_request += 2 * deltaB * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
    }

    STEPB_direction = STEPPER_Waiting;
}

void stepper_stop()
{
    STEPA_direction = STEPPER_Stopped;
    STEPA_position_request = STEPA_position_current;

    STEPB_direction = STEPPER_Stopped;
    STEPB_position_request = STEPB_position_current;
}

/*
void stepper_set_forward()
{
    STEPA_direction = STEPPER_Forward;
    printf("FW\n");
}

void stepper_set_backward()
{
    STEPA_direction = STEPPER_Backward;
    printf("BW\n");
}

bool StepperIsForward()
{
    return (STEPA_direction == STEPPER_Forward);
}

uint32_t getStepperPos()
{
    return STEPA_position_current;
}
*/
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
    STEPA_direction = STEPPER_Stopped;
    STEPA_position_current = 0;
    STEPA_position_request = 0;

    STEPB_direction = STEPPER_Stopped;
    STEPB_position_current = 0;
    STEPB_position_request = 0;
}
