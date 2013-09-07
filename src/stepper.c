#include "stepper.h"

#include "stm32f4xx_conf.h"

#include <stdio.h>

// consts
/// \todo move these constants in the stepper state structure to allow per stepper configuration
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


typedef enum
{
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_E,
    AXIS_NUM // For loops, contains the number of axis on this machine
} STEPPER_AXIS;

typedef struct
{
    STEPPER_DIR direction;
    uint32_t freq_request;
    uint32_t freq_current;
    int32_t position_request;
    int32_t position_current;
} STEPPER_STATE;

STEPPER_STATE stepper[4] = {
    {
        .direction = STEPPER_Stopped,
        .freq_request = 1,
        .freq_current = 1,
        .position_request = 0,
        .position_current = 0
    },

    {
        .direction = STEPPER_Stopped,
        .freq_request = 1,
        .freq_current = 1,
        .position_request = 0,
        .position_current = 0
    },

    {
        .direction = STEPPER_Stopped,
        .freq_request = 1,
        .freq_current = 1,
        .position_request = 0,
        .position_current = 0
    },

    {
        .direction = STEPPER_Stopped,
        .freq_request = 1,
        .freq_current = 1,
        .position_request = 0,
        .position_current = 0
    }
};

/// \todo move "move mode" in the stepper state structure to allow per stepper configuration
MOVE_MODE mode = MOVE_Relative;


void move_finished(STEPPER_AXIS axis);

/// \todo remove, debug only
extern bool g_Led2State;
extern bool g_Led3State;
extern bool g_Led4State;

void TIM2_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {

        if (stepper[AXIS_X].position_current < stepper[AXIS_X].position_request)
        {
            // Go Forward
            stepper[AXIS_X].direction = STEPPER_Forward;
        }
        else if (stepper[AXIS_X].position_current > stepper[AXIS_X].position_request)
        {
            // Go Backward
            stepper[AXIS_X].direction = STEPPER_Backward;
        }
        else
        {
            // Stop
            if (stepper[AXIS_X].direction != STEPPER_Stopped)
            {
                move_finished(AXIS_X);
            }
            stepper[AXIS_X].direction = STEPPER_Stopped; /// \todo should be part of the move_finished function.
        }


        if (stepper[AXIS_X].direction == STEPPER_Forward)
        {
            GPIO_WriteBit(GPIOE, GPIO_Pin_8, Bit_SET);
            GPIO_WriteBit(GPIOE, GPIO_Pin_12, Bit_SET); /// \todo remove : debug only
            stepper[AXIS_X].position_current++;
            g_Led2State = 1;
        }
        else if (stepper[AXIS_X].direction == STEPPER_Backward)
        {
            GPIO_WriteBit(GPIOE, GPIO_Pin_8, Bit_RESET);
            GPIO_WriteBit(GPIOE, GPIO_Pin_12, Bit_RESET); /// \todo remove : debug only
            stepper[AXIS_X].position_current--;
            g_Led2State = 0;
        }

        if (stepper[AXIS_X].direction != STEPPER_Stopped)
        {
            GPIO_ToggleBits(GPIOE, GPIO_Pin_7);
            GPIO_ToggleBits(GPIOE, GPIO_Pin_11); /// \todo remove : debug only
        }

        TIM_ARRPreloadConfig(TIM2, (uint32_t) ( (float)0x1000 / (float) stepper[AXIS_X].freq_current) + 3);
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void TIM5_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
    {

        if (stepper[AXIS_Y].position_current < stepper[AXIS_Y].position_request)
        {
            // Go Forward
            stepper[AXIS_Y].direction = STEPPER_Forward;
        }
        else if (stepper[AXIS_Y].position_current > stepper[AXIS_Y].position_request)
        {
            // Go Backward
            stepper[AXIS_Y].direction = STEPPER_Backward;
        }
        else
        {
            // Stop
            if (stepper[AXIS_Y].direction != STEPPER_Stopped)
            {
                move_finished(AXIS_Y);
            }
            stepper[AXIS_Y].direction = STEPPER_Stopped;
        }


        if (stepper[AXIS_Y].direction == STEPPER_Forward)
        {
            GPIO_WriteBit(GPIOE, GPIO_Pin_10, Bit_SET);
            GPIO_WriteBit(GPIOE, GPIO_Pin_14, Bit_SET); /// \todo remove : debug only
            stepper[AXIS_Y].position_current++;
            g_Led4State = 1;
        }
        else if (stepper[AXIS_Y].direction == STEPPER_Backward)
        {
            GPIO_WriteBit(GPIOE, GPIO_Pin_10, Bit_RESET);
            GPIO_WriteBit(GPIOE, GPIO_Pin_14, Bit_RESET); /// \todo remove : debug only
            stepper[AXIS_Y].position_current--;
            g_Led4State = 0;
        }

        if (stepper[AXIS_Y].direction != STEPPER_Stopped)
        {
            GPIO_ToggleBits(GPIOE, GPIO_Pin_9);
            GPIO_ToggleBits(GPIOE, GPIO_Pin_13); /// \todo remove : debug only
        }
       TIM_ARRPreloadConfig(TIM5, (uint32_t) ( (float)0x1000 / (float) stepper[AXIS_Y].freq_current) + 3);
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

void move_finished(STEPPER_AXIS axis)
{
    bool all_stopped = true;
    /// \todo add the check that all other movements are stopped
    for (int i=0; i<AXIS_NUM; i++)
    {
        if ( (i != axis ) // Skip the current axis, to only check if all other have finished
            && (stepper[i].direction != STEPPER_Stopped) ) // If one stepper has not finished, then notify it and exit
        {
            all_stopped = false;
            break;
        }
    }

    if (all_stopped)
    {
        puts("ok\n"); // Print only once when the last move finishes
    }
}


void stepper_init()
{
    for (int i=0; i<AXIS_NUM; i++)
    {
        stepper[i].freq_current = 1;
        stepper[i].freq_request = 1;
    }

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
        stepper[AXIS_X].position_request = 2 * deltaA * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
    }
    else
    {
        stepper[AXIS_X].position_request += 2 * deltaA * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
    }

    stepper[AXIS_X].direction = STEPPER_Waiting;

    if (mode == MOVE_Absolute)
    {
        stepper[AXIS_Y].position_request = 2 * deltaB * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
    }
    else
    {
        stepper[AXIS_Y].position_request += 2 * deltaB * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
    }

    stepper[AXIS_Y].direction = STEPPER_Waiting;
}

void stepper_stop()
{
    stepper[AXIS_X].direction = STEPPER_Stopped;
    stepper[AXIS_X].position_request = stepper[AXIS_X].position_current;

    stepper[AXIS_Y].direction = STEPPER_Stopped;
    stepper[AXIS_Y].position_request = stepper[AXIS_Y].position_current;
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
    stepper[AXIS_X].direction = STEPPER_Stopped;
    stepper[AXIS_X].position_current = 0;
    stepper[AXIS_X].position_request = 0;

    stepper[AXIS_Y].direction = STEPPER_Stopped;
    stepper[AXIS_Y].position_current = 0;
    stepper[AXIS_Y].position_request = 0;
}
