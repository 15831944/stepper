#include "stepper.h"

#include "stm32f4xx_conf.h"

#include <stdio.h>
#include <math.h>

// consts
/// \todo move these constants in the stepper state structure to allow per stepper configuration
//const float MICROSTEPS = 16;
#define MICROSTEPS      16
#define STEP_NUMBER     200.0
#define TOOTH_NUMBER    8.0
#define TOOTH_PITCH     5.0
const float mm2steps = MICROSTEPS * STEP_NUMBER / (TOOTH_NUMBER * TOOTH_PITCH); // microstep ratio * NbSteps / (Ntooth * PitchBelt)

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


typedef struct
{
    // I/O Config
    uint32_t step_AHB1Periph;
    GPIO_TypeDef* step_port;
    uint16_t step_pin;
    uint32_t dir_AHB1Periph;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;

    // Timer Config
    TIM_TypeDef* timer;
    uint32_t timerAPB1Periph;
    uint32_t timerAPB2Periph; /// \todo make some function to abstract more easily the timer (should only need to say which timer to use, not provide all fields directly related to it...)
    IRQn_Type timerIrq;

    // move state
    STEPPER_DIR direction;
    uint32_t period_request;
    uint32_t period_current;
    int32_t position_request;
    int32_t position_current;
} STEPPER_STATE;

STEPPER_STATE stepper[4] = {
    {
        .step_AHB1Periph = RCC_AHB1Periph_GPIOE,
        .step_port = GPIOE,
        .step_pin = GPIO_Pin_7,
        .dir_AHB1Periph = RCC_AHB1Periph_GPIOE,
        .dir_port = GPIOE,
        .dir_pin = GPIO_Pin_8,
        .timer = TIM10,
        .timerAPB1Periph = 0,
        .timerAPB2Periph = RCC_APB2ENR_TIM10EN,
        .timerIrq = TIM1_UP_TIM10_IRQn,
        .direction = STEPPER_Stopped,
        .period_request = 1,
        .period_current = 1,
        .position_request = 0,
        .position_current = 0
    },

    {
        .step_AHB1Periph = RCC_AHB1Periph_GPIOE,
        .step_port = GPIOE,
        .step_pin = GPIO_Pin_9,
        .dir_AHB1Periph = RCC_AHB1Periph_GPIOE,
        .dir_port = GPIOE,
        .dir_pin = GPIO_Pin_10,
        .timer = TIM11,
        .timerAPB1Periph = 0,
        .timerAPB2Periph = RCC_APB2ENR_TIM11EN,
        .timerIrq = TIM1_TRG_COM_TIM11_IRQn,
        .direction = STEPPER_Stopped,
        .period_request = 1,
        .period_current = 1,
        .position_request = 0,
        .position_current = 0
    },

    {
        .step_AHB1Periph = RCC_AHB1Periph_GPIOE,
        .step_port = GPIOE,
        .step_pin = GPIO_Pin_11,
        .dir_AHB1Periph = RCC_AHB1Periph_GPIOE,
        .dir_port = GPIOE,
        .dir_pin = GPIO_Pin_12,
        .timer = TIM13,
        .timerAPB1Periph = RCC_APB1ENR_TIM13EN,
        .timerAPB2Periph = 0,
        .timerIrq = TIM8_UP_TIM13_IRQn,
        .direction = STEPPER_Stopped,
        .period_request = 1,
        .period_current = 1,
        .position_request = 0,
        .position_current = 0
    },

    {
        .step_AHB1Periph = RCC_AHB1Periph_GPIOE,
        .step_port = GPIOE,
        .step_pin = GPIO_Pin_13,
        .dir_AHB1Periph = RCC_AHB1Periph_GPIOE,
        .dir_port = GPIOE,
        .dir_pin = GPIO_Pin_14,
        .timer = TIM14,
        .timerAPB1Periph = RCC_APB1ENR_TIM14EN,
        .timerAPB2Periph = 0,
        .timerIrq = TIM8_TRG_COM_TIM14_IRQn,
        .direction = STEPPER_Stopped,
        .period_request = 1,
        .period_current = 1,
        .position_request = 0,
        .position_current = 0
    }
};

/// \todo move "move mode" in the stepper state structure to allow per stepper configuration
MOVE_MODE mode = MOVE_Relative;


void move_finished(STEPPER_AXIS axis);
void Process_Stepper(STEPPER_AXIS axis); /// Function to call on interrupt to process the new step and the associated state machine
void init_stepper(STEPPER_AXIS axis);

/// \todo remove, debug only
extern bool g_LedState[4];

inline void Process_Stepper(STEPPER_AXIS axis)
{

    if (TIM_GetITStatus(stepper[axis].timer, TIM_IT_Update) == SET)
    {
        if (stepper[axis].position_current < stepper[axis].position_request)
        {
            // Go Forward
            stepper[axis].direction = STEPPER_Forward;
        }
        else if (stepper[axis].position_current > stepper[axis].position_request)
        {
            // Go Backward
            stepper[axis].direction = STEPPER_Backward;
        }
        else
        {
            // Stop
            if (stepper[axis].direction != STEPPER_Stopped)
            {
                move_finished(axis);
            }
            stepper[axis].direction = STEPPER_Stopped; /// \todo should be part of the move_finished function.
        }


        if (stepper[axis].direction == STEPPER_Forward)
        {
            GPIO_WriteBit(stepper[axis].dir_port, stepper[axis].dir_pin, Bit_SET);
            stepper[axis].position_current++;
            g_LedState[axis] = 1;
        }
        else if (stepper[axis].direction == STEPPER_Backward)
        {
            GPIO_WriteBit(stepper[axis].dir_port, stepper[axis].dir_pin, Bit_RESET);
            stepper[axis].position_current--;
            g_LedState[axis] = 0;
        }

        if (stepper[axis].direction != STEPPER_Stopped)
        {
            GPIO_ToggleBits(stepper[axis].step_port, stepper[axis].step_pin);
        }


        TIM_SetAutoreload(stepper[axis].timer, stepper[axis].period_current);
//        TIM_ARRPreloadConfig(stepper[axis].timer, (uint32_t) ( (float)0x1000 / (float) stepper[axis].period_current) + 3);

    }
    TIM_ClearITPendingBit(stepper[axis].timer, TIM_IT_Update);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    Process_Stepper(AXIS_X);
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    Process_Stepper(AXIS_Y);
}

void TIM8_UP_TIM13_IRQHandler(void)
{
    Process_Stepper(AXIS_Z);
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    Process_Stepper(AXIS_E);
}

void move_finished(STEPPER_AXIS axis)
{
    bool all_stopped = true;

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
        //puts("ok\n"); // Print only once when the last move finishes
        printf("ok x:%4d y:%4d z:%4d e:%4d period_x:%d period_y:%d pre_x=%d pre_y:%d\n", stepper[AXIS_X].position_current, stepper[AXIS_Y].position_current, stepper[AXIS_Z].position_current, stepper[AXIS_E].position_current, stepper[AXIS_X].period_current, stepper[AXIS_Y].period_current, TIM_GetPrescaler(stepper[AXIS_X].timer), TIM_GetPrescaler(stepper[AXIS_Y].timer));
    }
}

void init_stepper(STEPPER_AXIS axis)
{
    if (stepper[axis].timer != 0)
    {
        // Timer
        if (stepper[axis].timerAPB1Periph != 0)
        {
            RCC_APB1PeriphClockCmd(stepper[axis].timerAPB1Periph, ENABLE);
        }
        if (stepper[axis].timerAPB2Periph != 0)
        {
            RCC_APB2PeriphClockCmd(stepper[axis].timerAPB2Periph, ENABLE);
        }

        TIM_TimeBaseInitTypeDef timeBaseInitStruct;
        timeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
        timeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
        timeBaseInitStruct.TIM_Prescaler = 52500; // Default to 10mm/s
        timeBaseInitStruct.TIM_Period = 0x1000;
        timeBaseInitStruct.TIM_RepetitionCounter = 0; // Only TIM1 & TIM8

        TIM_TimeBaseInit(stepper[axis].timer, &timeBaseInitStruct);
        TIM_ARRPreloadConfig(stepper[axis].timer, ENABLE);

        TIM_ITConfig(stepper[axis].timer, TIM_IT_Update, ENABLE);
        NVIC_EnableIRQ(stepper[axis].timerIrq);

        TIM_Cmd(stepper[axis].timer, ENABLE);


        // Step Output
        RCC_AHB1PeriphClockCmd(stepper[axis].step_AHB1Periph, ENABLE);

        GPIO_InitTypeDef   GPIO_InitStructureStep;
        GPIO_InitStructureStep.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructureStep.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructureStep.GPIO_Pin = stepper[axis].step_pin;

        GPIO_Init(stepper[axis].step_port, &GPIO_InitStructureStep);


        // Dir Output

        RCC_AHB1PeriphClockCmd(stepper[axis].dir_AHB1Periph, ENABLE);

        GPIO_InitTypeDef   GPIO_InitStructureDir;
        GPIO_InitStructureDir.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructureDir.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructureDir.GPIO_Pin = stepper[axis].dir_pin;

        GPIO_Init(stepper[axis].dir_port, &GPIO_InitStructureDir);
    }
}

void stepper_init()
{
    for (int i=0; i<AXIS_NUM; i++)
    {
        stepper[i].period_current = 1;
        stepper[i].period_request = 1;

        init_stepper(i);
    }
}

void stepper_move(float delta[AXIS_NUM])
{

    // Compute norm of displacement on (X, Y, Z), to then compute speed along each axis
    float normDelta = sqrtf(powf(delta[AXIS_X], 2.0) + powf(delta[AXIS_Y], 2.0) + powf(delta[AXIS_Z], 2.0));

    for (int i=0; i<AXIS_NUM; i++)
    {
        if (delta[i] != 0)
        {
            if (mode == MOVE_Absolute)
            {
                stepper[i].position_request = 2 * delta[i] * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
            }
            else
            {
                stepper[i].position_request += 2 * delta[i] * mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
            }

            float period = 0.0;
            if (i != AXIS_E)
            {
                period = fabs(normDelta/delta[i]);
            }
            else
            {
                period = 1;
            }
            stepper[i].period_current = (uint32_t) period;
            TIM_SetAutoreload(stepper[i].timer, stepper[i].period_current);
//            TIM_ARRPreloadConfig(stepper[i].timer, (uint32_t) ( (float)0x1000 / (float) stepper[i].period_current) + 3);
            stepper[i].direction = STEPPER_Waiting;
        }
    }
}

void stepper_stop()
{
    for (int i=0; i<AXIS_NUM; i++)
    {
        stepper[i].direction = STEPPER_Stopped;
        stepper[i].position_request = stepper[i].position_current;
    }
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
    for (int i=0; i<AXIS_NUM; i++)
    {
        stepper[i].direction = STEPPER_Stopped;
        stepper[i].position_current = 0;
        stepper[i].position_request = 0;
    }
}


/// \todo allow for different feedrate on extruder and z axis
void stepper_set_feedrate(uint32_t feedrate_mmpermin)
{
    // Configure the timer prescalers so the smaller counter (1) goes to the max of the feedrate.
    // Leaves 16 bits of precision for subspeed (1/65535 at min speed)
    for (int i =0; i<AXIS_NUM; i++)
    {
        uint16_t prescaler = 0;
        uint32_t Fclk = 168000000/64; /// At CLK_DIV_1 => APB Timer clock = 168MHz main clock (PLL) /1 (HCLK) /4 (APBx) .... what else ? \see SetSysClock for more details
        prescaler = Fclk / (feedrate_mmpermin * mm2steps / 60);

//        prescaler = prescaler/64; /// \todo Correct this hack when found which clock is used for these Timers... !!

        // Prescaler change can be immediate or done at the next update event
        // Do it immedialty, as the update event can be a long time from now depending on the last config
        TIM_PrescalerConfig(stepper[i].timer, prescaler, TIM_PSCReloadMode_Immediate);
    }
}

