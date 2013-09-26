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
#define mm2stepsPulley (MICROSTEPS * STEP_NUMBER / (TOOTH_NUMBER * TOOTH_PITCH))

#define M_PI		3.14159265358979323846 /// \todo find why the include math.h don't define M_PI ?
#define EXTRUDER_AXIS_DIAMETER 8.0
#define mm2stepsExtruder (MICROSTEPS * STEP_NUMBER / (M_PI * EXTRUDER_AXIS_DIAMETER))

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
    MOVE_MODE mode;
    float mm2steps;
    bool is_inverted; // To invert the movement orders to the stepper referential
} STEPPER_STATE;

STEPPER_STATE stepper[4] = {
    // X
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
        .position_current = 0,
        .mode = MOVE_Absolute,
        .mm2steps = mm2stepsPulley,
        .is_inverted = false
    },

    // Y
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
        .position_current = 0,
        .mode = MOVE_Absolute,
        .mm2steps = mm2stepsPulley,
        .is_inverted = true
    },

    // Z
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
        .position_current = 0,
        .mode = MOVE_Absolute,
        .mm2steps = mm2stepsPulley,
        .is_inverted = true
    },

    // E
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
        .position_current = 0,
        .mode = MOVE_Absolute,
        .mm2steps = mm2stepsExtruder,
        .is_inverted = false
    }
};

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
        printf("ok x:%4d y:%4d z:%4d e:%4d period_x:%4u y:%4u z:%4u e:%4u pre_x=%5u y:%5u z:%5u e:%5u\n",
               (int)stepper[AXIS_X].position_current, (int)stepper[AXIS_Y].position_current, (int)stepper[AXIS_Z].position_current, (int)stepper[AXIS_E].position_current,
               (unsigned int)stepper[AXIS_X].period_current, (unsigned int)stepper[AXIS_Y].period_current, (unsigned int)stepper[AXIS_Z].period_current, (unsigned int)stepper[AXIS_E].period_current,
               (unsigned int) TIM_GetPrescaler(stepper[AXIS_X].timer), (unsigned int) TIM_GetPrescaler(stepper[AXIS_Y].timer), (unsigned int) TIM_GetPrescaler(stepper[AXIS_Z].timer), (unsigned int) TIM_GetPrescaler(stepper[AXIS_E].timer));
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
    for (int axis=0; axis<AXIS_NUM; axis++)
    {
        stepper[axis].period_current = 1;
        stepper[axis].period_request = 1;

        init_stepper(axis);
    }
}

void stepper_move(float pos[AXIS_NUM], bool set[AXIS_NUM])
{
    float delta[AXIS_NUM];

    for (int axis=0; axis<AXIS_NUM; axis++)
    {
        if (set[axis]) // Only moves this stepper if the position along this axis as been set in the G-Code command
        {
            if (stepper[axis].is_inverted)
            {
                pos[axis] = -pos[axis];
            }

            if (stepper[axis].mode == MOVE_Absolute)
            {
                stepper[axis].position_request = 2 * pos[axis] * stepper[axis].mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
            }
            else
            {
                stepper[axis].position_request += 2 * pos[axis] * stepper[axis].mm2steps; // x2 because of using toggle in interrupts (2 interrupts for one pulse)
            }

            delta[axis] = stepper[axis].position_request - stepper[axis].position_current;
        }
        else
        {
            delta[axis] = 0;
        }
    }

    bool isAllmovementsStopped = true; // Sometime a request is made using the point where we already are, so no move is done at all.

    for (int axis=0; axis<AXIS_NUM; axis++)
    {
        float period = 0.0;

        // Compute norm of displacement on (X, Y, Z), to then compute speed along each axis
        float normDelta = sqrtf(powf(delta[AXIS_X], 2.0) + powf(delta[AXIS_Y], 2.0) + powf(delta[AXIS_Z], 2.0));

        if (delta[axis] != 0)
        {

            if (normDelta != 0)
            {
                period = fabs(normDelta/delta[axis]);
            }
            else // For extruder only commands
            {
                period = 10; // Not too fast
            }

            if (period == 0) // To be sure the interrupt will get called
            {
                period = 10;
            }

            stepper[axis].period_current = (uint32_t) period;
            TIM_SetAutoreload(stepper[axis].timer, stepper[axis].period_current);
            stepper[axis].direction = STEPPER_Waiting;
            isAllmovementsStopped = false; // At least one move as been requested
        }
        else
        {
            stepper[axis].direction = STEPPER_Stopped; // To be sure of the state of stepper in case it don't have to move
        }
    }


    if(isAllmovementsStopped) // As no move is requested, need to send now the answer the move as been executed, as it will never reach the interrupt
    {
        puts("ok\n");
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

void stepper_set_absolute(STEPPER_AXIS axis)
{
    stepper[axis].mode = MOVE_Absolute;
}

void stepper_set_relative(STEPPER_AXIS axis)
{
    stepper[axis].mode = MOVE_Relative;
}

void stepper_reset()
{
    for (int axis=0; axis<AXIS_NUM; axis++)
    {
        stepper[axis].direction = STEPPER_Stopped;
        stepper[axis].position_current = 0;
        stepper[axis].position_request = 0;
    }
}

void stepper_set_position(STEPPER_AXIS axis, float position)
{
    stepper[axis].position_current = position;
    stepper[axis].position_request = position;
    stepper[axis].direction = STEPPER_Stopped;
}

/// \todo allow for different feedrate on extruder and z axis
void stepper_set_feedrate(STEPPER_AXIS axis, uint32_t feedrate_mmpermin)
{
    // Configure the timer prescalers so the smaller counter (1) goes to the max of the feedrate.
    // Leaves 16 bits of precision for subspeed (1/65535 at min speed)


    /// \todo move the max feedrate as a general parameter
    if (feedrate_mmpermin > 100)
    {
        feedrate_mmpermin = 100;
    }

    uint16_t prescaler = 0;
    uint32_t Fclk = 168000000/64; /// At CLK_DIV_1 => APB Timer clock = 168MHz main clock (PLL) /1 (HCLK) /4 (APBx) .... what else ? \see SetSysClock for more details
    prescaler = Fclk / (feedrate_mmpermin * stepper[axis].mm2steps / 60);

//        prescaler = prescaler/64; /// \todo Correct this hack when found which clock is used for these Timers... !!

    // Prescaler change can be immediate or done at the next update event
    // Do it immedialty, as the update event can be a long time from now depending on the last config
    TIM_PrescalerConfig(stepper[axis].timer, prescaler, TIM_PSCReloadMode_Immediate);

}
