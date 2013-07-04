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

    if (TIM2->SR & TIM_SR_UIF)
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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; // enable the clock to GPIOE
//    GPIOE->MODER |= (1 << (2*11));
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
