#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include "stm32f4xx_conf.h"

typedef enum
{
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_E,
    AXIS_NUM // For loops, contains the number of axis on this machine
} STEPPER_AXIS;


void stepper_init(void);

void stepper_move(float delta[AXIS_NUM]);
void stepper_stop();

void stepper_set_absolute(void);
void stepper_set_relative(void);
void stepper_reset(void);

void stepper_set_feedrate(uint32_t feedrate_mmpermin);

#endif // STEPPER_H
