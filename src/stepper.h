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

void stepper_set_absolute(STEPPER_AXIS axis);
void stepper_set_relative(STEPPER_AXIS axis);
void stepper_reset(void);

void stepper_set_feedrate(STEPPER_AXIS axis, uint32_t feedrate_mmpermin);
void stepper_set_position(STEPPER_AXIS axis, float position);

#endif // STEPPER_H
