#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include "stm32f4xx_conf.h"

void stepper_init(void);

void stepper_move(float deltaA, float deltaB);
void stepper_stop();
/*
void stepper_set_forward(void);
void stepper_set_backward(void);
bool StepperIsForward(void);
uint32_t getStepperPos(void);
*/
void stepper_set_absolute(void);
void stepper_set_relative(void);
void stepper_reset(void);

#endif // STEPPER_H
