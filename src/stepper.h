#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>

void stepper_init(void);

void stepper_set_forward(void);
void stepper_set_backward(void);
bool StepperIsForward(void);

#endif // STEPPER_H
