#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>

void pwm_init(void);
void pwm_setduty(float left, float right);

#endif