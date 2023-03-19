#ifndef PWM_CONF__H
#define PWM_CONF__H

#include <stdint.h>

#define PWM_RESIZE_FACTOR (65535)

void setup_pin_pwm(uint8_t pin);

#endif // PWM_CONF__H