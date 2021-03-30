#pragma once
#include <stdint.h>

void pwm_init(void);

void set_period(int freq, int duty);
