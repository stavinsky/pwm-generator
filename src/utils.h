#pragma once

#include "stdint.h"

extern volatile uint32_t system_millis;
void msleep(uint32_t delay);
