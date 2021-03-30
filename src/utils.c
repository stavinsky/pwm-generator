#include "utils.h"
volatile uint32_t system_millis=0;
void msleep(uint32_t delay)
{
    uint32_t wake = system_millis + delay;
    while (wake > system_millis);
}
