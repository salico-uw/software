#pragma once
#include <STM32FreeRTOS.h>

// Dummy task to blink the on board LED
// PA5
void initLedTask(UBaseType_t priority);
