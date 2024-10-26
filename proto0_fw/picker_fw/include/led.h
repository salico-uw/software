#pragma once
#include <STM32FreeRTOS.h>

#define TASK_PERIOD_MS 2000U

// Dummy task to blink the on board LED
// PA5
void initLedTask(UBaseType_t priority);
