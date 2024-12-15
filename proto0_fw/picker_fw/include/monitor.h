#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>

void initMonitorTask(UBaseType_t priority);
// Returns true when tripped
bool getMonitorTripped(void);
// Returns bits based on which monitor was tripped
uint16_t getMonitorTripBits(void);
