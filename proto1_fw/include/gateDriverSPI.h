#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>

void initGateDriverSPITask(UBaseType_t priority);
bool isGDInitFinished();
bool isGateDriverHealthy();
