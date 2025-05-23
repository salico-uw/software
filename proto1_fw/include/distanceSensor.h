#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>

void initDistanceSensorTask(UBaseType_t priority);
uint16_t getDistanceMM();
bool getDistanceSensorHealthy();
float getPositiveVelocity();
float getNegativeVelocity();
float getDirectionalVelocity();
