#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SimpleFOC.h>

void initRollerMotorTask(UBaseType_t priority);
BLDCMotor const * const getRollerMotor(void);
int16_t getRollerMotorSpeed(void);
int16_t getRollerMotorSpeedTarget(void);
int16_t getRollerMotorCurrent(void);
int16_t getRollerMotorCurrentLimit(void);
