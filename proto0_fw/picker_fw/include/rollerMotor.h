#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SimpleFOC.h>

#define CALIBRATION_MODE false

void initRollerMotorTask(UBaseType_t priority);
void setRollerMotorEnable(bool enable);
bool getRollerMotorEnabled(void);
float getRollerMotorAngle(void);
float getRollerMotorSpeed(void);
float getRollerMotorSpeedTarget(void);
float getRollerMotorCurrent(void);
float getRollerMotorCurrentLimit(void);
