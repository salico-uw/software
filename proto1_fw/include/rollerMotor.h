#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SimpleFOC.h>

#define CALIBRATION_MODE false

void initRollerMotorTask(UBaseType_t priority);
void setRollerMotorEnable(bool enable);
bool getRollerMotorEnabled(void);
bool getInSpeedMode(void);
float getRollerMotor1Angle(void);
float getRollerMotor2Angle(void);
float getRollerMotor1Speed(void);
float getRollerMotor2Speed(void);
float getRollerMotorSpeedTarget(void);
float getRollerMotor1Current(void);
float getRollerMotor2Current(void);
float getRollerMotorCurrentLimit(void);
