#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SimpleFOC.h>

// Run motor sensor calibration for first-time configuration
#define CALIBRATION_MODE false
// Toggle for open loop for testing
#define OPEN_LOOP true // Dont set when CALIBRATION_MODE == true

#if OPEN_LOOP && CALIBRATION_MODE
#error "Cannot calibrate in open loop mode"
#endif

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
