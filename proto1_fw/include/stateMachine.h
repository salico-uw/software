#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>

typedef enum {
  OFF_STATE,
  RETRACTED_IDLE_STATE,
  EXTENDING_ROLLING_STATE,
  RETRACTING_NO_ROLLING_STATE,
  RETRACTED_ROLLING_STATE,
  FAULT_STATE,
  STATE_COUNT
} State_E;

void initStateMachineTask(UBaseType_t priority);
State_E getState(void);
