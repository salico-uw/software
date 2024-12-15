#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>

typedef enum {
  OFF_STATE,
  SPEED_STATE,
  CURRENT_STATE,
  FAULT_STATE,
  STATE_COUNT
} State_E;

void initStateMachineTask(UBaseType_t priority);
State_E getState(void);
