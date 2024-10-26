#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "led.h"

// Private functions
static void TaskBlink(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;
    pinMode(LED_BUILTIN, OUTPUT);
    // Loop
    while (1)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(xDelay / 2);
        digitalWrite(LED_BUILTIN, LOW);        
        vTaskDelay(xDelay / 2);
    }
}

// Public functions
void initLedTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskBlink
    ,  (const portCHAR *)"Monitor"
    ,  128
    ,  NULL
    ,  priority
    ,  NULL );
}
