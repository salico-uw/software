#include <Arduino.h>
#include "monitor.h"
#include "rollerMotor.h"

#define TASK_PERIOD_MS 1U

#define MONITOR_ANGLE_SPEED_LIMIT 250.0f // rad/s
#define MONITOR_ANGLE_SPEED_LIMIT_COUNT 1U // cycles
#define MONITOR_HIGH_CURRENT_LIMIT 8.0f // amps
#define MONITOR_HIGH_CURRENT_TIMEOUT_MS 5000 // ms

uint32_t high_current_count = 0U;
uint32_t last_current_millis = 0U;
uint8_t angle_fault_count = 0U;
uint16_t monitor_bits = 0U; // bitfield of monitor trips (1 is tripped)

// Private functions
bool angleMonitor(BLDCMotor const * const motor)
{
    bool tripped = false;
    // angle sensor speed monitor
    if(fabs(motor->shaft_velocity) > MONITOR_ANGLE_SPEED_LIMIT) {
        if(angle_fault_count < MONITOR_ANGLE_SPEED_LIMIT_COUNT)
        {
            angle_fault_count++;
        }
        else {
            tripped = true;
            Serial.println("Faulted due to impossible angle velocity, fix fault and power cycle");
        }
    }
    return tripped;
}

bool motorOverloadMonitor(BLDCMotor const * const motor)
{
    bool tripped = false;
    // motor high current monitor, check every 1ms
    // Asymmetric counter to still trip when hovering around threshold
    if(fabs(motor->current.d + motor->current.q) >= MONITOR_HIGH_CURRENT_LIMIT)
    {
        if(high_current_count < (MONITOR_HIGH_CURRENT_TIMEOUT_MS*2U))
        {
            high_current_count += 2U;
        }
        else
        {
            tripped = true;
            Serial.println("Faulted due to high current timeout reached, fix fault and power cycle");
        }
    }
    else
    {
        if(high_current_count > 0)
        {
            high_current_count -= 1U;
        }
    }
    return tripped;
}

static void TaskMonitor(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;
    // Setup
    BLDCMotor const * const rollerMotor = getRollerMotor();
    // Loop
    while (1)
    {
        if(rollerMotor->enabled)
        {
            monitor_bits |= angleMonitor(rollerMotor) << 0U;
            monitor_bits |= motorOverloadMonitor(rollerMotor) << 1U;
        }

        vTaskDelay(xDelay);
    }
}

// Public functions
void initMonitorTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskMonitor
    ,  (const portCHAR *)"Monitor"
    ,  128
    ,  NULL
    ,  priority
    ,  NULL );
}

bool getMonitorTripped(void)
{
    return monitor_bits != 0U;
}

uint16_t getMonitorTripBits(void)
{
    return monitor_bits;
}
