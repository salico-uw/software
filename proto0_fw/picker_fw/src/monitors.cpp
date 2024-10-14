#include <Arduino.h>
#include "monitors.h"

#define MONITOR_ANGLE_SPEED_LIMIT 250.0f // rad/s
#define MONITOR_ANGLE_SPEED_LIMIT_COUNT 2U // cycles
#define MONITOR_HIGH_CURRENT_LIMIT 10.0f // amps
#define MONITOR_HIGH_CURRENT_TIMEOUT_MS 10000 // ms

uint32_t high_current_count = 0U;
uint32_t last_current_millis = 0U;
uint8_t angle_fault_count = 0U;

// Private functions
bool angleMonitor(BLDCMotor const * const motor)
{
    bool tripped = false;
    if(motor->enabled == true)
    {
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
    }
    return tripped;
}

bool motorOverloadMonitor(BLDCMotor const * const motor)
{
    bool tripped = false;
    if(motor->enabled == true)
    {
        // motor high current monitor, check every 1ms
        uint32_t current_millis = millis();
        if((current_millis-last_current_millis) >= 1U)
        {
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
        }
        last_current_millis = current_millis;
    }
    return tripped;
}

// Public functions
bool checkAllMonitors(BLDCMotor const * const motor)
{
    bool tripped = false;
    tripped |= angleMonitor(motor);
    tripped |= motorOverloadMonitor(motor);
    return tripped;
}