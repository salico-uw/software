#include <Arduino.h>
#include "monitor.h"
#include "rollerMotor.h"
#include "gateDriverSPI.h"

#define TASK_PERIOD_MS 200U

#define MONITOR_ANGLE_SPEED_LIMIT 120.0f // rad/s
#define MONITOR_ANGLE_SPEED_LIMIT_COUNT 1U // cycles
#define MONITOR_HIGH_CURRENT_LIMIT 6.0f // amps
#define MONITOR_HIGH_CURRENT_TIMEOUT_MS 5000 // ms
#define MONITOR_GD_UNHEALTHY_LIMIT 5U // 1s of unhealthy GD before tripping

uint32_t high_current_count = 0U;
uint32_t last_current_millis = 0U;
uint8_t angle_fault_count = 0U;
uint8_t gd_unhealthy_count = 0U;
uint16_t monitor_bits = 0U; // bitfield of monitor trips (1 is tripped)

// Private functions
bool angleMonitor(float motor_speed1, float motor_speed2)
{
    bool tripped = false;
    // angle sensor speed monitor
    if((fabs(motor_speed2) > MONITOR_ANGLE_SPEED_LIMIT) || (fabs(motor_speed2) > MONITOR_ANGLE_SPEED_LIMIT)) {
        if(angle_fault_count < MONITOR_ANGLE_SPEED_LIMIT_COUNT)
        {
            angle_fault_count++;
        }
        else {
            tripped = true;
            // Serial.println("Faulted due to impossible angle velocity, fix fault and power cycle");
        }
    }
    return tripped;
}

bool motorOverloadMonitor(float motor_current1, float motor_current2)
{
    bool tripped = false;
    // motor high current monitor, check every 1ms
    // Asymmetric counter to still trip when hovering around threshold
    if((fabs(motor_current1) >= MONITOR_HIGH_CURRENT_LIMIT) || (fabs(motor_current2) >= MONITOR_HIGH_CURRENT_LIMIT))
    {
        if(high_current_count < (MONITOR_HIGH_CURRENT_TIMEOUT_MS*2U))
        {
            high_current_count += 2U*TASK_PERIOD_MS;
        }
        else
        {
            tripped = true;
            // Serial.println("Faulted due to high current timeout reached, fix fault and power cycle");
        }
    }
    else if(high_current_count > 0U)
    {
        high_current_count -= 1U*TASK_PERIOD_MS;
    }
    return tripped;
}

bool gateDriverMonitor()
{
    // Trip if GD is faulted or not configured correctly
    bool tripped = false;
    if(!isGateDriverHealthy())
    {
        if(gd_unhealthy_count < MONITOR_GD_UNHEALTHY_LIMIT)
        {
            gd_unhealthy_count++;
        }
        else
        {
            tripped = true;
        }        
    }
    else
    {
        gd_unhealthy_count = 0U;
    }
    return tripped;
}

static void TaskMonitor(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;
    // Setup

    // Loop
    while (1)
    {
        monitor_bits |= gateDriverMonitor() << 0U;
        if(getRollerMotorEnabled())
        {
#if !OPEN_LOOP
            monitor_bits |= angleMonitor(getRollerMotor1Speed(), getRollerMotor2Speed()) << 1U; // Disable if not running closed loop
#endif
            monitor_bits |= motorOverloadMonitor(getRollerMotor1Current(), getRollerMotor2Current()) << 2U;
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
    ,  256
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
