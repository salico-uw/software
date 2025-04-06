#include "distanceSensor.h"
#include "stateMachine.h"

#include <Wire.h>
#include <VL53L0X_mod.h>

#define TASK_PERIOD_MS 100U

#define MAX_VALID_RANGE_MM 300U

static VL53L0X_mod sensor;
static uint16_t distance_mm = 0U;
static uint16_t prev_distance_mm = 0U;
static float velocity = 0.0f;
static bool healthy = false;

static void TaskDistanceSensor(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;
    // Setup
    Wire.setClock(400000);
	Wire.begin();

    sensor.setTimeout(TASK_PERIOD_MS);

    if (sensor.init())
    {
        healthy = true;
    }
    else
    {
        Serial.println("Failed to detect and initialize distance sensor!");
    }
    sensor.startContinuous();

    // Loop
    while (1)
    {
        uint16_t temp_dist = 0U;
        if(sensor.readRangeNoBlocking(temp_dist))
        {
            distance_mm = temp_dist;
            // if(distance_mm > MAX_VALID_RANGE_MM)
            // {
            //     distance_mm = MAX_VALID_RANGE_MM;
            // }
            // velocity up to 0.1 m/s
            velocity = ((float)(distance_mm) - (float)(prev_distance_mm)) / (float)(TASK_PERIOD_MS);
            
            // Serial.print(distance_mm);
            // Serial.print(" vel: ");
            // Serial.println(velocity);
            prev_distance_mm = distance_mm;
        }

        healthy &= !sensor.timeoutOccurred();

        // if (sensor.timeoutOccurred()) { Serial.print("Distance Sensor TIMEOUT"); }
        vTaskDelay(xDelay);
    }
}

void initDistanceSensorTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskDistanceSensor
    ,  (const portCHAR *)"Distance Sensor"
    ,  256
    ,  NULL
    ,  priority
    ,  NULL );
}

uint16_t getDistanceMM()
{
    return distance_mm;
}

float getPositiveVelocity()
{
    float vel_p = 0.0f;
    if(velocity > 0.0f)
    {
        vel_p = velocity;
    }
    return vel_p;
}

float getNegativeVelocity()
{
    float vel_n = 0.0f;
    if(velocity < 0.0f)
    {
        vel_n = fabs(velocity);
    }
    return vel_n;
}

float getDirectionalVelocity()
{
    float vel = 0.0f;
    if(getState() == EXTENDING_ROLLING_STATE)
    {
        vel = getPositiveVelocity();
    }
    else if(getState() == RETRACTING_NO_ROLLING_STATE)
    {
        vel = getNegativeVelocity();
    }
    return vel;
}

bool getDistanceSensorHealthy()
{
    return healthy;
}
