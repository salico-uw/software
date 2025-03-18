#include "distanceSensor.h"
#include "stateMachine.h"

#include <Wire.h>
#include <VL53L0X.h>

#define TASK_PERIOD_MS 200U

#define MAX_VALID_RANGE_MM 300U

static VL53L0X sensor;
static uint16_t distance_mm = 0U;
static uint16_t prev_distance_mm = 0U;
static float velocity = 0.0f;

static void TaskDistanceSensor(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;
    // Setup
    Wire.setClock(100000);
	Wire.begin();

    sensor.setTimeout(500);
    if (!sensor.init())
    {
        Serial.println("Failed to detect and initialize distance sensor!");
    }

    sensor.startContinuous();

    // Loop
    while (1)
    {
        distance_mm = sensor.readRangeContinuousMillimeters();
        if(distance_mm > MAX_VALID_RANGE_MM)
        {
            distance_mm = MAX_VALID_RANGE_MM;
        }
        // velocity up to 0.1 m/s
        velocity = ((float)(distance_mm) - (float)(prev_distance_mm)) / (float)(TASK_PERIOD_MS);
        
		// Serial.print(distance_mm);
        // Serial.print(" vel: ");
        // Serial.println(velocity);
        prev_distance_mm = distance_mm;

        if (sensor.timeoutOccurred()) { Serial.print("Distance Sensor TIMEOUT"); }
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
    if(getState() == EXTENDED_STATE)
    {
        vel = getPositiveVelocity();
    }
    else if(getState() == RETRACTED_STATE)
    {
        vel = getNegativeVelocity();
    }
    return vel;
}

bool getDistanceSensorHealthy()
{
    return sensor.init() && !sensor.timeoutOccurred();
}
