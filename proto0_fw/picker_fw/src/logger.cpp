#include "logger.h"

#include "stateMachine.h"
#include "rollerMotor.h"
#include "logger.h"

#define TASK_PERIOD_MS 1000U

static void TaskLogger(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;
    // Setup

    // Loop
    while (1)
    {
		Serial.print("mode: ");
		Serial.print(getState());
		Serial.print(" angle: ");
		Serial.print(getRollerMotorAngle());
		Serial.print(" speed: ");
		Serial.print(getRollerMotorSpeed());
		Serial.print(" target speed: ");
		Serial.print(getRollerMotorSpeedTarget());
		Serial.print(" current: ");
		Serial.print(getRollerMotorCurrent());
		Serial.print(" limit: ");
		Serial.println(getRollerMotorCurrentLimit());

		// dp_clear();
		// double value = 0.0f;
		// double target = 0.0f;
		// if(menuMode == OFF_MODE || menuMode == SPEED_MODE) {
		//   value = motor.shaft_velocity;
		//   target = speed_target;
		// } else if(menuMode == CURRENT_MODE) {
		//   value = motor.current.d + motor.current.q;
		//   target = motor.current_limit;
		// }
		// dp_draw_num(value, 0);
		// dp_draw_num(target, 1);
		// dp_draw_mode(menuMode);
		// dp_send();
        vTaskDelay(xDelay);
    }
}

void initLoggerTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskLogger
    ,  (const portCHAR *)"Logger"
    ,  128
    ,  NULL
    ,  priority
    ,  NULL );
}