#include "logger.h"

#include "stateMachine.h"
#include "rollerMotor.h"
#include "logger.h"
#include "display.h"

#define TASK_PERIOD_MS 1500U

static void TaskLogger(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;
    // Setup
	dp_setup();

    // Loop
    while (1)
    {
		
		State_E state = getState();
		float motorSpeed = getRollerMotorSpeed();
		float speedTarget = getRollerMotorSpeedTarget();
		float motorCurrent = getRollerMotorCurrent();
		float currentLimit = getRollerMotorCurrentLimit();
		// Serial.print("mode: ");
		// Serial.print(state);
		// Serial.print(" angle: ");
		// Serial.print(getRollerMotorAngle());
		// Serial.print(" speed: ");
		// Serial.print(motorSpeed);
		// Serial.print(" target speed: ");
		// Serial.print(speedTarget);
		// Serial.print(" current: ");
		// Serial.print(motorCurrent);
		// Serial.print(" limit: ");
		// Serial.println(currentLimit);

		dp_clear();
		double value = 0.0f;
		double target = 0.0f;
		if(state == OFF_STATE || state == SPEED_STATE) {
		  value = motorSpeed;
		  target = speedTarget;
		} else if(state == CURRENT_STATE) {
		  value = motorCurrent;
		  target = currentLimit;
		}
		dp_draw_num(value, 0);
		dp_draw_num(target, 1);
		dp_draw_state(state);
		dp_send();
        vTaskDelay(xDelay);
    }
}

void initLoggerTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskLogger
    ,  (const portCHAR *)"Logger"
    ,  512
    ,  NULL
    ,  priority
    ,  NULL );
}