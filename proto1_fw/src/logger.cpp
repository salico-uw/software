#include "logger.h"

#include "stateMachine.h"
#include "rollerMotor.h"
#include "logger.h"
#include "display.h"
#include "monitor.h"

#define TASK_PERIOD_MS 200U

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
		Serial.print("S: "); // state
		Serial.print(state);
		Serial.print(" A: "); // motor angle
		Serial.print(getRollerMotorAngle());
		Serial.print(" W: "); // motor speed (rad/s)
		Serial.print(motorSpeed);
		Serial.print(" TW: "); // target speed
		Serial.print(speedTarget);
		Serial.print(" C: "); // motor current (A)
		Serial.print(motorCurrent);
		Serial.print(" CL: "); // current limit
		Serial.print(currentLimit);
		Serial.print(" FB: "); // fault bits
		Serial.println(getMonitorTripBits(), BIN);
		// Serial.print("Hall A: ");
		// Serial.print(analogRead(PB3));
		// Serial.print("B: ");
		// Serial.print(analogRead(PB4));
		// Serial.print("C: ");
		// Serial.println(analogRead(PB5));

		// dp_clear();
		// double value = 0.0f;
		// double target = 0.0f;
		// if(state == OFF_STATE || state == SPEED_STATE) {
		//   value = motorSpeed;
		//   target = speedTarget;
		// } else if(state == CURRENT_STATE) {
		//   value = motorCurrent;
		//   target = currentLimit;
		// }
		// dp_draw_num(value, 0);
		// dp_draw_num(target, 1);
		// dp_draw_state(state);
		// dp_send();
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