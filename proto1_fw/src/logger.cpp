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
	// dp_setup();

    // Loop
    while (1)
    {
		State_E state = getState();
		float speedTarget = getRollerMotorSpeedTarget();
		float currentLimit = getRollerMotorCurrentLimit();

		Serial.print("S:"); // state
		Serial.print(state);
		Serial.print(",Sp/!Cu:"); // Speed or current Mode
		Serial.print(getInSpeedMode());
		Serial.print(",TW:"); // target speed
		Serial.print(speedTarget);
		Serial.print(",W1:"); // motor speed (rad/s)
		Serial.print(getRollerMotor1Speed());
		Serial.print(",W2:"); // motor speed (rad/s)
		Serial.print(getRollerMotor2Speed());
		Serial.print(",CL:"); // current limit
		Serial.print(currentLimit);
		Serial.print(",C1:"); // motor current (A)
		Serial.print(getRollerMotor1Current());
		Serial.print(",C2:"); // motor current (A)
		Serial.print(getRollerMotor2Current());
		Serial.print(",A1:"); // motor speed rad
		Serial.print(getRollerMotor1Angle());
		Serial.print(",A2:"); // motor angle rad
		Serial.print(getRollerMotor2Angle());
		Serial.print(",FB:"); // fault bits
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