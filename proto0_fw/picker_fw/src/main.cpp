#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SimpleFOC.h>
#include <U8g2lib.h>

#include "util.h"
#include "display.h"
#include "stateMachine.h"
#include "rollerMotor.h"
#include "monitor.h"

/* Using STMF401RE Nucleo with IHM08M1 motor sheild,
 * AS5047P SPI angle sensor
 * SSD1306 128x32 OLED I2C display
 * rotary encoder with pushbutton for mode control
 */

void setup()
{
	Serial.begin(115200);
	while (!Serial) {}

	initRollerMotorTask(2);
	initStateMachineTask(1);
	initMonitorTask(1);

	// dp_setup();
}

void loop()
{
// #if CALIBRATION_MODE == false
// 	// don't update display too fast, AFFECTS MOTOR CONTROL
// 	if (millis() % 5000 == 0)
// 	{
// 		Serial.print("mode: ");
// 		Serial.print(menuMode);
// 		Serial.print(" angle: ");
// 		Serial.print(angleSensor.getAngle());
// 		Serial.print(" speed: ");
// 		Serial.print(motor.shaft_velocity);
// 		Serial.print(" target speed: ");
// 		Serial.print(speed_target);
// 		Serial.print(" current: ");
// 		Serial.print(motor.current.d + motor.current.q);
// 		Serial.print(" limit: ");
// 		Serial.println(motor.current_limit);

// 		// dp_clear();
// 		// double value = 0.0f;
// 		// double target = 0.0f;
// 		// if(menuMode == OFF_MODE || menuMode == SPEED_MODE) {
// 		//   value = motor.shaft_velocity;
// 		//   target = speed_target;
// 		// } else if(menuMode == CURRENT_MODE) {
// 		//   value = motor.current.d + motor.current.q;
// 		//   target = motor.current_limit;
// 		// }
// 		// dp_draw_num(value, 0);
// 		// dp_draw_num(target, 1);
// 		// dp_draw_mode(menuMode);
// 		// dp_send();
// 	}
// #endif // CALIBRATION_MODE
}
