#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <SimpleFOC.h>
#include <U8g2lib.h>

#include "util.h"
#include "display.h"
#include "stateMachine.h"
#include "rollerMotor.h"
#include "monitor.h"
#include "logger.h"
#include "gateDriverSPI.h"

#define TEST_MODE true

/* Using STMF401RE Nucleo with IHM08M1 motor sheild,
 * AS5047P SPI angle sensor
 * SSD1306 128x32 OLED I2C display
 * rotary encoder with pushbutton for mode control
 */
void setup()
{
#if TEST_MODE
	Serial.begin(1000000);
	pinMode(13, OUTPUT);
#else
	Serial.begin(1000000);
	while (!Serial) {}
	Serial.print("Init RTOS with sysclock: ");
	Serial.println(SystemCoreClock);

#if !CALIBRATION_MODE
	initLoggerTask(1);
#endif // !CALIBRATION_MODE
	initGateDriverSPITask(2);
	initRollerMotorTask(3);
	initStateMachineTask(2);
	initMonitorTask(2);

	Serial.println("Start RTOS");
	
	// Start RTOS
	vTaskStartScheduler();
	while(1);
#endif
}

void loop()
{
#if TEST_MODE
		
	int state = 1;
	int mode = 2;
	float speed = 1.2345;
	float current = 5.6789;

	while(1) {
		speed += 0.2;
		current += 2.3;
		Serial.print("S:"); // state
		Serial.print(state);
		Serial.print(",Sp/!Cu:"); // Speed or current Mode
		Serial.print(mode);
		Serial.print(",TW:"); // target speed
		Serial.print(speed);
		Serial.print(",W1:"); // motor speed (rad/s)
		Serial.print(speed);
		Serial.print(",W2:"); // motor speed (rad/s)
		Serial.print(speed);
		Serial.print(",CL:"); // current limit
		Serial.print(current);
		Serial.print(",C1:"); // motor current (A)
		Serial.print(current);
		Serial.print(",C2:"); // motor current (A)
		Serial.print(current);
		Serial.print(",FB:"); // fault bits
		Serial.println(mode);
		delay(1000);
	}
#endif
}
