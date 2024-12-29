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
#include "led.h"

/* Using STMF401RE Nucleo with IHM08M1 motor sheild,
 * AS5047P SPI angle sensor
 * SSD1306 128x32 OLED I2C display
 * rotary encoder with pushbutton for mode control
 */

#define TEST_MODE true

void setup()
{
#if TEST_MODE
	pinMode(13, OUTPUT);
#else
	Serial.begin(115200);
	while (!Serial) {}
	Serial.print("Init RTOS with sysclock: ");
	Serial.println(SystemCoreClock);

#if !CALIBRATION_MODE
	initLoggerTask(1);
#endif // !CALIBRATION_MODE
	initRollerMotorTask(3);
	initStateMachineTask(2);
	initMonitorTask(2);
	initLedTask(4);

	Serial.println("Start RTOS");
	
	// Start RTOS
	vTaskStartScheduler();
	while(1);
#endif
}

void loop()
{
	digitalWrite(13, HIGH);
	delay(1000);
	digitalWrite(13, LOW);
	delay(1000);
}
