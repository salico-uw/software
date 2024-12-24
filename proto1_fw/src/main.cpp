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

/* Using STMF401RE Nucleo with IHM08M1 motor sheild,
 * AS5047P SPI angle sensor
 * SSD1306 128x32 OLED I2C display
 * rotary encoder with pushbutton for mode control
 */
void setup()
{
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
}

void loop()
{

}
