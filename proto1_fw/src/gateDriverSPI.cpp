#include "gateDriverSPI.h"
#include <SPI.h>

#define DUAL_GD true
#define DEBUG_SPI false

#define TASK_PERIOD_MS 200U

#define READ 0b1000000000000000
#define WRITE 0x00

#define FAULT_STATUS_ADDR 0x0
#define FAULT_STATUS2_ADDR 0x1
#define DRIVER_CONTROL_ADDR 0x2
#define GD_HS_ADDR 0x3
#define GD_LOW_ADDR 0x4
#define OCP_ADDR 0x5
#define CSA_ADDR 0x6
#define DATA_MASK 0b11111111111
#define PWM3_CONFIG (0b01<<5U)

#define GD_EN_PIN PC4
#define NCS_PIN1 PD2
#define NCS_PIN2 PA15
SPIClass GD_SPI(PC12, PC11, PC10);
bool healthy = true;

uint16_t transmitSPI(uint16_t mosi, uint8_t CS_PIN)
{
	digitalWrite(CS_PIN, LOW);
	uint16_t rx_buffer = GD_SPI.transfer16(mosi) & DATA_MASK;
	digitalWrite(CS_PIN, HIGH);
#if DEBUG_SPI
    Serial.print("GD#");
    Serial.print(int(CS_PIN == NCS_PIN1) + 1);
    Serial.print(" MOSI: ")
    Serial.print(mosi, BIN);
    Serial.print(" MISO: ")
	Serial.println(rx_buffer, BIN);
#endif // DEBUG_SPI
    return rx_buffer;
}

void writeSPIRegister(uint8_t addr, uint16_t data, uint8_t CS_PIN)
{
    uint16_t mosi = WRITE | (addr << 11U) | data;
    transmitSPI(mosi, CS_PIN);
}

uint16_t readSPIRegister(uint8_t addr, uint8_t CS_PIN)
{
    uint16_t mosi = READ | (addr << 11U);
    return transmitSPI(mosi, CS_PIN);
}

void setupSPI(void)
{
    pinMode(GD_EN_PIN, OUTPUT);
    pinMode(NCS_PIN1, OUTPUT);
    digitalWrite(GD_EN_PIN, HIGH);
    delay(10); // IMPORTANT delay for GD chip to enable
	digitalWrite(NCS_PIN1, HIGH);
	GD_SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE1));
    bool setupCorrect = true;
    // Check for no faults
    setupCorrect &= readSPIRegister(FAULT_STATUS_ADDR, NCS_PIN1) == 0U;
    // Write 3PWM config to GD
    writeSPIRegister(DRIVER_CONTROL_ADDR, PWM3_CONFIG, NCS_PIN1);
    // Read back config value to make sure config correct
    setupCorrect &= readSPIRegister(DRIVER_CONTROL_ADDR, NCS_PIN1) == PWM3_CONFIG;

#if DUAL_GD
    pinMode(NCS_PIN2, OUTPUT);
	digitalWrite(NCS_PIN2, HIGH);
    // Second GD board
    setupCorrect &= readSPIRegister(FAULT_STATUS_ADDR, NCS_PIN2) == 0U;
    // Write 3PWM config to GD
    writeSPIRegister(DRIVER_CONTROL_ADDR, PWM3_CONFIG, NCS_PIN2);
    // Read back config value to make sure config correct
    setupCorrect &= readSPIRegister(DRIVER_CONTROL_ADDR, NCS_PIN2) == PWM3_CONFIG;
#endif // DUAL_GD

    healthy = setupCorrect;
    Serial.print("GD Healthy: ");
    Serial.println(healthy);
}

bool checkGDRegisters()
{
    bool correct = true;
    // Check no faults and read if 3pwm config is still correct
    uint16_t fsa =  readSPIRegister(FAULT_STATUS_ADDR, NCS_PIN1);
    correct &= (fsa == 0U);
    uint16_t dca = readSPIRegister(DRIVER_CONTROL_ADDR, NCS_PIN1);
    correct &= (dca == PWM3_CONFIG);
    // if(!correct)
    // {
    //     Serial.println(fsa, BIN);
    //     Serial.println(dca, BIN);
    // }

#if DUAL_GD
    correct &= readSPIRegister(FAULT_STATUS_ADDR, NCS_PIN2) == 0U;
    correct &= readSPIRegister(DRIVER_CONTROL_ADDR, NCS_PIN2) == PWM3_CONFIG;
#endif // DUAL_GD
    return correct;
}

static void TaskGateDriverSPI(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;
    // Setup
	setupSPI();

    // Loop
    while (1)
    {
		healthy = checkGDRegisters();
        vTaskDelay(xDelay);
    }
}

void initGateDriverSPITask(UBaseType_t priority)
{
    xTaskCreate(
    TaskGateDriverSPI
    ,  (const portCHAR *)"Gate Driver SPI"
    ,  256
    ,  NULL
    ,  priority
    ,  NULL );
}

bool isGateDriverHealthy()
{
    return healthy;
}
