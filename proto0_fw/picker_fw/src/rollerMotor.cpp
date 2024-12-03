#include "rollerMotor.h"
#include "stateMachine.h"
#include "encoder.h"
#include "monitor.h"

// *****Adjustable defines*****
#define TASK_PERIOD_MS 2U

#define PWM_MODE 6 // For 3pwm make sure the pwms are tied together
#if (SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH) == true && (PWM_MODE == 6)
#error "L6398 has low side active low"
#endif

// Toggle for open loop for testing
#define OPEN_LOOP false // Dont set when CALIBRATION_MODE == true
#if OPEN_LOOP && CALIBRATION_MODE
#error "Cannot calibrate in open loop mode"
#endif

#define SPEED_INCREMENT 1.0f // rad/s
#define MAX_SPEED 200.0f // rad/s
#define CURRENT_INCREMENT 0.5f // rad/s
#if OPEN_LOOP == false
#define MAX_CURRENT 15.0f // amps
#else
#define MAX_CURRENT 5.0f // amps
#endif // OPEN_LOOP == false

#define SUPPLY_VOLTAGE (22U)
// Define specific motor we are using
#define MOTOR_AT3520

// *****Rest of defines*****
#ifdef MOTOR_F80
#define MOTOR_KV 1900U
#define MOTOR_POLE_PAIRS 7U
#define MOTOR_PHASE_RESISTANCE 0.15f
#endif // MOTOR_F80

#ifdef MOTOR_AT3520
#define MOTOR_KV 880U
#define MOTOR_POLE_PAIRS 7U
#define MOTOR_PHASE_RESISTANCE 0.075f
#endif // MOTOR_AT3520

#ifdef MOTOR_U8
#define MOTOR_KV 170U
#define MOTOR_POLE_PAIRS 21U
#define MOTOR_PHASE_RESISTANCE 0.15f
#endif // MOTOR_U8

#define UH PA8
#define UL PA7
#define VH PA9
#define VL PB0
#define WH PA10
#define WL PB1

#if PWM_MODE == 6
BLDCDriver6PWM driver = BLDCDriver6PWM(UH, UL, VH, VL, WH, WL);
#elif PWM_MODE == 3
BLDCDriver3PWM driver = BLDCDriver3PWM(UH, VH, WH);
#endif // PWM_MODE

#if CALIBRATION_MODE
// Need to use voltage limit mode for calibration, if specifying phase resistance it will use current limit
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
#else
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE);
#endif // CALIBRATION_MODE

MagneticSensorSPI angleSensor = MagneticSensorSPI(AS5047_SPI, PD2);
SPIClass SPI_3(PC12, PC11, PC10);

Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }

double speed_target = 0.0f; // rad/s
double current_limit = 3.0f; // amps
void checkEncoder() {
  State_E state = getState();
  if(state == SPEED_STATE || state == CURRENT_STATE)
  {
    // Initialize as speed mode
    double mode_increment = SPEED_INCREMENT;
    double upper_limit = MAX_SPEED;
    double lower_limit = -MAX_SPEED;
    double * target = &speed_target;
    if(state == CURRENT_STATE)
    {
      mode_increment = CURRENT_INCREMENT;
      upper_limit = MAX_CURRENT;
      lower_limit = 0.0f;
      target = &current_limit;
    }

    unsigned char direction = getEncoderDirection();
    if (direction == DIR_CW)
    {
      *target += mode_increment;
    }
    else if (direction == DIR_CCW)
    {
      *target -= mode_increment;
    }
    // Saturate speed
    if(*target > upper_limit)
    {
      *target = upper_limit;
    } else if (*target < lower_limit) {
      *target = lower_limit;
    }
  }
}

static void TaskRollerMotor(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;
    // Setup
    encoderSetup();

    commander.add('M',onMotor,"Motor 1");

    SimpleFOCDebug::enable(&Serial);
    motor.useMonitoring(Serial);
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.pwm_frequency = 10000; // Lower pwm freq (from 25kHz default) to reduce switching loss
    driver.init();
    motor.linkDriver(&driver);

    angleSensor.clock_speed = 4000000; // AS5047 SPI speed can go up to 10MHz 
    angleSensor.init(&SPI_3);
    motor.linkSensor(&angleSensor);

    motor.velocity_limit = 100;
#if OPEN_LOOP == false
    motor.controller = MotionControlType::velocity;
#else
    motor.controller = MotionControlType::velocity_openloop;
#endif // OPEN_LOOP == false
    motor.current_limit = current_limit;

    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 8.0;
    motor.PID_velocity.D = 0.0;
    motor.PID_velocity.output_ramp = 1000;
    motor.LPF_velocity.Tf = 0.01;

    motor.voltage_sensor_align = 0.5;
    #if CALIBRATION_MODE
    motor.voltage_limit = 0.5; // Set when running initFOC for CALIBRATION ONLY to be safe
    #else
    // Determined once with initFOC calibration
    motor.sensor_direction = Direction::CW;
    motor.zero_electric_angle = 6.0;
    #endif // CALIBRATION_MODE

    motor.KV_rating = MOTOR_KV;
    motor.init();

    motor.initFOC();
    motor.disable();

    // Loop
    while (1)
    {
        commander.run();
        checkEncoder();
        motor.current_limit = current_limit;
        motor.PID_velocity.limit = motor.current_limit;

        motor.loopFOC();

        motor.move(speed_target);

        vTaskDelay(xDelay);
    }
}

// Public functions
void initRollerMotorTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskRollerMotor
    ,  (const portCHAR *)"Roller Motor"
    ,  512
    ,  NULL
    ,  priority
    ,  NULL );
}

void setRollerMotorEnable(bool enable)
{
	if(enable)
	{
		motor.enable();
	}
	else
	{
		motor.disable();
	}
}

bool getRollerMotorEnabled(void)
{
    return motor.enabled;
}

float getRollerMotorAngle(void)
{
    return angleSensor.getAngle();
}

float getRollerMotorSpeed(void)
{
    return motor.shaft_velocity;
}

float getRollerMotorSpeedTarget(void)
{
    return speed_target;
}

float getRollerMotorCurrent(void)
{
    return motor.current.d + motor.current.q;
}

float getRollerMotorCurrentLimit(void)
{
    return current_limit;
}
