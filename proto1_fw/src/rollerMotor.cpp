#include "rollerMotor.h"
#include "stateMachine.h"
#include "encoder.h"
#include "monitor.h"

// *****Adjustable defines*****
#define TASK_PERIOD_MS 1U

#define DUAL_MOTOR false // BOTH MOTORS MUST BE THE SAME
#define PWM_MODE 3 // For 3pwm make sure the pwms are tied together
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
#define MAX_CURRENT 15.0f // amps

#define SUPPLY_VOLTAGE (24U)
// Define specific motor we are using
#define MOTOR_GEARBOX
// Define if we are using SPI for custom motor controller
#define USE_GD_SPI true

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

#ifdef MOTOR_GEARBOX
#define MOTOR_KV 145U
#define MOTOR_POLE_PAIRS 2U
#define MOTOR_PHASE_RESISTANCE 0.35f 
#endif // MOTOR_U8

#define UH1 PA8
#define UH2 PA7
#define VH1 PA9
#define VH2 PB0
#define WH1 PA10
#define WH2 PB1

BLDCDriver3PWM driver1 = BLDCDriver3PWM(UH1, VH1, WH1);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(UH2, VH2, WH2);

#if CALIBRATION_MODE
// Need to use voltage limit mode for calibration, if specifying phase resistance it will use current limit
BLDCMotor motor1 = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCMotor motor2 = BLDCMotor(MOTOR_POLE_PAIRS);
#else
BLDCMotor motor1 = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE);
BLDCMotor motor2 = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE);
#endif // CALIBRATION_MODE

HallSensor sensor1 = HallSensor(PB3, PB4, PB5, MOTOR_POLE_PAIRS);
HallSensor sensor2 = HallSensor(PB13, PB14, PB15, MOTOR_POLE_PAIRS);

Commander commander = Commander(Serial);
void onMotor1(char* cmd){ commander.motor(&motor1,cmd); }
void onMotor2(char* cmd){ commander.motor(&motor2,cmd); }

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

    commander.add('M',onMotor1,"Motor 1");
    commander.add('N',onMotor2,"Motor 2");

    SimpleFOCDebug::enable(&Serial);
    // Motor 1
    motor1.useMonitoring(Serial);
    driver1.voltage_power_supply = SUPPLY_VOLTAGE;
    driver1.pwm_frequency = 15000; // Lower pwm freq (from 25kHz default) to reduce switching loss
    driver1.init();
    motor1.linkDriver(&driver1);

    sensor1.pullup = Pullup::USE_INTERN;
    sensor1.init();
    motor1.linkSensor(&sensor1);

    motor1.velocity_limit = 100;

#if DUAL_MOTOR
    // Motor 2
    motor2.useMonitoring(Serial);
    driver2.voltage_power_supply = SUPPLY_VOLTAGE;
    driver2.pwm_frequency = 15000; // Lower pwm freq (from 25kHz default) to reduce switching loss
    driver2.init();
    motor2.linkDriver(&driver2);

    sensor2.pullup = Pullup::USE_INTERN;
    sensor2.init();
    motor2.linkSensor(&sensor2);

    motor2.velocity_limit = 100;
#endif // DUAL_MOTOR

#if OPEN_LOOP == false
    motor1.controller = MotionControlType::velocity;
    motor2.controller = MotionControlType::velocity;
#else
	motor1.controller = MotionControlType::velocity_openloop;
    motor2.controller = MotionControlType::velocity_openloop;
#endif // OPEN_LOOP == false

	// Motor 1
    motor1.current_limit = current_limit;

    motor1.PID_velocity.P = 0.4;
    motor1.PID_velocity.I = 1.0;
    motor1.PID_velocity.D = 0;
    motor1.PID_velocity.output_ramp = 1000;
    motor1.LPF_velocity.Tf = 0.4;

    motor1.voltage_sensor_align = 0.5;
    #if CALIBRATION_MODE
    motor1.voltage_limit = 0.5; // Set when running initFOC for CALIBRATION ONLY to be safe
    #else
    // Determined once with initFOC calibration
    motor1.sensor_direction = Direction::CW;
    motor1.zero_electric_angle = 3.14;
    #endif // CALIBRATION_MODE

    motor1.KV_rating = MOTOR_KV;
    motor1.init();

    motor1.initFOC();
    motor1.disable();

#if DUAL_MOTOR
	// Motor 2
	motor2.current_limit = current_limit;

    motor2.PID_velocity.P = 0.4;
    motor2.PID_velocity.I = 1.0;
    motor2.PID_velocity.D = 0;
    motor2.PID_velocity.output_ramp = 1000;
    motor2.LPF_velocity.Tf = 0.4;

    motor2.voltage_sensor_align = 0.5;
    #if CALIBRATION_MODE
    motor2.voltage_limit = 0.5; // Set when running initFOC for CALIBRATION ONLY to be safe
    #else
    // Determined once with initFOC calibration
    motor2.sensor_direction = Direction::CW;
    motor2.zero_electric_angle = 6.0;
    #endif // CALIBRATION_MODE

    motor2.KV_rating = MOTOR_KV;
    motor2.init();

    motor2.initFOC();
    motor2.disable();
#endif // DUAL_MOTOR

    // Loop
    while (1)
    {
        commander.run();
        checkEncoder();

        motor1.current_limit = current_limit;
        motor1.PID_velocity.limit = motor1.current_limit;
        motor1.loopFOC();
        motor1.move(speed_target);

#if DUAL_MOTOR
		motor2.current_limit = current_limit;
        motor2.PID_velocity.limit = motor2.current_limit;
		motor2.loopFOC();
		motor2.move(speed_target);
#endif // DUAL_MOTOR

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
		motor1.enable();
#if DUAL_MOTOR
		motor2.enable();
#endif // DUAL_MOTOR
	}
	else
	{
		motor1.disable();
#if DUAL_MOTOR
		motor2.disable();
#endif // DUAL_MOTOR
	}
}

bool getRollerMotorEnabled(void)
{
    return motor1.enabled || motor2.enabled;
}

float getRollerMotorAngle(void)
{
    return sensor1.getAngle();
}

float getRollerMotorSpeed(void)
{
    return motor1.shaft_velocity;
}

float getRollerMotorSpeedTarget(void)
{
    return speed_target;
}

float getRollerMotorCurrent(void)
{
    return motor1.current.d + motor1.current.q;
}

float getRollerMotorCurrentLimit(void)
{
    return current_limit;
}
