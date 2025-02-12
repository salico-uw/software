#include "rollerMotor.h"
#include "stateMachine.h"
#include "encoder.h"
#include "monitor.h"
#include "gateDriverSPI.h"
#include "distanceSensor.h"

// *****Adjustable defines*****
#define TASK_PERIOD_MS 1U

#define DUAL_MOTOR true // BOTH MOTORS MUST BE THE SAME
#define SPEED_INCREASE_ENABLE false
#define PWM_MODE 3 // For 3pwm make sure the pwms are tied together
#if (SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH) == true && (PWM_MODE == 6)
#error "L6398 has low side active low"
#endif

#define SPEED_INCREMENT 1.0f // rad/s
#define MAX_SPEED 200.0f // rad/s
#define PISTON_SPEED_GAIN 1.0f
#define MAX_SPEED_INCREASE 5.0f // rad/s
#define CURRENT_INCREMENT 0.5f // rad/s
#define MAX_CURRENT 7.0f // amps
#define PWM_FREQ 15000U // Hz - Lower pwm freq (from 25kHz default) to reduce switching loss
#define REVERSE_TIME_MS 10000U
#define BASE_ROLLER_SPEED 22.0f // rad/s

#define SUPPLY_VOLTAGE (24U)
// Define specific motor we are using
#define MOTOR_GEARBOX

// *****Rest of defines*****
#ifdef MOTOR_12V
#if SUPPLY_VOLTAGE > 12U
#error "Cannot use this motor with >12V supply"
#endif
#define MOTOR_KV 1000U
#define MOTOR_POLE_PAIRS 7U
#define MOTOR_PHASE_RESISTANCE 0.25f
#endif // MOTOR_12V

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

#define BUTTON_CHECK_INTERVAL_MS (100U)
#define BUTTON_DEBOUNCE_MS (250U) // ms
#define MOTOR_BUTTON_PIN PC5

#define UH1 PA8
#define UH2 PA7
#define VH1 PA9
#define VH2 PB0
#define WH1 PA10
#define WH2 PB1

bool prevMotorButton = true; // active low
uint32_t motor_last_millis = 0U;

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

bool in_speed_mode = true; // Start in speed mode, alternate between current mode on button press
double speed_target = 0.0f; // rad/s
double current_limit = 2.0f; // amps
uint16_t reverse_spin_count = 0U;

bool wasMotorButtonPressed(){
    bool pressed = false;
    // Only check button press in larger time intervals to ignore short transients
    if((millis() % BUTTON_CHECK_INTERVAL_MS) == 0)
    {
        // handle overflow case
        if (millis() < motor_last_millis)
        {
            motor_last_millis = millis();
        }
        bool currButton = digitalRead(MOTOR_BUTTON_PIN);
        if(currButton == false && prevMotorButton == true && (millis() - motor_last_millis) > BUTTON_DEBOUNCE_MS)
        {
            pressed = true;
            motor_last_millis = millis();
        }
        prevMotorButton = currButton;
    }
    return pressed;
}

void checkEncoder() {
  State_E state = getState();
  if(state != OFF_STATE)
  {
    // Initialize as speed mode
    double mode_increment = SPEED_INCREMENT;
    double upper_limit = MAX_SPEED;
    double lower_limit = -MAX_SPEED;
    double * target = &speed_target;
    if(in_speed_mode == false)
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
    pinMode(MOTOR_BUTTON_PIN, INPUT_PULLUP);
    encoderSetup();

    commander.add('M',onMotor1,"Motor 1");
    commander.add('N',onMotor2,"Motor 2");

    SimpleFOCDebug::enable(&Serial);
    // Motor 1
    motor1.useMonitoring(Serial);
    driver1.voltage_power_supply = SUPPLY_VOLTAGE;
    driver1.pwm_frequency = PWM_FREQ;
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
    driver2.pwm_frequency = PWM_FREQ;
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

    motor1.voltage_sensor_align = 0.7;
#if CALIBRATION_MODE
    motor1.voltage_limit = 0.7; // Set when running initFOC for CALIBRATION ONLY to be safe
#else
    // Determined once with initFOC calibration
    motor1.sensor_direction = Direction::CCW;
    motor1.zero_electric_angle = 1.05;
#endif // CALIBRATION_MODE

    motor1.KV_rating = MOTOR_KV;
    motor1.init();

    motor1.disable();

#if DUAL_MOTOR
	// Motor 2
	motor2.current_limit = current_limit;

    motor2.PID_velocity.P = 0.4;
    motor2.PID_velocity.I = 1.0;
    motor2.PID_velocity.D = 0;
    motor2.PID_velocity.output_ramp = 1000;
    motor2.LPF_velocity.Tf = 0.4;

    motor2.voltage_sensor_align = 0.7;
#if CALIBRATION_MODE
    motor2.voltage_limit = 0.7; // Set when running initFOC for CALIBRATION ONLY to be safe
#else
    // Determined once with initFOC calibration
    motor2.sensor_direction = Direction::CCW;
    motor2.zero_electric_angle = 1.05;
#endif // CALIBRATION_MODE

    motor2.KV_rating = MOTOR_KV;
    motor2.init();

    motor2.disable();
#endif // DUAL_MOTOR

    // Loop
    while (1)
    {
#if CALIBRATION_MODE
        if(motor1.motor_status == FOCMotorStatus::motor_uncalibrated && isGDInitFinished())
        {
            Serial.println("CALIBRATING MOTOR 1");
            motor1.enable();
            motor1.initFOC();
            motor1.disable();
        }
#if DUAL_MOTOR
        if(motor2.motor_status == FOCMotorStatus::motor_uncalibrated && isGDInitFinished())
        {
            Serial.println("CALIBRATING MOTOR 2");
            motor2.enable();
            motor2.initFOC();
            motor2.disable();
        }
#endif // DUAL_MOTOR
#else
        if(wasMotorButtonPressed())
        {
            in_speed_mode = !in_speed_mode;
        }
        commander.run();
        checkEncoder();

#if SPEED_INCREASE_ENABLE
        float speed_increase = PISTON_SPEED_GAIN * getDirectionalVelocity();
        if(speed_increase > MAX_SPEED_INCREASE)
        {
            speed_increase = MAX_SPEED_INCREASE;
        }
        speed_target += speed_increase;
#endif // SPEED_INCREASE_ENABLE

        float speed1 = -speed_target;
        float speed2 = speed_target;

        // Spin motors in opposite directions when retracting (with timeout)
        if(getState() == RETRACTED_STATE)
        {
            if(reverse_spin_count < REVERSE_TIME_MS / TASK_PERIOD_MS)
            {
                reverse_spin_count++;
                speed1 = -speed1;
                speed2 = -speed2;
            }
            else
            {
                // Disable motors
                setRollerMotorEnable(false);
            }
        }
        else
        {
            reverse_spin_count = 0U;
        }

        motor1.current_limit = current_limit;
        motor1.PID_velocity.limit = motor1.current_limit;
        motor1.loopFOC();
        motor1.move(speed1);

#if DUAL_MOTOR
		motor2.current_limit = current_limit;
        motor2.PID_velocity.limit = motor2.current_limit;
		motor2.loopFOC();
		motor2.move(speed2);
#endif // DUAL_MOTOR
#endif // CALIBRATION_MODE
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
    bool enabled = motor1.enabled;
#if DUAL_MOTOR
    enabled |= motor2.enabled;
#endif // DUAL_MOTOR
    return enabled;
}

bool getInSpeedMode(void)
{
    return in_speed_mode;
}

float getRollerMotor1Angle(void)
{
    return sensor1.getAngle();
}

float getRollerMotor2Angle(void)
{
#if DUAL_MOTOR
    return sensor2.getAngle();
#else
    return 0.0f;
#endif // DUAL_MOTOR
}

float getRollerMotor1Speed(void)
{
    return motor1.shaft_velocity;
}

float getRollerMotor2Speed(void)
{
#if DUAL_MOTOR
    return motor2.shaft_velocity;
#else
    return 0.0f;
#endif // DUAL_MOTOR
}

float getRollerMotorSpeedTarget(void)
{
    return speed_target;
}

float getRollerMotor1Current(void)
{
    return fabs(motor1.current.d) + fabs(motor1.current.q);
}

float getRollerMotor2Current(void)
{
#if DUAL_MOTOR
    return fabs(motor2.current.d) + fabs(motor2.current.q);
#else
    return 0.0f;
#endif // DUAL_MOTOR
}

float getRollerMotorCurrentLimit(void)
{
    return current_limit;
}
