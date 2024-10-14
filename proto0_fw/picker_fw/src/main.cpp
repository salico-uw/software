#include <Arduino.h>
#include <SimpleFOC.h>
#include <U8g2lib.h>
#include "util.h"
#include "display.h"
#include "encoder.h"
#include "monitors.h"
#include <rotary.h>

/* Using STMF401RE Nucleo with IHM08M1 motor sheild,
* AS5600 I2C angle sensor
* SSD1306 128x32 OLED display
* rotary encoder with pushbutton for mode control
*/

#define PWM_MODE 6 // For 3pwm make sure the pwms are tied together
#if (SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH) == true && (PWM_MODE == 6)
#error "L6398 has low side active low"
#endif

#define CALIBRATION_MODE false

// Define specific motor we are using
#define MOTOR_AT3520

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

#define UH PA8
#define UL PA7
#define VH PA9
#define VL PB0
#define WH PA10
#define WL PB1

#define SPEED_INCREMENT 1.0f // rad/s
#define MAX_SPEED 50.0f // rad/s
#define CURRENT_INCREMENT 0.5f // rad/s
#define MAX_CURRENT 20.0f // amps

#define SUPPLY_VOLTAGE (12U)
#define ENCODER_BUTTON_PIN PB12
#define BUTTON_DEBOUNCE_MS (100U) // ms

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
MagneticSensorI2C angleSensor = MagneticSensorI2C(AS5600_I2C);

Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }

Mode_E menuMode = OFF_MODE;

bool prevButton = true; // active low
uint32_t last_millis = 0U;
bool wasButtonPressed(){
  bool pressed = false;
  // handle overflow case
  if (millis() < last_millis)
  {
    last_millis = millis();
  }
  bool currButton = digitalRead(ENCODER_BUTTON_PIN);
  if(currButton == false && prevButton == true && (millis() - last_millis) > BUTTON_DEBOUNCE_MS)
  {
    pressed = true;
    last_millis = millis();
  }
  prevButton = currButton;
  return pressed;
}

double speed_target = 0.0f; // rad/s
double current_limit = 5.0f; // amps
void checkEncoder() {
  if(menuMode == SPEED_MODE || menuMode == CURRENT_MODE)
  {
    // Initialize as speed mode
    double mode_increment = SPEED_INCREMENT;
    double upper_limit = MAX_SPEED;
    double lower_limit = -MAX_SPEED;
    double * target = &speed_target;
    if(menuMode == CURRENT_MODE)
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

void setup() {
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
  encoderSetup();
  Serial.begin(115200);
  while(!Serial) {}
  _delay(1000);
  Serial.println("Start");

  commander.add('M',onMotor,"Motor 1");

  SimpleFOCDebug::enable(&Serial);
  motor.useMonitoring(Serial);
  // 25kHz pwm default
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.init();
  motor.linkDriver(&driver);

  angleSensor.init();
  Wire.setClock(100000); // needed to slow down i2c speed to stop sensor from freezing often
  motor.linkSensor(&angleSensor);

  motor.velocity_limit = 100;
  motor.controller = MotionControlType::velocity;
  motor.current_limit = current_limit;

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 5.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01;

  motor.voltage_sensor_align = 0.5;
#if CALIBRATION_MODE
  motor.voltage_limit = 0.5; // Set when running initFOC for CALIBRATION ONLY to be safe
#else
  // Determined once with initFOC calibration
  motor.sensor_direction = Direction::CW;
  motor.zero_electric_angle = 3.5;
#endif // CALIBRATION_MODE

  motor.KV_rating = MOTOR_KV;
  motor.init();
 
  motor.initFOC();
  motor.disable();
  dp_setup();
}

void loop() {
  commander.run();
  checkEncoder();
  if (checkAllMonitors(&motor) == true)
  {
    motor.disable();
    menuMode = FAULT_MODE;
  }

  motor.loopFOC();
  if (wasButtonPressed())
  {
    switch(menuMode) {
      case OFF_MODE:
        menuMode = SPEED_MODE;
        motor.enable();
        break;
      case SPEED_MODE:
        menuMode = CURRENT_MODE;
        break;
      case CURRENT_MODE:
        menuMode = SPEED_MODE;
        break;
    }
  }

  motor.current_limit = current_limit;
  motor.PID_velocity.limit = motor.current_limit;

#if CALIBRATION_MODE == false
  // don't update display too fast
  if(millis() % 100 == 0)
  {
    Serial.print("mode: ");
    Serial.print(menuMode);
    Serial.print(" angle: ");
    Serial.print(angleSensor.getAngle());
    Serial.print(" speed: ");
    Serial.print(angleSensor.getVelocity()*motor.sensor_direction);
    Serial.print(" target speed: ");
    Serial.print(speed_target);
    Serial.print(" limit: ");
    Serial.println(motor.current_limit);

    dp_clear();
    double value = 0.0f;
    double target = 0.0f;
    if(menuMode == OFF_MODE || menuMode == SPEED_MODE) {
      value = angleSensor.getVelocity()*motor.sensor_direction;
      target = speed_target;
    } else if(menuMode == CURRENT_MODE) {
      value = motor.current.d + motor.current.q;
      target = motor.current_limit;
    }
    dp_draw_num(value, 0);
    dp_draw_num(target, 1);
    dp_draw_mode(menuMode);
    dp_send();
  }
  #endif //CALIBRATION_MODE
  motor.move(speed_target);
}
