#include <Arduino.h>
#include <SimpleFOC.h>
// #include <rotary.h>

/* Using STMF401RE Nucleo with IHM08M1 motor sheild,
* AS5600 I2C angle sensor
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

#define STATUS_LED PC6
#define POT_PIN PA4 // pot ranges 740 - 785
#define ENCODER_PIN1 PA5
#define ENCODER_PIN2 PA6

#define ENCODER_SPEED_INCREMENT 1.0f // rad/s
#define MAX_SPEED 10.0f // rad/s

#define SUPPLY_VOLTAGE (12U)
#define BUTTON_DEBOUNCE_MS (300U) // ms
#if PWM_MODE == 6
BLDCDriver6PWM driver = BLDCDriver6PWM(UH, UL, VH, VL, WH, WL);
#elif PWM_MODE == 3
BLDCDriver3PWM driver = BLDCDriver3PWM(UH, VH, WH);
#endif // PWM_MODE
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE);
MagneticSensorI2C angleSensor = MagneticSensorI2C(AS5600_I2C);

// Rotary encoder = Rotary(ENCODER_PIN1, ENCODER_PIN2);

Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }

bool prevButton = true; // active low
uint32_t last_millis = 0U;
bool wasButtonPressed(){
  bool pressed = false;
  // handle overflow case (should never happen unless button unpressed for 50 days) for uint subtraction
  if (millis() < last_millis)
  {
    last_millis = millis();
  }
  bool currButton = digitalRead(USER_BTN);
  if(currButton == false && prevButton == true && (millis() - last_millis) > BUTTON_DEBOUNCE_MS)
  {
    pressed = true;
    last_millis = millis();
  }
  prevButton = currButton;
  return pressed;
}

double speed_target = 0.0f; // rad/s
// void check_encoder() {
//   // unsigned char result = encoder.process();
//   if (result == DIR_CW && speed_target <= MAX_SPEED) {
//     speed_target += ENCODER_SPEED_INCREMENT;
//     Serial.println(speed_target);
//   } else if (result == DIR_CCW && speed_target >= -MAX_SPEED) {
//     speed_target -= ENCODER_SPEED_INCREMENT;
//     Serial.println(speed_target);
//   }
// }

void setup() {
  pinMode(USER_BTN, INPUT);
  pinMode(POT_PIN, INPUT); // Pot
  // attachInterrupt(ENCODER_PIN1, check_encoder, CHANGE);
  // attachInterrupt(ENCODER_PIN2, check_encoder, CHANGE);
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
  motor.current_limit = 15;

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
  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 4.42;
#endif // CALIBRATION_MODE

  motor.KV_rating = MOTOR_KV;
  motor.init();
 
  motor.initFOC();
  motor.disable();
}

void loop() {
  commander.run();
  motor.loopFOC();

  if (wasButtonPressed())
  {
    if(motor.enabled == false) {
      motor.enable();
    }
    else {
      motor.disable();
    }
      // double max_speed = 20; // rad/s
      // speed_target = (analogRead(POT_PIN) - 740)/45.0*(max_speed);
      // if (speed_target > max_speed) {
      //   speed_target = max_speed;
      // }
      // else if (speed_target < -max_speed) {
      //   speed_target = -max_speed;
      // }
      // speed_target -= max_speed/2.0; // to get +-velocities
    // }
  }
  
  double max_current = 15; // A
  double current_limit = (analogRead(POT_PIN) - 740)/45.0*(max_current);
  if (current_limit > max_current) {
    current_limit = max_current;
  }
  else if (current_limit < 0) {
    current_limit = 0;
  }
  motor.current_limit = current_limit;
  motor.PID_velocity.limit = motor.current_limit;

  if(millis() % 10 == 0) // don't print too fast
  {
    Serial.print("angle: ");
    Serial.print(angleSensor.getSensorAngle());
    Serial.print(" speed: ");
    Serial.print(angleSensor.getVelocity()*motor.sensor_direction);
    Serial.print(" target speed: ");
    Serial.print(speed_target);
    Serial.print(" limit: ");
    Serial.println(motor.current_limit);
  }
  motor.move(8);
}
