#include <Arduino.h>
#include <SimpleFOC.h>

/* Using STMF401RE Nucleo with IHM08M1 motor sheild, f80 
* AS5600 I2C angle sensor
* F80 1900KV drone motor
*/

#if SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH == true
#error "L6398 has low side active low"
#endif

#define UH PA8
#define UL PA7
#define VH PA9
#define VL PB0
#define WH PA10
#define WL PB1

#define STATUS_LED PC6
#define POT_PIN PA4 // pot ranges 740 - 785
#define SUPPLY_VOLTAGE (12U)

BLDCDriver6PWM driver = BLDCDriver6PWM(UH, UL, VH, VL, WH, WL);
BLDCMotor motor = BLDCMotor(7, 0.15);
MagneticSensorI2C angleSensor = MagneticSensorI2C(AS5600_I2C);

Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor,cmd); }

bool pressed = false;
bool prevButton = true;
void setup() {
  pinMode(USER_BTN, INPUT);
  pinMode(POT_PIN, INPUT); // Pot
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
  motor.linkSensor(&angleSensor);

  motor.velocity_limit = 100;
  motor.controller = MotionControlType::velocity;
  motor.current_limit = 2;

  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 2.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 1000;

  motor.LPF_velocity.Tf = 0.01;

  motor.sensor_direction = Direction::CW;
  motor.zero_electric_angle = 3.89; // Determined once with initFOC
  motor.KV_rating = 1900;
  motor.init();
  motor.initFOC();
}

double speed_target = 0.0;
void loop() {
  commander.run();
  motor.loopFOC();
  bool currButton = digitalRead(USER_BTN);
  if(currButton == false && prevButton == true)
  {
    pressed = true;
  }
  prevButton = currButton;
  
  if (pressed)
  {
    double max_speed = 20; // rad/s
    speed_target = (analogRead(POT_PIN) - 740)/45.0*(max_speed);
    if (speed_target > max_speed) {
      speed_target = max_speed;
    }
    else if (speed_target < -max_speed) {
      speed_target = -max_speed;
    }
    speed_target -= max_speed/2.0; // to get +-velocities
  }
  Serial.print("Curr angle: ");
  Serial.print(angleSensor.getSensorAngle());
  Serial.print(" speed: ");
  Serial.print(angleSensor.getVelocity());
  Serial.print(" target: ");
  Serial.println(speed_target);
  motor.move(speed_target);
}
