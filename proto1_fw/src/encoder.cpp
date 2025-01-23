#include <Arduino.h>
#include "encoder.h"

// Encoder center pin GND, left and right pins are pulled up
#define ENCODER_PIN1 PC6
#define ENCODER_PIN2 PC8

#define ENCODER_DEBOUNCE_MS (75U)

Rotary encoder = Rotary(ENCODER_PIN1, ENCODER_PIN2);

unsigned char dir_debounced = DIR_NONE; // after debouncing
uint32_t last_encoder_millis = 0U;

void encoderCallback() {
  if (millis() < last_encoder_millis)
  {
    last_encoder_millis = millis();
  }

  unsigned char temp_result = encoder.process();
  if ((temp_result == DIR_CW) && ((millis() - last_encoder_millis) > ENCODER_DEBOUNCE_MS)) {
    dir_debounced = temp_result;
    last_encoder_millis = millis();
  } else if ((temp_result == DIR_CCW) && ((millis() - last_encoder_millis) > ENCODER_DEBOUNCE_MS)) {
    dir_debounced = temp_result;
    last_encoder_millis = millis();
  }
}

/* Public functions */
void encoderSetup()
{
    pinMode(ENCODER_PIN1, INPUT_PULLUP);
    pinMode(ENCODER_PIN2, INPUT_PULLUP);
    attachInterrupt(ENCODER_PIN1, encoderCallback, CHANGE);
    attachInterrupt(ENCODER_PIN2, encoderCallback, CHANGE);
}

// Polling function, ideally get rid of this and only rely on interrupts
uint8_t getEncoderDirection()
{
    uint8_t direction = dir_debounced;
    dir_debounced = DIR_NONE; // Clear direction after reading it
    return direction;
}