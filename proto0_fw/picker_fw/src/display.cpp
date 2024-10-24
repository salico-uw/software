#include "display.h"
#include "util.h"

#include <Arduino.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);

#define DP_YPOS_0 15
#define DP_YPOS_1 31

void dp_setup() {
  u8g2.begin();
  u8g2.setFont(
      u8g2_font_luRS14_tf);  // choose a suitable font at
                             // https://github.com/olikraus/u8g2/wiki/fntlistall
}

void dp_draw_num(float n, uint8_t line) {
  String str = "EE";

  // supports between -99999.99 to 99999.99
  float NUM_MIN = -9999.99;
  float NUM_MAX = 9999.99;
  int NUM_DP = 2;
  int ypos = line == 0 ? DP_YPOS_0 : DP_YPOS_1;

  if (n < NUM_MIN) {
    str = "-EE";
  } else if (n > NUM_MAX) {
    str = "+EE";
  } else {
    str = String(n, NUM_DP);
  }

  u8g2.drawStr(1, ypos, str.c_str());
};

void dp_draw_state(State_E state) {
  String state_char = "O";
  switch (state) {
    case OFF_STATE:
      state_char = "X";
      break;
    case SPEED_STATE:
      state_char = "S";
      break;
    case CURRENT_STATE:
      state_char = "C";
      break;
    case FAULT_STATE:
      state_char = "F";
      break;
  }
  u8g2.drawStr(100, DP_YPOS_0, state_char.c_str());
}

// hehe
void dp_fun() {
  u8g2.drawStr(1, DP_YPOS_0, "gm!");
  u8g2.drawStr(1, DP_YPOS_1, "ur cute :3");
};

void dp_clear() { u8g2.clearBuffer(); }

void dp_send() { u8g2.sendBuffer(); }