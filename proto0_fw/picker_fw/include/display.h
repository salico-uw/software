#pragma once
#include <U8g2lib.h>
#include "util.h"

void dp_setup();

void dp_draw_num(float n, uint8_t line);

void dp_draw_mode(Mode_E mode);

void dp_fun();

void dp_clear();

void dp_send();