#include <Arduino.h>
#include "pins.h"
#include "config.h"
#include "command.h"
#include "encoder.h"

const uint16_t encoder_pre_divider = ENCODER_PRE_DIV;
uint16_t current_pos = 0;                          // Current dome position
uint16_t ticks_per_turn = ENCODER_TICKS_PER_TURN;  // Encoder ticks per dome revolution
bool home_reached = false;

// Encoder interrupt service routine
void encoderISR() {
  static uint16_t encoder_fine_pos = 0;

  if (digitalRead(ENCODER2)) {
    // decrement direction
    if (encoder_fine_pos == 0) {
      encoder_fine_pos = encoder_pre_divider - 1;
      // decrement the current_pos
      if (current_pos == 0)
        current_pos = ticks_per_turn - 1;
      else
        current_pos--;
    } else
      encoder_fine_pos--;
  } else {
    // increment direction
    if (encoder_fine_pos >= encoder_pre_divider - 1) {
      encoder_fine_pos = 0;
      // increment the current_pos
      if (current_pos >= ticks_per_turn - 1)
        current_pos = 0;
      else
        current_pos++;
    } else
      encoder_fine_pos++;
  }
}

void check_home_sensor(void) {

  // store detected home position
  if (digitalRead(HOME_SENSOR)) {
    if (state == ST_HOMING) {
      home_pos = 0;
      current_pos = 0;
      home_reached = true;
    } else
      home_pos = current_pos;
  }
}
