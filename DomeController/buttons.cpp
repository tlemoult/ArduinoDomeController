#include <Arduino.h>
#include "buttons.h"
#include "command.h"
#include "pins.h"

// rotate the dome when ccw or cw buttons are pressed
void read_buttons_rotate() {
  static int prev_cw_button = 0, prev_ccw_button = 0;
  int cw_button = digitalRead(BUTTON_CW);
  int ccw_button = digitalRead(BUTTON_CCW);

  if (cw_button != prev_cw_button) {
    if (cw_button) {
      digitalWrite(LED_BUILTIN, HIGH);
      moveAzimuth(DIR_CW, false);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      stopAzimuth();
    }
  } else if (ccw_button != prev_ccw_button) {
    if (ccw_button) {
      digitalWrite(LED_BUILTIN, HIGH);
      moveAzimuth(DIR_CCW, false);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      stopAzimuth();
    }
  }
  prev_cw_button = cw_button;
  prev_ccw_button = ccw_button;
}

// open close the dome when open_chutter or close_shutter are pressed
void read_buttons_open_close() {
  static int prev_open_button = 0, prev_close_button = 0;
  int open_button = digitalRead(BUTTON_OPEN_SHUTTER);
  int close_button = digitalRead(BUTTON_CLOSE_SHUTTER);
  uint8_t cmd[4];

  if (open_button != prev_open_button) {
    if (open_button) {
      cmd[3] = OPEN_SHUTTER;
      cmdShutterCommand(cmd);
    } else {
      cmd[3] = ABORT_SHUTTER;
      cmdShutterCommand(cmd);
    }
  } else if (close_button != prev_close_button) {
    if (close_button) {
      cmd[3] = CLOSE_SHUTTER;
      cmdShutterCommand(cmd);
    } else {
      cmd[3] = ABORT_SHUTTER;
      cmdShutterCommand(cmd);
    }
  }
  prev_open_button = open_button;
  prev_close_button = close_button;
}
