/*******************************************************************************
ArduinoDomeShutter
Shutter controller for an astronomical dome using Arduino


The MIT License

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*******************************************************************************/
#include <bluefruit.h>
#include "SerialCommand.h"
#include "shutter.h"
#include "ble.h"

// Pin definitions
#define LED_ERR 13                // error LED
#define PIN_LIMIT_SWITCH_CLOSE 7  // shutter closed switch (NC)
#define PIN_LIMIT_SWITCH_OPEN 15  // shutter open switch (NO)

#define VBAT_PIN A1  // 12V input battery voltage reading

#define PIN_BUTTON_OPEN 29
#define PIN_BUTTON_CLOSE 28

// Timeouts in ms
#define COMMAND_TIMEOUT 60000  // Max. time from last command
#define SHUTTER_TIMEOUT 75000  // Max. time the shutter takes to open/close
#define FLAP_TIMEOUT 15000     // Max. time the flap takes to open/close

Shutter shutter(PIN_LIMIT_SWITCH_CLOSE, PIN_LIMIT_SWITCH_OPEN, SHUTTER_TIMEOUT);

SerialCommand sCmd;
unsigned long lastCmdTime = 0;

extern BLEUart bleuart;

// Return dome status by combining shutter and flap statuses
State domeStatus() {
  State sst = shutter.getState();

  if (sst == ST_ERROR)
    return ST_ERROR;
  else if (sst == ST_OPENING)
    return ST_OPENING;
  else if (sst == ST_CLOSING)
    return ST_CLOSING;
  else if (sst == ST_OPEN)
    return ST_OPEN;
  else if (sst == ST_CLOSED)
    return ST_CLOSED;

  return ST_ABORTED;
}

void cmdOpenShutter() {
  lastCmdTime = millis();
  shutter.open();
}

void cmdOpenBoth() {
  lastCmdTime = millis();
  shutter.open();
}

void cmdClose() {
  lastCmdTime = millis();
  shutter.close();
}

void cmdAbort() {
  lastCmdTime = millis();
  shutter.abort();
}

void cmdExit() {
  lastCmdTime = 0;
  shutter.close();
}

void cmdStatus() {
  lastCmdTime = millis();
  State st = domeStatus();
  char status_char = '0'+st;

  Serial.print("cmdStatus(): st=");
  Serial.print(st);
  Serial.print("status_char = ");
  Serial.println(status_char);
  
  bleuart.write(status_char);
}

void cmdGetVBat() {
  lastCmdTime = millis();
  int val = analogRead(VBAT_PIN);
  char buffer[8];
  sprintf(buffer, "v%04d", val);
  bleuart.write(buffer);
}

void setup() {
  pinMode(LED_ERR, OUTPUT);
  pinMode(PIN_BUTTON_OPEN, INPUT);
  pinMode(PIN_BUTTON_CLOSE, INPUT);
  pinMode(PIN_LIMIT_SWITCH_CLOSE, INPUT);
  pinMode(PIN_LIMIT_SWITCH_OPEN, INPUT);

  motor_setup();

  Serial.begin(115200);  // for console message
  Serial.println("Start Dome shutter");

  shutter.initState();  // Constructor init did work, due to late limit switch digital pin configuration.

  delay(100);

  // Map serial commands to functions
  sCmd.addCommand("open", cmdOpenBoth);
  sCmd.addCommand("open1", cmdOpenShutter);
  sCmd.addCommand("close", cmdClose);
  sCmd.addCommand("abort", cmdAbort);
  sCmd.addCommand("exit", cmdExit);
  sCmd.addCommand("stat", cmdStatus);
  sCmd.addCommand("vbat", cmdGetVBat);

  digitalWrite(LED_ERR, HIGH);
  ble_setup();  // for BLE exchange with the master board
  delay(100);
  digitalWrite(LED_ERR, LOW);
}


void display_status(void) {

  static int divider_display = 0;

  delay(1);
  divider_display++;
  if (divider_display == 1000) {
    divider_display = 0;
    int state = shutter.getState();
    Serial.print("shutter state =");
    Serial.print(StateString[state]);

    Serial.print(",  limit switch state: ");
    Serial.print("CLOSE ");
    if (digitalRead(PIN_LIMIT_SWITCH_CLOSE)) {
      Serial.print("trigered, ");
    } else {
      Serial.print("Free, ");
    }

    Serial.print("OPEN ");
    if (digitalRead(PIN_LIMIT_SWITCH_OPEN)) {
      Serial.println("trigered");
    } else {
      Serial.println("Free");
    }
  }
}

void loop() {
  if (digitalRead(PIN_BUTTON_OPEN)) {
    Serial.print("Button open pressed");
    shutter.open();
  }

  if (digitalRead(PIN_BUTTON_CLOSE)) {
    Serial.print("Button close pressed");
    shutter.close();
  }

  display_status();

  // Close the dome if time from last command > COMMAND_TIMEOUT
  if ((lastCmdTime > 0) && ((millis() - lastCmdTime) > COMMAND_TIMEOUT)) {
    Serial.println("Timeout last command, try to close dome shutter");
    if (domeStatus() != ST_CLOSED) {
      lastCmdTime = 0;
      shutter.close();
    }
  }



  int err = (shutter.getState() == ST_ERROR);
  digitalWrite(LED_ERR, err);

  shutter.update();
  sCmd.readSerial(&bleuart);
}
