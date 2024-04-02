#include <Arduino.h>
#include "shutter.h"
#include "ble.h"
#include "pins.h"
#include "command.h"

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
  char bleMsg[3];
  char status_char;

  lastCmdTime = millis();
  State st = shutter.getState();
  status_char = '0' + st;


  bleMsg[0] = 's';
  bleMsg[1] = status_char;
  bleMsg[2] = 0;

  Serial.print("cmdStatus(): st=");
  Serial.print(st);
  Serial.print("status_char = ");
  Serial.println(status_char);

  bleuart.write(bleMsg);
}

void cmdGetVBat() {
  char buffer[8];
  int val;

  lastCmdTime = millis();
  val = analogRead(VBAT_PIN);

  sprintf(buffer, "v%04d", val);
  Serial.print("cmdGetVbat(): vbat=");
  Serial.println(val);

  bleuart.write(buffer);
}

void cmdPing() {
  lastCmdTime = millis();
  Serial.println("cmdPing()");
}
