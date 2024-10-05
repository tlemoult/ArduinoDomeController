/*******************************************************************************
ArduinoDomeController
Azimuth control of an astronomical dome using Arduino


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
#include "pins.h"
#include "ble.h"
#include "serial_command.h"
#include "command.h"
#include "maxdome_protocol.h"
#include "shutterStatus.h"
#include "encoder.h"
#include "buttons.h"
#include "config.h"


void setup() {

  sCmd.addCommand(ABORT_CMD, 2, cmdAbort);
  sCmd.addCommand(HOME_CMD, 2, cmdHomeAzimuth);
  sCmd.addCommand(GOTO_CMD, 5, cmdGotoAzimuth);
  sCmd.addCommand(SHUTTER_CMD, 3, cmdShutterCommand);
  sCmd.addCommand(STATUS_CMD, 2, cmdStatus);
  sCmd.addCommand(SETPARK_CMD, 5, cmdSetPark);
  sCmd.addCommand(TICKS_CMD, 4, cmdSetTicks);
  sCmd.addCommand(ACK_CMD, 2, cmdAck);
  sCmd.addCommand(VBAT_CMD, 2, cmdVBat);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(HOME_SENSOR, INPUT);
  pinMode(BUTTON_CW, INPUT);
  pinMode(BUTTON_CCW, INPUT);
  pinMode(BUTTON_OPEN_SHUTTER, INPUT);
  pinMode(BUTTON_CLOSE_SHUTTER, INPUT);

  pinMode(MOTOR_JOG, OUTPUT);
  pinMode(MOTOR_CW, OUTPUT);
  pinMode(MOTOR_CCW, OUTPUT);
  digitalWrite(MOTOR_CW, LOW);
  digitalWrite(MOTOR_CCW, LOW);
  digitalWrite(MOTOR_JOG, LOW);

  pinMode(DEBUG_1, OUTPUT);
  digitalWrite(DEBUG_1, LOW);
  pinMode(DEBUG_2, OUTPUT);
  digitalWrite(DEBUG_2, HIGH);

  pinMode(ENCODER1, INPUT);
  pinMode(ENCODER2, INPUT);

  delay(100);

  attachInterrupt(digitalPinToInterrupt(ENCODER1), encoderISR, RISING);

  // RS232 port, PC controle Maxdome protocol
  Serial1.begin(BAUDRATE);

  // USB CDC, for logs
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  delay(10);   // for nrf52840 with native usb
  Serial.println("Starting Dome controler.");

  ble_setup();
}

void ping_slave_shutter(void) {
  static unsigned long last_ping = 0;
  unsigned long now_milli;

  now_milli = millis();

  if (now_milli-last_ping>PING_SHUTTER_MILLI) 
   {
    last_ping = now_milli;
    clientUart.println("ping");
  } 

}


void loop() {
  updateAzimuthFSM();
  read_buttons_rotate();
  read_buttons_open_close();
  check_home_sensor();
  ping_slave_shutter();
  sCmd.readSerial();
}
