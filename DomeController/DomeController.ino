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
#include "ble.h"
#include "serial_command.h"
#include "shutterStatus.h"


// Configuration
#define HAS_SHUTTER     // Uncomment if the shutter controller is available
#define USE_BUTTONS   // Uncomment if you want to move the dome with push buttons

#define AZ_TIMEOUT      30000   // Azimuth movement timeout (in ms)
#define AZ_TOLERANCE    4      	// Azimuth target tolerance in encoder ticks
#define AZ_SLOW_RANGE   16       // The motor will run at slower speed when the
                                // dome is at this number of ticks from the target

// pin definitions
#define ENCODER1 4      // Encoder
#define ENCODER2 5      // Encoder
#define HOME_SENSOR 16  // Home sensor (active high)
#define BUTTON_CW   12  // CW movement button (active high)
#define BUTTON_CCW  29  // CCW movement button (active high)
#define BUTTON_OPEN_SHUTTER 14    // OPEN shutter
#define BUTTON_CLOSE_SHUTTER  13  // CLOSE shutter

// motor pins 
#define MOTOR_CW 30      // Move motor clockwise
#define MOTOR_CCW 27     // Move motor counterclockwise
#define MOTOR_JOG 26     // Slow speed motor input (SHUTTER_OPEN Relay in our board)

// debug pin
#define DEBUG_1 28
#define DEBUG_2 20

// Message Destination
#define TO_MAXDOME  0x00
#define TO_COMPUTER 0x80

#define BAUDRATE 19200

// Commands
#define ABORT_CMD   0x03 // Abort azimuth movement
#define HOME_CMD    0x04 // Move until 'home' position is detected
#define GOTO_CMD    0x05 // Go to azimuth position
#define SHUTTER_CMD 0x06 // Send a command to shutter
#define STATUS_CMD  0x07 // Retrieve status
#define TICKS_CMD   0x09 // Set the number of tick per revolution of the dome
#define ACK_CMD     0x0A // ACK (?)
#define SETPARK_CMD 0x0B // Set park coordinates and shutter closing policy
#define VBAT_CMD    0x0C // Read shutter's battery voltage

// Shutter commands
#define OPEN_SHUTTER            0x01
#define OPEN_UPPER_ONLY_SHUTTER 0x02
#define CLOSE_SHUTTER           0x03
#define EXIT_SHUTTER            0x04 // Command sent to shutter on program exit
#define ABORT_SHUTTER           0x07

#define DIR_CW  0x01
#define DIR_CCW 0x02

// EEPROM addresses
#define ADDR_PARK_POS           0
#define ADDR_TICKS_PER_TURN     2
#define ADDR_PARK_ON_SHUTTER    4


enum MotorSpeed { MOTOR_STOP, MOTOR_SLOW, MOTOR_FAST };

enum AzimuthEvent {
    EVT_NONE,
    EVT_GOTO,
    EVT_HOME,
    EVT_ABORT
};

enum AzimuthStatus {
    ST_IDLE,
    ST_MOVING,
    ST_APPROACHING,
    ST_HOMING,
    ST_ERROR,
};

// MaxDome II azimuth status
enum MDAzimuthStatus {
    AS_IDLE = 1,
    AS_MOVING_CW,
    AS_MOVING_CCW,
    AS_IDLE2,
    AS_ERROR
};


extern BLEClientUart clientUart; // bleuart client
SerialCommand sCmd;


bool park_on_shutter = false;
bool home_reached = false;
bool parking = false;
uint8_t current_dir = DIR_CW;   // Current azimuth movement direction
uint16_t park_pos = 0;          // Parking position
uint16_t current_pos = 0;       // Current dome position
uint16_t target_pos = 0;        // Target dome position
uint16_t home_pos = 0;          // Home position
uint16_t ticks_per_turn = 600;  // Encoder ticks per dome revolution
const uint16_t encoder_pre_divider = 5;

AzimuthStatus state = ST_IDLE;
AzimuthEvent az_event = EVT_NONE;


// Convert two bytes to a uint16_t value (big endian)
uint16_t bytesToInt(uint8_t *data)
{
    uint16_t value1 = data[1];
    uint16_t value2 = data[0];
    return (value1 & 0xff) | ((value2 << 8) & 0xff00);
}

// Convert a uint16_t value to bytes (big endian)
void intToBytes(uint16_t value, uint8_t *data)
{
    data[1] = value & 0xff;
    data[0] = (value >> 8) & 0xff;
}

// Read a uint16_t value from EEPROM (little endian)
/*
uint16_t eepromReadUint16(int address)
{
    uint16_t value1 = EEPROM.read(address);
    uint16_t value2 = EEPROM.read(address + 1);
    return (value1 & 0xff) | ((value2 << 8) & 0xff00);
}

// Read a uint16_t value from EEPROM (little endian)
void eepromWriteUint16(int address, uint16_t value)
{
    uint8_t value1 = value & 0xff;
    uint8_t value2 = (value >> 8) & 0xff;
    EEPROM.write(address, value1);
    EEPROM.write(address + 1, value2);
}
*/

// Obtain the direction of the shortest path between two positons
uint8_t getDirection(uint16_t current, uint16_t target)
{
    if (target > current)
        return ((target - current) > ticks_per_turn / 2) ? DIR_CCW : DIR_CW;

    return ((current - target) > ticks_per_turn / 2) ? DIR_CW : DIR_CCW;
}

// Obtain the distance between two positons
uint16_t getDistance(uint16_t current, uint16_t target)
{
    // obtain the absolute value of the difference
    uint16_t diff = (target > current) ?  target - current : current - target;

    if (diff > ticks_per_turn / 2)
        return ticks_per_turn - diff;

    return diff;
}

inline void moveAzimuth(uint8_t dir, bool slow)
{
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(MOTOR_JOG, slow);
    digitalWrite(MOTOR_CW, dir == DIR_CW);
    digitalWrite(MOTOR_CCW, dir != DIR_CW);
}

inline void stopAzimuth()
{
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(MOTOR_JOG, LOW);
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
}

void cmdAbort(uint8_t *cmd)
{
#ifdef HAS_SHUTTER
    clientUart.println("abort");  // abort shutter movement
#endif

    az_event = EVT_ABORT;

    uint8_t resp[] = {START, 2, TO_COMPUTER | ABORT_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdHomeAzimuth(uint8_t *cmd)
{
    if (!parking) {
        current_dir = getDirection(current_pos, 0);
        az_event = EVT_HOME;
    }

    uint8_t resp[] = {START, 3, TO_COMPUTER | HOME_CMD, 0x01, 0x00};
    sCmd.sendResponse(resp, 5);
}

void cmdGotoAzimuth(uint8_t *cmd)
{
    if (!parking) {
        current_dir = cmd[3];
        target_pos = bytesToInt(cmd + 4);
        az_event = EVT_GOTO;
    }

    uint8_t resp[] = {START, 3, TO_COMPUTER | GOTO_CMD, 0x01, 0x00};
    sCmd.sendResponse(resp, 5);
}

void parkDome()
{
    parking = true;
    target_pos = park_pos;
    az_event = EVT_GOTO;
}

void cmdShutterCommand(uint8_t *cmd)
{
#ifdef HAS_SHUTTER
    switch(cmd[3]) {
    case OPEN_SHUTTER:
        clientUart.println("open");
        break;
    case OPEN_UPPER_ONLY_SHUTTER:
        clientUart.println("open1");
        break;
    case CLOSE_SHUTTER:
        if (park_on_shutter)
            parkDome();
        else
            clientUart.println("close");
        break;
    case EXIT_SHUTTER:
        clientUart.println("exit");
        break;
    case ABORT_SHUTTER:
        clientUart.println("abort");
        break;
    }
#endif

    uint8_t resp[] = {START, 2, TO_COMPUTER | SHUTTER_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdStatus(uint8_t *cmd)
{
    MDAzimuthStatus az_status;
    if (state == ST_IDLE) {
        az_status = AS_IDLE;
    }
    else if (state == ST_ERROR) {
        az_status = AS_ERROR;
    }
    else {
        if (current_dir == DIR_CW)
            az_status = AS_MOVING_CW;
        else
            az_status = AS_MOVING_CCW;
    }

    uint8_t sh_status = (uint8_t)shutterStatus_GetStatus();
    // uint8_t sh_status = (uint8_t)2;   // use for forcing status
    uint8_t resp[] = {START, 9, TO_COMPUTER | STATUS_CMD, sh_status, az_status,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                     };

    intToBytes(current_pos, resp + 5);
    intToBytes(home_pos, resp + 7);

    sCmd.sendResponse(resp, 11);
}



void cmdSetPark(uint8_t *cmd)
{
    park_on_shutter = cmd[3];
    park_pos = bytesToInt(cmd + 4);

    //EEPROM.write(ADDR_PARK_ON_SHUTTER, park_on_shutter);
    //eepromWriteUint16(ADDR_PARK_POS, park_pos);

    uint8_t resp[] = {START, 2, TO_COMPUTER | SETPARK_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdSetTicks(uint8_t *cmd)
{
    ticks_per_turn = bytesToInt(cmd + 3);
    //eepromWriteUint16(ADDR_TICKS_PER_TURN, ticks_per_turn);

    uint8_t resp[] = {START, 2, TO_COMPUTER | TICKS_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdVBat(uint8_t *cmd)
{
    uint8_t resp[] = {START, 4, TO_COMPUTER | VBAT_CMD, 0x00, 0x00, 0x00};

    int vbat = shutterStatus_GetVbat() * 100;
    intToBytes(vbat, resp + 3);

    sCmd.sendResponse(resp, 6);
}

void cmdAck(uint8_t *cmd)
{
    uint8_t resp[] = {START, 3, TO_COMPUTER | ACK_CMD, 0x03, 0x00};
    sCmd.sendResponse(resp, 5);
}

void updateAzimuthFSM()
{
    static unsigned long t0;

    switch(state) {
    case ST_HOMING:
        if (az_event == EVT_ABORT) {
            parking = false;
            stopAzimuth();
            state = ST_IDLE;
        }
        else if (home_reached) {
            stopAzimuth();
            state = ST_IDLE;
            home_reached = false;
        }
        else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_MOVING:
        if (az_event == EVT_ABORT) {
            parking = false;
            stopAzimuth();
            state = ST_IDLE;
        }
        else if (getDistance(current_pos, target_pos) < AZ_SLOW_RANGE) {
            moveAzimuth(current_dir, true);
            state = ST_APPROACHING;
        }
        else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_APPROACHING:
        if (az_event == EVT_ABORT) {
            parking = false;
            stopAzimuth();
            state = ST_IDLE;
        }
        else if (getDistance(current_pos, target_pos) < AZ_TOLERANCE) {
            stopAzimuth();

            // close shutter after parking
            if (parking) {
                parking = false;
#ifdef HAS_SHUTTER
                clientUart.println("close");
#endif
            }

            state = ST_IDLE;
        }
        else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_ERROR:
    case ST_IDLE:
        if (az_event == EVT_HOME) {
            t0 = millis();
            state = ST_HOMING;
            moveAzimuth(current_dir, false);
        }
        else if (az_event == EVT_GOTO) {
            t0 = millis();
            state = ST_MOVING;
            moveAzimuth(current_dir, false);
        }
        break;
    }
    az_event = EVT_NONE;
}

// Encoder interrupt service routine
void encoderISR()
{
  static uint16_t encoder_fine_pos = 0;
 
    if(digitalRead(ENCODER2)) {
      // decrement direction
        if (encoder_fine_pos == 0)
          {
            encoder_fine_pos = encoder_pre_divider - 1;
            // decrement the current_pos
            if (current_pos == 0)
              current_pos = ticks_per_turn - 1;
            else
              current_pos--;
          }
        else
            encoder_fine_pos--;
    }
    else 
    {
        // increment direction
        if (encoder_fine_pos >= encoder_pre_divider - 1)
          {
            encoder_fine_pos = 0;
            // increment the current_pos
            if (current_pos >= ticks_per_turn - 1)
              current_pos = 0;
            else
              current_pos++;
          }
        else
            encoder_fine_pos++;
    }

}

void setup()
{

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
    pinMode(BUTTON_OPEN_SHUTTER,INPUT);
    pinMode(BUTTON_CLOSE_SHUTTER,INPUT);

    pinMode(MOTOR_JOG, OUTPUT);
    pinMode(MOTOR_CW, OUTPUT);
    pinMode(MOTOR_CCW, OUTPUT);
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
    digitalWrite(MOTOR_JOG,LOW);

    pinMode(DEBUG_1,OUTPUT);
    digitalWrite(DEBUG_1,LOW);
    pinMode(DEBUG_2,OUTPUT);
    digitalWrite(DEBUG_2,HIGH);

    pinMode(ENCODER1,INPUT);
    pinMode(ENCODER2,INPUT);

    delay(100);

    attachInterrupt(digitalPinToInterrupt(ENCODER1), encoderISR, RISING);

    /*
    park_pos = eepromReadUint16(ADDR_PARK_POS);
    park_on_shutter = EEPROM.read(ADDR_PARK_ON_SHUTTER);
    ticks_per_turn = eepromReadUint16(ADDR_TICKS_PER_TURN);
*/

    Serial.begin(19200);

    ble_setup();

}

// rotate the dome when ccw or cw buttons are pressed
void read_buttons_rotate()
{
    static int prev_cw_button = 0, prev_ccw_button = 0;
    int cw_button = digitalRead(BUTTON_CW);
    int ccw_button = digitalRead(BUTTON_CCW);

    if (cw_button != prev_cw_button) {
        if (cw_button) {
            digitalWrite(LED_BUILTIN, HIGH);
	    moveAzimuth(DIR_CW, false);
        }
        else {
            digitalWrite(LED_BUILTIN, LOW);
	    stopAzimuth();
        }
    }
    else if (ccw_button != prev_ccw_button) {
        if (ccw_button) {
            digitalWrite(LED_BUILTIN, HIGH);
	    moveAzimuth(DIR_CCW, false);
        }
        else {
            digitalWrite(LED_BUILTIN, LOW);
	    stopAzimuth();
        }
    }
    prev_cw_button = cw_button;
    prev_ccw_button = ccw_button;
}

// open close the dome when open_chutter or close_shutter are pressed
void read_buttons_open_close()
{
  static int prev_open_button = 0, prev_close_button = 0;
  int open_button = digitalRead(BUTTON_OPEN_SHUTTER);
  int close_button = digitalRead(BUTTON_CLOSE_SHUTTER);
  uint8_t cmd[4];

  if (open_button != prev_open_button) {
        if (open_button) {
            cmd[3] = OPEN_SHUTTER;
            cmdShutterCommand(cmd);
        }
        else {
            cmd[3] = ABORT_SHUTTER;
            cmdShutterCommand(cmd);
        }
    }
    else if (close_button != prev_close_button) {
        if (close_button) {
            cmd[3] = CLOSE_SHUTTER;
            cmdShutterCommand(cmd);
        }
        else {
            cmd[3] = ABORT_SHUTTER;
            cmdShutterCommand(cmd);
        }
    }
    prev_open_button = open_button;
    prev_close_button = close_button;
}




void send_periodic_status(void) 
{
  
}

void loop()
{
    sCmd.readSerial();
    updateAzimuthFSM();
    read_buttons_rotate();
    read_buttons_open_close();

    // store detected home position
    if (digitalRead(HOME_SENSOR)) {
        if (state == ST_HOMING) {
            home_pos = 0;
            current_pos = 0;
            home_reached = true;
        }
        else
            home_pos = current_pos;
    }
}
