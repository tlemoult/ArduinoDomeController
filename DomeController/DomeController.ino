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

#include <EEPROM.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include "serial_command.h"


#define BAUDRATE 19200

// pin definitions
#define HC12_RX 4       // Recieve Pin on HC12
#define HC12_TX 5       // Transmit Pin on HC12
#define ENCODER1 2      // Azimuth encoder
#define ENCODER2 3      // Azimuth encoder
#define MOTOR_JOG 7     // Motor jog mode (low speed)
#define MOTOR_CW 8      // Move motor clockwise
#define MOTOR_CCW 9     // Move motor counterclockwise
#define HALL_PROBE 10   // Home probe

// Message Destination
#define TO_MAXDOME  0x00
#define TO_COMPUTER 0x80

// Commands
#define ABORT_CMD   0x03 // Abort azimuth movement
#define HOME_CMD    0x04 // Move until 'home' position is detected
#define GOTO_CMD    0x05 // Go to azimuth position
#define SHUTTER_CMD 0x06 // Send a command to shutter
#define STATUS_CMD  0x07 // Retrieve status
#define TICKS_CMD   0x09 // Set the number of tick per revolution of the dome
#define ACK_CMD     0x0A // ACK (?)
#define SETPARK_CMD 0x0B // Set park coordinates and shutter closing policy

// Shutter commands
#define OPEN_SHUTTER            0x01
#define OPEN_UPPER_ONLY_SHUTTER 0x02
#define CLOSE_SHUTTER           0x03
#define EXIT_SHUTTER            0x04 // Command sent to shutter on program exit
#define ABORT_SHUTTER           0x07

#define DIR_CW  0x01
#define DIR_CCW 0x01

#define AZ_TOLERANCE    4       // Azimuth target tolerance in encoder ticks
#define AZ_SLOW_RANGE   16      //
#define AZ_TIMEOUT      30000   // Azimuth movement timeout (in ms)

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
    ST_GOING,
    ST_GOING_SLOW,
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

// MaxDome II shutter status
enum ShutterStatus {
    SS_CLOSED = 0,
    SS_OPENING,
    SS_OPEN,
    SS_CLOSING,
    SS_ABORTED,
    SS_ERROR
};


SoftwareSerial HC12(HC12_TX, HC12_RX); // Create Software Serial Port
SerialCommand sCmd;

bool park_on_shutter = false;
bool home_reached = false;
uint8_t current_dir = DIR_CW;   // Current azimuth movement direction
uint16_t park_pos = 0;          // Parking position
uint16_t current_pos = 0;       // Current dome position
uint16_t target_pos = 0;        // Target dome position
uint16_t home_pos = 0;          // Home position
uint16_t ticks_per_turn = 360;  // Encoder ticks per dome revolution
AzimuthStatus state = ST_IDLE;
AzimuthEvent az_event = EVT_NONE;


// Convert two bytes to a uint16_t value (big endian)
uint16_t bytesToInt(uint8_t *data) {
    uint16_t value1 = data[1];
    uint16_t value2 = data[0];
    return (value1 & 0xff) | ((value2 << 8) & 0xff00);
}

// Convert a uint16_t value to bytes (big endian)
void intToBytes(uint16_t value, uint8_t *data) {
    data[1] = value & 0xff;
    data[0] = (value >> 8) & 0xff;
}

// Read a uint16_t value from EEPROM (little endian)
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


// Obtain the direction of the shortest path between two positons
uint8_t getDirection(uint16_t current, uint16_t target)
{
    if (target > current)
        return (target - current > ticks_per_turn/2) ? DIR_CCW : DIR_CW;

    return (current - target > ticks_per_turn/2) ? DIR_CW : DIR_CCW;
}

// Obtain the distance between two positons
uint16_t getDistance(uint16_t current, uint16_t target)
{
    // obtain the absolute value of the difference
    uint16_t diff = (target > current) ?  target - current : current - target;

    if (diff > ticks_per_turn/2)
        return ticks_per_turn - diff;

    return diff;
}

inline void moveAzimuth(uint8_t dir)
{
    digitalWrite(MOTOR_CW, dir == DIR_CW);
    digitalWrite(MOTOR_CCW, dir != DIR_CW);
}

inline void stopAzimuth()
{
    digitalWrite(MOTOR_JOG, LOW);
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
}

inline void setJog(bool enable)
{
    digitalWrite(MOTOR_JOG, enable);
}

ShutterStatus getShutterStatus() {
    ShutterStatus st = SS_ERROR;

    HC12.flush();
    for (int i=0; i<4; i++) {
        HC12.println("stat");
        delay(100);

        char c = 0;
        while (HC12.available() > 0) {
            c = HC12.read();
        }

        if (c >= '0' && c <= ('0' + SS_ERROR)) {
            st = (ShutterStatus)(c - '0');
            break;
        }
    }

    return st;
}


void cmdAbortAzimuth(uint8_t *cmd)
{
    az_event = EVT_ABORT;

    uint8_t resp[] = {START, 2, TO_COMPUTER | ABORT_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdHomeAzimuth(uint8_t *cmd)
{
    current_dir = getDirection(current_pos, home_pos);
    az_event = EVT_HOME;

    uint8_t resp[] = {START, 2, TO_COMPUTER | HOME_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdGotoAzimuth(uint8_t *cmd)
{
    current_dir = cmd[3];
    target_pos = bytesToInt(cmd + 4);
    az_event = EVT_GOTO;

    uint8_t resp[] = {START, 2, TO_COMPUTER | GOTO_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdShutterCommand(uint8_t *cmd)
{
    switch(cmd[3]) {
    case OPEN_SHUTTER:
        HC12.println("open");
        break;
    case OPEN_UPPER_ONLY_SHUTTER:
        HC12.println("open1");
        break;
    case CLOSE_SHUTTER:
        HC12.println("close");
        break;
    case EXIT_SHUTTER:
        HC12.println("exit");
        break;
    case ABORT_SHUTTER:
        HC12.println("abort");
        break;
    }

    uint8_t resp[] = {START, 2, TO_COMPUTER | SHUTTER_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdStatus(uint8_t *cmd)
{
    MDAzimuthStatus az_status;
    if (state == ST_IDLE) {
        az_status = AS_IDLE;
    } else if (state == ST_ERROR) {
        az_status = AS_ERROR;
    } else {
        if (current_dir == DIR_CW)
            az_status = AS_MOVING_CW;
        else
            az_status = AS_MOVING_CCW;
    }

    uint8_t sh_status = (uint8_t)getShutterStatus();
    uint8_t resp[] = {START, 8, TO_COMPUTER | STATUS_CMD, sh_status, az_status,
        0x00, 0x00, 0x00, 0x00, 0x00};

    intToBytes(current_pos, resp + 5);
    intToBytes(home_pos, resp + 7);

    sCmd.sendResponse(resp, 10);
}

void cmdSetPark(uint8_t *cmd)
{
    park_on_shutter = cmd[3];
    park_pos = bytesToInt(cmd + 4);

    EEPROM.write(ADDR_PARK_ON_SHUTTER, park_on_shutter);
    eepromWriteUint16(ADDR_PARK_POS, park_pos);

    uint8_t resp[] = {START, 2, TO_COMPUTER | SETPARK_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdSetTicks(uint8_t *cmd)
{
    ticks_per_turn = bytesToInt(cmd + 3);
    eepromWriteUint16(ADDR_TICKS_PER_TURN, ticks_per_turn);

    uint8_t resp[] = {START, 2, TO_COMPUTER | TICKS_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdAck(uint8_t *cmd)
{
    uint8_t resp[] = {START, 2, TO_COMPUTER | ACK_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void updateAzimuthFSM()
{
    static unsigned long t0;

    switch(state) {
    case ST_HOMING:
        if (az_event == EVT_ABORT) {
            stopAzimuth();
            state = ST_IDLE;
        } else if (home_reached) {
            stopAzimuth();
            state = ST_IDLE;
            home_reached = false;
        } else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_GOING:
        if (az_event == EVT_ABORT) {
            stopAzimuth();
            state = ST_IDLE;
        } else if (getDistance(current_pos, target_pos) < AZ_SLOW_RANGE) {
            moveAzimuth(current_dir);
            setJog(true);
            state = ST_GOING_SLOW;
        } else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_GOING_SLOW:
        if (az_event == EVT_ABORT) {
            stopAzimuth();
            state = ST_IDLE;
        } else if (getDistance(current_pos, target_pos) < AZ_TOLERANCE) {
            stopAzimuth();
            state = ST_IDLE;
        } else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_ERROR:
    case ST_IDLE:
        if (az_event == EVT_HOME) {
            t0 = millis();
            state = ST_HOMING;
            moveAzimuth(current_dir);
        } else if (az_event == EVT_GOTO) {
            t0 = millis();
            state = ST_GOING;
            moveAzimuth(current_dir);
        }
        break;
    }
    az_event = EVT_NONE;
}

// Encoder interrupt service routine
void encoderISR()
{
    if(digitalRead(ENCODER1) == digitalRead(ENCODER2)) {
        if (current_pos == 0)
            current_pos = ticks_per_turn - 1;
        else
            current_pos--;
    } else {
        if (current_pos >= ticks_per_turn)
            current_pos = 0;
        else
            current_pos++;
    }

    // store detected home position
    if (!digitalRead(HALL_PROBE)) {
        if (state == ST_HOMING) {
            home_pos = 0;
            current_pos = 0;
            home_reached = true;
        } else
            home_pos = current_pos;
    }
}

void setup()
{
    wdt_disable();
    wdt_enable(WDTO_2S);

    sCmd.addCommand(ABORT_CMD, 2, cmdAbortAzimuth);
    sCmd.addCommand(HOME_CMD, 2, cmdHomeAzimuth);
    sCmd.addCommand(GOTO_CMD, 5, cmdGotoAzimuth);
    sCmd.addCommand(SHUTTER_CMD, 3, cmdShutterCommand);
    sCmd.addCommand(STATUS_CMD, 2, cmdStatus);
    sCmd.addCommand(SETPARK_CMD, 5, cmdSetPark);
    sCmd.addCommand(TICKS_CMD, 4, cmdSetTicks);
    sCmd.addCommand(ACK_CMD, 2, cmdAck);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(MOTOR_JOG, OUTPUT);
    pinMode(MOTOR_CW, OUTPUT);
    pinMode(MOTOR_CCW, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER1), encoderISR, CHANGE);

    park_pos = eepromReadUint16(ADDR_PARK_POS);
    park_on_shutter = EEPROM.read(ADDR_PARK_ON_SHUTTER);
    ticks_per_turn = eepromReadUint16(ADDR_TICKS_PER_TURN);

    Serial.begin(19200);
    HC12.begin(9600);   // Open serial port to HC12

}


void loop()
{
    sCmd.readSerial();
    updateAzimuthFSM();
    wdt_reset();
}