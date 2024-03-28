#include <Arduino.h>
#include "pins.h"
#include "command.h"
#include "serial_command.h"
#include "maxdome_protocol.h"
#include "shutterStatus.h"
#include "config.h"
#include "ble.h"
#include "encoder.h"

bool park_on_shutter = false;
bool parking = false;
uint8_t current_dir = DIR_CW;   // Current azimuth movement direction
uint16_t park_pos = HOME_POSITION;          // Parking position
uint16_t target_pos = HOME_POSITION;        // Target dome position
uint16_t home_pos = HOME_POSITION;          // Home position

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
uint8_t getDirection(uint16_t current, uint16_t target) {
  if (target > current)
    return ((target - current) > ticks_per_turn / 2) ? DIR_CCW : DIR_CW;

  return ((current - target) > ticks_per_turn / 2) ? DIR_CW : DIR_CCW;
}

// Obtain the distance between two positons
uint16_t getDistance(uint16_t current, uint16_t target) {
  // obtain the absolute value of the difference
  uint16_t diff = (target > current) ? target - current : current - target;

  if (diff > ticks_per_turn / 2)
    return ticks_per_turn - diff;

  return diff;
}

void moveAzimuth(uint8_t dir, bool slow) {
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(MOTOR_JOG, slow);
  digitalWrite(MOTOR_CW, dir == DIR_CW);
  digitalWrite(MOTOR_CCW, dir != DIR_CW);
}

void stopAzimuth() {
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(MOTOR_JOG, LOW);
  digitalWrite(MOTOR_CW, LOW);
  digitalWrite(MOTOR_CCW, LOW);
}

void cmdAbort(uint8_t *cmd) {
#ifdef HAS_SHUTTER
  clientUart.println("abort");  // abort shutter movement
#endif

  az_event = EVT_ABORT;

  uint8_t resp[] = { START, 2, TO_COMPUTER | ABORT_CMD, 0x00 };
  sCmd.sendResponse(resp, 4);
}

void cmdHomeAzimuth(uint8_t *cmd) {
  if (!parking) {
    current_dir = getDirection(current_pos, 0);
    az_event = EVT_HOME;
  }

  uint8_t resp[] = { START, 3, TO_COMPUTER | HOME_CMD, 0x01, 0x00 };
  sCmd.sendResponse(resp, 5);
}

void cmdGotoAzimuth(uint8_t *cmd) {
  if (!parking) {
    current_dir = cmd[3];
    target_pos = bytesToInt(cmd + 4);
    az_event = EVT_GOTO;
  }

  uint8_t resp[] = { START, 3, TO_COMPUTER | GOTO_CMD, 0x01, 0x00 };
  sCmd.sendResponse(resp, 5);
}

void parkDome() {
  parking = true;
  target_pos = park_pos;
  az_event = EVT_GOTO;
}

void cmdShutterCommand(uint8_t *cmd) {
#ifdef HAS_SHUTTER
  switch (cmd[3]) {
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

  uint8_t resp[] = { START, 2, TO_COMPUTER | SHUTTER_CMD, 0x00 };
  sCmd.sendResponse(resp, 4);
}

void cmdStatus(uint8_t *cmd) {
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

  uint8_t sh_status = (uint8_t)shutterStatus_GetStatus();
  // uint8_t sh_status = (uint8_t)2;   // use for forcing status
  uint8_t resp[] = { START, 9, TO_COMPUTER | STATUS_CMD, sh_status, az_status,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  intToBytes(current_pos, resp + 5);
  intToBytes(home_pos, resp + 7);

  sCmd.sendResponse(resp, 11);
}



void cmdSetPark(uint8_t *cmd) {
  park_on_shutter = cmd[3];
  park_pos = bytesToInt(cmd + 4);

  //EEPROM.write(ADDR_PARK_ON_SHUTTER, park_on_shutter);
  //eepromWriteUint16(ADDR_PARK_POS, park_pos);

  uint8_t resp[] = { START, 2, TO_COMPUTER | SETPARK_CMD, 0x00 };
  sCmd.sendResponse(resp, 4);
}

void cmdSetTicks(uint8_t *cmd) {
  ticks_per_turn = bytesToInt(cmd + 3);
  //eepromWriteUint16(ADDR_TICKS_PER_TURN, ticks_per_turn);

  uint8_t resp[] = { START, 2, TO_COMPUTER | TICKS_CMD, 0x00 };
  sCmd.sendResponse(resp, 4);
}

void cmdVBat(uint8_t *cmd) {
  uint8_t resp[] = { START, 4, TO_COMPUTER | VBAT_CMD, 0x00, 0x00, 0x00 };

  int vbat = shutterStatus_GetVbat() * 100;
  intToBytes(vbat, resp + 3);

  sCmd.sendResponse(resp, 6);
}

void cmdAck(uint8_t *cmd) {
  uint8_t resp[] = { START, 3, TO_COMPUTER | ACK_CMD, 0x03, 0x00 };
  sCmd.sendResponse(resp, 5);
}

void updateAzimuthFSM() {
  static unsigned long t0;

  switch (state) {
    case ST_HOMING:
      if (az_event == EVT_ABORT) {
        parking = false;
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

    case ST_MOVING:
      if (az_event == EVT_ABORT) {
        parking = false;
        stopAzimuth();
        state = ST_IDLE;
      } else if (getDistance(current_pos, target_pos) < AZ_SLOW_RANGE) {
        moveAzimuth(current_dir, true);
        state = ST_APPROACHING;
      } else if (millis() - t0 > AZ_TIMEOUT) {
        stopAzimuth();
        state = ST_ERROR;
      }
      break;

    case ST_APPROACHING:
      if (az_event == EVT_ABORT) {
        parking = false;
        stopAzimuth();
        state = ST_IDLE;
      } else if (getDistance(current_pos, target_pos) < AZ_TOLERANCE) {
        stopAzimuth();

        // close shutter after parking
        if (parking) {
          parking = false;
#ifdef HAS_SHUTTER
          clientUart.println("close");
#endif
        }

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
        moveAzimuth(current_dir, false);
      } else if (az_event == EVT_GOTO) {
        t0 = millis();
        state = ST_MOVING;
        moveAzimuth(current_dir, false);
      }
      break;
  }
  az_event = EVT_NONE;
}
