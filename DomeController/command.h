#ifndef _h_command_
#define _h_command_

typedef enum { MOTOR_STOP,
               MOTOR_SLOW,
               MOTOR_FAST } MotorSpeed;

typedef enum {
  EVT_NONE,
  EVT_GOTO,
  EVT_HOME,
  EVT_ABORT
} AzimuthEvent;

typedef enum {
  ST_IDLE,
  ST_MOVING,
  ST_APPROACHING,
  ST_HOMING,
  ST_ERROR,
} AzimuthStatus;

// MaxDome II azimuth status
typedef enum {
  AS_IDLE = 1,
  AS_MOVING_CW,
  AS_MOVING_CCW,
  AS_IDLE2,
  AS_ERROR
} MDAzimuthStatus;


// Shutter commands
#define OPEN_SHUTTER 0x01
#define OPEN_UPPER_ONLY_SHUTTER 0x02
#define CLOSE_SHUTTER 0x03
#define EXIT_SHUTTER 0x04  // Command sent to shutter on program exit
#define ABORT_SHUTTER 0x07

#define DIR_CW 0x01
#define DIR_CCW 0x02

void cmdAbort(uint8_t *cmd);
void cmdHomeAzimuth(uint8_t *cmd);
void cmdGotoAzimuth(uint8_t *cmd);
void cmdShutterCommand(uint8_t *cmd);
void cmdStatus(uint8_t *cmd);
void cmdSetPark(uint8_t *cmd);
void cmdSetTicks(uint8_t *cmd);
void cmdVBat(uint8_t *cmd);
void cmdAck(uint8_t *cmd);
void updateAzimuthFSM();
void moveAzimuth(uint8_t dir, bool slow);
void stopAzimuth();

extern AzimuthStatus state;
extern uint16_t home_pos;          // Home position
#endif