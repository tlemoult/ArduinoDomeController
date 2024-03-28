#ifndef h_max_dome_protocol_
#define h_max_dome_protocol_

// Message Destination
#define TO_MAXDOME 0x00
#define TO_COMPUTER 0x80

#define BAUDRATE 19200

// maxdome protocol Commands (use by serial_command.cpp)
#define ABORT_CMD 0x03    // Abort azimuth movement
#define HOME_CMD 0x04     // Move until 'home' position is detected
#define GOTO_CMD 0x05     // Go to azimuth position
#define SHUTTER_CMD 0x06  // Send a command to shutter
#define STATUS_CMD 0x07   // Retrieve status
#define TICKS_CMD 0x09    // Set the number of tick per revolution of the dome
#define ACK_CMD 0x0A      // ACK (?)
#define SETPARK_CMD 0x0B  // Set park coordinates and shutter closing policy
#define VBAT_CMD 0x0C     // Read shutter's battery voltage

#endif 