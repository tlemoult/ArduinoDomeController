#ifndef _h_pins_
#define _h_pins_

// pin definitions
#define ENCODER1 16               // Encoder
#define ENCODER2 17               // Encoder
#define HOME_SENSOR  13          // Home sensor (active high)
#define BUTTON_CW 26             // CW movement button (active high)
#define BUTTON_CCW 19            // CCW movement button (active high)
#define BUTTON_OPEN_SHUTTER 24   // OPEN shutter
#define BUTTON_CLOSE_SHUTTER 25  // CLOSE shutter

// motor pins
#define MOTOR_CW 6   // Move motor clockwise
#define MOTOR_CCW 5  // Move motor counterclockwise
#define MOTOR_JOG 23  // Slow speed motor input (SHUTTER_OPEN Relay in our board)

// debug pin
#define DEBUG_1 18
#define DEBUG_2 2

#endif