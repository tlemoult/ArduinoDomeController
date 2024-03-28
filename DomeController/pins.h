#ifndef _h_pins_
#define _h_pins_

// pin definitions
#define ENCODER1 4               // Encoder
#define ENCODER2 5               // Encoder
#define HOME_SENSOR 16           // Home sensor (active high)
#define BUTTON_CW 12             // CW movement button (active high)
#define BUTTON_CCW 29            // CCW movement button (active high)
#define BUTTON_OPEN_SHUTTER 14   // OPEN shutter
#define BUTTON_CLOSE_SHUTTER 13  // CLOSE shutter

// motor pins
#define MOTOR_CW 30   // Move motor clockwise
#define MOTOR_CCW 27  // Move motor counterclockwise
#define MOTOR_JOG 26  // Slow speed motor input (SHUTTER_OPEN Relay in our board)

// debug pin
#define DEBUG_1 28
#define DEBUG_2 20

#endif