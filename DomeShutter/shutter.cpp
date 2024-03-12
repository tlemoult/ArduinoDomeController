/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the ArduinoDomeController project:
        https://github.com/juanmb/ArduinoDomeController

*******************************************************************/

#include <Arduino.h>
#include "shutter.h"
#include "motor.h"

#define DEFAULT_TIMEOUT 30000 // shutter timeout (in ms)

// Shutter constructor.
// motor: pointer to an instance of Motor
// sw1: Limit switch (closed)
// sw2: Limit switch (fully open)
// swInt: Interference switch
Shutter::Shutter(int closedSwitch, int openSwitch, unsigned long timeout)
{
    swClosed = closedSwitch;    // normally closed (1 if shutter is closed)
    swOpen = openSwitch;        // normally open (0 if shutter is fully open)
    runTimeout = timeout;
    nextAction = DO_NONE;
    motor_setup();
    initState();
}


void Shutter::initState()
{
    if (digitalRead(swClosed))
        state = ST_CLOSED;
    else if (!digitalRead(swOpen))
        state = ST_OPEN;
    else
        state = ST_ABORTED;
}


void Shutter::open() { nextAction = DO_OPEN; }
void Shutter::close() { nextAction = DO_CLOSE; }
void Shutter::abort() { nextAction = DO_ABORT; }

State Shutter::getState() { return state; }


// Shutter state machine
void Shutter::update()
{
    Action action;
    static unsigned long t0;

      noInterrupts();
      action = nextAction;
      nextAction = DO_NONE;
      interrupts();

    switch (state) {
    case ST_CLOSED:
        if (action == DO_OPEN) {
            t0 = millis();
            state = ST_OPENING;
        }
        break;
    case ST_OPEN:
        if (action == DO_CLOSE) {
            t0 = millis();
            state = ST_CLOSING;
        }
        break;
    case ST_ABORTED:
    case ST_ERROR:
        if (action == DO_OPEN) {
            t0 = millis();
            state = ST_OPENING;
        } else if (action == DO_CLOSE) {
            t0 = millis();
            state = ST_CLOSING;
        }
        break;
    case ST_OPENING:
        if (!digitalRead(swOpen)) {
            state = ST_OPEN;
            motor_stop();
        } else if (action == DO_ABORT || action == DO_CLOSE) {
            state = ST_ABORTED;
            motor_stop();
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor_stop();
        }
        break;
    case ST_CLOSING:
        if (digitalRead(swClosed)) {
            state = ST_CLOSED;
            motor_stop();
        } else if (action == DO_ABORT || action == DO_OPEN) {
            state = ST_ABORTED;
            motor_stop();
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor_stop();
        }
        break;
    }
}
