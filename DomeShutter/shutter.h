/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the ArduinoDomeController project:
        https://github.com/juanmb/ArduinoDomeController

*******************************************************************/

#ifndef _shutter_h_
#define _shutter_h_


// Lid states
enum State {
    ST_CLOSED,
    ST_OPENING,
    ST_OPEN,
    ST_CLOSING,
    ST_ABORTED,
    ST_ERROR,
};

const char StateString[6][8] =
{ "CLOSED",
  "OPENING",
  "OPEN",
  "CLOSING",
  "ABORTED",
  "ERROR"
};

enum Action {
    DO_NONE,
    DO_OPEN,
    DO_CLOSE,
    DO_ABORT,
};

// Define a pointer to a function for checking shutter/flap interference
typedef bool (*interFn)(State st);


class Shutter {
public:
    Shutter( int closedSwitch, int openSwitch, unsigned long timeout);
    void open();
    void close();
    void abort();
    void update();
    void initState();
    State getState();
private:    
    State state;
    Action nextAction;
    unsigned long runTimeout;
    int swClosed;
    int swOpen;
};

extern Shutter shutter;

#endif
