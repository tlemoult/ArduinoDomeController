#ifndef shutterStatus_h
#define shutterStatus_h

#define VBAT_FACTOR (3.3/1024.0)
#define VBAT_OFFSET 0.5

// MaxDome II shutter status
typedef enum {
    SS_CLOSED = 0,
    SS_OPENING,
    SS_OPEN,
    SS_CLOSING,
    SS_ABORTED,
    SS_ERROR
} ShutterStatus;

void shutterStatus_SetVbat(int adc);
float shutterStatus_GetVbat(void);

void shutterStatus_SetStatus(ShutterStatus state);
ShutterStatus shutterStatus_GetStatus(void);

#endif
