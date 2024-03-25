#include "shutterStatus.h"

static int shutterAdcVbatValue=0;
static ShutterStatus shutterStatusValue= SS_ABORTED;

void shutterStatus_SetVbat(int adcValue) {
  shutterAdcVbatValue = adcValue;
}

float shutterStatus_GetVbat(void) {
  return (float)shutterAdcVbatValue * VBAT_FACTOR + VBAT_OFFSET;
}

void shutterStatus_SetStatus(ShutterStatus state) {
  shutterStatusValue = state;
}

ShutterStatus shutterStatus_GetStatus(void) {
  return shutterStatusValue;
}
