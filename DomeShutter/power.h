#include "arduino.h"

#define POWER_MILLIS_TIME_OUT (5000)
#define PIN_MOTOR_POWER 16

void init_power(void);
void set_power(void);
void clear_power(void);
void check_power_sleep(void);
