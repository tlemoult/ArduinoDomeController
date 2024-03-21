#include <Arduino.h>
#include "power.h"

unsigned long time_of_power_start;
bool is_power = 0;

void init_power(void) {
  pinMode(PIN_MOTOR_POWER, OUTPUT);
  set_power();
}

void set_power(void)
{
  time_of_power_start = millis();
  is_power = 1;
  digitalWrite(PIN_MOTOR_POWER,HIGH);
}

void clear_power(void)
{
  is_power = 0;
  digitalWrite(PIN_MOTOR_POWER,LOW);
}

void check_power_sleep(void)
{
  unsigned long current_time = millis();
  if ( (is_power==1) && ((current_time-time_of_power_start) > POWER_MILLIS_TIME_OUT ) )
  {
    clear_power();
  }

}