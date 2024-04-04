#include <Arduino.h>
#include "motor.h"
#include "power.h"

void motor_open(void)
{
  digitalWrite(PIN_MOTOR_OPEN, HIGH);
  digitalWrite(PIN_MOTOR_CLOSE, LOW);
  set_power();
}

void motor_close(void)
{
  digitalWrite(PIN_MOTOR_OPEN, LOW);
  digitalWrite(PIN_MOTOR_CLOSE, HIGH);
  set_power();
}

void motor_stop(void)
{
  digitalWrite(PIN_MOTOR_OPEN, LOW);
  digitalWrite(PIN_MOTOR_CLOSE, LOW);
  set_power();
}

void motor_setup(void)
{
  pinMode(PIN_MOTOR_OPEN, OUTPUT);
  pinMode(PIN_MOTOR_CLOSE, OUTPUT);
  digitalWrite(PIN_MOTOR_OPEN, LOW);
  digitalWrite(PIN_MOTOR_CLOSE, LOW);
  init_power();
}
