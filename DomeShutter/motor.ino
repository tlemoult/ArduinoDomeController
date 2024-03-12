#include "motor.h"

void motor_open(void)
{
  digitalWrite(PIN_MOTOR_OPEN, HIGH);
  digitalWrite(PIN_MOTOR_CLOSE, LOW);
}

void motor_close(void)
{
  digitalWrite(PIN_MOTOR_OPEN, LOW);
  digitalWrite(PIN_MOTOR_CLOSE, HIGH);
}

void motor_stop(void)
{
  digitalWrite(PIN_MOTOR_OPEN, LOW);
  digitalWrite(PIN_MOTOR_CLOSE, LOW);
}

void motor_setup(void)
{
  pinMode(PIN_MOTOR_OPEN, OUTPUT);
  pinMode(PIN_MOTOR_CLOSE, OUTPUT);
  motor_stop();
}
