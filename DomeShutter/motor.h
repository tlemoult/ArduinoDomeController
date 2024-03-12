#ifndef _motor_h_
#define _motor_h_

#define PIN_MOTOR_OPEN  26
#define PIN_MOTOR_CLOSE 25

void motor_open(void);
void motor_close(void);
void motor_stop(void);
void motor_setup(void);

#endif