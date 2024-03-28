#ifndef _h_encoder_
#define _h_encoder_

extern uint16_t current_pos;       // Current dome position
extern uint16_t ticks_per_turn;
extern bool home_reached;

void encoderISR();
void check_home_sensor(void);

#endif