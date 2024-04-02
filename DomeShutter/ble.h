#ifndef _BLE_H   
#define _BLE_H   

#include <bluefruit.h>

// function prototype for ble.cpp
void ble_setup(void);
void ble_startAdv(void);

extern BLEUart bleuart;

#endif // _BLE_H 