#ifndef _hall_handlers_h
#define _hall_handlers_h
#include "arduino.h"
#include "config.h"
#include "log.h"

extern uint16_t cranks_count;
extern uint16_t wheel_count;

void ICACHE_RAM_ATTR cranks_low();
void ICACHE_RAM_ATTR cranks_high();
void ICACHE_RAM_ATTR wheel_low();
void ICACHE_RAM_ATTR wheel_high();
char getHall3();
void hall_setup();
#endif