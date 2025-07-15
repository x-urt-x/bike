#ifndef _LED_h
#define _LED_h

#include "arduino.h"

struct Led_cfg 
{
	Led_cfg() :phase(0), last_time(0){}
	uint8_t pin;
	uint8_t phase;
	uint32_t last_time;
	uint16_t* timings;
	uint8_t timings_len;
	bool state;
	bool cyc;
};

bool led(Led_cfg* led_cfg, uint32_t now);

#endif