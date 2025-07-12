#ifndef _btn_h
#define _btn_h

#include "arduino.h"
#include "config.h"
#include "log.h"
#include "mpu.h"
#include "server.h"
#include "mem.h"
#include "Header.h"

extern unsigned int btn_last;
extern unsigned int btn_pressed_time;
extern unsigned int btn_count;
extern unsigned int btn_press;

void btnTick();
void btn_setup();
void ICACHE_RAM_ATTR btn_func();
#endif