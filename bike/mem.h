#ifndef _mem_h
#define _mem_h

#include "arduino.h"
#include "config.h"
#include "log.h"

struct Data {
	uint16_t cranks_count;
	uint16_t wheel_count;
	char pos;
	float ang1, ang2, ang3;
} __attribute__((packed));

extern alignas(4) uint32_t flash_buffer[];

extern uint32_t start_sector;
extern uint32_t current_sector;
extern uint32_t curent_sector_number;
extern uint16_t buffer_index;
extern uint32_t buffer_start_time;


void initBuffer();
void flushBuffer();
void flushPartBuffer();
void addDataEntry(const Data& entry);
void findLastSector();
void getSector(int offset, uint32_t* buffer);
void loadSector(uint32_t pos);
void memReset();
#endif