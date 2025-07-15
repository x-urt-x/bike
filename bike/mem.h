#ifndef _mem_h
#define _mem_h

#include "arduino.h"
#include "config.h"
#include "log.h"

extern alignas(4) uint32_t flash_buffer[];

extern uint32_t start_sector;
extern uint32_t start_sector_number;
extern uint16_t start_index;
extern uint32_t current_sector;
extern uint32_t curent_sector_number;
extern uint16_t buffer_index;



void initBuffer();
void flushBuffer();
void flushPartBuffer();
void addDataEntry(const uint32_t entry);
void findLastSector();
void getSector(int offset, uint32_t* buffer);
void loadSector(uint32_t pos);
void memReset();
uint32_t pack(uint8_t crank, uint8_t wheel, uint8_t pos, int16_t angle);

#endif