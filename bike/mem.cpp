#include "mem.h"

alignas(4) uint32_t flash_buffer[SECTOR_SIZE / 4];

uint32_t start_sector = 0;
uint32_t start_sector_number;
uint16_t start_index = 0;
uint32_t current_sector = 0;
uint32_t curent_sector_number = 0;
uint16_t buffer_index = 0;

void initBuffer() 
{
	curent_sector_number += 1;
	memset((uint8_t*)flash_buffer, 0xFF, SECTOR_SIZE);
	memcpy((uint8_t*)flash_buffer, &curent_sector_number, sizeof(curent_sector_number));
	LOG_SECTOR("init buffer with num %d\n", curent_sector_number);
}

void flushBuffer() 
{
	if (buffer_index == 0) return;


	//memcpy((uint8_t*)flash_buffer + SECTOR_SIZE - 4, &SECTOR_KEY, sizeof(SECTOR_KEY)); //add deadbeef
	flash_buffer[SECTOR_SIZE / 4 - 1] = SECTOR_KEY;
	
	uint32_t addr = USER_FLASH_START + current_sector * SECTOR_SIZE;

	uint32_t sector = (addr - FLASH_BASE) / SECTOR_SIZE;

	ESP.flashEraseSector(sector);
	ESP.flashWrite(addr - FLASH_BASE, flash_buffer, SECTOR_SIZE);

	//LOG_SECTOR("flushBuffer pos %d adr %d num %d\n", current_sector, addr - FLASH_BASE, flash_buffer[0]);
	//for (int i = 0; i < 8; i++) {
	//	LOG_SECTOR(" 0x%08X ", flash_buffer[i]);
	//}
	//LOG_SECTOR("\n");
	//for (int i = SECTOR_SIZE / 4 - 1 - 16; i < SECTOR_SIZE / 4; i++) {
	//	LOG_SECTOR(" 0x%08X ", flash_buffer[i]);
	//}
	//LOG_SECTOR("\n");
	//uint32_t key = 0;
	//ESP.flashRead(addr - FLASH_BASE + SECTOR_SIZE - 4, &key, 4);
	//LOG_SECTOR("mem key 0x%08X adr %d\n", key, addr - FLASH_BASE + SECTOR_SIZE - 4);
	current_sector++;
	buffer_index = 0;
	if (current_sector >= MAX_SECTORS) current_sector = 0;
}

void flushPartBuffer() 
{
	if (buffer_index == 0) return;

	memcpy((uint8_t*)flash_buffer + 4 + buffer_index * DATA_ENTRY_SIZE, &SECTOR_KEY, sizeof(SECTOR_KEY)); //add deadbeef

	uint32_t addr = USER_FLASH_START + current_sector * SECTOR_SIZE;
	uint32_t sector = (addr - FLASH_BASE) / SECTOR_SIZE;
	ESP.flashEraseSector(sector);
	ESP.flashWrite(addr - FLASH_BASE, flash_buffer, SECTOR_SIZE);
	LOG_SECTOR("flushPartBuffer elements %d pos %d adr %d num %d\n", buffer_index, current_sector, addr - FLASH_BASE, flash_buffer[0]);
}

void addDataEntry(const uint32_t entry)
{
	//LOG_SECTOR("Add data on %d index\n", buffer_index);
	if (buffer_index >= MAX_DATA_ENTRIES)
	{
		flushBuffer();
		initBuffer();
	}
	memcpy((uint8_t*)flash_buffer + 4 + buffer_index * DATA_ENTRY_SIZE, &entry, DATA_ENTRY_SIZE);
	buffer_index++;
}

void findLastSector()
{
	LOG_SECTOR("start last sector search\n");
	uint32_t last = 0, cur = 0, pos = 0, key = 0;
	ESP.flashRead(USER_FLASH_START - FLASH_BASE, &last, sizeof(last));
	last--;
	for (; pos < MAX_SECTORS; pos++)
	{
		ESP.flashRead(USER_FLASH_START + pos * SECTOR_SIZE - FLASH_BASE, &cur, sizeof(cur));
		if (cur != last+1 || cur == 0xFFFF'FFFF) //old or raw sector
		{
			LOG_SECTOR("old or raw sector on %d [last: %d cur: %d]\n", pos, last, cur);
			curent_sector_number = last;
			initBuffer();
			break; 
		}
		uint32_t adr = USER_FLASH_START + pos * SECTOR_SIZE + SECTOR_SIZE - 4 - FLASH_BASE;
		ESP.flashRead(adr, &key, sizeof(key));
		LOG_SECTOR("read key 0x%08X pos = %d adr %d\n", key, pos, adr);
		if (key != SECTOR_KEY) //not full sector
		{
			LOG_SECTOR("not full sector on %d key = 0x%08X\n", pos, key);
			curent_sector_number = last;
			initBuffer();
			loadSector(pos);
			break;
		}
		last = cur;
	}
	start_sector = pos;
	start_sector_number = curent_sector_number;
	current_sector = pos;
}

void loadSector(uint32_t pos)
{
	LOG_SECTOR("load sector %d\n", pos);
	buffer_index = 0;
	const uint32_t start = USER_FLASH_START + pos * SECTOR_SIZE - FLASH_BASE + 4;
	for (uint16_t entry_pos = 0; entry_pos < MAX_DATA_ENTRIES; entry_pos++)
	{
		uint32_t data;
		ESP.flashRead(start + entry_pos * DATA_ENTRY_SIZE, (uint8_t*) & data, DATA_ENTRY_SIZE);
		if (((uint32_t*)&data)[0] == SECTOR_KEY) //no data struct. end of array
		{
			LOG_SECTOR("found end on %d\n", entry_pos);
			break;
		}
		memcpy((uint8_t*)flash_buffer + 4 + buffer_index * DATA_ENTRY_SIZE, &data, sizeof(data));
		buffer_index++;
	}
	start_index = buffer_index;
}

uint32_t pack(uint8_t crank, uint8_t wheel, uint8_t pos, int16_t angle)
{
	return  ((uint32_t)(crank & 0x3F)) |  // 6 bits
		((uint32_t)(wheel & 0x3F) << 6) |  // 6 bits
		((uint32_t)(pos & 0x0F) << 12) |  // 4 bits
		((uint32_t)angle << 16);             // 16 bits
}

void getSector(int offset, uint32_t* buffer)
{
	if (!buffer) return;

	if (offset == 0)
	{
		// return the RAM buffer with appended  key
		LOG_SECTOR("RAM buffer  cur index = %d\n", buffer_index);
		memcpy((uint8_t*)buffer, (uint8_t*)flash_buffer, 4 + buffer_index * DATA_ENTRY_SIZE);
		memcpy((uint8_t*)buffer + 4 + buffer_index * DATA_ENTRY_SIZE, &SECTOR_KEY, sizeof(SECTOR_KEY));
		return;
	}
	uint32_t sector;
	if (offset < 0)
	{
		offset = -offset;
		if (offset >= MAX_SECTORS) offset = MAX_SECTORS - 1;
		sector = current_sector + MAX_SECTORS - offset;
	}
	else
	{
		sector = offset-1;
	}
	while (sector >= MAX_SECTORS) sector -= MAX_SECTORS;
	uint32_t addr = USER_FLASH_START + sector * SECTOR_SIZE;
	uint32_t flash_offset = addr - FLASH_BASE;
	ESP.flashRead(flash_offset, buffer, SECTOR_SIZE);
	//uint32_t key = 0;
	//ESP.flashRead(flash_offset + SECTOR_SIZE - 4, &key, 4);
	//LOG_SECTOR("mem key 0x%08X adr %d\n", key, flash_offset + SECTOR_SIZE - 4);
	//LOG_SECTOR("read sector pos %d adr %d num %d\n", sector, flash_offset, buffer[0]);
	//for (int i = 0; i < 8; i++) {
	//	LOG_SECTOR(" 0x%08X ", buffer[i]);
	//	if(i % 16 == 0 && i != 0) LOG_SECTOR("\n");
	//}
	//LOG_SECTOR("\n");
	//for (int i = SECTOR_SIZE / 4 - 1 - 16; i < SECTOR_SIZE / 4; i++) {
	//	LOG_SECTOR(" 0x%08X ", buffer[i]);
	//}
	//LOG_SECTOR("\n");
}

void memReset()
{
	start_sector = 0;
	current_sector = 0;
	curent_sector_number = 0;
	buffer_index = 0;
	initBuffer();
}