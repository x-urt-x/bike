#include "hall_handlers.h"

uint16_t cranks_count = 0;
uint16_t wheel_count = 0;

bool cranks_side = false; //false - lastc call was low. true - last call was high
void ICACHE_RAM_ATTR cranks_low()
{
	if (!cranks_side)
	{
		cranks_count++;
		cranks_side = true;
	}
}
void ICACHE_RAM_ATTR cranks_high()
{
	if (cranks_side)
	{
		cranks_count++;
		cranks_side = false;
	}
}

bool wheel_side = false; //false - last call was low. true - last call was high
void ICACHE_RAM_ATTR wheel_low()
{
	if (!wheel_side)
	{
		wheel_count++;
		wheel_side = true;
	}
}
void ICACHE_RAM_ATTR wheel_high()
{
	if (wheel_side)
	{
		wheel_count++;
		wheel_side = false;
	}
}

char getHall3()
{
	int pos = analogRead(DERAILLEUR_PIN);
	if (pos < POS1_2) return 1;
	if (pos < POS2_3) return 2;
	if (pos < POS3_4) return 3;
	if (pos < POS4_5) return 4;
	if (pos < POS5_6) return 5;
	if (pos < POS6_7) return 6;
	if (pos < POS7_8) return 7;
	if (pos < POS8_9) return 8;
	if (pos < POS9_10) return 9;
	return 10;
}

void hall_setup()
{
	pinMode(CRANKS_LOW_PIN, INPUT); //cant be pulled up
	attachInterrupt(CRANKS_LOW_PIN, cranks_low, RISING);
	pinMode(CRANKS_HIGH_PIN, INPUT_PULLUP);
	attachInterrupt(CRANKS_HIGH_PIN, cranks_high, FALLING);

	pinMode(WHEEL_LOW_PIN, INPUT_PULLUP);
	attachInterrupt(WHEEL_LOW_PIN, wheel_low, FALLING);
	pinMode(WHEEL_HIGH_PIN, INPUT_PULLUP);
	attachInterrupt(WHEEL_HIGH_PIN, wheel_high, FALLING);
	pinMode(DERAILLEUR_PIN, INPUT);
}
