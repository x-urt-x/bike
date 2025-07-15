#include "btn.h"

unsigned int btn_last = 0;
unsigned int btn_pressed_time = 0;
unsigned int btn_count = 0;
unsigned int btn_press = 0;
bool btn_state = true;

void btnTick()
{
	if (!btn_state || (millis() - btn_last < BTN_TIMEOUT)) return;
	do {
		Led_cfg led_cfg;
		led_cfg.cyc = false;
		if (btn_count == 1 && !(btn_press & 0x1))
		{
			LOG_BTN("single short\n");
			addDataEntry(0xEEEE'EEEE);
			uint16_t led_t[] = { 300, 50 };
			led_cfg.timings = led_t;
			led_cfg.timings_len = 2;
			while (!led(&led_cfg, millis()));
			break;
		}
		if (btn_count == 1 && (btn_press & 0x1))
		{
			LOG_BTN("single long\n");
			connectOrStartAP();
			Led_cfg led_cfg;
			led_cfg.cyc = false;
			uint16_t led_t[] = { 300, 200};
			led_cfg.timings = led_t;
			led_cfg.timings_len = 2;
			while (!led(&led_cfg, millis()));
			break;
		}
		if (btn_count == 2 && !(btn_press & 0x1) && !(btn_press & 0x2))
		{
			LOG_BTN("double short\n");
			turnOffWiFi();
			uint16_t led_t[] = { 300, 50, 80, 50 };
			led_cfg.timings = led_t;
			led_cfg.timings_len = 4;
			while (!led(&led_cfg, millis()));
			break;
		}
		if (btn_count == 2 && (btn_press & 0x1) && (btn_press & 0x2))
		{
			LOG_BTN("double long\n");
			flushPartBuffer();
			uint16_t led_t[] = { 300, 200, 80, 200 };
			led_cfg.timings = led_t;
			led_cfg.timings_len = 4;
			while (!led(&led_cfg, millis()));
			break;
		}
		if (btn_count == 2 && (btn_press & 0x1) && !(btn_press & 0x2))
		{
			LOG_BTN("long short\n");
			mpu_base();
			uint16_t led_t[] = { 300, 200, 80, 50};
			led_cfg.timings = led_t;
			led_cfg.timings_len = 4;
			while (!led(&led_cfg, millis()));
			break;
		}
		if (btn_count == 3 && (btn_press & 0x1) && (btn_press & 0x2) && (btn_press & 0x4))
		{
			LOG_BTN("triple long\n");
			memReset();
			uint16_t led_t[] = { 300, 200, 80, 200, 80, 200 };
			led_cfg.timings = led_t;
			led_cfg.timings_len = 6;
			while (!led(&led_cfg, millis()));
			break;
		}
	} while (false);
	btn_count = 0;
	btn_press = 0;
}

void btn_setup()
{
	pinMode(BTN_PIN, INPUT_PULLUP);
	attachInterrupt(BTN_PIN, btn_func, CHANGE);
}

void ICACHE_RAM_ATTR btn_func()
{
	btn_state = digitalRead(BTN_PIN);
	unsigned int now = millis();
	if (now - btn_last < BTN_DEBOUNCE) return;
	if (!btn_state)
	{
		btn_pressed_time = now;
	}
	else
	{
		if (btn_count < sizeof(btn_press) * 8)
		{
			if (now - btn_pressed_time > BTN_LONG_TIME)
				btn_press |= 1 << btn_count;
			btn_count++;
		}
	}
	btn_last = now;
}