#include "led.h"

bool led(Led_cfg* led_cfg, uint32_t now)
{
	if (led_cfg->phase >= led_cfg->timings_len)
		if (led_cfg->cyc)
			led_cfg->phase = 0;
		else
			return true;
	if (led_cfg->timings[led_cfg->phase] <= now - led_cfg->last_time)
	{
		led_cfg->state = !led_cfg->state;
		digitalWrite(led_cfg->pin, led_cfg->state);
		led_cfg->phase += 1;
		led_cfg->last_time = now;
	}
	return false;
}