#include "led.h"
#include "log.h"

#include "config.h"
#include "btn.h"
#include "hall_handlers.h"
#include "mpu.h"
#include "server.h"
#include "mem.h"

void setup()
{
#ifdef LOG_USB_ENABLE
	Serial.begin(115200);
	delay(2000);
	Serial.println();
#endif
	mpu_setup();
	hall_setup();
	btn_setup();
	findLastSector();

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
	if (!fs_setup()) return;

	WiFi.mode(WIFI_OFF);
	WiFi.forceSleepBegin();
	mpu_base();
	server_setup();
	//connectOrStartAP();
}

unsigned long next;
void loop()
{
	//Serial.printf("pos %d raw %d\n", getHall3(), analogRead(DERAILLEUR_PIN));
	server.handleClient();
	mpu_loop();
	btnTick();
	unsigned long now = millis();
	while ((int32_t)(now - next) >= 0)
	{
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetEuler(euler, &q);
		float angle = euler[2] - base_angle;
		if (angle > M_PI) angle -= 2 * M_PI;
		if (angle < M_PI) angle += 2 * M_PI;

		addDataEntry(pack(cranks_count / 2, wheel_count / 2, getHall3(), (int16_t)(angle * RAD_TO_INT16_SCALE)));

		cranks_count = 0;
		wheel_count = 0;
		next += SEND_TIME_MS;
	}
}