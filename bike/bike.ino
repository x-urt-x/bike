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
	delay(1);

	server_setup();
	delay(100);
	connectOrStartAP();
}

unsigned long next;
void loop()
{
	server.handleClient();
	mpu_loop();
	btnTick();
	unsigned long now = millis();
	while ((int32_t)(now - next) >= 0)
	{
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetEuler(euler, &q);
		Data d;
		d.cranks_count = cranks_count / 2;
		d.wheel_count = wheel_count / 2;
		d.pos = getHall3();
		d.ang1 = euler[0] * TODEGR;
		d.ang2 = euler[1] * TODEGR;
		d.ang3 = euler[2] * TODEGR;
		addDataEntry(d);
		addDataEntry(d);
		addDataEntry(d);
		addDataEntry(d);
		addDataEntry(d);
		addDataEntry(d);
		addDataEntry(d);
		addDataEntry(d);
		addDataEntry(d);
		addDataEntry(d);

		cranks_count = 0;
		wheel_count = 0;
		next += SEND_TIME_MS;
	}
}