#include <EncButton.h>
#define EB_NO_COUNTER
#define EB_NO_BUFFER
#include <ESP8266WiFi.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "log.h"

#define GYRO_INTERRUPT_PIN D2 // use pin 15 on ESP8266
#define GYRO_SCL_PIN D3
#define GYRO_SDA_PIN D1
#define CRANKS_LOW_PIN D8
#define CRANKS_HIGH_PIN D7
#define WHEEL_LOW_PIN D6
#define WHEEL_HIGH_PIN D5
#define DERAILLEUR_PIN A0
#define BTN_PIN D4

#define SEND_TIME_MS 2000

#define POS1_2 350
#define POS2_3 430
#define POS3_4 485
#define POS4_5 505
#define POS5_6 519
#define POS6_7 523
#define POS7_8 527
#define POS8_9 530
#define POS9_10 533

#define BTN_LONG_TIME 200
#define BTN_TIMEOUT 1000
#define BTN_DEBOUNCE 50

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

float euler[3];

volatile bool mpuInterrupt = false;
void ICACHE_RAM_ATTR dmpDataReady() {
	mpuInterrupt = true;
}

unsigned int cranks_count = 0;
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

unsigned int wheel_count = 0;
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

unsigned int btn_last = 0;
unsigned int btn_pressed_time = 0;
unsigned int btn_count = 0;
unsigned int btn_press;

void ICACHE_RAM_ATTR btn_func()
{
	unsigned int now = millis();
	if (now - btn_last < BTN_DEBOUNCE) return;
	bool state = digitalRead(BTN_PIN);
	if (!state)
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

void mpu_setup()
{
	Wire.begin(GYRO_SDA_PIN, GYRO_SCL_PIN);
	Wire.setClock(400000);

	LOG_MPU("Initializing I2C devices...\n");
	mpu.initialize();
	pinMode(GYRO_INTERRUPT_PIN, INPUT);

	LOG_MPU("Testing device connections...\n");
	LOG_MPU(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

	LOG_MPU("Initializing DMP...\n");
	devStatus = mpu.dmpInitialize();

	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);

	if (devStatus == 0) {
		LOG_MPU("Enabling DMP...\n");
		mpu.setDMPEnabled(true);
		LOG_MPU("Enabling interrupt detection...\n");
		attachInterrupt(GYRO_INTERRUPT_PIN, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		LOG_MPU("DMP ready! Waiting for first interrupt...\n");
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		LOG_MPU("DMP Initialization failed (code %d)\n", devStatus);
	}
}

void setup()
{
#ifdef LOG_USB_ENABLE
	Serial.begin(115200);
	delay(2000);
#endif
	mpu_setup();
	pinMode(CRANKS_LOW_PIN, INPUT); //cant be pulled up
	attachInterrupt(CRANKS_LOW_PIN, cranks_low, RISING);
	pinMode(CRANKS_HIGH_PIN, INPUT_PULLUP);
	attachInterrupt(CRANKS_HIGH_PIN, cranks_high, FALLING);

	pinMode(WHEEL_LOW_PIN, INPUT_PULLUP);
	attachInterrupt(WHEEL_LOW_PIN, wheel_low, FALLING);
	pinMode(WHEEL_HIGH_PIN, INPUT_PULLUP);
	attachInterrupt(WHEEL_HIGH_PIN, wheel_high, FALLING);
	pinMode(DERAILLEUR_PIN, INPUT);

	pinMode(BTN_PIN, INPUT_PULLUP);
	attachInterrupt(BTN_PIN, btn_func, CHANGE);
}

void mpu_loop()
{
	if (!dmpReady) return;
	if (!mpuInterrupt && fifoCount < packetSize) return;
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();

	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		mpu.resetFIFO();
	}
	else if (mpuIntStatus & 0x02) {
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetEuler(euler, &q);
	}
}
void btnTick()
{
	if (millis() - btn_last < BTN_TIMEOUT) return;
	do{
		if (btn_count == 1 && !(btn_press & 0x1))
		{
			LOG_BTN("single short\n");
			break;
		}
		if (btn_count == 1 && (btn_press & 0x1))
		{
			LOG_BTN("single long\n");
			break;
		}
		if (btn_count == 2 && !(btn_press & 0x1) && !(btn_press & 0x2))
		{
			LOG_BTN("double short\n");
			break;
		}
		if (btn_count == 2 && (btn_press & 0x1) && (btn_press & 0x2))
		{
			LOG_BTN("double long\n");
			break;
		}
	} while (false);
	btn_count = 0;
	btn_press = 0;
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

unsigned long next;
void loop()
{
	mpu_loop();
	btnTick();
	unsigned long now = millis();
	//Serial.printf("%d\n", analogRead(DERAILLEUR_PIN));
	while ((int32_t)(now - next) >= 0)
	{
		//Serial.printf("cranks: %d\twheel: %d\ttransmission: %d\n", cranks_count / 2, wheel_count / 2, getHall3());
		cranks_count = 0;
		wheel_count = 0;
		next += SEND_TIME_MS;
	}
}
