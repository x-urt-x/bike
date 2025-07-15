#include "mpu.h"

MPU6050 mpu;
float base_angle = 0;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;

float euler[3];

volatile bool mpuInterrupt = false;
void ICACHE_RAM_ATTR dmpDataReady() {
	mpuInterrupt = true;
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

bool mpu_loop()
{
	if (!dmpReady) return false;
	if (!mpuInterrupt && fifoCount < packetSize) return false;
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
	return true;
}

void mpu_base()
{
	Led_cfg led_cfg;
	led_cfg.cyc = true;
	uint16_t mpu_led[] = { 80, 80, 80, 1000 };
	led_cfg.timings = mpu_led;
	led_cfg.timings_len = 4;

	uint32_t start = millis();
	uint32_t now = start;
	while (now < start + 20000)
	{
		mpu_loop();
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetEuler(euler, &q);
		delay(10);
		LOG_MPU("base_angle unstable %f\n", euler[2]);
		led(&led_cfg, now);
		now = millis();
	}
	LOG_MPU("\n");
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetEuler(euler, &q);
	base_angle = euler[2];
	LOG_MPU("base_angle %f\n", base_angle);
}