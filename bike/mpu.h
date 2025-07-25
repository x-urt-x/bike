#ifndef _mpu_h
#define _mpu_h
#include "arduino.h"
#include "config.h"
#include "log.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "led.h"

extern MPU6050 mpu;
extern float base_angle;

// DMP control/status
extern bool dmpReady;
extern uint8_t mpuIntStatus;
extern uint8_t devStatus;
extern uint16_t packetSize;
extern uint16_t fifoCount;
extern uint8_t fifoBuffer[64];

// orientation/motion vars
extern Quaternion q;
extern VectorInt16 aa;
extern VectorInt16 aaReal;
extern VectorInt16 aaWorld;
extern VectorFloat gravity;
extern float euler[3];
#define TODEGR 180/M_PI
#define RAD_TO_INT16_SCALE (32767.0f / M_PI)

extern volatile bool mpuInterrupt;
void ICACHE_RAM_ATTR dmpDataReady();

void mpu_setup();
bool mpu_loop();
void mpu_base();

#endif