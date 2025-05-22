#ifndef LOG_CFG_H
#define LOG_CFG_H

#define LOG_USB_ENABLE

#ifdef LOG_USB_ENABLE

//#define LOG_MPU_ENABLE
#define LOG_STARTUP_ENABLE
//#define LOG_HALL12_ENABLE
//#define LOG_HALL3_ENABLE
#define LOG_BTN_ENABLE
#define LOG_FS_ENABLE

#endif // LOG_USB_ENABLE


#ifdef LOG_USB_ENABLE
#define LOG_USB_MATR(matr)						\
{												\
	Serial.printf("%s\n", #matr);				\
	for (int i = 0; i < MATR_SIZE; i++)			\
	{											\
		for (int j = 0; j < MATR_SIZE; j++)		\
			Serial.printf("%d ", matr[j][i]);	\
		Serial.printf("\n");					\
	}											\
}
#endif

#ifdef LOG_MPU_ENABLE
#define LOG_MPU(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
#define LOG_MPU(format, ...) ;
#endif

#ifdef LOG_STARTUP_ENABLE
#define LOG_STARTUP(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
#define LOG_STARTUP(format, ...) ;
#endif

#ifdef LOG_HALL12_ENABLE
#define LOG_HALL12(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
#define LOG_HALL12(format, ...) ;
#endif

#ifdef LOG_HALL3_ENABLE
#define LOG_HALL3(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
#define LOG_HALL3(format, ...) ;
#endif

#ifdef LOG_BTN_ENABLE
#define LOG_BTN(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
#define LOG_BTN(format, ...) ;
#endif

#ifdef LOG_FS_ENABLE
#define LOG_FS(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
#define LOG_FS(format, ...) ;
#endif


#endif