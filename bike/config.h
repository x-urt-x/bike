#ifndef CONFIG_H
#define CONFIG_H

#define GYRO_INTERRUPT_PIN D2
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

#define BTN_LONG_TIME 400
#define BTN_TIMEOUT 1000
#define BTN_DEBOUNCE 50

#define WIFI_TIMEOUT 10000
#define WIFI_SOFT_AP_SSID "esp_stat"
#define WIFI_SOFT_AP_PASS "12345678"
#endif // !CONFIG_H
