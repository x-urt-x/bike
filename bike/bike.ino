#define EB_NO_COUNTER
#define EB_NO_BUFFER
#include "log.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <LittleFS.h>
#include "config.h"

#include "pass.h" //  define in this file SSID and PASS of ur wifi

struct Data {
	uint16_t cranks_count;
	uint16_t wheel_count;
	char pos;
	float ang1, ang2, ang3;
} __attribute__((packed));

alignas(4) uint32_t flash_buffer[SECTOR_SIZE / 4];

File uploadFile;

const char* ssid = SSID;
const char* password = PASS;

ESP8266WebServer server(80);
const IPAddress local_ip(192, 168, 1, 50);
const IPAddress gateway(192, 168, 1, 1);
const IPAddress subnet(255, 255, 255, 0);
const IPAddress primaryDNS(8, 8, 8, 8);
const IPAddress secondaryDNS(8, 8, 4, 4);

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

#define TODEGR 180/M_PI
float euler[3];

volatile bool mpuInterrupt = false;
void ICACHE_RAM_ATTR dmpDataReady() {
	mpuInterrupt = true;
}

uint8_t sector_buffer[SECTOR_SIZE];
uint32_t current_sector = 0;
uint16_t buffer_index = 0;
uint32_t buffer_start_time = 0;

void initBuffer() {
	memset(sector_buffer, 0xFF, SECTOR_SIZE);
	buffer_start_time = millis();
	memcpy(sector_buffer, &buffer_start_time, sizeof(uint32_t));
	buffer_index = 0;
}

void flushBuffer() {
	if (buffer_index == 0) return;

	uint32_t end_time = millis();
	memcpy(sector_buffer + 4 + buffer_index * DATA_ENTRY_SIZE, &end_time, sizeof(uint32_t));
	memcpy(sector_buffer + 4 + buffer_index * DATA_ENTRY_SIZE + 4, &SECTOR_KEY, sizeof(uint32_t));

	uint32_t addr = USER_FLASH_START + current_sector * SECTOR_SIZE;
	uint32_t sector_num = (addr - FLASH_BASE) / SECTOR_SIZE;

	ESP.flashEraseSector(sector_num);
	ESP.flashWrite(addr - FLASH_BASE, (uint32_t*)sector_buffer, SECTOR_SIZE / 4);

	current_sector = (current_sector + 1) % MAX_SECTORS;
	initBuffer();
}

void addDataEntry(const Data& entry) {
	if (buffer_index >= MAX_DATA_ENTRIES) {
		flushBuffer();
	}
	memcpy(sector_buffer + 4 + buffer_index * DATA_ENTRY_SIZE, &entry, DATA_ENTRY_SIZE);
	buffer_index++;
}

void findLastSector() {
	uint32_t last_time = 0;
	current_sector = 0;
	for (uint32_t i = 0; i < MAX_SECTORS; ++i) {
		uint32_t addr = USER_FLASH_START + i * SECTOR_SIZE;
		uint32_t start_time;
		memcpy(&start_time, (void*)addr, sizeof(uint32_t));

		if (start_time == 0xFFFFFFFF) continue;

		if (start_time < last_time) {
			current_sector = i;
			break;
		}
		last_time = start_time;
	}
	initBuffer();
}

bool getSector(int offset, uint8_t* buffer)
{
	if (!buffer) return false;

	uint32_t index;
	if (offset == 0)
	{
		// return the RAM buffer with appended timestamp and key
		LOG_SECTOR("RAM buffer");
		uint32_t now = millis();
		memcpy(sector_buffer + 4 + buffer_index * DATA_ENTRY_SIZE, &now, 4);
		memcpy(sector_buffer + 4 + buffer_index * DATA_ENTRY_SIZE + 4, &SECTOR_KEY, 4);
		memcpy(buffer, sector_buffer, SECTOR_SIZE);
		return true;
	}
	if (offset > 0) {
		uint32_t base = current_sector + 1;
		if (base >= MAX_SECTORS) base -= MAX_SECTORS;
		index = base + offset - 1;
		if (index >= MAX_SECTORS) index -= MAX_SECTORS;
	}
	else {
		int32_t idx = (int32_t)current_sector + offset - 1;
		if (idx < 0) idx += MAX_SECTORS;
		index = (uint32_t)idx;
	}

	uint32_t addr = USER_FLASH_START + index * SECTOR_SIZE;
	uint32_t flash_offset = addr - 0x40200000;

	// boundary check
	if (addr < USER_FLASH_START || addr + SECTOR_SIZE > USER_FLASH_END) {
		LOG_SECTOR("Address out of valid range");
		return false;
	}

	// important: cast to uint32_t* and size in 4-byte words
	bool res = ESP.flashRead(flash_offset, (uint32_t*)buffer, SECTOR_SIZE / 4);
	return res;
}

uint16_t cranks_count = 0;
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

uint16_t wheel_count = 0;
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
bool btn_state = true;
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
bool fs_setup()
{
	if (!LittleFS.begin()) {
		LOG_FS("error on LittleFS\n");
		return false;
	}

	FSInfo fs_info;
	LittleFS.info(fs_info);

	LOG_FS("LittleFS info:\n");
	LOG_FS("Total size: %u bytes\n", fs_info.totalBytes);
	LOG_FS("Used size: %u bytes\n", fs_info.usedBytes);
	LOG_FS("Free space: %u bytes\n", fs_info.totalBytes - fs_info.usedBytes);
	return true;
}

void handleSector()
{
	int offset = server.arg("i").toInt();
	if (getSector(offset, (uint8_t*)flash_buffer)) {
		server.sendHeader("Content-Type", "application/octet-stream");
		server.send_P(200, "application/octet-stream", (char*)flash_buffer, SECTOR_SIZE);
	}
	else
		server.send(404, "text/plain", "Invalid sector");
}
void handleRoot() {
	File file = LittleFS.open("/index.html", "r");
	if (!file) {
		server.send(404, "text/plain", "index.html not found");
		return;
	}
	server.streamFile(file, "text/html");
	file.close();
}
void handleUpload() {
	HTTPUpload& upload = server.upload();
	if (upload.status == UPLOAD_FILE_START) {
		String filename = "/" + upload.filename;
		uploadFile = LittleFS.open(filename, "w");
	}
	else if (upload.status == UPLOAD_FILE_WRITE) {
		if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
	}
	else if (upload.status == UPLOAD_FILE_END) {
		if (uploadFile) uploadFile.close();
		server.sendHeader("Location", "/");
		server.send(303);
	}
}

void server_setup()
{
	server.begin();
	server.on("/", HTTP_GET, handleRoot);
	server.on("/sector", handleSector);
	server.on("/upload", HTTP_GET, []() {
		server.send(200, "text/html",
		"<form method='POST' action='/upload' enctype='multipart/form-data'>"
		"<input type='file' name='file'>"
			"<input type='submit' value='Upload'></form>");
		});

	
	server.on("/update", HTTP_POST, []() {
		server.send(200, "text/plain", "Upload complete");
		}, handleUpload);
	LOG_HTTP("server started\n");
}

void connectOrStartAP() {
	WiFi.forceSleepWake();
	delay(1);

	WiFi.mode(WIFI_STA);
	WiFi.config(local_ip, gateway, subnet, primaryDNS, secondaryDNS);
	WiFi.begin(ssid, password);

	unsigned long startAttempt = millis();
	while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000 && btn_state)
	{
		LOG_WIFI(".");
		delay(100);
	}
	LOG_WIFI("\n");
	if (WiFi.status() != WL_CONNECTED || !btn_state)
	{
		WiFi.disconnect();
		WiFi.mode(WIFI_AP);
		WiFi.softAPConfig(local_ip, gateway, subnet);
		WiFi.softAP(WIFI_SOFT_AP_SSID, WIFI_SOFT_AP_PASS);
		LOG_WIFI("Started AP mode: IP=%s\n", WiFi.softAPIP().toString().c_str());
	}
	else {
		LOG_WIFI("Connected to WiFi: IP=%s\n", WiFi.localIP().toString().c_str());
	}
}

void turnOffWiFi() {
	WiFi.disconnect(true);
	WiFi.mode(WIFI_OFF);
	WiFi.forceSleepBegin();
	delay(1);
	LOG_WIFI("WiFi turned off\n");
}

void setup()
{
#ifdef LOG_USB_ENABLE
	Serial.begin(115200);
	delay(2000);
	Serial.println();
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

	findLastSector();
	if (!fs_setup()) return;

	WiFi.mode(WIFI_OFF);
	WiFi.forceSleepBegin();
	delay(1);

	server_setup();
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
	if (!btn_state || (millis() - btn_last < BTN_TIMEOUT)) return;
	do {
		if (btn_count == 1 && !(btn_press & 0x1))
		{
			LOG_BTN("single short\n");
			break;
		}
		if (btn_count == 1 && (btn_press & 0x1))
		{
			LOG_BTN("single long\n");
			connectOrStartAP();
			break;
		}
		if (btn_count == 2 && !(btn_press & 0x1) && !(btn_press & 0x2))
		{
			LOG_BTN("double short\n");
			turnOffWiFi();
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
	server.handleClient();
	mpu_loop();
	btnTick();
	unsigned long now = millis();
	//Serial.printf("%d\n", analogRead(DERAILLEUR_PIN));
	while ((int32_t)(now - next) >= 0)
	{
		//Serial.printf("cranks: %d\twheel: %d\ttransmission: %d\n", cranks_count / 2, wheel_count / 2, getHall3());
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

		cranks_count = 0;
		wheel_count = 0;
		next += SEND_TIME_MS;
	}
}