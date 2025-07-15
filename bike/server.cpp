#include "server.h"

File uploadFile;
uint32_t buffer[SECTOR_SIZE/4];

const char* ssid = mySSID;
const char* password = myPASS;

ESP8266WebServer server(80);

const IPAddress local_ip(192, 168, 1, 50);
const IPAddress gateway(192, 168, 1, 1);
const IPAddress subnet(255, 255, 255, 0);
const IPAddress primaryDNS(8, 8, 8, 8);
const IPAddress secondaryDNS(8, 8, 4, 4);

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
	getSector(offset, buffer);
	server.sendHeader("Content-Type", "application/octet-stream");
	server.send_P(200, "application/octet-stream", (char*)buffer, SECTOR_SIZE);
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
		LOG_HTTP("upload start\n");
		String filename = "/" + upload.filename;
		uploadFile = LittleFS.open(filename, "w");
	}
	else if (upload.status == UPLOAD_FILE_WRITE) {
		LOG_HTTP("upload in process\n");
		if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
	}
	else if (upload.status == UPLOAD_FILE_END) {
		LOG_HTTP("upload end\n");
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
	server.on("/log", HTTP_GET, []() {
		String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Log</title></head><body>";
		html += "<h1>Log Information</h1>";
		html += "<ul>";
		html += "<li>current_sector = " + String(current_sector) + "</li>";
		html += "<li>curent_index = " + String(buffer_index) + "</li>";
		html += "<li>start_sector = " + String(start_sector) + "</li>";
		html += "</ul>";
		html += "</body></html>";
		server.send(200, "text/html", html);
		});
	server.on("/upload", HTTP_POST, []() {
		server.send(200, "text/plain", "Upload complete");
		LOG_HTTP("upload init\n");
		}, handleUpload);
	server.on("/start_params", HTTP_GET, []() {
		server.send(200, "application/json",
		String(
			"{\"start_sector\":") + start_sector +
		",\"start_index\":" + start_index + 
		",\"start_index_number\":" + start_sector_number +
			"}");
		});
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
		digitalWrite(LED_PIN, !digitalRead(LED_PIN));
		delay(100);
	}
	digitalWrite(LED_PIN, LOW);
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
	digitalWrite(LED_PIN, HIGH);
	WiFi.disconnect(true);
	WiFi.mode(WIFI_OFF);
	WiFi.forceSleepBegin();
	delay(10);
	digitalWrite(LED_PIN, LOW);
	LOG_WIFI("WiFi turned off\n");
}