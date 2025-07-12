#ifndef _server_h
#define _server_h

#include "arduino.h"
#include "config.h"
#include "pass.h" //  define in this file SSID and PASS of ur wifi
#include "log.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include "Header.h"
#include "mem.h"

extern File uploadFile;
extern uint32_t buffer[];

// WiFi
extern const char* ssid;
extern const char* password;

// Server
extern ESP8266WebServer server;
extern const IPAddress local_ip;
extern const IPAddress gateway;
extern const IPAddress subnet;
extern const IPAddress primaryDNS;
extern const IPAddress secondaryDNS;

bool fs_setup();

void handleSector();
void handleRoot();
void handleUpload();
void server_setup();
void connectOrStartAP();
void turnOffWiFi();

#endif