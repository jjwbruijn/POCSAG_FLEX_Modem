#include <Arduino.h>
#include "ESPAsyncWebServer.h"

#ifndef CERT
#define CERT

extern const uint8_t der[];
extern size_t derlen;
extern char macStr[13];
extern String uuid;

struct settingsstruct {
    struct {
        bool ppsenabled;
    } gps;
    struct {
        bool loginRequired;
        char username[32];
        char password[32];
        char ntpserver[128];
    } webportal;
    struct {
        bool enabled;
        char host[64];
        uint16_t port;
        char username[32];
        char password[32];
    } mqtt;
    struct {
        uint8_t defaultmode;
        uint32_t defaultfrequency;
        uint16_t defaultbaud;
        uint16_t defaultoffset;
        uint8_t defaultxmitpower;
    } rf;
    struct {
        uint8_t mode;
        uint32_t freq;
        uint16_t baud;
        bool flexsynced;
    } current;
    struct {
        bool typebymessagecontent;
        bool markerrors;
    } pocsag;
    struct {
        bool autosendidleframes;
        bool allowSendToRX;
        uint8_t collapse;
    } flex;
    struct {
        bool lcdenabled;
        uint8_t ledsbrightness;
    } indicators;
    struct {
        char url[128];
        bool useSSL;
    } update;
};

#define LED_WIFI_CONNECTED 0x01
#define LED_WIFI_PORTAL 0x02
#define LED_WIFI_DISCONNECTED 0x04
#define LED_MQTT_CONNECTED 0x08
#define LED_MQTT_ACTIVITY 0x10
#define LED_MQTT_DISCONNECTED 0x20
#define LED_SYNC_SYNCED_TO_CHANNEL 0x40
#define LED_SYNC_SYNCED_NTP 0x80
#define LED_RF_RX_NOSIGNAL 0x100
#define LED_RF_RX_SIGNAL 0x200
#define LED_RF_TX 0x400
#define LED_RF_OFF 0x800
#define LED_WIFI_ACTIVITY 0x1000
#define LED_SYNC_ACTIVITY 0x2000
#define LED_SYNC_NOSYNC 0x4000


#define LCD_MODE_IDLE 0x01
#define LCD_MODE_RX 0x02
#define LCD_MODE_TX 0x04
#define LCD_MODECHANGE 0x08
#define LCD_UPDATERSSI 0x10
#define LCD_FREQCHANGE 0x1000
#define LCD_FRAMECHANGE 0x2000
#define LCD_IPCHANGE 0x4000

extern struct settingsstruct settings;

extern void setupSettings();
extern bool loadJSONFile();
extern void saveJSONFile();
extern void getSettings(AsyncWebServerRequest *request);
extern void saveSettings(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
extern void saveSettingsComplete(AsyncWebServerRequest *request);
extern void getOTACert(AsyncWebServerRequest *request);
extern void uploadOTACert(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
extern void uploadOTACertComplete(AsyncWebServerRequest *request);
extern void getMode(AsyncWebServerRequest *request);
extern void setMode(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);

#define MODE_FLEX 0
#define MODE_POCSAG 1
#define MODE_TX_ONLY 2
#define MODE_FLEX_SYNCED 3


#endif