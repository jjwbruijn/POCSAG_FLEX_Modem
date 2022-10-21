#include "jwifi.h"

#include <Arduino.h>
#include <ESPAsync_WiFiManager.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_wifi.h>

#include "ESPAsyncWebServer.h"
#include "httpd.h"
#include "led.h"
#include "settings.h"
#include "statustracker.h"

#define AP_NAME "PagerModem"
#define AP_PASSWORD "password"

AsyncDNSServer dnsServer;
extern AsyncWebServer server;
char macStr[13] = {0};
char shortmac[6] = {0};
enum wifistatusenum wifiStatus;
enum wifistatusenum prevWifiStatus;
#define RECONNECT_INTERVAL 10
uint8_t reconnectCountdown = RECONNECT_INTERVAL;
uint16_t reconnectAttempts = 0;
volatile bool wifiConnected = false;

void wifiConfigModeCallback(ESPAsync_WiFiManager *myWiFiManager);
void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case SYSTEM_EVENT_STA_START:
        case SYSTEM_EVENT_WIFI_READY:
            wifiStatus = wifistatusenum::READY;
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            wifiStatus = wifistatusenum::CONNECTED;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
        case SYSTEM_EVENT_STA_STOP:
        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        case SYSTEM_EVENT_STA_LOST_IP:
            wifiStatus = wifistatusenum::DISCONNECTED;
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            wifiStatus = wifistatusenum::GOT_IP;
            break;
        default:
            Serial.printf("[WiFi-event] event: %d\n", event);
            break;
    }
}

void WiFiEventPre(WiFiEvent_t event) {
    if (event == 5) {
        ets_printf("Reconnecting from WiFiEvent\n");
        WiFi.reconnect();
    }
}

void wifiProcess(void *parameter) {
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    if (status.ledupdater) xTaskNotify(status.ledupdater, LED_WIFI_DISCONNECTED, eSetBits);
    Serial.println("Launching WiFi Process...");
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    sprintf(shortmac, "%02X%02X", mac[4], mac[5]);
    Serial.print(F("MAC: "));
    Serial.println(macStr);
    wifiInit();
    WiFi.onEvent(WiFiEvent);

    if ((WiFi.status() == WL_CONNECTED) &&
        WiFi.localIP() &&
        WiFi.isConnected()) {
        wifiStatus = wifistatusenum::GOT_IP;
        ets_printf("Connected to WiFi - IP: ");
        sendStatus(STATUS_NEWIP);
        Serial.println(WiFi.localIP());
        wifiConnected = true;
        if (status.ledupdater) xTaskNotify(status.ledupdater, LED_WIFI_CONNECTED, eSetBits);
    } else {
        wifiStatus = wifistatusenum::DISCONNECTED;
        if (status.ledupdater) xTaskNotify(status.ledupdater, LED_WIFI_DISCONNECTED, eSetBits);
    }
    prevWifiStatus = wifiStatus;
    sendStatus(STATUS_UPDATERSSI);
    uint8_t doRSSI = 0;
    while (true) {
        if (prevWifiStatus != wifiStatus) {  // status change
            if (wifiStatus == wifistatusenum::GOT_IP) {
                ets_printf("Connected to WiFi - IP: ");
                Serial.println(WiFi.localIP());
                wifiConnected = true;
                HTTPrestartNeeded = true;
                reconnectAttempts = 0;
                sendStatus(STATUS_NEWIP);
                if (status.ledupdater) xTaskNotify(status.ledupdater, LED_WIFI_CONNECTED, eSetBits);
            } else {
                if (status.ledupdater) xTaskNotify(status.ledupdater, LED_WIFI_DISCONNECTED, eSetBits);
                ets_printf("Disconnected!!!\n");
                wifiConnected = false;
            }
            prevWifiStatus = wifiStatus;
        }
        if (wifiStatus != wifistatusenum::GOT_IP) {
            // not online;
            reconnectCountdown--;
            if (!reconnectCountdown) {
                reconnectCountdown = RECONNECT_INTERVAL;
                reconnectAttempts++;
                if (reconnectAttempts == 180) {
                    wifiReset();
                }
                if (reconnectAttempts >= 360) {
                    ESP.restart();
                }
                ets_printf("Reconnecting...\n");
                WiFi.reconnect();
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (doRSSI == 10) {
            doRSSI = 0;
            sendStatus(STATUS_UPDATERSSI);
        } else {
            doRSSI++;
        }
    }
}

void wifiResetConfig() {
    //DNSServer dnsServer;
    Serial.println("Removing settings and resetting..");
    vTaskDelay(50 / portTICK_PERIOD_MS);
    WiFi.disconnect(true, true);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP.restart();
}

void wifiInit() {
    AsyncDNSServer dnsServer;
    String ssid = AP_NAME + String(" ") + String(shortmac);
    ESPAsync_WiFiManager wifiManager(&server, &dnsServer, (const char *)ssid.c_str());
    WiFi.onEvent(WiFiEventPre);
    WiFi.setHostname((const char *)ssid.c_str());
    wifiManager.setConnectTimeout(10);
    wifiManager.setDebugOutput(false);
    wifiManager.setBreakAfterConfig(true);
    wifiManager.setTimeout(180);
    //wifiManager.setAPCallback(wifiConfigModeCallback);
    //wifiManager.setSaveConfigCallback(wifiSaveConfigCallback);
    ssid = AP_NAME + String(" Setup ") + String(shortmac);
    if (!wifiManager.autoConnect((const char *)ssid.c_str(), AP_PASSWORD)) {
        // set LED to red
        if (status.ledupdater) xTaskNotify(status.ledupdater, LED_WIFI_DISCONNECTED, eSetBits);
        Serial.println("failed to connect, reset and try again..");
        delay(500);
        wifiReset();
        ESP.restart();
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
}

void wifiConfigModeCallback(ESPAsync_WiFiManager *myWiFiManager) {
    if (status.ledupdater) xTaskNotify(status.ledupdater, LED_WIFI_PORTAL, eSetBits);
    Serial.print("Entered config mode: ");
    Serial.print(WiFi.softAPIP());
    Serial.print(" on ");
    Serial.println(myWiFiManager->getConfigPortalSSID());
}

void wifiSaveConfigCallback() {
    Serial.println(F("Saved config"));
}

void wifiReset() {
    esp_wifi_disconnect();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    esp_wifi_set_mode(WIFI_MODE_STA);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    wifi_country_t country = {
        .cc = "JP",
        .schan = 1,
        .nchan = 14,
        .max_tx_power = 126,
        .policy = WIFI_COUNTRY_POLICY_AUTO,
    };
    esp_wifi_set_country(&country);
}