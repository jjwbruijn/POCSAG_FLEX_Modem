#include "settings.h"

#include <Arduino.h>
#include <ArduinoJson.h>
//#include <SPIFFS.h>
#include <AsyncJson.h>
//#include <LITTLEFS.h>
#include <LittleFS.h>

#include <iostream>
#include <vector>

#include "ESPAsyncWebServer.h"
#include "RF4463.h"
#include "decoder.h"
#include "statustracker.h"

extern AsyncWebSocket ws;

struct settingsstruct settings;

void setupSettings() {
    if (!LittleFS.begin()) {
        if (!LittleFS.format()) {
            ets_printf("Error occurred while mounting FS\n");
            ets_printf("Error formatting FS!!!\n");
        } else {
            ets_printf("Filesystem created ");
            if (!LittleFS.begin()) {
                ets_printf("but failed to mount :(\n");
            } else {
                ets_printf("and mounted\n");
            }
        }
    } else {
        ets_printf("Filesystem mounted, %u/%u bytes used\n", LittleFS.usedBytes(), LittleFS.totalBytes());
    }
    loadJSONFile();
}

void getSettings(AsyncWebServerRequest *request) {
    ets_printf("getsettings called\n");
    if (settings.webportal.loginRequired) {
        if (!request->authenticate(settings.webportal.username, settings.webportal.password)) {
            return request->requestAuthentication();
        }
    }
    request->send(LittleFS, "/settings.json", "application/json");
}

void saveSettingsComplete(AsyncWebServerRequest *request) {
    if (settings.webportal.loginRequired) {
        if (!request->authenticate(settings.webportal.username, settings.webportal.password)) {
            return;
        }
    }
    if (!loadJSONFile()) {
        request->send(200, "application/json", "{\"status\":\"CONFIG_INVALID\"}");
    } else {
        request->send(200, "application/json", "{\"status\":\"CONFIG_VALID\"}");
    }
}

void saveSettings(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    if (settings.webportal.loginRequired) {
        if (!request->authenticate(settings.webportal.username, settings.webportal.password)) {
            return request->requestAuthentication();
        }
    }
    ets_printf("Save New Settings\n");
    File file = LittleFS.open("/settings.json", "w");
    file.write(data, len);
    file.close();
}
void uploadOTACertComplete(AsyncWebServerRequest *request) {
    if (settings.webportal.loginRequired) {
        if (!request->authenticate(settings.webportal.username, settings.webportal.password)) {
            return;
        }
    }
    request->send(200, "application/json", "{\"status\":\"OTACERT_SAVED\"}");
}
void uploadOTACert(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (settings.webportal.loginRequired) {
        if (!request->authenticate(settings.webportal.username, settings.webportal.password)) {
            return request->requestAuthentication();
        }
    }
    if (!index) {
        LittleFS.remove("/otacert.pem");
    }
    File file = LittleFS.open("/otacert.pem", "a");
    file.write(data, len);
    file.close();
}
void getOTACert(AsyncWebServerRequest *request) {
    ets_printf("getotacert called\n");
    if (settings.webportal.loginRequired) {
        if (!request->authenticate(settings.webportal.username, settings.webportal.password)) {
            return request->requestAuthentication();
        }
    }
    request->send(LittleFS, "/otacert.pem", "application/json");  // todo, right content type
}
void getMode(AsyncWebServerRequest *request) {
    StaticJsonDocument<100> data;
    switch (settings.current.mode) {
        case 0:
            data["mode"] = "MODE_FLEX";
            break;
        case 1:
            data["mode"] = "MODE_POCSAG";
            break;
        case 2:
            data["mode"] = "MODE_TX_ONLY";
            break;
        case 3:
            data["mode"] = "MODE_FLEX_SYNCED";
            break;
    }
    data["freq"] = settings.current.freq;
    data["baud"] = settings.current.baud;
    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
}
void setMode(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<512> doc;
    bool configValid = false;
    DeserializationError error = deserializeJson(doc, data);
    if (error) {
        ets_printf("Failed to read json file, config corrupted? Using defaults\n");
    } else {
        bool freqValid = true;

        if (doc["mode"] == "MODE_FLEX") {
            freqValid = RF4463::checkAvail(doc["freq"], RF4463::FLEX_1600_RX);
        } else if (doc["mode"] == "MODE_POCSAG") {
            uint16_t tempbaud = doc["baud"] | 1200;
            switch (tempbaud) {
                case 512:
                    freqValid = RF4463::checkAvail(doc["freq"], RF4463::POCSAG_512_RX);
                    break;
                case 1200:
                    freqValid = RF4463::checkAvail(doc["freq"], RF4463::POCSAG_1200_RX);
                    break;
                case 2400:
                    freqValid = RF4463::checkAvail(doc["freq"], RF4463::POCSAG_2400_RX);
                    break;
            }
        } else if (doc["mode"] == "MODE_FLEX_SYNCED") {
            freqValid = RF4463::checkAvail(doc["freq"], RF4463::FLEX_1600_RX);
        }

        if (freqValid) {
            settings.current.baud = doc["baud"] | 1200;
            if (doc["mode"] == "MODE_FLEX") {
                settings.current.mode = MODE_FLEX;
            } else if (doc["mode"] == "MODE_POCSAG") {
                settings.current.mode = MODE_POCSAG;
            } else if (doc["mode"] == "MODE_TX_ONLY") {
                settings.current.mode = MODE_TX_ONLY;
            } else if (doc["mode"] == "MODE_FLEX_SYNCED") {
                settings.current.mode = MODE_FLEX_SYNCED;
            }
            settings.current.freq = doc["freq"];
            resetRX = true;
        }

        if (status.websocketUpdater) xTaskNotifyFromISR(status.websocketUpdater, 0x04, eSetBits, NULL);
    }
}
void saveJSONFile() {
    File file = LittleFS.open("/settings.json", "w");
    StaticJsonDocument<2048> doc;
    JsonObject gps = doc.createNestedObject("gps");
    gps["ppsenabled"] = settings.gps.ppsenabled;

    JsonObject webportal = doc.createNestedObject("webportal");
    webportal["loginrequired"] = settings.webportal.loginRequired;
    webportal["username"] = settings.webportal.username;
    webportal["password"] = settings.webportal.password;
    webportal["ntpserver"] = settings.webportal.ntpserver;

    JsonObject mqtt = doc.createNestedObject("mqtt");
    mqtt["enabled"] = settings.mqtt.enabled;
    mqtt["host"] = settings.mqtt.host;
    mqtt["port"] = settings.mqtt.port;
    mqtt["username"] = settings.mqtt.username;
    mqtt["password"] = settings.mqtt.password;

    JsonObject rf = doc.createNestedObject("rf");
    rf["defaultmode"] = settings.rf.defaultmode;
    rf["defaultfrequency"] = settings.rf.defaultfrequency;
    rf["defaultoffset"] = settings.rf.defaultoffset;
    rf["defaultxmitpower"] = settings.rf.defaultxmitpower;
    rf["defaultbaud"] = settings.rf.defaultbaud;

    JsonObject pocsag = doc.createNestedObject("pocsag");
    pocsag["typebymessagecontent"] = settings.pocsag.typebymessagecontent;
    pocsag["markerrors"] = settings.pocsag.markerrors;

    JsonObject flex = doc.createNestedObject("flex");
    flex["autosendidleframes"] = settings.flex.autosendidleframes;
    flex["allowSendToRX"] = settings.flex.allowSendToRX;
    flex["collapse"] = settings.flex.collapse;

    JsonObject indi = doc.createNestedObject("indicators");
    indi["lcdEnabled"] = settings.indicators.lcdenabled;
    indi["ledsBrightness"] = settings.indicators.ledsbrightness;

    JsonObject update = doc.createNestedObject("update");
    update["url"] = settings.update.url;
    update["usessl"] = settings.update.useSSL;

    if (serializeJson(doc, file) == 0) {
        Serial.println(F("Failed to write to file"));
    }
    file.close();
}
bool loadJSONFile() {
    StaticJsonDocument<2048> doc;
    bool configValid = false;
    File file = LittleFS.open("/settings.json");
    Serial.println("Loading config: ");
    while (file.available()) {
        Serial.write(file.read());
    }
    Serial.println("");
    Serial.println("----------------");
    file.seek(0);

    if (!file) {
        ets_printf("Can't read config file, we'll be using defaults\n");
    } else {
        DeserializationError error = deserializeJson(doc, file);
        if (error) {
            ets_printf("Failed to read json file, config corrupted? Using defaults\n");
        } else {
            configValid = true;
        }
    }
    file.close();
    settings.gps.ppsenabled = doc["gps"]["ppsenabled"] | false;

    settings.webportal.loginRequired = doc["webportal"]["loginrequired"] | false;
    strlcpy(settings.webportal.username, doc["webportal"]["username"] | "", sizeof(settings.webportal.username));
    strlcpy(settings.webportal.password, doc["webportal"]["password"] | "", sizeof(settings.webportal.password));
    strlcpy(settings.webportal.ntpserver, doc["webportal"]["ntpserver"] | "", sizeof(settings.webportal.ntpserver));

    settings.mqtt.enabled = doc["mqtt"]["enabled"] | false;
    strlcpy(settings.mqtt.host, doc["mqtt"]["host"] | "", sizeof(settings.mqtt.host));
    settings.mqtt.port = doc["mqtt"]["port"] | 1883;
    strlcpy(settings.mqtt.username, doc["mqtt"]["username"] | "", sizeof(settings.mqtt.username));
    strlcpy(settings.mqtt.password, doc["mqtt"]["password"] | "", sizeof(settings.mqtt.password));

    settings.rf.defaultmode = doc["rf"]["defaultmode"] | 0;
    settings.rf.defaultfrequency = doc["rf"]["defaultfrequency"] | 169650000;
    settings.rf.defaultoffset = doc["rf"]["defaultoffset"] | 87;
    settings.rf.defaultxmitpower = doc["rf"]["defaultxmitpower"] | 1;
    settings.rf.defaultbaud = doc["rf"]["defaultbaud"] | 1200;

    settings.pocsag.typebymessagecontent = doc["pocsag"]["typebymessagecontent"] | true;
    settings.pocsag.markerrors = doc["pocsag"]["markerrors"] | false;

    settings.flex.autosendidleframes = doc["flex"]["autosendidleframes"] | false;
    settings.flex.allowSendToRX = doc["flex"]["allowSendToRX"] | false;
    settings.flex.collapse = doc["flex"]["collapse"] | 4;

    settings.indicators.lcdenabled = doc["indicators"]["lcdEnabled"] | true;
    settings.indicators.ledsbrightness = doc["indicators"]["ledsBrightness"] | 10;

    strlcpy(settings.update.url, doc["update"]["url"] | "OTAHOST", sizeof(settings.update.url));
    settings.update.useSSL = doc["update"]["usessl"];

    if (!configValid) {
        ets_printf("Saving new file\n");
        saveJSONFile();
        return false;
    }
    return true;
}
