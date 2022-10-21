
#include "pagerkeepalive.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32TimerInterrupt.h>
#include <ESPPerfectTime.h>
#include <LittleFS.h>

#include <vector>

#include "ESPAsyncWebServer.h"
#include "encoder.h"
#include "settings.h"

std::vector<struct idlelistentry> flexidlelist;
#define FLEXIDLEFRAMETIMEOUT 127
#define POCSAGIDLEFRAMETIMEOUT 128

bool registerFrame(uint32_t freq, uint32_t ric, char prefix) {
    //check if the pager is listening on the synced RX frequency, if so, don't register it's frame.
    if ((settings.current.mode == MODE_FLEX_SYNCED) && (freq == settings.current.freq)) {
        return false;
    }

    // check if base frame for this frequency is already in it's idle list
    uint8_t frame = flexaddress::getFrame(ric, prefix);
    for (uint16_t c = 0; c < flexidlelist.size(); c++) {
        if ((flexidlelist[c].frame == frame) && (flexidlelist[c].freq == freq)) {
            return false;
        }
    }

    // add to the idle list
    struct idlelistentry newentry;
    newentry.frame = frame;
    newentry.freq = freq;
    newentry.timeout = FLEXIDLEFRAMETIMEOUT;
    flexidlelist.push_back(newentry);
    return true;
}

bool registerPOCSAG(uint32_t freq, uint16_t baud) {
    for (uint16_t c = 0; c < flexidlelist.size(); c++) {
        if ((flexidlelist[c].frame == baud) && (flexidlelist[c].freq == freq)) {
            return false;
        }
    }
    struct idlelistentry newentry;
    newentry.frame = baud;
    newentry.freq = freq;
    newentry.timeout = POCSAGIDLEFRAMETIMEOUT;
    flexidlelist.push_back(newentry);
    return true;
}

void IRAM_ATTR resetIdleCounter(uint32_t freq, uint16_t frame) {
    for (uint16_t c = 0; c < flexidlelist.size(); c++) {
        if (flexidlelist[c].freq == freq) {
            if (flexstatus.collapse == 0) {
                if (flexidlelist[c].frame > 500) {
                    if (frame == flexidlelist[c].frame) {
                        flexidlelist[c].timeout = POCSAGIDLEFRAMETIMEOUT;
                    }
                } else {
                    flexidlelist[c].timeout = FLEXIDLEFRAMETIMEOUT;
                }
            } else {
                if (flexidlelist[c].frame > 500) {
                    if (frame == flexidlelist[c].frame) {
                        flexidlelist[c].timeout = POCSAGIDLEFRAMETIMEOUT;
                    }
                } else {
                    uint8_t match = ((1 << flexstatus.collapse) - 1);
                    if ((frame & match) == (flexidlelist[c].frame & match)) {
                        flexidlelist[c].timeout = FLEXIDLEFRAMETIMEOUT;
                    }
                }
            }
        }
    }
}

void IRAM_ATTR decrementIdleCounter() {
    for (uint16_t c = 0; c < flexidlelist.size(); c++) {
        if (flexidlelist[c].timeout) flexidlelist[c].timeout--;
    }
}

uint32_t IRAM_ATTR shouldSendIdleFlexFrame(uint8_t frame) {
    for (uint16_t c = 0; c < flexidlelist.size(); c++) {
        if ((flexidlelist[c].timeout == 0) && (flexidlelist[c].frame < 500)) {
            uint8_t match = ((1 << flexstatus.collapse) - 1);
            if ((frame & match) == (flexidlelist[c].frame & match)) {
                return flexidlelist[c].freq;
            }
        }
    }
    return 0;
}

idlelistentry IRAM_ATTR shouldSendIdlePOCSAG() {
    for (uint16_t c = 0; c < flexidlelist.size(); c++) {
        if (flexidlelist[c].timeout == 0) {
            if (flexidlelist[c].frame > 500) {
                return flexidlelist[c];
            }
        }
    }
    idlelistentry leeg;
    leeg.freq = 0;
    return leeg;
}

void getIdleList(AsyncWebServerRequest *request) {
    DynamicJsonDocument data(30 + 80 * flexidlelist.size());
    JsonArray pagers = data.createNestedArray("pagers");
    for (uint8_t c = 0; c < flexidlelist.size(); c++) {
        JsonObject pager = pagers.createNestedObject();
        pager["frame"] = flexidlelist[c].frame;
        pager["freq"] = flexidlelist[c].freq;
        pager["timeout"] = flexidlelist[c].timeout;
    }
    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
}

void saveIdleList() {
    DynamicJsonDocument data(30 + 50 * flexidlelist.size());
    JsonArray pagers = data.createNestedArray("pagers");
    for (uint8_t c = 0; c < flexidlelist.size(); c++) {
        JsonObject pager = pagers.createNestedObject();
        pager["frame"] = flexidlelist[c].frame;
        pager["freq"] = flexidlelist[c].freq;
    }
    File file = LittleFS.open("/idlelist.json", "w");
    if (serializeJson(data, file) == 0) {
        Serial.println(F("Failed to write to file"));
    }
    file.close();
}

void loadIdleList() {
    //StaticJsonDocument<2048> doc;
    DynamicJsonDocument doc(1500);
    File file = LittleFS.open("/idlelist.json");
    Serial.println("Loading Pager Idle list: ");
    while (file.available()) {
        Serial.write(file.read());
    }
    Serial.println("");
    Serial.println("----------------");
    file.seek(0);

    if (!file) {
        ets_printf("Can't read idlelist file, bailing\n");
        return;
    } else {
        DeserializationError error = deserializeJson(doc, file);
        if (error) {
            ets_printf("Failed to read json file, idlelist corrupted?\n");
            return;
        }
    }
    file.close();
    JsonArray array = doc["pagers"].as<JsonArray>();
    for (JsonVariant v : array) {
        idlelistentry pager;
        pager.frame = v["frame"].as<int>();
        pager.freq = v["freq"].as<int>();
        pager.timeout = 64;
        flexidlelist.push_back(pager);
    }
}

void delFromIdleList(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<150> doc;
    DeserializationError error = deserializeJson(doc, data);
    if (error) {
        return;
    } else {
        uint16_t frameno = doc["frame"];
        uint32_t freq = doc["freq"];
        for (uint16_t c = 0; c < flexidlelist.size(); c++) {
            if ((flexidlelist[c].frame == frameno) && (flexidlelist[c].freq == freq)) {
                flexidlelist.erase(flexidlelist.begin() + c);
                saveIdleList();
                return;
            }
        }
    }
}