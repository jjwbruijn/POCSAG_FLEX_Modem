#include "statustracker.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <LITTLEFS.h>
#include <WiFi.h>

#include "ESPAsyncWebServer.h"
#include "RF4463.h"
#include "decoder.h"
#include "encoder.h"
#include "httpd.h"
#include "jwifi.h"
#include "pagerkeepalive.h"
#include "settings.h"

#define UPDATEQUEUESIZE 15
QueueHandle_t updatequeue;
struct statusstruct status;

void statustrackerTask(void* parameter) {
    updatequeue = xQueueCreate(UPDATEQUEUESIZE, sizeof(uint32_t));
    bool signal = false;
    bool rx = false;
    bool tx = false;
    bool flexsyncchannel = false;
    bool flexsyncntp = false;
    uint32_t lastseenfreq;
    uint32_t update;
    status.currentmode = 0;
    while (true) {
        portYIELD();
        BaseType_t updateq = xQueueReceive(updatequeue, &update, portMAX_DELAY);
        if (updateq == pdTRUE) {
            switch (update) {
                case STATUS_WIFI_ACTIVITY:
                    if (status.ledupdater) xTaskNotify(status.ledupdater, LED_WIFI_ACTIVITY, eSetBits);
                    break;
                case STATUS_FRAMELIST_UPDATE:
                    if (status.ledupdater) xTaskNotify(status.ledupdater, LED_SYNC_ACTIVITY, eSetBits);
                    if (status.websocketUpdater) xTaskNotify(status.websocketUpdater, WS_SEND_FRAME_STATUS, eSetBits);
                    if (status.lcdupdater) xTaskNotify(status.lcdupdater, LCD_FRAMECHANGE, eSetBits);
                    break;
                case STATUS_TX: {
                    if (rx) {
                        rx = false;
                        status.rxSignal = false;
                        status.rxActive = false;
                        signal = false;
                    }
                    if (!tx) {
                        tx = true;
                        signal = false;
                        status.txActive = true;
                        if (status.ledupdater) xTaskNotify(status.ledupdater, LED_RF_TX, eSetBits);
                        if (status.websocketUpdater) xTaskNotify(status.websocketUpdater, WS_SEND_MODE_STATUS, eSetBits);
                        if (status.lcdupdater) xTaskNotify(status.lcdupdater, LCD_MODE_TX, eSetBits);
                    }
                    if (status.freq != lastseenfreq) {
                        lastseenfreq = status.freq;
                        //if (status.websocketUpdater) xTaskNotify(status.websocketUpdater, WS_SEND_MODE_STATUS, eSetBits);
                        if (status.lcdupdater) xTaskNotify(status.lcdupdater, LCD_FREQCHANGE, eSetBits);
                    }
                } break;
                case STATUS_RX_NO_SIGNAL: {
                    if (tx) {
                        tx = false;
                        status.txActive = false;
                    }
                    if (!rx) {
                        rx = true;
                        status.rxActive = true;
                        status.rxSignal = false;
                        signal = false;
                        if (status.ledupdater) xTaskNotify(status.ledupdater, LED_RF_RX_NOSIGNAL, eSetBits);
                        if (status.websocketUpdater) xTaskNotify(status.websocketUpdater, WS_SEND_MODE_STATUS, eSetBits);
                        if (status.lcdupdater) xTaskNotify(status.lcdupdater, LCD_MODE_RX, eSetBits);
                    }
                    if (status.freq != lastseenfreq) {
                        lastseenfreq = status.freq;
                        if (status.lcdupdater) xTaskNotify(status.lcdupdater, LCD_FREQCHANGE, eSetBits);
                        //if (status.websocketUpdater) xTaskNotify(status.websocketUpdater, WS_SEND_MODE_STATUS, eSetBits);
                    }
                    if (signal) {
                        signal = false;
                        status.rxSignal = false;
                        if (status.ledupdater) xTaskNotify(status.ledupdater, LED_RF_RX_NOSIGNAL, eSetBits);
                        if (status.websocketUpdater) xTaskNotify(status.websocketUpdater, WS_SEND_SIGNAL, eSetBits);
                    }
                } break;
                case STATUS_RX_SIGNAL:
                    if (!signal) {
                        signal = true;
                        status.rxSignal = true;
                        if (status.ledupdater) xTaskNotify(status.ledupdater, LED_RF_RX_SIGNAL, eSetBits);
                        if (status.websocketUpdater) xTaskNotify(status.websocketUpdater, WS_SEND_SIGNAL | WS_SEND_FRAME_STATUS, eSetBits);
                    }
                    break;
                case STATUS_NEW_WS_CONNECTION:
                    if (status.websocketUpdater) xTaskNotify(status.websocketUpdater, WS_SEND_FRAME_STATUS | WS_SEND_MODE_STATUS | WS_SEND_MODMODE | WS_SEND_SIGNAL, eSetBits);
                    break;
                case STATUS_SYNCED_FLEX:
                    if (!flexsyncchannel) {
                        flexsyncchannel = true;
                        flexsyncntp = false;
                        if (status.ledupdater) xTaskNotify(status.ledupdater, LED_SYNC_SYNCED_TO_CHANNEL, eSetBits);
                    }
                    break;
                case STATUS_SYNCED_GPS:
                    if (!flexsyncntp) {
                        flexsyncntp = true;
                        flexsyncchannel = false;
                        if (status.ledupdater) xTaskNotify(status.ledupdater, LED_SYNC_SYNCED_NTP, eSetBits);
                    }
                case STATUS_SYNCED_NOSYNC:
                    if (flexsyncchannel || flexsyncntp) {
                        flexsyncchannel = false;
                        flexsyncntp = false;
                        if (status.ledupdater) xTaskNotify(status.ledupdater, LED_SYNC_NOSYNC, eSetBits);
                    }
                case STATUS_MODE_FLEX1600:
                case STATUS_MODE_FLEX3200_2:
                case STATUS_MODE_FLEX3200_4:
                case STATUS_MODE_FLEX6400_4:
                case STATUS_MODE_POCSAG512:
                case STATUS_MODE_POCSAG1200:
                case STATUS_MODE_POCSAG2400:
                    if (update != status.currentmode) {
                        status.currentmode = update;
                        if (status.lcdupdater) xTaskNotify(status.lcdupdater, LCD_MODECHANGE, eSetBits);
                    }
                    break;
                case STATUS_NEWIP:
                    if (status.lcdupdater) xTaskNotify(status.lcdupdater, LCD_IPCHANGE, eSetBits);
                    break;
                case STATUS_UPDATERSSI:
                    if (status.lcdupdater) xTaskNotify(status.lcdupdater, LCD_UPDATERSSI, eSetBits);
                    break;
                default:
                    ets_printf("Notification: %u\n", update);
                    break;
            }
        }
    }
}

void sendStatus(uint32_t statusval) {
    xQueueSend(updatequeue, &statusval, 0);
}

void IRAM_ATTR sendStatusISR(uint32_t statusval) {
    xQueueSendFromISR(updatequeue, &statusval, NULL);
}