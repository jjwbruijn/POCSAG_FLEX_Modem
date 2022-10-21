#include "led.h"

#include <Arduino.h>
#include <FastLED.h>

#include "settings.h"
#include "statustracker.h"

FASTLED_USING_NAMESPACE
//#define FASTLED_FORCE_SOFTWARE_SPI

#define NUM_LEDS 5
#define DATA_PIN 22

#define LED_POWER 0
#define LED_WIFI 1
#define LED_MQTT 2
#define LED_SYNC 3
#define LED_RF 4
#define LED_OFF CRGB::Black

CRGB leds[NUM_LEDS];

void ledSetup() {
    FastLED.addLeds<SK6812, DATA_PIN, GRB>(leds, NUM_LEDS);
    leds[LED_POWER] = CRGB::Green;
    leds[LED_WIFI] = LED_OFF;
    leds[LED_MQTT] = LED_OFF;
    leds[LED_SYNC] = LED_OFF;
    leds[LED_RF] = LED_OFF;
    FastLED.setBrightness(settings.indicators.ledsbrightness);
    FastLED.show();
}

void ledTask(void *parameter) {
    ledSetup();
    status.ledupdater = xTaskGetCurrentTaskHandle();
    bool wifiActivity = false;
    bool mqttActivity = false;
    bool syncActivity = false;
    bool syncNTP = false;
    bool syncChannel = false;
    uint32_t action = 0;
    while (true) {
        if (wifiActivity || mqttActivity || syncActivity) {
            action = ulTaskNotifyTake(pdTRUE, 25 / portTICK_RATE_MS);
        } else {
            action = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        if (action == 0) {  //timeout
            if (wifiActivity) {
                leds[LED_WIFI] = CRGB::Green;
                wifiActivity = false;
                FastLED.show();
            }
            if (mqttActivity) {
                leds[LED_MQTT] = CRGB::Green;
                mqttActivity = false;
                FastLED.show();
            }
            if (syncActivity) {
                if (syncNTP)
                    leds[LED_SYNC] = CRGB::Green;
                if (syncChannel)
                    leds[LED_SYNC] = CRGB::White;
                syncActivity = false;
                FastLED.show();
            }
        } else {
            if (action & LED_WIFI_CONNECTED) leds[LED_WIFI] = CRGB::Green;
            if (action & LED_WIFI_DISCONNECTED) leds[LED_WIFI] = CRGB::DarkRed;
            if (action & LED_WIFI_PORTAL) leds[LED_WIFI] = CRGB::DarkBlue;
            if (action & LED_WIFI_ACTIVITY) {
                leds[LED_WIFI] = CRGB::Black;
                wifiActivity = true;
            }
            if (action & LED_MQTT_CONNECTED) leds[LED_MQTT] = CRGB::Green;
            if (action & LED_MQTT_DISCONNECTED) leds[LED_MQTT] = CRGB::Red;
            if (action & LED_MQTT_ACTIVITY) {
                leds[LED_MQTT] = CRGB::Black;
                mqttActivity = true;
            }
            if (action & LED_SYNC_NOSYNC) {
                leds[LED_SYNC] = CRGB::Black;
                syncNTP = false;
                syncChannel = false;
                syncActivity = false;
            }
            if (action & LED_SYNC_SYNCED_NTP) {
                leds[LED_SYNC] = CRGB::Green;
                syncNTP = true;
            }
            if (action & LED_SYNC_SYNCED_TO_CHANNEL) {
                leds[LED_SYNC] = CRGB::Yellow;
                syncChannel = true;
            }
            if (action & LED_SYNC_ACTIVITY) {
                leds[LED_SYNC] = CRGB::Black;
                syncActivity = true;
            }
            if (action & LED_RF_RX_NOSIGNAL) leds[LED_RF] = CRGB::Green;
            if (action & LED_RF_RX_SIGNAL) leds[LED_RF] = CRGB::Blue;
            if (action & LED_RF_TX) leds[LED_RF] = CRGB::Yellow;
            if (action & LED_RF_OFF) leds[LED_RF] = CRGB::Black;

            FastLED.show();
        }
    }
}