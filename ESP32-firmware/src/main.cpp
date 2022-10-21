#include <Arduino.h>
#include <Wire.h>
#include <esp_wifi.h>

#include "BCH3121.h"
#include "RF4463.h"
#include "WiFi.h"
#include "jwifi.h"
#include "lcd.h"
#include "led.h"
#include "statustracker.h"

#define DEBUG
#define SERDEBUG
#include "bitutils.h"
#include "decoder.h"
#include "encoder.h"
#include "flexutils.h"
#include "httpd.h"
#include "pocsagutils.h"
#include "settings.h"
#include "soc/rtc_wdt.h"

RF4463 rf4463;

extern void initRF(void);

char buff[512];

void processOTAUpdate(void);

void setup() {
    Serial.begin(115200);

    ets_printf("class/struct            size\n");
    ets_printf("--------------------------------\n");
    ets_printf("flexbiw:                %u\n", sizeof(flexbiw));
    ets_printf("flexsync:               %u\n", sizeof(flexsync));
    ets_printf("flexfiw:                %u\n", sizeof(flexfiw));
    ets_printf("flexaddress:            %u\n", sizeof(flexaddress));
    ets_printf("flextempaddrmapping:    %u\n", sizeof(flextempaddrmapping));
    ets_printf("flexvector:             %u\n", sizeof(flexvector));
    ets_printf("flexphase:              %u\n", sizeof(flexphase));
    ets_printf("flexdatablock:          %u\n", sizeof(flexdatablock));
    ets_printf("flexmsg:                %u\n", sizeof(flexmsg));
    ets_printf("flexframe:              %u\n", sizeof(flexframe));
    ets_printf("--------------------------------\n");

    rf4463.powerOnReset();
    setupSettings();
    xTaskCreate(lcdTask, "LCD Process", 5000, NULL, 2, NULL);
    xTaskCreate(ledTask, "LED Process", 5000, NULL, 2, NULL);
    xTaskCreate(statustrackerTask, "statustrackerTask", 5000, NULL, 2, NULL);
    xTaskCreate(wifiProcess, "WiFi Process", 3000, NULL, 2, NULL);
    while (!wifiConnected) {
    }
    xTaskCreate(httpdProcess, "HTTPD Process", 3000, NULL, 4, NULL);
    xTaskCreate(flexTask, "flexTask", 3000, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(pocsagTask, "pocsagTask", 5000, NULL, 5, NULL);
    xTaskCreate(syncTask, "syncTask", 4000, NULL, 2, NULL);
}

void loop() {
    //rtc_wdt_feed();
    vTaskDelay(1);
}