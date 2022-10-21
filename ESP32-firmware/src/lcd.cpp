#define FS_NO_GLOBALS
#include "lcd.h"

#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <Wire.h>

#include "RF4463.h"
#include "encoder.h"
#include "fonts.h"
#include "images.h"
#include "settings.h"
#include "statustracker.h"

TFT_eSPI tft = TFT_eSPI(135, 240);

uint32_t lcd_lastric = 0;
uint32_t lcd_lastric_drawn = 0;
uint8_t lcd_lastfunc;
bool lcd_msg_changed = true;

void lcdSetup() {
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setSwapBytes(true);
    tft.pushImage(0, 0, 240, 135, bg);

    tft.loadFont(lcars24);  // Load another different font
    tft.setTextDatum(BL_DATUM);

    //blauw
    tft.setTextColor(0x3A38, TFT_BLACK);  // Change the font colour and the background colour
    tft.fillRect(40, 20, 128, 22, 0x0000);
    tft.drawString("460.91875 MHz", 40, 47);

    tft.fillRect(186, 20, 55, 20, 0x0000);
    tft.drawString("RX", 190, 47);

    tft.drawLine(17, 44, 229, 44, 0x3A38);

    // oranje block
    tft.fillRect(40, 47, 107, 20, 0x0000);
    tft.setTextColor(0xFBE4, TFT_BLACK);  // Change the font colour and the background colour
    tft.drawString("POCSAG1200", 40, 72);

    tft.fillRect(190, 47, 50, 20, 0x0000);
    tft.setTextColor(0xFBE4, TFT_BLACK);  // Change the font colour and the background colour
    tft.drawString("127", 190, 72);

    tft.drawLine(17, 69, 222, 69, 0xFBE4);
    tft.unloadFont();

    // paars block
    tft.setTextColor(0xA254);
    tft.setCursor(40, 77);
    tft.fillRect(36, 71, 92, 15, 0x0000);
    //tft.print(WiFi.localIP());

    tft.setTextColor(0xA254);
    tft.fillRect(190, 71, 49, 15, 0x0000);
    tft.setCursor(190, 77);
    //tft.print(WiFi.RSSI());
    //tft.print(" dBm");

    tft.drawLine(17, 86, 229, 86, 0xA254);
}

void updateState() {
    char buffer[16];
    /*
    if (flex_rx_state > 1) {
        tft.pushImage(55, 72, 50, 10, icon_synced);
    } else {
        tft.pushImage(55, 72, 50, 10, icon_nosignal);
        tft.fillRect(106, 72, 103, 10, 0x0000);
    }
    if (flex_rx_state == 10) {
        tft.pushImage(106, 72, 50, 10, icon_message);
    }
    if (flex_rx_state > 10) {
        tft.pushImage(158, 72, 50, 10, icon_idle);
    }
    */
}

uint32_t screenfreq;
bool fontLoaded = false;

void lcdTask(void *parameter) {
    lcdSetup();
    status.lcdupdater = xTaskGetCurrentTaskHandle();
    uint32_t action = 0;
    while (1) {
        action = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (action == 0) {
            // timeout
        } else {
            if (action & LCD_FREQCHANGE) {
                if (screenfreq != status.freq) {
                    screenfreq = status.freq;
                    double freq = (double)status.freq / 1000000;
                    if (!fontLoaded) {
                        tft.loadFont(lcars24);
                        fontLoaded = true;
                    }
                    tft.setTextColor(0x3A38, TFT_BLACK);
                    tft.fillRect(40, 20, 128, 22, 0x0000);
                    tft.setCursor(40, 24);
                    tft.printf("%.8g MHz", freq);
                }
            }
            if (action & LCD_MODE_RX) {
                if (!fontLoaded) {
                    tft.loadFont(lcars24);
                    fontLoaded = true;
                }
                tft.setTextColor(0x3A38, TFT_BLACK);
                tft.fillRect(190, 20, 50, 22, 0x0000);
                tft.setCursor(190, 24);
                tft.print("RX");
            }
            if (action & LCD_MODE_TX) {
                if (!fontLoaded) {
                    tft.loadFont(lcars24);
                    fontLoaded = true;
                }
                tft.setTextColor(0x3A38, TFT_BLACK);
                tft.fillRect(190, 20, 49, 22, 0x0000);
                tft.setCursor(190, 24);
                tft.print("TX");
            }
            if (action & LCD_MODECHANGE) {
                if (!fontLoaded) {
                    tft.loadFont(lcars24);
                    fontLoaded = true;
                }
                tft.fillRect(40, 47, 107, 20, 0x0000);
                tft.setTextColor(0xFBE4, TFT_BLACK);  // Change the font colour and the background colour
                tft.setCursor(40, 49);
                switch (status.currentmode) {
                    case STATUS_MODE_FLEX1600:
                        tft.print("FLEX 1600");
                        break;
                    case STATUS_MODE_FLEX3200_2:
                        tft.print("FLEX 3200 2");
                        break;
                    case STATUS_MODE_FLEX3200_4:
                        tft.print("FLEX 3200 4");
                        break;
                    case STATUS_MODE_FLEX6400_4:
                        tft.print("FLEX 6400 4");
                        break;
                    case STATUS_MODE_POCSAG512:
                        tft.print("POCSAG 512");
                        break;
                    case STATUS_MODE_POCSAG1200:
                        tft.print("POCSAG 1200");
                        break;
                    case STATUS_MODE_POCSAG2400:
                        tft.print("POCSAG 2400");
                        break;
                }
            }

            if (action & LCD_FRAMECHANGE) {
                if (!fontLoaded) {
                    tft.loadFont(lcars24);
                    fontLoaded = true;
                }
                tft.fillRect(190, 47, 50, 20, 0x0000);
                tft.setTextColor(0xFBE4, TFT_BLACK);  // Change the font colour and the background colour
                tft.setCursor(190, 49);
                tft.print(flexstatus.frame);
            }

            if (action & LCD_MODE_IDLE) {
                if (!fontLoaded) {
                    tft.loadFont(lcars24);
                    fontLoaded = true;
                }
                tft.setTextColor(0x3A38, TFT_BLACK);
                tft.fillRect(190, 20, 50, 20, 0x0000);
                tft.setCursor(190, 24);
                tft.print("IDLE");
            }

            if (fontLoaded) {
                tft.unloadFont();
                fontLoaded = false;
            }

            if (action & LCD_IPCHANGE) {
                tft.setTextColor(0xA254);
                tft.setCursor(40, 76);
                tft.fillRect(36, 71, 92, 14, 0x0000);
                tft.print(WiFi.localIP());
            }
            if (action & LCD_UPDATERSSI) {
                if (WiFi.isConnected()) {
                    tft.setTextColor(0xA254);
                    tft.fillRect(190, 71, 49, 14, 0x0000);
                    tft.setCursor(190, 76);
                    tft.print(WiFi.RSSI());
                    tft.print(" dBm");
                }
            }
        }
    }
}