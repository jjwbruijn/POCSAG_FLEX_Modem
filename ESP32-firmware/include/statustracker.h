#include <Arduino.h>

#define STATUS_WIFI_ACTIVITY 0x01
#define STATUS_RX_NO_SIGNAL 0x02
#define STATUS_RX_SIGNAL 0x03
#define STATUS_TX 0x04
#define STATUS_IDLE 0x05
#define STATUS_MODE_FLEX1600 0x06
#define STATUS_MODE_FLEX3200_2 0x07
#define STATUS_MODE_FLEX3200_4 0x08
#define STATUS_MODE_FLEX6400_4 0x09
#define STATUS_MODE_POCSAG512 0x0A
#define STATUS_MODE_POCSAG1200 0x0B
#define STATUS_MODE_POCSAG2400 0x0C
#define STATUS_MODE_IDLE 0x0D
#define STATUS_NEW_WS_CONNECTION 0x0E
#define STATUS_SYNCED_FLEX 0x0F
#define STATUS_SYNCED_GPS 0x10
#define STATUS_SYNCED_NOSYNC 0x11
#define STATUS_FRAMELIST_UPDATE 0x12
#define STATUS_NEWIP 0x13
#define STATUS_UPDATERSSI 0x14

extern QueueHandle_t updatequeue;

struct statusstruct {
    bool txActive = false;
    uint8_t currentmode = 0;
    bool rxActive = false;
    bool rxSignal = false;
    uint32_t framesReceived = 0;
    uint32_t freq = 0;
    TaskHandle_t websocketUpdater;
    TaskHandle_t ledupdater;
    TaskHandle_t lcdupdater;
    TaskHandle_t updater;
};

extern struct statusstruct status;

void statustrackerTask(void *parameter);
void sendStatus(uint32_t statusval);
void sendStatusISR(uint32_t statusval);