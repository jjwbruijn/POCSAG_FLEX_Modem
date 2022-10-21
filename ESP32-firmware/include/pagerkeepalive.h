#include <Arduino.h>
#include "ESPAsyncWebServer.h"
#include <vector>

struct idlelistentry {
    uint16_t frame;
    uint32_t freq;
    uint16_t timeout;
};
//extern std::vector<struct idlelistentry> flexidlelist;

extern bool registerFrame(uint32_t freq, uint32_t ric, char prefix);
bool registerPOCSAG(uint32_t freq, uint16_t baud);
void resetIdleCounter(uint32_t freq, uint16_t frame);
void decrementIdleCounter();
uint32_t shouldSendIdleFlexFrame(uint8_t frame);
idlelistentry shouldSendIdlePOCSAG();
void delFromIdleList(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void getIdleList(AsyncWebServerRequest* request);
void loadIdleList();
void saveIdleList();
