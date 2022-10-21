#include <Arduino.h>

#include <vector>

#include "flexutils.h"
#include "pocsagutils.h"

struct txframe {
    bool blocked;
    enum frameformat { FORMAT_FLEX,
                       FORMAT_POCSAG,
                       FORMAT_BLOCKED,
                       FORMAT_IDLE } format;
    struct flexframe *flex = NULL;
    class pocsagdata *pocsag = NULL;
    ~txframe();
    txframe(enum frameformat format);
};

struct statusframe {
    uint8_t frameno;
    enum txframe::frameformat txformat;
    enum flexsynctype rxtype;
    bool isTX;
    bool txCancelled;
    uint32_t freq;
};

#define STATUSFRAMELISTSIZE 10
extern struct statusframe* statusframearr[];

void syncTask(void *parameter);
void eachFrame();
void startFrameAlignedTX(bool channelClear, bool shortPreamble);
void shiftStatusFrames();
int addPOCSAG(pocsagaddr* addr, uint32_t freq, uint16_t baudrate, char* data);
int addFLEX(char pre, uint32_t ric, uint32_t freq, char *data, bool isNumeric);
