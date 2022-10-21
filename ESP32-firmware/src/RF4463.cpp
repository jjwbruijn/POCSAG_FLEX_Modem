#include <RF4463.h>
#include <SPI.h>
// Generated with wireless development suite by silicon labs
//#include "radio_config_Si4463.h"

#include "decoder.h"
#include "rfconfig.h"
#include "settings.h"
#include "statustracker.h"

#define SDN 12
#define GPIO0 2
#define GPIO1 15
#define SDO 27
#define SDI 26
#define CLK 25
#define NSEL 33
#define RELAY 17
#define NIRQ 13

#define _nIRQPin NIRQ
#define _sdnPin SDN
#define _nSELPin NSEL

//#define REVERSE_POL

SPIClass rf(HSPI);
static const int spiClk = 15000000;  //10 - 17 mhz works. 22 is too high

uint32_t currentfrequency = 0;
uint8_t currentmode = INIT_TX;
bool interruptSet = false;
SemaphoreHandle_t CTSBusy;
//#define DEBUG

extern RF4463 rf4463;

void RF4463::startPOCSAG(void* parameter) {
    rf4463.init((uint32_t)parameter, RF4463::rftype::POCSAG_2400_TX);
    rf4463.startTX();
    vTaskDelete(NULL);
}

void RF4463::stopTX(void* parameter) {
    rf4463.enterStandbyMode();
    restartDecoder();
    vTaskDelete(NULL);
}
bool restartDecoder() {
    switch (settings.current.mode) {
        case MODE_FLEX:
        case MODE_FLEX_SYNCED:
            //FLEX-mode
            if (!rf4463.init(settings.current.freq, RF4463::rftype::FLEX_1600_RX)) {
            } else {
                rf4463.startRX(flexsynchelper);
            }
            break;
        case MODE_POCSAG:
            //POCSAG-mode
            switch (settings.current.baud) {
                case 512:
                    rf4463.init(settings.current.freq, RF4463::rftype::POCSAG_512_RX);
                    break;
                case 1200:
                    rf4463.init(settings.current.freq, RF4463::rftype::POCSAG_1200_RX);
                    break;
                case 2400:
                    rf4463.init(settings.current.freq, RF4463::rftype::POCSAG_2400_RX);
                    break;
            }
            rf4463.startRX(pocsagRXInterrupt);
            break;
        case MODE_TX_ONLY:
            sendStatus(STATUS_MODE_IDLE);
            sendStatus(STATUS_IDLE);
            return false;
            break;
    }
    return true;
}

void IRAM_ATTR sendMark() {
#ifdef REVERSE_POL
    digitalWrite(GPIO1, HIGH);
#else
    digitalWrite(GPIO1, LOW);
#endif
}
void IRAM_ATTR sendSpace() {
#ifdef REVERSE_POL
    digitalWrite(GPIO1, LOW);
#else
    digitalWrite(GPIO1, HIGH);
#endif
}

RF4463::RF4463(uint8_t nIRQPin, uint8_t sdnPin, uint8_t nSELPin) {
    //	_nIRQPin = nIRQPin;
    //	_sdnPin = sdnPin;
    //	_nSELPin = nSELPin;
    CTSBusy = xSemaphoreCreateMutex();
    ets_printf("Mutex created\n");
    xSemaphoreGive(CTSBusy);
}
void RF4463::spiInit() {
    rf.begin(CLK, SDO, SDI, NSEL);  //CLK,MISO,MOIS,SS
    pinMode(_nSELPin, OUTPUT);
    digitalWrite(_nSELPin, HIGH);
}
void RF4463::pinInit() {
    pinMode(_sdnPin, OUTPUT);
    digitalWrite(_sdnPin, HIGH);
    pinMode(_nIRQPin, INPUT);
    pinMode(NIRQ, INPUT_PULLUP);
}
bool RF4463::init(uint32_t freq, rftype inittype) {
    uint8_t buf[20];
    type = inittype;

    bool isTX = false;
    bool freqfound = false;

    if (interruptSet) {
        detachInterrupt(GPIO1);
        interruptSet = false;
    }

    status.freq = round(freq / 10) * 10;

    switch (inittype) {
        case rftype::FLEX_6400_TX:
        case rftype::POCSAG_2400_TX:
            isTX = true;
            break;
        default:
            break;
    }

    if (isTX) {
        sendStatus(STATUS_TX);
    } else {
        sendStatus(STATUS_RX_NO_SIGNAL);
        switch (inittype) {
            case rftype::FLEX_1600_RX:
                sendStatus(STATUS_MODE_FLEX1600);
                break;
            case rftype::FLEX_3200_2_RX:
                sendStatus(STATUS_MODE_FLEX3200_2);
                break;
            case rftype::FLEX_3200_4_RX:
                sendStatus(STATUS_MODE_FLEX3200_4);
                break;
            case rftype::FLEX_6400_RX:
                sendStatus(STATUS_MODE_FLEX6400_4);
                break;
            case rftype::POCSAG_512_RX:
                sendStatus(STATUS_MODE_POCSAG512);
                break;
            case rftype::POCSAG_1200_RX:
                sendStatus(STATUS_MODE_POCSAG1200);
                break;
            case rftype::POCSAG_2400_RX:
                sendStatus(STATUS_MODE_POCSAG2400);
                break;
        }
    }

    freq += inittype;

    if (isTX) {
        //currentmode = mode;
        uint8_t c = 0;
        for (c = 0; c < TX_FREQS; c++) {
            if (txFrequencies[c] == freq) {
                freqfound = true;
                channel = 0;
                break;
            }
        }

        if (!freqfound) {
            c--;
            while (!freqfound && c < 250) {
                uint32_t diff = (freq / 10) - (txFrequencies[c] / 10);
                diff *= 10;
                if (diff < (25000 * 255)) {
                    if ((txFrequencies[c] % 10) == (freq % 10)) {  // check if type (pocsag/flex/modulation) is the same
                        channel = diff / 25000;
                        if (diff % 25000 == 0) {
                            freqfound = true;
                        } else {
                            c--;
                        }
                    } else {
                        c--;
                    }
                } else {
                    c--;
                }
            }
        }
        if (freqfound) {
            if (currentconfig == txFrequencies[c]) {
                return true;
            } else {
                currentconfig = txFrequencies[c];
                setRawConfig(configs[c + RX_FREQS]);
            }
        } else {
            ets_printf("TX Freq %u not found...\n", freq / 10);
            return false;
        }
    } else {  // RX
        //currentmode = mode;
        uint8_t c = 0;
        for (c = 0; c < RX_FREQS; c++) {
            if (rxFrequencies[c] == freq) {
                freqfound = true;
                channel = 0;
                break;
            }
        }
        if (!freqfound) {
            c--;
            // no exact match...
            while (!freqfound && c < 250) {
                uint32_t diff = (freq / 10) - (rxFrequencies[c] / 10);
                diff *= 10;
                if (diff < (25000 * 255)) {
                    if ((rxFrequencies[c] % 10) == (freq % 10)) {
                        channel = diff / 25000;
                        freqfound = true;
                    } else {
                        c--;
                    }
                } else {
                    c--;
                }
            }
        }
        if (freqfound) {
            if (currentconfig == rxFrequencies[c]) {
                return true;
            } else {
                currentconfig = rxFrequencies[c];
                setRawConfig(configs[c]);
            }
        } else {
#ifdef DEBUG
            ets_printf("RX Freq %u not found for mode %u...\n", (freq / 10) * 10, freq % 10);
#endif
            return false;
        }
    }
    if (isTX) {                          // setup
        buf[0] = RF4463_GPIO_NO_CHANGE;  // disabled :(
        buf[0] = RF4463_GPIO_NO_CHANGE;
        if (type == rftype::POCSAG_2400_TX) {
            buf[1] = RF4463_GPIO_INPUT;
            pinMode(GPIO1, OUTPUT);
            CTSLineActive = false;
        } else {
            buf[1] = RF4463_GPIO_CTS;
            pinMode(GPIO1, INPUT_PULLUP);
            CTSLineActive = true;
        }
        buf[2] = RF4463_GPIO_RX_STATE;
        buf[3] = RF4463_GPIO_TX_STATE;
        buf[4] = RF4463_NIRQ_INTERRUPT_SIGNAL;
        buf[5] = RF4463_GPIO_SPI_DATA_OUT;
        setCommand(6, RF4463_CMD_GPIO_PIN_CFG, buf);

    } else {
        buf[0] = RF4463_GPIO_NO_CHANGE;  // disabled :(
        buf[1] = RF4463_GPIO_RX_DATA_CLOCK;
        buf[2] = RF4463_GPIO_RX_STATE;
        buf[3] = RF4463_GPIO_TX_STATE;
        buf[4] = RF4463_GPIO_RX_DATA;
        buf[5] = RF4463_GPIO_SPI_DATA_OUT;
        setCommand(6, RF4463_CMD_GPIO_PIN_CFG, buf);
        pinMode(GPIO1, INPUT_PULLUP);
        CTSLineActive = false;
    }

    // frequency adjust
    //buf[0] = 98; // dev board 88 -- second board 86
    buf[0] = 86;
    setProperties(RF4463_PROPERTY_GLOBAL_XO_TUNE, 1, buf);

    if (type == rftype::POCSAG_2400_TX) {
        buf[0] = RF4463_TX_DIRECT_MODE_TYPE_SYNCHRONOUS | RF4463_TX_DIRECT_MODE_GPIO1 | RF4463_MOD_SOURCE_DIRECT_MODE | RF4463_MOD_TYPE_2GFSK;
        //buf[0] = RF4463_TX_DIRECT_MODE_TYPE_ASYNCHRONOUS | RF4463_TX_DIRECT_MODE_GPIO1 | RF4463_MOD_SOURCE_DIRECT_MODE | RF4463_MOD_TYPE_2FSK;
        setProperties(RF4463_PROPERTY_MODEM_MOD_TYPE, 1, buf);
    } else {
        buf[0] = 0x40;
        setProperties(RF4463_PROPERTY_PKT_LEN, 1, buf);
    }
    setTxPower(127);
    enterStandbyMode();
    return true;
}
bool RF4463::checkAvail(uint32_t freq, rftype inittype) {
    bool isTX = false;
    bool freqfound = false;

    switch (inittype) {
        case rftype::FLEX_6400_TX:
        case rftype::POCSAG_2400_TX:
            isTX = true;
            break;
        default:
            break;
    }

    freq += inittype;

    if (isTX) {
        uint8_t c = 0;
        for (c = 0; c < TX_FREQS; c++) {
            if (txFrequencies[c] == freq) {
                return true;
            }
        }
        c--;
        while (!freqfound && c < 250) {
            uint32_t diff = (freq / 10) - (txFrequencies[c] / 10);
            diff *= 10;
            if (diff < (25000 * 255)) {
                if ((txFrequencies[c] % 10) == (freq % 10)) {  // check if type (pocsag/flex/modulation) is the same
                    if (diff % 25000 == 0) {
                        return true;
                    } else {
                        c--;
                    }
                } else {
                    c--;
                }
            } else {
                c--;
            }
        }
        return false;
    } else {
        uint8_t c = 0;
        for (c = 0; c < RX_FREQS; c++) {
            if (rxFrequencies[c] == freq) {
                return true;
            }
        }
        c--;
        while (!freqfound && c < 250) {  // loop until c wraps and becomes >250
            uint32_t diff = (freq / 10) - (rxFrequencies[c] / 10);
            diff *= 10;
            if (diff < (25000 * 255)) {
                if ((rxFrequencies[c] % 10) == (freq % 10)) {
                    return true;
                } else {
                    c--;
                }
            } else {
                c--;
            }
        }
        return false;
    }
}
void RF4463::powerOnReset() {
    ets_printf("RF4463 POR\n");
    CTSLineActive = false;
    uint8_t buf[] = {0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80};
    pinInit();
    spiInit();
    digitalWrite(_sdnPin, HIGH);
    delay(100);
    digitalWrite(_sdnPin, LOW);
    delay(20);  // wait for RF4463 stable
    // send power up command
    digitalWrite(_nSELPin, LOW);
    spiWriteBuf(sizeof(buf), buf);
    digitalWrite(_nSELPin, HIGH);
    if (!checkDevice()) {
        ets_printf("RF4463 failed :(\n");
    }
}
void RF4463::setRawConfig(const uint8_t* config) {
    uint8_t buf[6];

    pinMode(GPIO1, INPUT_PULLUP);
    buf[0] = RF4463_GPIO_NO_CHANGE;  // disabled :(
    buf[1] = RF4463_GPIO_CTS;
    buf[2] = RF4463_GPIO_RX_STATE;
    buf[3] = RF4463_GPIO_TX_STATE;
    buf[4] = RF4463_NIRQ_INTERRUPT_SIGNAL;
    buf[5] = RF4463_GPIO_SPI_DATA_OUT;
    setCommand(6, RF4463_CMD_GPIO_PIN_CFG, buf);
    CTSLineActive = true;
    uint8_t configfields = *config;
    config++;
    uint8_t len = *config;
    config++;        // move to first byte of config
    config += len;   // skip powerup
    configfields--;  // skip powerup
    while (configfields) {
        len = *config;  // read len
        config++;       // set to first byte of config row
        //        if (*config != RF4463_PROPERTY_GLOBAL_XO_TUNE && *config != RF4463_PROPERTY_MODEM_MOD_TYPE && *config != RF4463_CMD_GPIO_PIN_CFG) {
        if (*config != RF4463_PROPERTY_GLOBAL_XO_TUNE && *config != RF4463_CMD_GPIO_PIN_CFG) {
            digitalWrite(_nSELPin, LOW);
            spiWriteBuf(len, (uint8_t*)config);
            digitalWrite(_nSELPin, HIGH);
            checkCTS();
        }
        configfields--;
        config += len;
    }
}
void RF4463::setConfig(const uint8_t* parameters, uint16_t paraLen) {
    // command buf starts with length of command in RADIO_CONFIGURATION_DATA_ARRAY
    uint8_t cmdLen;
    uint8_t command;
    uint16_t pos;
    uint8_t buf[30];

    parameters++;
    paraLen--;
    paraLen = paraLen - 1;
    cmdLen = parameters[0];
    pos = cmdLen + 1;

    while (pos < paraLen) {
        cmdLen = parameters[pos++] - 1;         // get command lend
        command = parameters[pos++];            // get command
        memcpy(buf, parameters + pos, cmdLen);  // get parameters

        setCommand(cmdLen, command, buf);
        pos = pos + cmdLen;
    }
}
bool RF4463::checkDevice() {
    uint8_t buf[9];
    uint16_t partInfo;
    if (!getCommand(9, RF4463_CMD_PART_INFO, buf))  // read part info to check if 4463 works
        return false;

    partInfo = buf[2] << 8 | buf[3];
    if (partInfo != 0x4463) {
        return false;
    }
    return true;
}
bool RF4463::waitnIRQ() {
    return !digitalRead(_nIRQPin);  // inquire interrupt
}
void RF4463::startTX() {
    uint8_t buf[] = {0x00, 0x30, 0x00, 0x00};
    buf[0] = channel;
    setCommand(4, RF4463_CMD_START_TX, buf);
}
void RF4463::startPacketTX(uint16_t len) {
    uint8_t buf[] = {0x00, 0x30, 0x00, 0x00};
    buf[0] = channel;
    buf[2] = (len / 256) & 0x1F;
    buf[3] = len % 256;
    setCommand(4, RF4463_CMD_START_TX, buf);
}
void RF4463::startPacketRX(uint16_t len) {
    uint8_t buf[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08};
    buf[0] = channel;
    buf[2] = (len >> 8) & 0x1F;
    buf[3] = len & 0xFF;
    setCommand(7, RF4463_CMD_START_RX, buf);
}
void RF4463::startRX() {
    uint8_t buf[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08};
    buf[0] = channel;
    setCommand(7, RF4463_CMD_START_RX, buf);
}
bool RF4463::startRX(void (*clockinterrupt)(void)) {
    startRX();  // enter RX mode
    if (clockinterrupt != NULL) {
        interruptSet = true;
        attachInterrupt(15, clockinterrupt, RISING);
    } else {
        detachInterrupt(GPIO1);
        interruptSet = false;
    }
    return true;
}
bool RF4463::enterStandbyMode() {
    uint8_t data = 0x01;
    return setCommand(1, RF4463_CMD_CHANGE_STATE, &data);
}
bool RF4463::enterReadyMode() {
    uint8_t data = 0x03;
    return setCommand(1, RF4463_CMD_CHANGE_STATE, &data);
}
void RF4463::writeTxFifo(uint8_t* databuf, uint8_t length) {
    //setProperties(RF4463_PROPERTY_PKT_FIELD_2_LENGTH_7_0, sizeof(length), &length);
    //uint8_t buf[length + 1];
    //buf[0] = length;
    //memcpy(buf + 1, databuf, length);
    setCommandNoCTS(length, RF4463_CMD_TX_FIFO_WRITE, databuf);  // hmmm
    //setCommand(length, RF4463_CMD_TX_FIFO_WRITE, databuf);
}
void RF4463::readRxFifo(uint8_t* databuf, uint8_t len) {
    if (!checkCTS())
        return;
    digitalWrite(_nSELPin, LOW);
    spiByte(RF4463_CMD_RX_FIFO_READ);
    spiReadBuf(len, databuf);
    digitalWrite(_nSELPin, HIGH);
}
int8_t RF4463::getRXFIFOSize() {
    if (!checkCTS())
        return -1;
    uint8_t buffer[3];
    if (!getCommand(3, RF4463_CMD_FIFO_INFO, buffer)) return -1;
    return buffer[1];
}
int8_t RF4463::getTXFIFOAvail() {
    if (!checkCTS())
        return -1;
    uint8_t buffer[3];
    getCommand(3, RF4463_CMD_FIFO_INFO, buffer);
    return buffer[2];
}
void RF4463::fifoReset() {
    uint8_t data = 0x03;
    setCommand(sizeof(data), RF4463_CMD_FIFO_INFO, &data);
}
bool RF4463::setTxPower(uint8_t power) {
    if (power > 127)  // max is 127
        return false;

    uint8_t buf[4] = {0x08, 0x00, 0x00, 0x3d};
    buf[1] = power;

    return setProperties(RF4463_PROPERTY_PA_MODE, sizeof(buf), buf);
}
bool RF4463::setCommand(uint8_t length, uint8_t command, uint8_t* paraBuf) {
    if (!checkCTS())
        return false;

    digitalWrite(_nSELPin, LOW);
    spiByte(command);              // send command
    spiWriteBuf(length, paraBuf);  // send parameters
    digitalWrite(_nSELPin, HIGH);

    return true;
}
bool RF4463::setCommandNoCTS(uint8_t length, uint8_t command, uint8_t* paraBuf) {
    digitalWrite(_nSELPin, LOW);
    spiByte(command);              // send command
    spiWriteBuf(length, paraBuf);  // send parameters
    digitalWrite(_nSELPin, HIGH);
    return true;
}
bool RF4463::getCommand(uint8_t length, uint8_t command, uint8_t* paraBuf) {
    if (!checkCTS())
        return false;

    digitalWrite(_nSELPin, LOW);
    spiByte(command);  // set command to read
    digitalWrite(_nSELPin, HIGH);

    if (!checkCTS())  // check if RF4463 is ready
        return false;

    digitalWrite(_nSELPin, LOW);
    spiByte(RF4463_CMD_READ_BUF);  // turn to read command mode
    spiReadBuf(length, paraBuf);   // read parameters
    digitalWrite(_nSELPin, HIGH);
    return true;
}
bool RF4463::setProperties(uint16_t startProperty, uint8_t length, uint8_t* paraBuf) {
    uint8_t buf[4];

    if (!checkCTS())
        return false;

    buf[0] = RF4463_CMD_SET_PROPERTY;
    buf[1] = startProperty >> 8;    // GROUP
    buf[2] = length;                // NUM_PROPS
    buf[3] = startProperty & 0xff;  // START_PROP

    digitalWrite(_nSELPin, LOW);
    spiWriteBuf(4, buf);           // set start property and read length
    spiWriteBuf(length, paraBuf);  // set parameters
    digitalWrite(_nSELPin, HIGH);

    return true;
}
bool RF4463::getProperties(uint16_t startProperty, uint8_t length, uint8_t* paraBuf) {
    if (!checkCTS())
        return false;

    uint8_t buf[4];
    buf[0] = RF4463_CMD_GET_PROPERTY;
    buf[1] = startProperty >> 8;    // GROUP
    buf[2] = length;                // NUM_PROPS
    buf[3] = startProperty & 0xff;  // START_PROP

    digitalWrite(_nSELPin, LOW);
    spiWriteBuf(4, buf);  // set start property and read length
    digitalWrite(_nSELPin, HIGH);

    if (!checkCTS())
        return false;

    digitalWrite(_nSELPin, LOW);
    spiByte(RF4463_CMD_READ_BUF);  // turn to read command mode
    spiReadBuf(length, paraBuf);   // read parameters
    digitalWrite(_nSELPin, HIGH);
    return true;
}
bool RF4463::checkCTS() {
    if (xSemaphoreTake(CTSBusy, 20 / portTICK_PERIOD_MS) == pdTRUE) {
        if (CTSLineActive) {
            unsigned long start = millis();
            while (!digitalRead(GPIO1)) {
                if ((millis() - start) > 5) {
                    xSemaphoreGive(CTSBusy);
                    return false;
                }
                portYIELD();
            }
            xSemaphoreGive(CTSBusy);
            return true;
        }
        uint16_t timeOutCnt = RF4463_CTS_TIMEOUT;
        while (timeOutCnt--) {
            digitalWrite(_nSELPin, LOW);
            spiByte(RF4463_CMD_READ_BUF);  // send READ_CMD_BUFF command
            if (spiByte(0) == RF4463_CTS_REPLY) {
                digitalWrite(_nSELPin, HIGH);
                xSemaphoreGive(CTSBusy);
                return true;
            }
            digitalWrite(_nSELPin, HIGH);
            portYIELD();
        }
        xSemaphoreGive(CTSBusy);
        return false;
    } else {
        ets_printf("Failed to take CTS mutex\n");
        return false;
    }
}
void RF4463::spiWriteBuf(uint8_t writeLen, uint8_t* writeBuf) {
    rf.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    rf.transferBytes(writeBuf, NULL, writeLen);
    rf.endTransaction();
}
void RF4463::spiReadBuf(uint8_t readLen, uint8_t* readBuf) {
    while (readLen--)
        *readBuf++ = spiByte(0);
}
uint8_t RF4463::spiByte(uint8_t writeData) {
    uint8_t readData;
    rf.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    readData = rf.transfer(writeData);
    rf.endTransaction();
    return readData;
}
