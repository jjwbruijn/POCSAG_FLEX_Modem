
#include "encoder.h"

#include <Arduino.h>
#include <ESP32TimerInterrupt.h>
#include <ESPPerfectTime.h>

#include <vector>

#include "RF4463.h"
#include "bitutils.h"
#include "flexutils.h"
#include "jwifi.h"
#include "pagerkeepalive.h"
#include "pocsagutils.h"
#include "settings.h"
#include "statustracker.h"

#define POCSAG_SEND_MARK sendMark()
#define POCSAG_SEND_SPACE sendSpace()

struct txframe *frame[128] = {0};
struct statusframe *statusframearr[STATUSFRAMELISTSIZE];
enum pocsagtxstate { POCSAG_STATE_PREAMBLE,
                     POCSAG_STATE_FRAMESYNC,
                     POCSAG_STATE_BATCH };
struct pocsagstatusstruct {
    class pocsagdata *pocsagdata;
    uint8_t curbit;
    uint8_t curword;
    uint16_t curbatch;
    uint16_t preambleremain;
    pocsagtxstate state;
} pocsagstatus;

extern RF4463 rf4463;
ESP32Timer frametimer(0);
ESP32Timer bittimer(2);
bool frametimerstarted = false;

const char *ntpServer = "pool.ntp.org";

void IRAM_ATTR shiftStatusFrames() {
    if (statusframearr[STATUSFRAMELISTSIZE - 1]) {
        delete statusframearr[STATUSFRAMELISTSIZE - 1];
    }
    for (uint8_t c = STATUSFRAMELISTSIZE - 1; c > 0; c--) {
        statusframearr[c] = statusframearr[c - 1];
    }
    statusframearr[0] = new statusframe;
    statusframearr[0]->frameno = (uint8_t(flexstatus.frame + (STATUSFRAMELISTSIZE / 2))) % 128;
    statusframearr[0]->txCancelled = false;
    statusframearr[0]->isTX = false;
    statusframearr[0]->freq = 0;
    statusframearr[0]->rxtype = flexsynctype::SYNC_NOSYNC;
    statusframearr[0]->txformat = txframe::frameformat::FORMAT_IDLE;
    switch (settings.current.mode) {
        case MODE_FLEX:
            flexstatus.frame = (flexstatus.frame + 1) % 128;
            if (flexstatus.frame == 0) flexstatus.cycle = (flexstatus.cycle + 1) % 15;
            break;
        case MODE_FLEX_SYNCED:
        case MODE_TX_ONLY:
            for (uint8_t c = 0; c < ((STATUSFRAMELISTSIZE / 2) + 1); c++) {
                if (statusframearr[c]) {
                    if (frame[statusframearr[c]->frameno]) {
                        statusframearr[c]->txformat = frame[statusframearr[c]->frameno]->format;
                        if (frame[statusframearr[c]->frameno]->format == txframe::frameformat::FORMAT_POCSAG) {
                            statusframearr[c]->freq = frame[statusframearr[c]->frameno]->pocsag->freq;
                            statusframearr[c]->isTX = true;
                        }
                        if (frame[statusframearr[c]->frameno]->format == txframe::frameformat::FORMAT_FLEX) {
                            statusframearr[c]->freq = frame[statusframearr[c]->frameno]->flex->frequency;
                            statusframearr[c]->isTX = true;
                        }
                    } else {
                        statusframearr[c]->txformat = txframe::FORMAT_IDLE;
                    }
                }
            }
            break;
    }
    if (xPortInIsrContext() == pdTRUE) {
        sendStatusISR(STATUS_FRAMELIST_UPDATE);
    } else {
        sendStatus(STATUS_FRAMELIST_UPDATE);
    }
}

void IRAM_ATTR printTime(struct tm *tm, suseconds_t usec) {
    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d.%06ld ",
                  tm->tm_year + 1900,
                  tm->tm_mon + 1,
                  tm->tm_mday,
                  tm->tm_hour,
                  tm->tm_min,
                  tm->tm_sec,
                  usec);
}
void IRAM_ATTR sendPOCSAGBit() {
    switch (pocsagstatus.state) {
        case pocsagtxstate::POCSAG_STATE_PREAMBLE:
            if (pocsagstatus.curbatch % 2) {
                POCSAG_SEND_SPACE;
            } else {
                POCSAG_SEND_MARK;
            }
            pocsagstatus.curbatch--;
            if (pocsagstatus.curbatch == 0) {
                pocsagstatus.curbatch = 0;
                pocsagstatus.state = pocsagtxstate::POCSAG_STATE_FRAMESYNC;
                //ets_printf("<-PREAMBLE\n");
                pocsagstatus.curbit = 0;
            }
            break;
        case pocsagtxstate::POCSAG_STATE_FRAMESYNC:
            if ((SYNCWORD >> (31 - pocsagstatus.curbit)) & 0x01) {
                POCSAG_SEND_MARK;
            } else {
                POCSAG_SEND_SPACE;
            }
            pocsagstatus.curbit++;
            if (pocsagstatus.curbit == 32) {
                pocsagstatus.curbit = 0;
                pocsagstatus.curword = 0;
                pocsagstatus.state = pocsagtxstate::POCSAG_STATE_BATCH;
                //ets_printf("<- SYNC!\nBATCH:\n");
            }
            break;
        case pocsagtxstate::POCSAG_STATE_BATCH:
            if ((((uint32_t *)&(pocsagstatus.pocsagdata->batch[pocsagstatus.curbatch]))[pocsagstatus.curword] >> (31 - pocsagstatus.curbit)) & 0x01) {
                POCSAG_SEND_MARK;
            } else {
                POCSAG_SEND_SPACE;
            }
            pocsagstatus.curbit++;
            if (pocsagstatus.curbit == 32) {
                //ets_printf("<- curbatch = %u curword = %u\n", pocsagstatus.curbatch, pocsagstatus.curword);
                pocsagstatus.curbit = 0;
                pocsagstatus.curword++;
                if (pocsagstatus.curword == 16) {
                    pocsagstatus.curbatch++;
                    if (pocsagstatus.curbatch == pocsagstatus.pocsagdata->batchcount) {
                        //ets_printf("Done sending POCSAG\n");
                        bittimer.stopTimer();
                        //bittimer.disableTimer();
                        xTaskCreate(RF4463::stopTX, "stopTX", 5000, NULL, configMAX_PRIORITIES - 1, NULL);
                        delete pocsagstatus.pocsagdata;
                        pocsagstatus.pocsagdata = nullptr;
                    } else {
                        pocsagstatus.curword = 0;
                        pocsagstatus.state = pocsagtxstate::POCSAG_STATE_FRAMESYNC;
                    }
                }
            }
            break;
    }
}

void IRAM_ATTR startFrameAlignedTX(bool channelClear, bool shortPreamble) {
    struct txframe *curframe;
    uint8_t oldframe = flexstatus.frame;
    uint32_t idlefreq;
    flexstatus.frame = (flexstatus.frame + 1) % 128;
    if (flexstatus.frame == 0) flexstatus.cycle = (flexstatus.cycle + 1) % 15;
    curframe = frame[flexstatus.frame];

    //tm = pftime::localtime(nullptr, &usec);

    //printTime(tm, usec);
    ets_printf("Frame: %u cycle %u > ", flexstatus.frame, flexstatus.cycle);

    // if frame is not set (empty, check if we should be sending an idle frame for a flex pager)
    if (!curframe) {
        idlefreq = shouldSendIdleFlexFrame(flexstatus.frame);
        if (idlefreq) {  // send idle flex frame
            frame[flexstatus.frame] = new txframe(txframe::frameformat::FORMAT_FLEX);
            frame[flexstatus.frame]->flex = new flexframe(flexsynctype::SYNC_FLEX_1600);
            frame[flexstatus.frame]->flex->frequency = idlefreq;
            curframe = frame[flexstatus.frame];
        } else {
            frame[flexstatus.frame] = new txframe(txframe::frameformat::FORMAT_IDLE);
            curframe = frame[flexstatus.frame];
        }
    }

    switch (curframe->format) {
        case txframe::frameformat::FORMAT_BLOCKED:
            ets_printf("BLOCKED \n");
            break;
        case txframe::frameformat::FORMAT_POCSAG:
            ets_printf("POCSAG START \n");
            pocsagstatus.curbatch = 576;
            pocsagstatus.state = pocsagtxstate::POCSAG_STATE_PREAMBLE;
            pocsagstatus.pocsagdata = curframe->pocsag;
            xTaskCreate(RF4463::startPOCSAG, "startPOCSAGTX", 2000, (void *)curframe->pocsag->freq, configMAX_PRIORITIES - 1, NULL);
            switch (curframe->pocsag->baudrate) {
                case 512:
                    bittimer.changeFrequency(512);  //, sendPOCSAGBit);
                    sendStatusISR(STATUS_MODE_POCSAG512);
                    break;
                case 1200:
                    bittimer.changeFrequency(1200);  //, sendPOCSAGBit);
                    sendStatusISR(STATUS_MODE_POCSAG1200);
                    break;
                case 2400:
                    bittimer.changeFrequency(2400);  //, sendPOCSAGBit);
                    sendStatusISR(STATUS_MODE_POCSAG2400);
                    break;
            }
            bittimer.restartTimer();
            sendPOCSAGBit();
            portYIELD_FROM_ISR();
            resetIdleCounter(curframe->pocsag->freq, curframe->pocsag->baudrate);
            break;
        case txframe::frameformat::FORMAT_IDLE:
            // Nothing to send, not even an idle frame;
            ets_printf("IDLE \n");
            if (flexstatus.sendingflex) {
                xTaskCreate(RF4463::stopTX, "stopTX", 5000, NULL, configMAX_PRIORITIES - 1, NULL);
                portYIELD_FROM_ISR();
                flexstatus.sendingflex = false;
            }
            break;
        case txframe::frameformat::FORMAT_FLEX:
            ets_printf("FLEX \n");
            if (curframe->flex) {
                curframe->flex->rf4463 = &rf4463;
                curframe->flex->fiww.cycle = flexstatus.cycle;
                curframe->flex->fiww.frame = flexstatus.frame;
                curframe->flex->shortPreamble = shortPreamble;
                if ((channelClear) || (curframe->flex->frequency != settings.current.freq)) {
                    curframe->flex->startTX();
                    portYIELD_FROM_ISR();
                } else {
                    ets_printf("SKIPPED! \n");
                    for (uint8_t i = 0; i < STATUSFRAMELISTSIZE; i++) {
                        if (statusframearr[i]) {
                            if (statusframearr[i]->frameno == flexstatus.frame) {
                                statusframearr[i]->txCancelled = true;
                            }
                        }
                    }
                }
                // update status
                switch (curframe->flex->type) {
                    case flexsynctype::SYNC_FLEX_1600:
                        sendStatusISR(STATUS_MODE_FLEX1600);
                        break;
                    case flexsynctype::SYNC_FLEX_3200_2:
                        sendStatusISR(STATUS_MODE_FLEX3200_2);
                        break;
                    case flexsynctype::SYNC_FLEX_3200_4:
                        sendStatusISR(STATUS_MODE_FLEX3200_4);
                        break;
                    case flexsynctype::SYNC_FLEX_6400:
                        sendStatusISR(STATUS_MODE_FLEX6400_4);
                        break;
                }
            }
            resetIdleCounter(curframe->flex->frequency, flexstatus.frame);
            break;
        default:
            ets_printf("??? ");
            break;
    }
    decrementIdleCounter();

    if (frame[oldframe] != nullptr) {
        delete frame[oldframe];
        frame[oldframe] = nullptr;
    }
}

void IRAM_ATTR eachFrame() {
    if (settings.current.mode == MODE_TX_ONLY) {
        startFrameAlignedTX(true, false);
        shiftStatusFrames();
    }
}

void flexFrameCycleAlign(struct tm *tm, suseconds_t usec) {
    if (settings.current.mode != MODE_TX_ONLY) return;
    uint8_t sec = tm->tm_sec;
    uint8_t min = tm->tm_min;
    //  if (usec >= 500000) {
    sec += 1;
    if (sec == 60) {
        sec = 0;
        min++;
        if (min == 60) {
            min = 0;
        }
    }
    //}
    flexstatus.frame = ((min % 4) * 32) + ((32 * (uint32_t)sec) / 60);
    flexstatus.cycle = (min / 4);
    if (flexstatus.frame == 0) {
        flexstatus.frame = 127;
        if (flexstatus.cycle == 0) {
            flexstatus.cycle = 14;
        } else {
            flexstatus.cycle--;
        }
    } else {
        flexstatus.frame--;
    }
    eachFrame();
}

int addPOCSAG(pocsagaddr *addr, uint32_t freq, uint16_t baudrate, char *data) {
    bool spotFound = false;

    switch (baudrate) {
        case 512:
        case 1200:
        case 2400:
            if (!RF4463::checkAvail(freq, RF4463::POCSAG_2400_TX)) return -1;
            break;
        default:
            return -1;
    }

    if (registerPOCSAG(freq, baudrate)) saveIdleList();

    pocsagdata temp;
    temp.baudrate = baudrate;
    temp.addMsg(*addr, data);
    uint8_t cleanSize = temp.getFrameSize();

    //    uint8_t cleanSize = 8;
    for (uint8_t c = (flexstatus.frame + 1) % 128; c != flexstatus.frame;) {
        if (frame[c] != nullptr) {
            if (frame[c]->format == txframe::frameformat::FORMAT_POCSAG) {
                if (frame[c]->pocsag->freq == freq && frame[c]->pocsag->baudrate == baudrate) {
                    // found a pocsag frame that matches frequency and baudrate
                    uint8_t newsize = frame[c]->pocsag->getFrameSize(frame[c]->pocsag->batchcount + frame[c]->pocsag->requiredSize(*addr, data));
                    ets_printf("We need room for %u frames\n", newsize);
                    spotFound = true;
                    for (uint8_t d = 1; d < newsize; d++) {  // check if the frame will hold the new size
                        if (frame[(c + d) % 128] != nullptr) {
                            if (frame[(c + d) % 128]->format != txframe::frameformat::FORMAT_BLOCKED) {
                                spotFound = false;  // not gonna fit, we've encountered a different frame
                                c = (c + d) % 128;
                                break;
                            }
                        } else {  // empty frame
                        }
                    }
                    if (spotFound == true) {  // this new location will hold the new, larger frame.
                        frame[c]->pocsag->addMsg(*addr, data);
                        for (uint8_t d = 1; d < frame[c]->pocsag->getFrameSize(); d++) {
                            if (frame[(c + d) % 128] == nullptr)
                                frame[(c + d) % 128] = new txframe(txframe::frameformat::FORMAT_BLOCKED);
                        }
                        return c;
                        break;
                    }
                }
            } else {
                // other frame type
                //ets_printf("- Other frame found at %u\n", c);
                c = (c + 1) % 128;
            }
        } else {  // empty frame
            spotFound = true;
            for (uint8_t d = 1; d < cleanSize; d++) {
                if (frame[(c + d) % 128] != nullptr) {
                    if (frame[(c + d) % 128]->format != txframe::frameformat::FORMAT_BLOCKED) {
                        spotFound = false;  // not gonna fit, we've encountered a different frame
                        c = (c + d) % 128;
                        break;
                    } else {
                        // already blocked? weird but sure.
                    }
                } else {
                    // empty frame;
                }
            }
            if (spotFound == true) {
                pocsagdata *pocsag = new pocsagdata();
                frame[c] = new txframe(txframe::frameformat::FORMAT_POCSAG);
                frame[c]->pocsag = pocsag;
                frame[c]->pocsag->freq = freq;
                frame[c]->pocsag->baudrate = baudrate;
                frame[c]->pocsag->addMsg(*addr, data);
                //ets_printf("Spot found at frame %u - %u frames\n", c, pocsag->getFrameSize());
                for (uint8_t d = 0; d < frame[c]->pocsag->getFrameSize(); d++) {
                    if (frame[(c + d) % 128] == nullptr) {
                        frame[(c + d) % 128] = new txframe(txframe::frameformat::FORMAT_BLOCKED);
                        //ets_printf("  %u is blocked\n", (c + d) % 128);
                        delay(2);
                    } else {
                    }
                }
                //ets_printf(" Done allocating\n");
                delay(2);
                return c;
            }
        }
    }
    return -1;
}

int addFLEX(char pre, uint32_t ric, uint32_t freq, char *data, bool isNumeric) {
    if (!RF4463::checkAvail(freq, RF4463::rftype::FLEX_6400_TX)) return -1;
    // TODO: This needs a lot of work...
    flexmsg *msg = new flexmsg;
    msg->addAddress(ric);
    if (!isNumeric) {
        msg->encode(flexvector::vectortype::VECTOR_ALPHA, data);
    } else {
        msg->encode(flexvector::vectortype::VECTOR_NUMERIC, data);
    }
    uint8_t firstframe;

    // check if we've addressed this specific pager/frame before. If not, send an extra frame before the message itself in order for
    // the pager to sync up to the frequency. TODO: this fails if another pager with same frame assignment/frequency has already been
    // addressed
    if (registerFrame(freq, ric, pre)) {
        saveIdleList();
        for (firstframe = 0; firstframe < 128; firstframe++) {
            //check if frame is empty
            if (!frame[(firstframe + flexstatus.frame + 1) % 128]) {
                //make an idle frame
                frame[(firstframe + flexstatus.frame + 1) % 128] = new txframe(txframe::frameformat::FORMAT_FLEX);
                frame[(firstframe + flexstatus.frame + 1) % 128]->flex = new flexframe(flexsynctype::SYNC_FLEX_1600);
                frame[(firstframe + flexstatus.frame + 1) % 128]->flex->frequency = freq;
                break;
            }
        }
        firstframe = (firstframe + flexstatus.frame + 2) % 128;
    } else {
        firstframe = flexstatus.frame + 1;
    }

    // check if we're sending on a slave-synced FLEX network. In that case, we need to follow the RX frame collapse for this pager
    bool rxcollapse = false;
    if ((freq == settings.current.freq) && (settings.current.mode == MODE_FLEX_SYNCED)) {
        rxcollapse = true;
    }

    // look for the nearest frame that the pager should listen to
    uint8_t eligibleframe = flexaddress::getNextFrame(ric, pre, firstframe, rxcollapse);
    uint8_t firsttxframe = eligibleframe;
    ets_printf("Next position is %u \n", eligibleframe);
    bool msgDone = false;
    bool cycleComplete = false;
    uint8_t firsteligible = eligibleframe;

    // loop until we've queued the entire message, or finished the entire cycle
    while (!msgDone && !cycleComplete) {
        // check if the frame is in use
        if (frame[eligibleframe] != nullptr) {
            if (frame[eligibleframe]->format == txframe::FORMAT_FLEX) {
                // check if the flex frame is on the same frequency
                if (frame[eligibleframe]->flex->frequency == freq) {
                    uint8_t freeWords = frame[eligibleframe]->flex->phase[0]->freeWords();
                    uint8_t needed = 7;
                    if (isNumeric) needed = msg->wordcount;
                    if (freeWords > needed) {  // free space in current phase
                        if (isNumeric) {
                            frame[eligibleframe]->flex->phase[0]->addBlock(msg->getBlock());
                        } else {
                            frame[eligibleframe]->flex->phase[0]->addBlock(msg->getBlock(freeWords - 3));
                        }
                        eligibleframe = (eligibleframe + 1) % 128;
                    } else {  // no free space
                        eligibleframe = flexaddress::getNextFrame(ric, pre, eligibleframe + 1, rxcollapse);
                    }
                    // flex frame but on the wrong frequency.
                } else {
                    eligibleframe = flexaddress::getNextFrame(ric, pre, eligibleframe + 1, rxcollapse);
                }
                // not a flex frame
            } else {
                eligibleframe = flexaddress::getNextFrame(ric, pre, eligibleframe + 1, rxcollapse);
            }
            // frame not in use
        } else {
            // create a frame
            frame[eligibleframe] = new txframe(txframe::frameformat::FORMAT_FLEX);
            frame[eligibleframe]->flex = new flexframe(flexsynctype::SYNC_FLEX_6400);
            frame[eligibleframe]->flex->frequency = freq;
            frame[eligibleframe]->flex->phase[0]->addBlock(msg->getBlock(70));
            eligibleframe = (eligibleframe + 1) % 128;
        }
        if (msg->wordcount == 0) msgDone = true;
        if (eligibleframe == firsteligible) cycleComplete = true;
    }
    delete msg;
    if (cycleComplete) {
        return -1;
    } else {
        return firsttxframe;
    }
}

void pocsagTimeoutTask(void *parameter) {
    // check if it's been a while since we've shown activity on a previously used channel
    while (true) {
        if ((settings.current.mode == MODE_TX_ONLY) || (settings.current.mode == MODE_FLEX_SYNCED)) {
            idlelistentry pocsagidle = shouldSendIdlePOCSAG();
            if (pocsagidle.freq) {
                pocsagaddr adr(1337, pocsagaddr::TONE, pocsagaddr::A);
                addPOCSAG(&adr, pocsagidle.freq, pocsagidle.frame, (char *)"");
                resetIdleCounter(pocsagidle.freq, pocsagidle.frame);
                ets_printf("Added expiring pocsag\n");
            }
        }
        vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}

void syncTask(void *parameter) {
    // load list with pagers to keepalive
    loadIdleList();
    // start task to keep pocsag pagers alive
    xTaskCreate(pocsagTimeoutTask, "PocsagTimeoutTask", 1000, NULL, 1, NULL);

    //uint8_t min;
    time_t t;
    struct tm *tm;
    suseconds_t usec;
    flexstatus.sendingflex = false;
    bittimer.attachInterrupt(1200, sendPOCSAGBit);
    bittimer.stopTimer();

    while(1){
        vTaskDelay(200);
    }
    /*
    tm = pftime::localtime(nullptr, &usec);
    addFLEX('E', 6477833, 929612500, (char *)"Dit is een test\0");

    frame[7] = new txframe(txframe::frameformat::FORMAT_FLEX);
    frame[8] = new txframe(txframe::frameformat::FORMAT_FLEX);
    frame[9] = new txframe(txframe::frameformat::FORMAT_FLEX);

    frame[10] = new txframe(txframe::frameformat::FORMAT_POCSAG);
    frame[10]->pocsag = new pocsagdata();
    frame[10]->pocsag->freq = 460918750;
    frame[10]->pocsag->baudrate = 1200;
    pocsagaddr adr(11548, pocsagaddr::ALPHA, pocsagaddr::D);
    frame[10]->pocsag->addMsg(adr, "Minister Cora van Nieuwenhuizen (Infrastructuur) heeft vrijdag de deur op een kiertje gezet voor KLM om niet aan de aankomende sneltestplicht te hoeven voldoen. De luchtvaartmaatschappij moet wel, in overleg met het RIVM, voor een veilig alternatief zorgen. KLM zei eerder deze week als gevolg van de sneltestplicht, waarbij alle passagiers voor vertrek naar Nederland een negatieve sneltest moeten kunnen tonen, de langeafstandsvluchten te moeten schrappen. De luchtvaartmaatschappij is onder meer bang dat ze hierdoor bemanningsleden die positief testen in het buitenland moeten achterlaten.\0");
    frame[10]->pocsag->addMsg(adr, "Dit is een test!!\0");
    for (uint8_t c = 1; c < frame[10]->pocsag->getFrameSize(); c++) {
        frame[10 + c] = frame[13] = new txframe(txframe::frameformat::FORMAT_BLOCKED);
        ets_printf("Frame %u blocked!\n", 10 + c);
    }


    frame[7]->flex = new flexframe(flexsynctype::SYNC_FLEX_6400);
    frame[7]->flex->frequency = 929612500;

    frame[8]->flex = new flexframe(flexsynctype::SYNC_FLEX_6400);
    frame[8]->flex->frequency = 929612500;

    frame[9]->flex = new flexframe(flexsynctype::SYNC_FLEX_6400);
    frame[9]->flex->frequency = 929612500;

    flexmsg amsg;
    amsg.addAddress(9296125);
    amsg.encode(flexvector::vectortype::VECTOR_ALPHA, (char *)"Socialemediaplatformen nemen in toenemende mate afstand van president Donald Trump\0");
    frame[7]->flex->phase[0]->addBlock(amsg.getBlock(70));

    flexmsg bmsg;
    bmsg.addAddress(9296125);
    bmsg.encode(flexvector::vectortype::VECTOR_ALPHA, (char *)"Maar ook van platformen waarop veel van zijn aanhangers zich bevinden. Met name de permaban op Twitter moet de Amerikaanse president pijn hebben gedaan. Die zei in het verleden al eens dat hij zonder zijn Twitter-account nooit zover zou zijn gekomen in zijn korte politieke carrière.\0");
    frame[8]->flex->phase[0]->addBlock(bmsg.getBlock(70));

    flexmsg cmsg;
    cmsg.addAddress(9296124);
    cmsg.encode(flexvector::vectortype::VECTOR_ALPHA, (char *)"Socialemediaplatformen nemen in toenemende mate afstand van president Donald Trump\0");
    frame[9]->flex->phase[0]->addBlock(cmsg.getBlock(70));
    frame[9]->flex->phase[0]->addBlock(bmsg.getBlock(70));
    ets_printf("Free words in frame 9: %u \n", frame[9]->flex->phase[0]->freeWords());

*/


    while (!wifiConnected) {
        vTaskDelay(3000);
    }
    pftime::configTzTime(PSTR("UTC0"), ntpServer);
    vTaskDelay(3000);

    Serial.println("Starting Time Sync");
    pftime::configTzTime(PSTR("UTC0"), ntpServer);
    while (true) {
        t = pftime::time(nullptr);
        if (t > 1607730840) break;
        if (t > 59 && t % 60 == 0) {
            Serial.println("Resyncing time");
            Serial.println("Time is now:");
            Serial.println(String(t));
            pftime::configTzTime(PSTR("UTC0"), ntpServer);
        }
        vTaskDelay(1000);
    }

    Serial.println("Time obtained:");
    tm = pftime::localtime(nullptr, &usec);
    printTime(tm, usec);
    vTaskDelay(1000 * (14 - (tm->tm_sec % 15)));
    Serial.println("back!");
    while (1) {
        tm = pftime::localtime(nullptr, &usec);
        switch (tm->tm_sec) {
            case 14:
            case 29:
            case 44:
                //if (frametimerstarted) break;
            case 59:
                delay(1);
                pftime::localtime(nullptr, &usec);
                if (usec > 930000) {
                    if (frametimerstarted) frametimer.stopTimer();
                    pftime::localtime(nullptr, &usec);
                    ets_delay_us((1000000 - 28) - usec);
                    if (frametimerstarted) {
                        frametimer.restartTimer();
                    } else {
                        frametimerstarted = true;
                        frametimer.attachInterruptInterval(1875000, eachFrame);
                        ets_printf("synctask stack words free: %u \n", uxTaskGetStackHighWaterMark(NULL));
                        vTaskDelay(5800 / portTICK_PERIOD_MS);
                    }
                    flexFrameCycleAlign(tm, usec);
                    Serial.println("Sync");
                    vTaskDelay(800);
                }
                break;
            default:
                vTaskDelay(800);
                break;
        }
        vTaskDelay(40);
    }
}

// STRUCT FUNCTIONS

txframe::txframe(enum frameformat formatc) {
    switch (formatc) {
        case frameformat::FORMAT_FLEX:
            break;
        case frameformat::FORMAT_POCSAG:
            break;
        case frameformat::FORMAT_BLOCKED:
            break;
        default:
            break;
    }
    format = formatc;
}

txframe::~txframe() {
    if (flex) {
        delete flex;
    }
    if (pocsag) {
        //delete pocsag;
    }
}