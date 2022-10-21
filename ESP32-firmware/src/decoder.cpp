#include "decoder.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32TimerInterrupt.h>
#include <ESP32_ISR_Timer.h>
#include <ESPPerfectTime.h>
#include <Wire.h>
#include <esp_wifi.h>

#include "BCH3121.h"
#include "RF4463.h"
#include "WiFi.h"
#include "bitutils.h"
#include "encoder.h"
#include "flexutils.h"
#include "pocsagutils.h"
#include "settings.h"
#include "statustracker.h"

CBCH3121 bch;
extern RF4463 rf4463;
flexsync flsync;
QueueHandle_t frameQueue;
QueueHandle_t pocsagQueue;
SemaphoreHandle_t wsMutex = NULL;
#define FRAMEQUEUESIZE 20

//std::vector<flexframe*> flexframes;

extern std::vector<flextempaddrmapping*> flexmappings;
extern std::vector<flexmsg*> flexmessages;


volatile bool resetRX;
ESP32Timer syncedflex(1);
volatile bool syncedslew = false;
pocsagdata pocsagrx;
extern AsyncWebSocket ws;

struct ringbuffer {
    volatile uint8_t buffer[256];
    volatile uint8_t tail = 0;
    volatile uint8_t head = 0;
    uint8_t bitcount = 0;
} pocsagring;

struct pocsagrx {
    enum pocsagstate {
        STATE_WAIT_PREAMBLE,
        STATE_SYNC,
        STATE_BATCH
    } state;
    uint32_t prevword = 0;
    uint32_t curword = 0;
    uint8_t curbit = 0;
    uint8_t wordcount = 0;
} pocsagRX;

void pocsagReadBit(uint8_t val) {
    uint16_t errors = 0;
    uint32_t copy;
    bool bchtest = false;
    switch (pocsagRX.state) {
        case pocsagrx::pocsagstate::STATE_WAIT_PREAMBLE:
        case pocsagrx::pocsagstate::STATE_SYNC:  // waiting for sync
            if (pocsagRX.curword & 0x80000000) {
                pocsagRX.prevword <<= 1;
                pocsagRX.prevword |= 1;
            } else {
                pocsagRX.prevword <<= 1;
            }

            pocsagRX.curword <<= 1;
            if (val == 0) {
                pocsagRX.curword |= 1;
            }

            // Check for syncframe
            if (countbits(pocsagRX.prevword ^ 0xAAAAAAAA) < 4) {
                copy = pocsagRX.curword;
                if ((bchtest = bch.decode(copy, errors))) {
                    if (pocsagRX.curword == SYNCWORD) {
                        //ets_printf("Sync!\n");
                        pocsagRX.state = pocsagrx::pocsagstate::STATE_BATCH;
                        sendStatus(STATUS_RX_SIGNAL);
                        pocsagRX.curbit = 32;
                        pocsagRX.wordcount = 0;
                    }
                } else if ((countbits(pocsagRX.curword ^ SYNCWORD) <= 4)) {
                    Serial.println("!!! NOISY START!");
                    pocsagRX.state = pocsagrx::pocsagstate::STATE_BATCH;
                    sendStatus(STATUS_RX_SIGNAL);
                    pocsagRX.curbit = 32;
                    pocsagRX.wordcount = 0;
                }
            }
            break;
        case pocsagrx::pocsagstate::STATE_BATCH:
            pocsagRX.curword <<= 1;
            if (val == 0) pocsagRX.curword |= 1;
            pocsagRX.curbit--;

            // SYNCWORD WINDOW
            if ((pocsagRX.wordcount == 16 && pocsagRX.curbit < 2) || (pocsagRX.wordcount == 16 && pocsagRX.curbit > 32)) {
                uint32_t copy = pocsagRX.curword;
                bchtest = bch.decode(copy, errors);
                if (bchtest && copy == SYNCWORD) {
                    if (pocsagRX.curbit != 0) {
                        Serial.print("bit: ");
                        Serial.print(pocsagRX.curbit);
                        Serial.println(" DRIFTED SYNC");
                    } else {
                        //ets_printf("RESYNC\n");
                    }
                    pocsagRX.wordcount = 0;
                    pocsagRX.curbit = 32;
                    pocsagRX.curword = 0;
                } else if (bchtest && pocsagRX.curbit == 0) {  // valid word but not sync
                    pocsagrx.addWord(IDLEWORD, pocsagRX.wordcount, !bchtest);
                    pocsagRX.state = pocsagrx::pocsagstate::STATE_SYNC;
                    sendStatus(STATUS_RX_NO_SIGNAL);
                } else if ((countbits(pocsagRX.curword ^ SYNCWORD) <= 4) && (pocsagRX.curbit == 0)) {  // max hamming distance = 4 from sync word (noisy sync)
                    Serial.print("bit: ");
                    Serial.print(pocsagRX.curbit);
                    Serial.println("NOISY SYNC!~");
                    pocsagRX.wordcount = 0;
                    pocsagRX.curbit = 32;
                    pocsagRX.curword = 0;
                } else if (countbits(pocsagRX.curword ^ SYNCWORD) <= 3) {  // max hamming distance = 4 from sync word (drifted sync)
                    Serial.print("bit: ");
                    Serial.print(pocsagRX.curbit);
                    Serial.println("DRIFTED NOISY SYNC!~");
                    pocsagRX.wordcount = 0;
                    pocsagRX.curbit = 32;
                    pocsagRX.curword = 0;
                } else if (pocsagRX.curbit > 32 && pocsagRX.curbit <= 253) {  // lost sync somewhere
                    pocsagrx.addWord(IDLEWORD, pocsagRX.wordcount, false);
                    pocsagRX.state = pocsagrx::pocsagstate::STATE_SYNC;
                    sendStatus(STATUS_RX_NO_SIGNAL);
                } else {
                    // continue
                }
            } else if (!pocsagRX.curbit) {  // not in syncword window
                bchtest = bch.decode(pocsagRX.curword, errors);
                pocsagrx.addWord(pocsagRX.curword, pocsagRX.wordcount, !bchtest);
                pocsagRX.wordcount++;
                pocsagRX.curbit = 32;
            }
            break;
    }
}
void IRAM_ATTR pocsagRXInterrupt() {
    // Gets called on every rising clock edge (generated by RF module). Saves clocked bits into a ringbuffer
    // 1200 bits per second, stored in 255 bytes. At the very minimum, the flexTask should be fired every second (1.7s of storage);
    pocsagring.buffer[pocsagring.head] <<= 1;
    pocsagring.buffer[pocsagring.head] |= digitalRead(13);
    //ets_printf("%u",digitalRead(13));
    pocsagring.bitcount++;
    if (pocsagring.bitcount == 8) {
        pocsagring.bitcount = 0;
        pocsagring.head++;
    }
}
void pocsagTask(void* parameter) {
    xTaskCreate(pocsagProcessing, "pocsagProcessing", 10000, NULL, 2, NULL);
    pocsagRX.state = pocsagrx::pocsagstate::STATE_SYNC;
    // reads data from the ringbuffer, calling readbit for every bit
    while (1) {
        while (pocsagring.tail != pocsagring.head) {
            for (uint8_t c = 0; c < 8; c++) {
                pocsagReadBit((pocsagring.buffer[pocsagring.tail]) >> 7);
                pocsagring.buffer[pocsagring.tail] <<= 1;
            }
            pocsagring.tail++;
        }
        vTaskDelay(100);
    }
}

void pocsagProcessing(void* parameter) {
    pocsagQueue = xQueueCreate(FRAMEQUEUESIZE, sizeof(pocsagmsg*));
    pocsagmsg* msg;
    while (true) {
        BaseType_t rx = xQueueReceive(pocsagQueue, &msg, portMAX_DELAY);
        if (rx == pdTRUE) {
            char* alpha = msg->alphadata;
            char* numeric = msg->numdata;
            pocsagaddr addr = msg->addr;
            enum {
                ALPHA,
                NUMERIC,
                TONE
            } msgtype = ALPHA;

            // determine encoding
            if (settings.pocsag.typebymessagecontent) {
                int rating = pocsagdata::getRating(msg->alphadata, msg->numdata, msg->addr);
                if (rating >= 0) {
                    msgtype = ALPHA;
                } else {
                    msgtype = NUMERIC;
                }
                if (strlen(numeric) <= 1) {
                    msgtype = TONE;
                }
            } else {
                switch (addr.func) {
                    case pocsagaddr::D:
                        msgtype = ALPHA;
                        break;
                    case pocsagaddr::B:
                    case pocsagaddr::C:
                        msgtype = TONE;
                        break;
                    case pocsagaddr::A:
                        msgtype = NUMERIC;
                        break;
                }
            }
            // end determine encoding

            uint16_t errorStateSwitchLen = 0;
            bool errorState = false;
            char* outputbuffer = 0;
            char* writep;
            switch (msgtype) {
                case ALPHA:
                    if (settings.pocsag.markerrors) {
                        for (uint16_t c = 0; c < strlen(alpha); c++) {
                            if (!errorState && (alpha[c] & 0x80)) {
                                errorStateSwitchLen += 3;
                                errorState = true;
                            } else if (errorState && !(alpha[c] & 0x80)) {
                                errorStateSwitchLen += 4;
                                errorState = false;
                            }
                        }
                        if (errorState) errorStateSwitchLen += 4;
                    }
                    outputbuffer = (char*)malloc(strlen(alpha) + errorStateSwitchLen + 1);
                    writep = outputbuffer;
                    errorState = false;
                    for (uint16_t c = 0; c < strlen(alpha); c++) {
                        if (settings.pocsag.markerrors) {
                            if (!errorState && (alpha[c] & 0x80)) {
                                errorState = true;
                                *writep = 0x3C;
                                writep++;
                                *writep = 0x69;
                                writep++;
                                *writep = 0x3E;
                                writep++;
                            } else if (errorState && !(alpha[c] & 0x80)) {
                                errorState = false;
                                *writep = 0x3C;
                                writep++;
                                *writep = 0x2F;
                                writep++;
                                *writep = 0x69;
                                writep++;
                                *writep = 0x3E;
                                writep++;
                            }
                        }
                        if ((alpha[c] & 0x7F) > 0x1F) {
                            *writep = (alpha[c] & 0x7F);
                        } else if ((alpha[c] & 0x7F) == 0x0D) {
                            *writep = 0x0D;
                        } else {
                            *writep = 0x20;
                        }

                        *writep = alpha[c] & 0x7F;
                        writep++;
                    }
                    *writep = 0;
                    Serial.println(outputbuffer);
                    break;
                case NUMERIC:
                    if (settings.pocsag.markerrors) {
                        for (uint16_t c = 0; c < strlen(numeric); c++) {
                            if (!errorState && (numeric[c] & 0x80)) {
                                errorStateSwitchLen += 3;
                                errorState = true;
                            } else if (errorState && !(numeric[c] & 0x80)) {
                                errorStateSwitchLen += 4;
                                errorState = false;
                            }
                        }
                        if (errorState) errorStateSwitchLen += 4;
                    }
                    outputbuffer = (char*)malloc(strlen(numeric) + errorStateSwitchLen + 1);
                    writep = outputbuffer;
                    errorState = false;
                    for (uint16_t c = 0; c < strlen(numeric); c++) {
                        if (settings.pocsag.markerrors) {
                            if (!errorState && (numeric[c] & 0x80)) {
                                errorState = true;
                                *writep = 0x3C;
                                writep++;
                                *writep = 0x69;
                                writep++;
                                *writep = 0x3E;
                                writep++;
                            } else if (errorState && !(numeric[c] & 0x80)) {
                                errorState = false;
                                *writep = 0x3C;
                                writep++;
                                *writep = 0x2F;
                                writep++;
                                *writep = 0x69;
                                writep++;
                                *writep = 0x3E;
                                writep++;
                            }
                        }
                        if ((numeric[c] & 0x7F) > 0x1F) {
                            *writep = (numeric[c] & 0x7F);
                        } else if ((numeric[c] & 0x7F) == 0x0D) {
                            *writep = 0x0D;
                        } else {
                            *writep = 0x20;
                        }

                        *writep = numeric[c] & 0x7F;
                        writep++;
                    }
                    *writep = 0;
                    break;
                case TONE:
                    break;
            }

            DynamicJsonDocument doc(100 + strlen(alpha) + errorStateSwitchLen + 1);
            doc["type"] = "POCSAG";
            doc["ric"] = addr.ric & 0x7FFFFFFF;

            ets_printf("%u|", addr.ric & 0x7FFFFFFF);
            switch (addr.func) {
                case pocsagaddr::A:
                    doc["function"] = "A";
                    ets_printf("A> ");
                    break;
                case pocsagaddr::B:
                    doc["function"] = "B";
                    ets_printf("B> ");
                    break;
                case pocsagaddr::C:
                    doc["function"] = "C";
                    ets_printf("C> ");
                    break;
                case pocsagaddr::D:
                    doc["function"] = "D";
                    ets_printf("D> ");
                    break;
            }
            switch (msgtype) {
                case ALPHA:
                    doc["msg"] = outputbuffer;
                    doc["encoding"] = "alpha";
                    ets_printf(outputbuffer);

                    break;
                case NUMERIC:
                    doc["msg"] = outputbuffer;
                    doc["encoding"] = "numeric";
                    ets_printf(outputbuffer);
                    break;
                case TONE:
                    doc["msg"] = "<TONE ONLY>";
                    doc["encoding"] = "tone";
                    ets_printf("<TONE ONLY>");
                    break;
            }
            doc["time"] = pftime::time(nullptr);
            ets_printf("\n");
            size_t len = measureJson(doc);
            xSemaphoreTake(wsMutex, portMAX_DELAY);
            auto buffer = std::make_shared<std::vector<uint8_t>>(len);
            serializeJson(doc, buffer->data(), len);
            ws.textAll(buffer);
            xSemaphoreGive(wsMutex);
            delete msg;
            if (outputbuffer) free(outputbuffer);
        }
        vTaskDelay(150 / portTICK_PERIOD_MS);
    }
}

// called every bit (1600x every second) for flex sync recognition
void IRAM_ATTR flexsynchelper() {
    flsync.bitInterrupt(digitalRead(13));
}

// this task takes the FLEX messages from the queue and processes them (to websocket/MQTT?)
void flexProcessing(void* parameter) {
    frameQueue = xQueueCreate(FRAMEQUEUESIZE, sizeof(flexframe*));
    flexframe* f;
    while (true) {
        BaseType_t rx = xQueueReceive(frameQueue, &f, portMAX_DELAY);
        if (rx == pdTRUE) {
            //delete f;
            std::vector<flexmsg*>* messages = f->decode();
            flexstatus.rxcollapse = f->phase[0]->biwone.collapse;
            // calculate needed json buffer space
            uint16_t buffersize = 256;
            for (uint8_t c = 0; c < messages->size(); c++) {
                taskYIELD();
                flexmsg* msg = messages->at(c);
                if (msg) {
                    switch (msg->type) {
                        case flexvector::vectortype::VECTOR_ALPHA:
                            buffersize += strlen(msg->getAlphaData());
                            buffersize += 160;
                            for (uint8_t d = 0; d < msg->addrarr.size(); d++) {
                                buffersize += 16;
                            }
                            break;
                        case flexvector::vectortype::VECTOR_NUMERIC:
                        case flexvector::vectortype::VECTOR_NUMBEREDNUMERIC:
                        case flexvector::vectortype::VECTOR_SPECIAL:
                            buffersize += strlen(msg->getNumericData());
                            buffersize += 160;
                            for (uint8_t d = 0; d < msg->addrarr.size(); d++) {
                                buffersize += 16;
                            }
                            break;
                    }
                }
            }
            DynamicJsonDocument doc(buffersize);
            doc["frame"] = f->fiww.frame;
            doc["cycle"] = f->fiww.cycle;
            doc["time"] = pftime::time(nullptr);
            JsonArray msgarr = doc.createNestedArray("messages");
            switch (f->type) {
                case flexsynctype::SYNC_FLEX_1600:
                    doc["type"] = "FLEX_1600_2";
                    break;
                case flexsynctype::SYNC_FLEX_3200_2:
                    doc["type"] = "FLEX_3200_2";
                    break;
                case flexsynctype::SYNC_FLEX_3200_4:
                    doc["type"] = "FLEX_3200_4";
                    break;
                case flexsynctype::SYNC_FLEX_6400:
                    doc["type"] = "FLEX_6400";
                    break;
                default:
                    break;
            }

            // add the messages to the JSON array of messages
            for (uint8_t c = 0; c < messages->size(); c++) {
                taskYIELD();
                flexmsg* msg = messages->at(c);
                if (msg) {
                    JsonObject jsonmsg = msgarr.createNestedObject();
                    JsonArray addrarr = jsonmsg.createNestedArray("addr");
                    switch (msg->type) {
                        case flexvector::vectortype::VECTOR_ALPHA:
                            jsonmsg["retrieval"] = msg->retrieval;
                            jsonmsg["id"] = msg->messageno;
                            jsonmsg["maildrop"] = msg->maildrop;
                            for (uint8_t d = 0; d < msg->addrarr.size(); d++) {
                                addrarr.add(msg->addrarr.at(d)->ric);
                            }
                            jsonmsg["msg"] = msg->getAlphaData();
                            delete msg;
                            break;
                        case flexvector::vectortype::VECTOR_NUMERIC:
                        case flexvector::vectortype::VECTOR_NUMBEREDNUMERIC:
                        case flexvector::vectortype::VECTOR_SPECIAL:
                            jsonmsg["retrieval"] = msg->retrieval;
                            jsonmsg["id"] = msg->messageno;
                            jsonmsg["maildrop"] = msg->maildrop;
                            for (uint8_t d = 0; d < msg->addrarr.size(); d++) {
                                addrarr.add(msg->addrarr.at(d)->ric);
                            }
                            jsonmsg["msg"] = msg->getNumericData();
                            delete msg;
                            break;
                        default:
                            ets_printf("Unhandled message type in decoder.cpp/flexProcessing at <adding to json array>\n");
                            break;
                    }
                }
            }
            size_t len = measureJson(doc);
            xSemaphoreTake(wsMutex, portMAX_DELAY);
            auto buffer = std::make_shared<std::vector<uint8_t>>(len);
            serializeJson(doc, buffer->data(), len);
            heap_caps_check_integrity_all(true);
            ws.textAll(buffer);
            heap_caps_check_integrity_all(true);
            xSemaphoreGive(wsMutex);
            delete f;
            delete messages;
            f = nullptr;
            heap_caps_check_integrity_all(true);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// this task removes stale mappings and messages
void removeTask(void* parameter) {
    while (1) {
        flextempaddrmapping::removeStaleMappings();
        flexmsg::removeStaleMessages();
        flexmessages.shrink_to_fit();
        flexmappings.shrink_to_fit();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //ets_printf("removeTask stack words free: %u \n", uxTaskGetStackHighWaterMark(NULL));
    }
}

// this function is called on the frame boundary + 4k-ish µS. Checks if we received anything that might resemble a bitsync pattern
void IRAM_ATTR checkFrameActive() {
    if (syncedslew) {
        syncedflex.changeInterval(1875000);
        syncedflex.restartTimer();
        syncedslew = false;
        sendStatusISR(STATUS_SYNCED_FLEX);
    }

    if (settings.current.mode == MODE_FLEX_SYNCED) {
        //ets_printf("FIWword = %X \n", sync.FIWword);
        if (((flsync.FIWword & 0xFF) == 0xAA) ||
            ((flsync.FIWword & 0xFF) == 0x55) ||
            ((flsync.FIWword & 0xF) == 0xA) ||
            ((flsync.FIWword == 0x05))) {
            //ets_printf("SYNCED ^^^^ NEW FRAME\n");
            startFrameAlignedTX(false, true);  // channel not clear
        } else {
            startFrameAlignedTX(true, true);  // channel clear
            if (status.rxSignal) sendStatusISR(STATUS_RX_NO_SIGNAL);
        }
    } else {
    }
    if (settings.current.mode != MODE_POCSAG) {
        shiftStatusFrames();
    }
}

void flexTask(void* parameter) {
    uint32_t ulNotificationValue;
    uint8_t buffer[64];
    int8_t count = 0;
    int16_t left = 352;
    uint16_t errors;
    uint8_t loopcount = 0;
    flexframe* f;

    flsync.flexRXTaskId = xTaskGetCurrentTaskHandle();
    flexstatus.rxcollapse = 4;

    ets_printf("Starting FLEX RX\n");
    xTaskCreate(flexProcessing, "flexProcessing", 10000, NULL, 2, NULL);
    xTaskCreate(removeTask, "removeTask", 2000, NULL, 2, NULL);
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    settings.current.freq = settings.rf.defaultfrequency;
    settings.current.baud = settings.rf.defaultbaud;
    settings.current.mode = settings.rf.defaultmode;
    restartDecoder();
    syncedflex.attachInterruptInterval(1875000, checkFrameActive);
    while (true) {
        ulNotificationValue = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));
        if (ulNotificationValue == 1) {  // loop until the semaphore tells us a bit sync has occurred
            switch (flsync.type) {
                // switch to the correct modulation type and sync header
                case flexsynctype::SYNC_FLEX_1600:
                    rf4463.enterReadyMode();
                    break;
                case flexsynctype::SYNC_FLEX_3200_2:
                    rf4463.init(settings.current.freq, RF4463::rftype::FLEX_3200_2_RX);
                    break;
                case flexsynctype::SYNC_FLEX_3200_4:
                    rf4463.init(settings.current.freq, RF4463::rftype::FLEX_3200_4_RX);
                    break;
                case flexsynctype::SYNC_FLEX_6400:
                    rf4463.init(settings.current.freq, RF4463::rftype::FLEX_6400_RX);
                    break;
                default:
                    ets_printf("Didn't reinit!\n");
                    break;
            }
            rf4463.fifoReset();
            rf4463.startPacketRX(352 * flexframe::getMultiplex(flsync.type));
            bch.decode(flsync.FIWword, errors);
            f = new flexframe(flsync.type, flsync.FIWword);
            for (uint8_t i = 0; i < STATUSFRAMELISTSIZE; i++) {
                if (statusframearr[i]) {
                    if (statusframearr[i]->frameno == f->fiww.frame) {
                        statusframearr[i]->freq = settings.current.freq;
                        statusframearr[i]->rxtype = flsync.type;
                        statusframearr[i]->isTX = false;
                    }
                }
            }
            sendStatus(STATUS_RX_SIGNAL);
            left = 352 * f->multiplex;
            loopcount = 0;
            ets_printf("Frame %u - Cycle %u         - Heap: %u + %u (map)+ %u (msg)\n", f->fiww.frame, f->fiww.cycle, ESP.getFreeHeap(), flexmappings.capacity(), flexmessages.capacity());
            flexstatus.frame = f->fiww.frame;
            flexstatus.cycle = f->fiww.cycle;
            while (flsync.isSynced == true) {
                count = rf4463.getRXFIFOSize();
                if (count > 0) {
                    if (count > left) count = left;
                    rf4463.readRxFifo(buffer, count);
                    for (uint8_t c = 0; c < count; c++) {
                        if (f) f->addByte(buffer[c]);
                        heap_caps_check_integrity_all(true);
                    }
                    left -= count;
                } else if (count == -1) {
                    ets_printf("RF4463 failed during FLEX RX, resetting");
                    loopcount = 251;
                    rf4463.powerOnReset();
                } else {
                    loopcount++;
                }
                if (left == 0 || ((loopcount > 100) && (left > (16 * f->multiplex)))) {
                    if ((loopcount > 100) && (left > (16 * f->multiplex))) {
                        ets_printf("--------2nd sync failed :( This shouldn't happen!!\n");
                        delete f;
                        f = nullptr;
                    } else {
                        if (f) {
                            BaseType_t queuestatus = xQueueSend(frameQueue, &f, 0);
                            if (queuestatus == pdFALSE) {
                                delete f;
                            }
                            f = nullptr;
                        }
                    }
                    // reconfigure rf4463 for 1600 baud reception (for the header);
                    syncedflex.restartTimer();
                    syncedflex.changeInterval(3800);  // was 4000
                    syncedflex.restartTimer();
                    if (flsync.type != flexsynctype::SYNC_FLEX_1600) rf4463.init(settings.current.freq, RF4463::rftype::FLEX_1600_RX);
                    flsync.clearSync();
                    syncedslew = true;
                    rf4463.startRX(flexsynchelper);
                    status.framesReceived++;
                    sendStatus(STATUS_SYNCED_FLEX);
                    // frame reception done, shift frames
                    break;
                } else if (left >= (16 * f->multiplex)) {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                } else {
                    // no delay, keep thread alive
                }
            }
        } else {  // tasknotify timeout
            //ets_printf("flextask stack words free: %u \n", uxTaskGetStackHighWaterMark(NULL));
            if (resetRX) {
                resetRX = false;
                ets_printf("Restarting RX\n");
                restartDecoder();
            }
        }
    }
}