#include "pocsagutils.h"

#include <Arduino.h>

#include "bitutils.h"

#define DEBUG

extern QueueHandle_t pocsagQueue;
// pocsag addr
uint32_t pocsagaddr::getWord() {
    uint32_t ret = 0;
    switch (func) {
        case A:
            ret |= addValue(19, 2, 0);
            break;
        case B:
            ret |= addValue(19, 2, 2);
            break;
        case C:
            ret |= addValue(19, 2, 1);
            break;
        case D:
            ret |= addValue(19, 2, 3);
            break;
    };
    uint32_t tric = ric;
    tric >>= 3;
    tric <<= 13;
    ret |= tric;
    ret = createcrc(ret);
    return ret;
}
pocsagaddr::pocsagaddr(uint32_t addrword, uint8_t word) {
}
uint32_t pocsagaddr::decodeAddress(uint32_t word, uint8_t wordcount) {
    word >>= 11;
    switch (word & 0x03) {
        case 0x00:
            func = pocsagaddr::A;
            break;
        case 0x01:
            func = pocsagaddr::B;
            break;
        case 0x02:
            func = pocsagaddr::C;
            break;
        case 0x03:
            func = pocsagaddr::D;
            break;
    }
    word >>= 2;
    word <<= 3;
    word |= (wordcount / 2) & 0x03;
    ric = word;
    return word;
}
pocsagaddr::pocsagaddr(uint32_t ric_c, enum type type_c, enum func function_c) {
    ric = ric_c;
    frameno = ric & 0x07;
    type = type_c;
    func = function_c;
}
pocsagaddr::pocsagaddr() {
}
// pocsag data
pocsagdata::pocsagdata() {
}
pocsagdata::pocsagdata(pocsagaddr ric, char* data) {
    addMsg(ric, data);
}
pocsagdata::~pocsagdata() {
    if (batch) free(batch);
    if (errorflags) free(errorflags);
}
uint16_t pocsagdata::requiredSize(pocsagaddr ric, char* data) {
    uint16_t wordsneeded = 1;  // we always need 1 address word;
    uint32_t len = strlen(data);
    uint16_t framesneeded = 0;

    switch (ric.type) {
        case pocsagaddr::ALPHA:
            wordsneeded += (len * 7) / 20;
            if ((len * 7) % 20) {
                wordsneeded++;
            }
            break;
        case pocsagaddr::NUMERIC:
            wordsneeded += (len * 4) / 20;
            if ((len * 4) % 20) {
                wordsneeded++;
            }
            break;
        case pocsagaddr::TONE:
            break;
    }
    if (batchcount == 0) {
        framesneeded++;
        if (((8 - ric.frameno) * 2) < wordsneeded) {
            wordsneeded -= ((8 - ric.frameno) * 2);
        } else {
            wordsneeded = 0;
        }
    } else {
        if (batch[batchcount - 1].frame[ric.frameno].word[0] != IDLEWORD) {
            if (batch[batchcount - 1].frame[ric.frameno].word[1] != IDLEWORD) {
                // new frame
                framesneeded++;
                if (((8 - ric.frameno) * 2) < wordsneeded) {
                    wordsneeded -= ((8 - ric.frameno) * 2);
                } else {
                    wordsneeded = 0;
                }
            } else {
                // second free
                if (((8 - ric.frameno) * 2) - 1 < wordsneeded) {
                    wordsneeded -= ((8 - ric.frameno) * 2) - 1;
                } else {
                    wordsneeded = 0;
                }
            }
        } else {
            if (((8 - ric.frameno) * 2) < wordsneeded) {
                wordsneeded -= ((8 - ric.frameno) * 2);
            } else {
                wordsneeded = 0;
            }
            // first word free
        }
    }
    //wordsneeded++;
    framesneeded += wordsneeded / 16;
    if (wordsneeded % 16) {
        framesneeded++;
    } else if (((wordsneeded % 16) == 0) && (wordsneeded > 0)) {
        framesneeded++;  // hmmm.
    }
    //ets_printf("Length: %u Frames needed: %u  - words needed %u\n", len, framesneeded, wordsneeded);
    return framesneeded;
}
void pocsagdata::addMsg(pocsagaddr ric, char* data) {
    uint16_t curbatch = batchcount - 1;
    uint16_t curword = 0;
    if (batchcount == 0) curbatch = 0;
    growBatches(requiredSize(ric, data));
    if (batch[batchcount - 1].frame[ric.frameno].word[0] != IDLEWORD) {
        if (batch[batchcount - 1].frame[ric.frameno].word[1] != IDLEWORD) {
            // new frame
            curbatch++;
            curword = ric.frameno * 2;
        } else {
            // second free
            curword = (ric.frameno * 2) + 1;
        }
    } else {
        // first word free
        curword = ric.frameno * 2;
    }
    ((uint32_t*)&(batch[curbatch]))[curword] = ric.getWord();
    curword++;
    uint32_t tempword = 0;
    switch (ric.type) {
        case pocsagaddr::ALPHA:
            uint8_t bitsleft, bitalign;
            bitalign = 0;
            while (*data != 0x00) {
                tempword = 0;
                bitsleft = 20;
                if (bitalign) {  // if bitalign is not 0, there were bits for a previous character left. Shift them in!
                    tempword |= bitswitch(((*data) & 0x7F));
                    bitsleft -= bitalign;
                    bitalign = 0;
                    if (*data != 0) {
                        data++;
                    }
                }
                // Now lets see how many full characters we can fit in there (usually 1 or 2)
                while ((*data != 0) && (bitsleft >= 7)) {
                    tempword <<= 7;
                    tempword |= bitswitch(((*data) & 0x7F));
                    data++;
                    bitsleft -= 7;
                }
                while ((*data == 0) && (bitsleft >= 7)) {
                    tempword <<= 7;
                    tempword |= bitswitch(((0x00) & 0x7F));
                    //data++; //hmmm
                    bitsleft -= 7;
                }
                // Now 'OR' the first bits of the next character onto the frame, and set how many bits we've fit
                // in there in bitalign
                tempword = (tempword << bitsleft);
                bitalign = 7 - bitsleft;
                if (*data != 0) {
                    tempword |= (bitswitch(((*data) & 0x7F)) >> bitalign);
                } else {
                    tempword |= (bitswitch(((0x00) & 0x7F)) >> bitalign);
                }
                tempword = (tempword << 11);
                tempword |= 0x80000000;
                tempword = createcrc(tempword);
                //ets_printf("Storing %u into %u , %u \n", tempword, curbatch, curword);
                ((uint32_t*)&(batch[curbatch]))[curword] = tempword;
                curword++;
            }
            break;
        case pocsagaddr::TONE:
            break;
        case pocsagaddr::NUMERIC:
            uint8_t charcount;
            while ((*data != 0)) {  // loop for as long as messagep isn't 0
                charcount = 0;
                tempword = 0;
                while ((*data != 0) && (charcount < 5)) {  // check if the pointer doesnt point to the 0 character, and load characters as long as it doesnt.
                    tempword <<= 4;
                    tempword |= encodeNumber(*data);
                    data++;
                    charcount++;
                }
                while (charcount < 5) {  // if we enter this block, we ran out of characters before the frame was full. Fill it up with 0-pointers
                    tempword <<= 4;
                    tempword |= 0x03;
                    charcount++;
                }
                // calculate CRC for the frame
                tempword = (tempword << 11);
                tempword |= 0x80000000;
                tempword = createcrc(tempword);
                ((uint32_t*)&(batch[curbatch]))[curword] = tempword;
                curword++;
            }
            break;
    }
}
void pocsagdata::growBatches(uint16_t growby) {
    //ets_printf("GrowBatches called with %u \n", growby);
    if (batch != nullptr) {
        batch = (struct batchs*)realloc(batch, (batchcount + growby) * sizeof(struct batchs)); // Common realloc mistake: 'batch' nulled but not freed upon failure
        errorflags = (uint16_t*)realloc(errorflags, (batchcount + growby) * sizeof(uint16_t)); // this one too. I kinda assume the realloc never fails. If it does, the shit hits the fan
    } else {
        batch = (struct batchs*)calloc(growby, sizeof(struct batchs));
        errorflags = (uint16_t*)calloc(growby, sizeof(uint16_t));
    }
    for (uint8_t c = 0; c < growby; c++) {
        for (uint8_t d = 0; d < 8; d++) {
            batch[(batchcount + c)].frame[d].word[0] = IDLEWORD;
            batch[(batchcount + c)].frame[d].word[1] = IDLEWORD;
        }
    }
    batchcount += growby;
    //ets_printf("Batchcount is now %u \n", batchcount);
}
void pocsagdata::addWord(uint32_t word, uint8_t wordcount, bool hasErrors) {
    bool doDecode = false;
    batchs* batchcopyp;
    uint8_t batchcountcopy;
    uint16_t msgwordcountcopy;
    uint16_t* errorflagspcopy;
    switch (getType(word)) {
        case pocsagdata::wordtype::TYPE_IDLE:
            if (msgStarted) {
                batchcopyp = batch;
                batchcountcopy = batchcount;
                msgwordcountcopy = msgwordcount;
                errorflagspcopy = errorflags;

                batch = nullptr;
                errorflags = nullptr;
                batchcount = 0;
                msgStarted = false;
                doDecode = true;
            }
            break;
        case pocsagdata::wordtype::TYPE_ADDR:
            if (msgStarted) {
                batchcopyp = batch;
                batchcountcopy = batchcount;
                msgwordcountcopy = msgwordcount;
                errorflagspcopy = errorflags;

                errorflags = nullptr;
                batch = nullptr;
                batchcount = 0;
                doDecode = true;
            } else {
                msgStarted = true;
            }
            growBatches(1);
            msgwordcount = 1;
            ((uint32_t*)&(batch[batchcount - 1]))[wordcount] = word;
            if (hasErrors) errorflags[batchcount - 1] |= (1 << wordcount);
            break;
        case pocsagdata::wordtype::TYPE_MSG:
            if (msgStarted) {
                msgwordcount++;
                if (wordcount == 0) {
                    growBatches(1);
                }
                ((uint32_t*)&(batch[batchcount - 1]))[wordcount] = word;
            }
            break;
    }
    if (doDecode) decodeMessage(batchcopyp, batchcountcopy, msgwordcountcopy, errorflagspcopy);  // send pointer to decodeMessage; the batch block is freed there.
}
uint8_t pocsagdata::getFrameSize() {
    uint8_t ret = 0;
    uint32_t totalbits = 1000 * ((17 * 32 * batchcount) + 576) / baudrate;
    ret = (totalbits / 1875);
    if (totalbits % 1875) ret++;
    return ret;
}
uint8_t pocsagdata::getFrameSize(uint16_t batchcount) {
    uint8_t ret = 0;
    uint32_t totalbits = 1000 * ((17 * 32 * batchcount) + 576) / baudrate;
    ret = (totalbits / 1875);
    if (totalbits % 1875) ret++;
    return ret;
}
uint8_t pocsagdata::encodeNumber(char c) {
    // This is a lookup for the relevant numbers for alpha pagers.
    switch (c) {
        case 0x30:
            return 0;
        case 0x31:
            return 0x08;
        case 0x32:
            return 0x04;
        case 0x33:
            return 0x0C;
        case 0x34:
            return 0x02;
        case 0x35:
            return 0x0A;
        case 0x36:
            return 0x06;
        case 0x37:
            return 0x0E;
        case 0x38:
            return 0x01;
        case 0x39:
            return 0x09;
        case 0x2F:  //spare is slash
        case 0x41:  // A
            return 0x05;
        case 0x55:  // U is urgent?
        case 0x42:  // B
            return 0x0D;
        case 0x20:  // space
        case 0x43:  // C
            return 0x03;
        case 0x2D:  // --
        case 0x44:  // D
            return 0x0B;
        case 0x5D:  // [
        case 0x45:  // E
            return 0x07;
        case 0x5B:  // ]
        case 0x46:  // F
            return 0x0F;
        default:
            return 0x05;
            break;
    }
}
char pocsagdata::decodeNumber(uint8_t c) {
    c &= 0x0F;
    switch (c) {
        case 8:
            return '1';
        case 4:
            return '2';
        case 12:
            return '3';
        case 2:
            return '4';
        case 10:
            return '5';
        case 6:
            return '6';
        case 14:
            return '7';
        case 1:
            return '8';
        case 9:
            return '9';
        case 0:
            return '0';
        case 5:
            return '/';
        case 13:
            return 'U';
        case 3:
            return ' ';
        case 11:
            return '-';
        case 7:
            return ']';
        case 15:
            return '[';
    }
    return ' ';
}
pocsagdata::wordtype pocsagdata::getType(uint32_t word) {
    if (isIdle(word)) return pocsagdata::wordtype::TYPE_IDLE;
    if (isAddr(word)) return pocsagdata::wordtype::TYPE_ADDR;
    return pocsagdata::wordtype::TYPE_MSG;
}
bool pocsagdata::isMsg(uint32_t word) {
    if (!isIdle(word)) {
        return ((word & 0x80000000));
    } else {
        return false;
    }
}
bool pocsagdata::isAddr(uint32_t word) {
    if (!isIdle(word)) {
        return (!(word & 0x80000000));
    }
    return false;
}
bool pocsagdata::isIdle(uint32_t word) {
    return (word == 2055848343);
}
void pocsagdata::decodeMessage(batchs* batchp, uint8_t batchcount, uint16_t wordcount, uint16_t* errorflagsp) {
    uint32_t temp;
    pocsagaddr addr;

    uint16_t alphacount = 2 + (((wordcount - 1) * 20) / 7);
    uint16_t numcount = 2 + (wordcount * 5);
    uint8_t errorMarker = 0;
    char* alphabuffer = (char*)calloc(1, alphacount);
    char* numbuffer = (char*)calloc(1, numcount);
    char* alphabufferp = alphabuffer;
    char* numbufferp = numbuffer;

    uint32_t tempw = 0;
    char tempchar;
    char unaligned = 0;
    uint8_t bitsleft;
    uint8_t rxbitalign = 0;
    uint8_t unalignederror = 0;

    for (uint8_t c = 0; c < batchcount; c++) {
        for (uint8_t d = 0; d < 16; d++) {
            temp = ((uint32_t*)&(batchp[c]))[d];
            if (isAddr(temp)) {
                addr.decodeAddress(temp, d);
            } else if (isMsg(temp)) {  // Decode msg word
                temp &= 0x7FFFFFFF;
                tempw = temp;
                if (errorflagsp[c] & (1 << d)) {
                    errorMarker = 0x80;
                } else {
                    errorMarker = 0x00;
                }
                // decode as numeric
                *numbufferp = errorMarker | decodeNumber((temp >> 27));
                numbufferp++;
                *numbufferp = errorMarker | decodeNumber((temp >> 23));
                numbufferp++;
                *numbufferp = errorMarker | decodeNumber((temp >> 19));
                numbufferp++;
                *numbufferp = errorMarker | decodeNumber((temp >> 15));
                numbufferp++;
                *numbufferp = errorMarker | decodeNumber((temp >> 11));
                numbufferp++;
                // decode as alpha
                bitsleft = 20;
                while (bitsleft) {
                    if (rxbitalign != 0) {
                        tempchar = (char)(tempw >> (bitsleft + 4 + (7 - rxbitalign))) & 0x7F;
                        tempchar |= unaligned;
                        bitsleft -= rxbitalign;
                        rxbitalign = 0;
                        tempchar = bitswitch(tempchar);
                        *alphabufferp = (tempchar | errorMarker) | unalignederror;
                        alphabufferp++;
                    } else {
                        if (bitsleft >= 7) {
                            tempchar = (char)(tempw >> (bitsleft + 4)) & 0x7F;
                            tempchar = bitswitch(tempchar);
                            *alphabufferp = tempchar | errorMarker;
                            alphabufferp++;
                            bitsleft -= 7;
                        } else {
                            unaligned = (char)(tempw >> 11) & 0x7F;
                            rxbitalign = 7 - bitsleft;
                            unaligned <<= rxbitalign;
                            unaligned &= 0x7F;
                            bitsleft = 0;
                            unalignederror = errorMarker;
                        }
                    }
                }  // end while bitsleft loop
            }      // end if msg
        }          // end word-loop
    }              // end batch-loop
    *alphabufferp = 0x00;
    *numbufferp = 0x00;
    pocsagmsg* msg = new pocsagmsg(addr, alphabuffer, numbuffer);
    queueMessage(msg);
    if (batchp) free(batchp);
    if (errorflagsp) free(errorflagsp);
}
int pocsagdata::getRating(char* alpha, char* numeric, pocsagaddr addr) {
    int rating = 0;
    if (addr.func == pocsagaddr::A) {
        rating -= 250;
    } else if (addr.func == pocsagaddr::D) {
        rating += 250;
    }
    uint16_t alphalen = 0;
    uint16_t nonprint = 0;
    alphalen = strlen(alpha);
    for (uint16_t c = 0; c < alphalen; c++) {
        if ((alpha[c] & 0x7F) < 0x20) {
            nonprint++;
        }
    }
    if ((alpha[alphalen - 1] & 0x7F) == 0x00) {
        nonprint--;
        if ((alpha[alphalen - 2] & 0x7F) == 0x00) {
            nonprint--;
        }
    }

    if (nonprint == 0) {
        rating += 200;
    } else if (nonprint < 2) {
        rating += 100;
    } else if (nonprint >= 3) {
        rating -= 200;
    } else if (nonprint >= 2) {
        rating -= 100;
    }

    uint16_t numericlen = 0;
    uint16_t nonnumbers = 0;
    numericlen = strlen(numeric)|1;
    for (uint16_t c = 0; c < numericlen; c++) {
        if (((numeric[c] & 0x7F) >= 0x30) && ((numeric[c] & 0x7F) <= 0x39)) {
        } else {
            nonnumbers++;
        }
    }
    for (uint8_t c = 0; c < 4; c++) {
        if ((numeric[(numericlen - 1) - c] & 0x7F) == 0x20) {
            nonnumbers--;
        } else {
            break;
        }
    }
    if (nonnumbers > (numericlen / 4)) {
        rating += 100;
    } else {
        rating -= 100;
    }
    if (numericlen - nonnumbers == 10) {
        rating -= 100;
    }

    if (numericlen > 60) {
        rating += 400;
    } else if (numericlen > 40) {
        rating += 200;
    } else if (numericlen > 20) {
        rating += 100;
    }
    return rating;
}
void pocsagdata::queueMessage(pocsagmsg* msg) {
    if (msg) {
        BaseType_t queuestatus = xQueueSend(pocsagQueue, &msg, 0);
        if (queuestatus == pdFALSE) {
            delete msg;
            ets_printf("!!! Failed to queue message. Maybe the processing queue has stalled?\n");
        }
        msg = nullptr;
    }
}

pocsagmsg::~pocsagmsg() {
    delete alphadata;
    delete numdata;
}
pocsagmsg::pocsagmsg(pocsagaddr ad, char* alpha, char* num) {
    addr = ad;
    alphadata = alpha;
    numdata = num;
}