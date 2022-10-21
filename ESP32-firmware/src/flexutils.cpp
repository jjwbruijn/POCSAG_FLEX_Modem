#include "flexutils.h"

#include <Arduino.h>

#include <iostream>
#include <vector>

#include "BCH3121.h"
#include "RF4463.h"
#include "bitutils.h"
#include "settings.h"
#include "statustracker.h"

std::vector<flextempaddrmapping*> flexmappings;
std::vector<flexmsg*> flexmessages;
struct flexstatusstruct flexstatus;

// flextempaddrmapping
flextempaddrmapping* flextempaddrmapping::getTempMapping(uint8_t frame, uint8_t tempaddr, bool allMappings) {
    uint8_t timeout = 238;
    flextempaddrmapping* tempmap;
    if (allMappings) timeout = 255;
    for (uint8_t c = 0; c < flexmappings.size(); c++) {
        tempmap = flexmappings.at(c);
        if (tempmap->frame == frame && tempmap->tempaddr == tempaddr && tempmap->timeout <= timeout) {
            return tempmap;
        }
    }
    return nullptr;
}
void flextempaddrmapping::addTempMapping(uint8_t frame, uint8_t tempaddr, flexaddress* addr, uint8_t curframe) {
    flextempaddrmapping* tempmap;
    tempmap = getTempMapping(frame, tempaddr, true);
    if ((tempmap && curframe != frame) || (tempmap && curframe == frame && tempmap->timeout > 238)) {
        tempmap->addresslist.push_back(addr);
    } else {
        tempmap = new flextempaddrmapping;
        flexmappings.push_back(tempmap);
        tempmap->addresslist.push_back(addr);
        tempmap->frame = frame;
        tempmap->tempaddr = tempaddr;
        tempmap->timeout = ceil((double)(((frame - curframe) % 128) * 1.875));
        if (curframe == frame) tempmap->timeout = 240;
    }
}
void flextempaddrmapping::deleteTempMapping(flextempaddrmapping* mapping) {
    for (uint8_t c = 0; c < flexmappings.size(); c++) {
        if (flexmappings.at(c) == mapping) {
            flexmappings.erase(flexmappings.begin() + c);
            delete mapping;
            break;
        }
    }
}
void flextempaddrmapping::removeStaleMappings() {
    flextempaddrmapping* tempmap;
    for (uint8_t c = 0; c < flexmappings.size(); c++) {
        tempmap = flexmappings.at(c);
        if (tempmap->timeout) {
            tempmap->timeout--;
        } else {
            flexmappings.erase(flexmappings.begin() + c);
            delete tempmap;
        }
    }
}
flextempaddrmapping::flextempaddrmapping() {
}
flextempaddrmapping::~flextempaddrmapping() {
    for (uint8_t c = 0; c < addresslist.size(); c++) {
        delete addresslist.at(c);
    }
}

//flexsync
void IRAM_ATTR flexsync::bitInterrupt(bool bit) {
    if (isSynced == false) {
        bs1 <<= 1;
        if (Aword & 0x80000000) bs1 |= 0x01;
        Aword <<= 1;
        if (Bword & 0x8000) Aword |= 0x01;
        Bword <<= 1;
        if (notAword & 0x80000000) Bword |= 0x01;
        notAword <<= 1;
        if (FIWword & 0x80000000) notAword |= 0x01;
        FIWword <<= 1;
        FIWword |= bit;
        if (validate()) {
            isSynced = true;
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR(flexRXTaskId, &woken);
            portYIELD_FROM_ISR();
        }
    }
}
bool IRAM_ATTR flexsync::validate() {
    if (countbits(bs1 ^ 0xAAAA) < 8) {
        if (countbits(Bword ^ 0x5555) < 3) {
            if ((countbits(Aword ^ FLEX_A1) < 5) && (countbits(notAword ^ FLEX_A1) > 27)) {
                type = flexsynctype::SYNC_FLEX_1600;
                //ets_printf("Sync A1 ");
                return true;
            } else if ((countbits(Aword ^ FLEX_A2) < 5) && (countbits(notAword ^ FLEX_A2) > 27)) {
                type = flexsynctype::SYNC_FLEX_3200_2;
                //ets_printf("Sync A2\n");
                return true;
            } else if ((countbits(Aword ^ FLEX_A3) < 5) && (countbits(notAword ^ FLEX_A3) > 27)) {
                type = flexsynctype::SYNC_FLEX_3200_4;
                //ets_printf("Sync A3\n");
                return true;
            } else if ((countbits(Aword ^ FLEX_A4) < 5) && (countbits(notAword ^ FLEX_A4) > 27)) {
                type = flexsynctype::SYNC_FLEX_6400;
                //ets_printf("Sync A4\n");
                return true;
            } else if ((countbits(Aword ^ FLEX_AR) < 5) && (countbits(notAword ^ FLEX_AR) > 27)) {
                ets_printf("Resync\n");
            } else {
                type = flexsynctype::SYNC_NOSYNC;
            }
        }
    }
    return false;
}
void flexsync::clearSync() {
    notAword = 0;
    bs1 = 0;
    Aword = 0;
    FIWword = 0;
    isSynced = false;
}
// fiw
uint32_t flexfiw::output() {
    uint32_t ret = 0;
    ret |= addValue(4, 4, cycle);
    ret |= addValue(8, 7, frame);
    ret |= addValue(16, 1, repeat);
    ret |= addValue(17, 1, traffic[0]);
    ret |= addValue(18, 1, traffic[1]);
    ret |= addValue(19, 1, traffic[2]);
    ret |= addValue(20, 1, traffic[3]);
    ret = motchecksum(ret);
    ret = createcrc(ret);
    return ret;
}
flexfiw::flexfiw() {
}
void flexfiw::input(uint32_t word) {
    if (validateMotChecksum(word)) {
        isValid = true;
        cycle = getValue(4, 4, word);
        frame = getValue(8, 7, word);
        repeat = getValue(16, 1, word);
        traffic[0] = getValue(17, 1, word);
        traffic[1] = getValue(18, 1, word);
        traffic[2] = getValue(19, 1, word);
        traffic[3] = getValue(20, 1, word);
    } else {
        isValid = false;
    }
}
flexfiw::flexfiw(uint32_t word) {
    input(word);
}

// biw
flexbiw::flexbiw(enum biwtype_e type) {
    biwtype = type;
}
flexbiw::flexbiw() {
}
flexbiw::flexbiw(uint32_t word) {
    input(word, 0);
}
uint32_t flexbiw::output() {
    uint32_t ret = 0;
    switch (biwtype) {
        case BIW:
            //ets_printf("DEBUG: Generating BIW - vectorstart: %u, blocksize: %u \n", vectorstart, blocksize);
            ret |= addValue(4, 4, priority);
            ret |= addValue(8, 2, blocksize - 1);
            ret |= addValue(10, 6, vectorstart);
            ret |= addValue(16, 2, carryon);
            ret |= addValue(18, 3, collapse);
            break;
        case DATE:
            ret |= addValue(4, 3, 1);  // func
            /*Serial.printf("Time1: %04d/%02d/%02d %02d:%02d:%02d \n",
                          time.year,
                          time.month,
                          time.day,
                          time.hours,
                          time.mins,
                          time.secs);*/
            ret |= addValue(7, 5, (uint8_t)time.year - 1994);
            ret |= addValue(12, 5, (uint8_t)time.day);
            ret |= addValue(17, 4, (uint8_t)time.month);
            //ret |= addValue(7, 5, 10);
            //ret |= addValue(12, 5, 10);
            //ret |= addValue(17, 4, 10);
            //Serial.println(ret);
            break;
        case TIME:
            ret |= addValue(4, 3, 2);  // func
            ret |= addValue(18, 3, time.secs / 7.5);
            ret |= addValue(12, 6, time.mins);
            ret |= addValue(7, 5, time.hours);
            break;
        default:
            ets_printf("Invalid BIW generated...");
            break;
    }
    ret |= motchecksum(ret);
    ret = createcrc(ret);
    return ret;
}
void flexbiw::input(uint32_t in, uint8_t pos) {
    if (validateMotChecksum(in)) {
        isValid = true;
    } else {
        isValid = false;
    }
    if (!pos) {  // BIW 1;
        priority = getValue(4, 4, in);
        blocksize = getValue(8, 2, in) + 1;
        vectorstart = getValue(10, 6, in);
        carryon = getValue(16, 2, in);
        collapse = getValue(18, 3, in);
        //ets_printf("BIW: Blocksize: %u, vector start: %u\n", blocksize, vectorstart);
    } else {
        uint8_t func = 0;
        func = getValue(4, 3, in);
        switch (func) {
            case 0:  // local ID
                break;
            case 1:  // DATE
                biwtype = biwtype_e::DATE;
                time.year = getValue(7, 5, in);
                time.day = getValue(12, 5, in) - 6;
                time.month = getValue(17, 4, in);
                //ets_printf("BIW2: Date: %u/%u/%u\n", time.day, time.month, time.year);
                break;
            case 2:  // TIME
                biwtype = biwtype_e::TIME;
                time.secs = getValue(18, 3, in) * 7.5;
                time.mins = getValue(12, 6, in);
                time.hours = getValue(7, 5, in);
                //ets_printf("BIW2: Time: %u:%u:%u\n", time.hours, time.mins, time.secs);
                break;
        }
    }
}

// address
void flexaddress::decodeRIC(uint32_t address) {
    decodeRIC(address, 0);
}
void flexaddress::decodeRIC(uint32_t address, uint8_t offset) {
    ric = address;
    phase = ((address - offset) / 4) % 4;
    frame = ((address - offset) / 16) % 128;
    if (address >= 4291000001) {  // Info Svc, Long Adr 2-3
        addrtype = ADDRT_2_3;
        addruse = ADDRUSE_INFO_SVC_RESERVED;
    } else if (address >= 4290000001) {  // Info Svr, Long 2-3
        addrtype = ADDRT_2_3;
        addruse = ADDRUSE_INFO_SVC_GLOBAL;
    } else if (address >= 4285000001) {  // Info Svr, Long 2-3
        addrtype = ADDRT_2_3;
        addruse = ADDRUSE_INFO_SVC_COUNTRY1;
    } else if (address >= 4280000001) {  // Info Svr, Long 2-3
        addrtype = ADDRT_2_3;
        addruse = ADDRUSE_INFO_SVC_GLOBAL;
    } else if (address >= 3923326751) {  // Long 2-3
        addrtype = ADDRT_2_3;
        addruse = ADDRUSE_INFO_SVC_RESERVED;
    } else if (address >= 3223326721) {  // Long 2-3
        addrtype = ADDRT_2_3;
        addruse = ADDRUSE_BY_COUNTRY2;
    } else if (address >= 2149584897) {  // Long 1-4
        addrtype = ADDRT_1_4;
        addruse = ADDRUSE_GLOBAL3;
    } else if (address >= 1075843073) {  // Long 1-3
        addrtype = ADDRT_1_3;
        addruse = ADDRUSE_GLOBAL2;
    } else if (address >= 402101251) {  // Long 1-2
        addrtype = ADDRT_1_2;
        addruse = ADDRUSE_GLOBAL1;
    } else if (address >= 102101251) {  // Long 1-2
        addrtype = ADDRT_1_2;
        addruse = ADDRUSE_BY_COUNTRY1;
    } else if (address >= 2101249) {  // Long 1-2
        addrtype = ADDRT_1_2;
        addruse = ADDRUSE_UNCOORDINATED;
    } else if (address >= 2031615) {  // Unused
        addrtype = ADDRT_SHORT;
        addruse = ADDRUSE_ILLEGAL;
    } else if (address >= 2029600) {  // Reserved
        addrtype = ADDRT_SHORT;
        addruse = ADDRUSE_RESERVED2;
    } else if (address >= 2029584) {  // Operator Messaging
        addrtype = ADDRT_SHORT;
        addruse = ADDRUSE_OPERATOR_MSG;
    } else if (address >= 2029568) {  // Temporary Address
        addrtype = ADDRT_SHORT;
        addruse = ADDRUSE_TEMPORARY;
    } else if (address >= 2025472) {  // Network Address
        addrtype = ADDRT_SHORT;
        addruse = ADDRUSE_NETWORK_ADDR;
    } else if (address >= 2009088) {  // Information Services
        addrtype = ADDRT_SHORT;
        addruse = ADDRUSE_INFORMATION_SVC_ADDR;
    } else if (address >= 1998849) {  // Reserved
        addrtype = ADDRT_SHORT;
        addruse = ADDRUSE_RESERVED1;
    } else if (address >= 1933313) {  // Illegal
        addrtype = ADDRT_SHORT;
        addruse = ADDRUSE_ILLEGAL;
    } else if (address >= 1) {  // Short
        addrtype = ADDRT_SHORT;
        addruse = ADDRUSE_SHORT;
    } else {  // Unused
        addruse = ADDRUSE_ILLEGAL;
    }

    switch (addrtype) {
        case ADDRT_1_2:
            addresswords = 2;
            word[0].value = ((address - 2068481) % 32768) + 1;
            word[1].value = 2097151 - ((address - 2068481) / 32768);
            break;
        case ADDRT_1_3:
        case ADDRT_1_4:
            addresswords = 2;
            word[0].value = ((address - 2068481) % 32768) + 1;
            word[1].value = 1933312 + ((address - 2068481) / 32768);
            break;
        case ADDRT_2_3:
            addresswords = 2;
            word[0].value = ((address - 2068479) % 32768) + 2064383;
            word[1].value = 1867776 + ((address - 2068479) / 32768);
            break;
        default:  // short
            word[0].value = address + 32768;
            addresswords = 1;
            break;
    }
}
enum flexaddress::shortaddrtype flexaddress::getShortType(uint32_t address) {
    if (address == 0x1FFFFF) {
        return shortaddrtype::SHORT_IDLE2;
    } else if (address >= 0x1F7FFF) {
        return shortaddrtype::SHORT_LONG_2;
    } else if (address >= 0x1F7820) {
        return shortaddrtype::SHORT_RESERVED2;
    } else if (address >= 0x1F7810) {
        return shortaddrtype::SHORT_OPERATOR;
    } else if (address >= 0x1F7800) {
        return shortaddrtype::SHORT_TEMP;
    } else if (address >= 0x1F6800) {
        return shortaddrtype::SHORT_NETWORK;
    } else if (address >= 0x1F2800) {
        return shortaddrtype::SHORT_INFO_SERVICE;
    } else if (address >= 0x1F0001) {
        return shortaddrtype::SHORT_RESERVED1;
    } else if (address >= 0x1E8001) {
        return shortaddrtype::SHORT_LONG_4;
    } else if (address >= 0x1E0001) {
        return shortaddrtype::SHORT_LONG_3;
    } else if (address >= 0x008001) {
        return shortaddrtype::SHORT;
    } else if (address >= 0x000001) {
        return shortaddrtype::SHORT_LONG_1;
    } else {
        return shortaddrtype::SHORT_IDLE1;
    }
}
void flexaddress::decodeWord(uint32_t* addrwordp) {
    uint32_t temp = *addrwordp;
    temp &= 0b11111111111111111111100000000000;
    temp = swap32(temp);
    word[0].type = getShortType(temp);
    word[0].value = temp;
    addresswords = 1;
    if ((word[0].type == shortaddrtype::SHORT_LONG_3) || (word[0].type == shortaddrtype::SHORT_LONG_4)) {
        word[0].type = shortaddrtype::SHORT_INVALID;
        ric = 0;
        return;
    } else if ((word[0].type == shortaddrtype::SHORT_LONG_1) || (word[0].type == shortaddrtype::SHORT_LONG_2)) {
        temp = *(addrwordp + 1);
        temp &= 0b11111111111111111111100000000000;
        temp = swap32(temp);
        word[1].type = getShortType(temp);
        word[1].value = temp;
        switch (word[0].type) {
            case shortaddrtype::SHORT_LONG_1:
                switch (word[1].type) {
                    case shortaddrtype::SHORT_LONG_2:
                        addrtype = ADDRT_1_2;
                        break;
                    case shortaddrtype::SHORT_LONG_3:
                        addrtype = ADDRT_1_3;
                        break;
                    case shortaddrtype::SHORT_LONG_4:
                        addrtype = ADDRT_1_4;
                        break;
                    default:
                        addrtype = ADDRT_INVALID;
                        word[0].type = shortaddrtype::SHORT_INVALID;
                        word[1].type = shortaddrtype::SHORT_INVALID;
                        ric = 0;
                        return;
                        break;
                };
                break;
            case shortaddrtype::SHORT_LONG_2:
                switch (word[1].type) {
                    case shortaddrtype::SHORT_LONG_3:
                        addrtype = ADDRT_2_3;
                        break;
                    default:
                        addrtype = ADDRT_INVALID;
                        word[0].type = shortaddrtype::SHORT_INVALID;
                        word[1].type = shortaddrtype::SHORT_INVALID;
                        ric = 0;
                        return;
                        break;
                };
                break;
            default:
                break;
        }
        // decode long address
        switch (addrtype) {
            case ADDRT_1_2:
                ric = ((2097151 - word[1].value) * 32768) + 2068481;
                ric += (word[0].value) + 1;
                break;
            case ADDRT_1_3:
            case ADDRT_1_4:
                break;
            case ADDRT_2_3:
                break;
            default:
                break;
        }
        addresswords = 2;
    } else {  // not long
        ric = word[0].value;
        if (word[0].type == SHORT) {
            ric -= 32768;
        }
        addresswords = 1;
    }
    decodeRIC(ric);
}
void IRAM_ATTR flexaddress::RICtoWord(uint32_t* wordp) {
    uint32_t temp;
    if (word[0].value) {
        temp = word[0].value;
        temp = swap32(temp);
        temp = createcrc(temp);
        *wordp = temp;
    }
    if (addresswords == 2) {
        temp = word[1].value;
        temp = swap32(temp);
        temp = createcrc(temp);
        *(wordp + 1) = temp;
    }
}
uint32_t flexaddress::temp(uint8_t tempaddr) {
    return 0x1EF800 + (tempaddr % 16);
}
uint8_t flexaddress::getPhase(uint32_t address, char prefix) {
    uint8_t offset = getSubstract(prefix);
    return ((address - offset) / 4) % 4;
}
uint8_t flexaddress::getFrame(uint32_t address, char prefix) {
    uint8_t offset = getSubstract(prefix);
    return ((address - offset) / 16) % 128;
}
uint8_t flexaddress::getSubstract(char prefix) {
    switch (prefix) {
        case 'A':
            return 0;
        case 'B':
            return 1;
        case 'C':
            return 2;
        case 'D':
            return 3;
        case 'E':
            return 0;
        case 'F':
            return 1;
        case 'G':
            return 2;
        case 'H':
            return 3;
        case 'I':
            return 0;
        case 'J':
            return 1;
        case 'K':
            return 2;
        case 'L':
            return 3;
        default:
            return 0;
    }
}
uint8_t flexaddress::getNextFrame(uint32_t address, char prefix, uint8_t start, bool rxcollapse) {
    uint8_t c = start;
    uint8_t frame = getFrame(address, prefix);
    uint8_t fcollapse = 4;  // minimum frame collapse for most pagers

    if (rxcollapse) {
        if (flexstatus.rxcollapse < 4) fcollapse = flexstatus.rxcollapse;
    } else {
        if (flexstatus.collapse < 4) fcollapse = flexstatus.collapse;
    }
    //ets_printf("System collapse = %u\n", flexstatus.collapse);
    if (fcollapse == 0) return c;
    fcollapse = ((1 << fcollapse) - 1);
    frame &= fcollapse;
    start = (start - 1) % 128;
    while (c != start) {
        if ((c & fcollapse) == frame) break;
        c++;
    }
    return c % 128;
}
flexaddress::flexaddress(uint32_t* wordp) {
    decodeWord(wordp);
}
flexaddress::flexaddress() {
}

// flex data block decoding
flexmsg* flexdatablock::decode(flexphase* phase, uint8_t addrword, uint8_t curframe) {
    flexmsg* msg;
    if (phase->getWord(addrword) == 0) {
        // empty address word, (0), it must be already taken care off in earlier processing.
        return nullptr;
    }

    flexaddress* addr = nullptr;
    flexvector* vect = new flexvector(phase->getWordP(phase->biwone.vectorstart + (addrword - phase->biwone.blocksize)));
    switch (vect->type) {
        case flexvector::vectortype::VECTOR_ALPHA:
            //ets_printf("ALPHA> start %u, len %u\n", vect->blockstart, vect->blocklength);
            msg = decodeAlpha(phase, addrword, curframe, vect);
            if (msg) msg->type = vect->type;
            delete vect;
            return msg;
            break;
        case flexvector::vectortype::VECTOR_SHORT_INSTRUCTION:
            addr = new flexaddress(phase->getWordP(addrword));
            //ets_printf(" Assigned %u to temp address %u in frame %u\n", addr->ric, vect->tempAddress, vect->tempAssignedFrame);
            flextempaddrmapping::addTempMapping(vect->tempAssignedFrame, vect->tempAddress, addr, curframe);
            delete vect;
            return nullptr;
            break;
        case flexvector::vectortype::VECTOR_NUMERIC:
            ets_printf("numeric vector length = %u, start = %u \n", vect->blocklength, vect->blockstart);
            msg = decodeNumeric(phase, addrword, curframe, vect);
            if (msg) msg->type = vect->type;
            delete vect;
            return msg;
            break;
        default:
            addr = new flexaddress(phase->getWordP(addrword));
            addr->decodeWord(phase->getWordP(addrword));
            ets_printf("Vector type e for addr %u, word value = %u \n", addr->ric, phase->getWord(phase->biwone.vectorstart + (addrword - phase->biwone.blocksize)));
            delete addr;
            delete vect;
            return nullptr;
            break;
    }
}
flexmsg* flexdatablock::decodeNumeric(flexphase* phase, uint8_t addrword, uint8_t curframe, flexvector* vect) {
    flexaddress* addr = new flexaddress(phase->getWordP(addrword));
    flexmsg* msg;
    uint8_t addrindex = 0;
    addrindex = addrword - phase->biwone.blocksize;

    uint32_t* firstnumword = 0;
    if (addr->addresswords == 1) {  // hmmm
        firstnumword = phase->getWordP(vect->blockstart);
        vect->blockstart++;
        if (vect->blocklength) vect->blocklength--;
    } else {
        firstnumword = phase->getWordP(phase->biwone.vectorstart + addrindex + 1);
    }

    // first part of the numeric checksum is in the vector, second part is the first two bits of the first word
    uint8_t numchecksum = vect->numchecksum;
    numchecksum |= (getValue(0, 2, *firstnumword)) << 4;

    msg = new flexmsg();
    if (msg) {
        uint8_t* buffer = (uint8_t*)calloc(42, 1);
        uint8_t* bufferp = &(buffer[4]);
        buffer[0] = getValue(2, 4, *firstnumword);
        buffer[1] = getValue(6, 4, *firstnumword);
        buffer[2] = getValue(10, 4, *firstnumword);
        buffer[3] = getValue(14, 4, *firstnumword);
        uint8_t bitsremaining = getValue(18, 3, *firstnumword);
        uint8_t bitcountremaining = 1;
        uint8_t bitcounter = 0;
        uint32_t tempword = 0;

        // add all other words containing numeric data, if any
        while (vect->blocklength) {
            bitcounter = 0;
            //bitsremaining <<= bitcountremaining;
            tempword = phase->getWord(vect->blockstart);
            uint8_t part2bits = getValue(0, bitcountremaining, tempword);
            part2bits <<= (4-bitcountremaining);
            bitsremaining |= part2bits;
            *bufferp = bitsremaining;
            bufferp++;
            bitsremaining = 0;
            bitcounter = bitcountremaining;
            // add all other full (4-bit) characters
            while (bitcounter < 18) {
                *bufferp = getValue(bitcounter, 4, tempword);
                bufferp++;
                bitcounter += 4;
            }

            // add unaligned remaining bits
            if (bitcounter < 21) {
                bitsremaining = getValue(bitcounter, 21 - bitcounter, tempword);
            }
            bitcountremaining = 4 - (21 - bitcounter);

            // decrement length and move index to next word
            vect->blocklength--;
            vect->blockstart++;
        }
        msg->primaryAddress = addr->ric;
        msg->addAddress(curframe, addr);
        msg->addNumericData(buffer, bufferp - buffer);
        ets_printf(">TO:>");
        for (uint8_t c = 0; c < msg->addrarr.size(); c++) {
            ets_printf(" %u", msg->addrarr[c]->ric);
        }
        ets_printf("\nMSG:> ");
        ets_printf(msg->getNumericData());
        ets_printf("\n----------------------\n");

        return msg;
        free(buffer);
        return msg;
    }
    return nullptr;
}
flexmsg* flexdatablock::decodeAlpha(flexphase* phase, uint8_t addrword, uint8_t curframe, flexvector* vect) {
    flexaddress* addr = new flexaddress(phase->getWordP(addrword));
    uint8_t addrsize = addr->addresswords;
    flexmsg* msg;
    uint8_t addrindex = 0;
    addrindex = addrword - phase->biwone.blocksize;

    uint32_t* firstalphaword = 0;
    if (addr->addresswords == 1) {  // hmm, is this right? shouldn't we decrement the total amount of words?
        firstalphaword = phase->getWordP(vect->blockstart);
        vect->blockstart++;
    } else {
        firstalphaword = phase->getWordP(phase->biwone.vectorstart + addrindex + 1);
    }

    //uint16_t fchecksum = getValue(0, 10, *firstalphaword); // <<IMPLEMENT ME! / TODO
    uint8_t continued = getValue(10, 1, *firstalphaword);
    uint8_t fragmentno = getValue(11, 2, *firstalphaword);
    uint8_t messageno = getValue(13, 6, *firstalphaword);
    //uint8_t fragmentcontrol = getValue(19, 2, *firstalphaword); // <<IMPLEMENT ME! / TODO

    // start of a new message
    if (fragmentno == 3) {  // first fragment
        msg = new flexmsg();
        msg->primaryAddress = addr->ric;

        msg->addAddress(curframe, addr);

        // iterate over all address words, trying to find vectors pointing to the same message
        for (uint8_t c = (addrword + addrsize); c < phase->biwone.vectorstart;) {
            flexaddress* tempaddr = new flexaddress(phase->getWordP(c));
            flexvector* tempvect = new flexvector(phase->getWordP(phase->biwone.vectorstart + (c - phase->biwone.blocksize)));
            if (tempaddr->addresswords == 1) tempvect->blockstart++;  // cut first header location may be different if long/short messages are mixed
            if ((tempvect->blockstart == vect->blockstart) && (tempvect->type == vect->type)) {
                ets_printf("!!! Message deduplicated!\n");
                msg->addAddress(curframe, tempaddr);
                // delete address words
                //*(phase->getWordP(c)) = (uint32_t)NULL; //**UNCOMMENT ME** hmm
                //if (tempaddr->addresswords == 2) *(phase->getWordP(c + 1)) = (uint32_t)NULL; //**UNCOMMENT ME
            } else {
                delete tempaddr;
            }
            delete tempvect;
            c += tempaddr->addresswords;
        }

        msg->fragmentid = fragmentno;
        msg->messageno = messageno;
        msg->signature = getValue(0, 7, phase->getWord(vect->blockstart));
    } else {  // not the first fragment
        msg = flexmsg::getIncomplete(addr->ric, messageno);
        if (msg == nullptr) {
            ets_printf("!!! Couldn't find message :(\n");
        } else {
            // continuing message
        }
        delete addr;
    }
    //ets_printf("alpha checksum = %u, continued = %u, fragmentno = %u, messageno = %u \n", fchecksum, continued, fragmentno, messageno);

    if (msg) {
        msg->addAlphaWords(phase, fragmentno, vect->blockstart, vect->blocklength);
        if (continued) {
            flexmsg::addIncompleteMessage(msg);
        } else {
            ets_printf(">TO:>");
            for (uint8_t c = 0; c < msg->addrarr.size(); c++) {
                ets_printf(" %u", msg->addrarr[c]->ric);
            }
            ets_printf("\nMSG:> ");
            ets_printf(msg->getAlphaData());
            ets_printf("\n----------------------\n");

            return msg;
        }
    }
    return nullptr;
}
flexdatablock::flexdatablock(uint8_t words) {
    wordcount = words;
    //ets_printf("%u words reserved for block\n", words);
    if (words) block = (uint32_t*)calloc(words, sizeof(uint32_t));
}
flexdatablock::~flexdatablock() {
    if (block != NULL) {
        free(block);
        block = NULL;
    }
    // delete all entries in the address array
    for (uint8_t d = 0; d < addrarr.size(); d++) {
        delete addrarr.at(d);
        addrarr.at(d) = nullptr;
    }
}
uint8_t flexdatablock::addrBlockSize() {
    uint8_t ret = 0;
    for (uint8_t c = 0; c < addrarr.size(); c++) {
        ret += addrarr.at(c)->addresswords;
    }
    return ret;
}
uint8_t flexdatablock::blockSize() {
    uint8_t ret = 0;
    ret += 2 * addrBlockSize();  // address + vector size
    ret += wordcount;
    return ret;
}

// vector parent
enum flexvector::vectortype flexvector::getVectorType(uint32_t* wordp) {
    uint32_t temp = *wordp;
    return getVectorType(temp);
};
enum flexvector::vectortype flexvector::getVectorType(uint32_t temp) {
    if (!validateMotChecksum(temp)) {
        return flexvector::vectortype::VECTOR_INVALID;
    }
    switch (getValue(4, 3, temp)) {
        case 0:
            return flexvector::vectortype::VECTOR_SECURE;
            break;
        case 1:
            return flexvector::vectortype::VECTOR_SHORT_INSTRUCTION;
            break;
        case 2:
            return flexvector::vectortype::VECTOR_SHORT_MESSAGE;
            break;
        case 3:
            return flexvector::vectortype::VECTOR_NUMERIC;
            break;
        case 4:
            return flexvector::vectortype::VECTOR_SPECIAL;
            break;
        case 5:
            return flexvector::vectortype::VECTOR_ALPHA;
            break;
        case 6:
            return flexvector::vectortype::VECTOR_HEX;
            break;
        case 7:
            return flexvector::vectortype::VECTOR_NUMBEREDNUMERIC;
            break;
        default:
            return flexvector::vectortype::VECTOR_INVALID;
    }
};
uint32_t flexvector::getNumericVector(uint8_t start, uint8_t words, uint8_t checksum) {
    // generate the vector
    uint32_t word = 0;

    uint8_t wordcount = words;
    uint8_t datapos = start;

    word |= addValue(4, 3, 3);  // numeric type
    word |= addValue(7, 7, datapos);
    word |= addValue(14, 3, wordcount - 1);
    word |= addValue(17, 4, checksum);
    word = motchecksum(word);
    word = createcrc(word);
    return word;
}
uint32_t flexvector::getAlphaVector(uint8_t start, uint8_t words) {
    uint32_t word;
    word = addValue(4, 3, 5);
    word |= addValue(7, 7, start);
    word |= addValue(14, 7, words);  // dit was + 1
    word = motchecksum(word);
    word = createcrc(word);
    return word;
}
uint32_t flexvector::getShortInstruction(uint8_t address, uint8_t frame) {
    uint32_t word;
    word = addValue(4, 3, 1);  // short instruction
    word |= addValue(7, 3, 0);
    word |= addValue(10, 7, frame);
    word |= addValue(17, 4, address);
    word = motchecksum(word);
    word = createcrc(word);
    return word;
}
uint32_t flexvector::getShortInstruction(uint16_t event) {
    uint32_t word;
    word = addValue(4, 3, 1);  // short instruction
    word = addValue(7, 3, 1);  // event
    word = motchecksum(word);
    word = createcrc(word);
    return word;
}
flexvector::flexvector(uint32_t* wordp) {
    type = getVectorType(wordp);
    switch (type) {
        case vectortype::VECTOR_NUMERIC:
        case vectortype::VECTOR_NUMBEREDNUMERIC:
        case vectortype::VECTOR_SPECIAL:
            decodeNumeric(wordp);
            break;
        case vectortype::VECTOR_SHORT_INSTRUCTION:
            decodeShortInstruction(wordp);
            break;
        case vectortype::VECTOR_SHORT_MESSAGE:
            decodeShortMessage(wordp);
            break;
        case vectortype::VECTOR_HEX:
        case vectortype::VECTOR_SECURE:
            ets_printf("!!! Unimplemented Hex/Secure Vector Type (%u)\n", getValue(4, 3, *wordp));
        case vectortype::VECTOR_ALPHA:
            decodeAlpha(wordp);
            break;
        default:
            ets_printf("!!! Unimplemented Vector Type (%u)\n", getValue(4, 3, *wordp));
            break;
    }
}
void flexvector::decodeNumeric(uint32_t* wordp) {
    uint32_t temp = *wordp;
    blockstart = getValue(7, 7, temp);
    blocklength = 1 + getValue(14, 3, temp);
    numchecksum = getValue(17, 4, temp);
}
void flexvector::decodeShortMessage(uint32_t* wordp) {
    uint32_t temp = *wordp;
    switch (getValue(7, 2, temp)) {
        case 0:  // short numeric message or service identifier, depending on address TODO: we need the address here in order to determine which it is.
            uint8_t buffer[4];
            buffer[0] = (uint8_t)((temp >> 11) & 0x0F);
            buffer[1] = (uint8_t)((temp >> 15) & 0x0F);
            buffer[2] = (uint8_t)((temp >> 19) & 0x0F);
            ets_printf("Short message: %X%X%X\n", buffer[0], buffer[1], buffer[2]);
            break;
        default:
            ets_printf("!!! Short Message Vector decoding needs to be implemented (further)\n");
    }
}
void flexvector::decodeShortInstruction(uint32_t* wordp) {
    uint32_t temp = *wordp;
    uint8_t dshorttype = getValue(7, 3, temp);
    switch (dshorttype) {
        case 0x00:
            shortInstructionType = shorttype::VECTOR_SHORT_TEMP_ADDRESS;
            tempAddress = getValue(17, 4, temp);
            tempAssignedFrame = getValue(10, 7, temp);
            break;
        case 0x01:
            shortInstructionType = shorttype::VECTOR_SYSTEM_EVENT;
            systemEvent = getValue(10, 11, temp);
            break;
    }
}
void flexvector::decodeAlpha(uint32_t* wordp) {
    uint32_t temp = *wordp;
    blockstart = getValue(7, 7, temp);
    blocklength = getValue(14, 7, temp);
    //ets_printf("Vector start %u, length %u\n", blockstart, blocklength);
}

// flex message content
//      TX
void flexmsg::encode(flexvector::vectortype msgtype, char* datap) {
    type = msgtype;
    switch (type) {
        case flexvector::vectortype::VECTOR_ALPHA:
            encodeAlpha(datap);
            break;
        case flexvector::vectortype::VECTOR_NUMERIC:
            encodeNumeric(datap);
            break;
        default:
            ets_printf("UNHANDLED FLEX MESSAGE TYPE!\n");
            break;
    }
}
void flexmsg::encode(flexvector::vectortype msgtype, uint8_t tempaddress, uint8_t frame) {
    // currently only used for short instruction vectors (temporary address assignment)
    type = msgtype;
    vectorsize = frame;
    messageno = tempaddress;
}
void flexmsg::encodeAlpha(char* datap) {
    // calculate word count
    bytes = strlen(datap);
    bytesleft = bytes;
    alphadata = (char*)calloc(1, bytes + 1);
    if (alphadata) {
        readp = alphadata;
        strcpy(alphadata, datap);
    }

    char* dataindex = alphadata;
    while (*((uint8_t*)dataindex) != 0x00) {
        signature += *((uint8_t*)dataindex++);
        signature &= 0x7F;
    }
    signature += ((3 - ((bytesleft - 2) % 3)) % 3) * 0x03;
    signature &= 0x7F;
    signature = (0xFF ^ signature);
    wordcount = countWords(readp);
}
void flexmsg::encodeNumeric(char* datap) {
    // calculate word count
    bytes = strlen(datap);
    if (bytes > 41) bytes = 41;  // max 41 bytes of numeric data
    numericdata = (char*)calloc(1, bytes + 1);
    if (numericdata) {
        strncpy(numericdata, datap, bytes);
    }
    uint32_t bits = (bytes * 4) + 2;
    wordcount = (uint8_t)(bits / 21);
    if (bits % 21) {
        wordcount++;
    }
    if (wordcount > 8) wordcount = 8;  //max wordcount = 8; (this should've been caught by the bytes check)
}
flexdatablock* flexmsg::getBlock() {
    return getBlock(80);
}
flexdatablock* flexmsg::getBlock(uint8_t words) {
    flexdatablock* freshblock;
    delay(10);
    switch (type) {
        case flexvector::vectortype::VECTOR_ALPHA:
            freshblock = getAlphaBlock(words);
            break;
        case flexvector::vectortype::VECTOR_NUMERIC:
            freshblock = getNumericBlock();
            break;
        case flexvector::vectortype::VECTOR_SHORT_INSTRUCTION:
            freshblock = getShortInstructionBlock();
            break;
        default:
            ets_printf("UNHANDLED FLEX MESSAGE TYPE!\n");
            return nullptr;
            break;
    }
    //make a copy of the entire address array. Using this based on RIC is probably not enough, as this doesn't cope with RIC's with offset / TODO
    // this could be done using a shared_ptr to the address object
    for (uint8_t d = 0; d < addrarr.size(); d++) {
        flexaddress* newaddr = new flexaddress();
        newaddr->decodeRIC(addrarr.at(d)->ric);
        freshblock->addrarr.push_back(newaddr);
    }
    return freshblock;
}
flexdatablock* flexmsg::getAlphaBlock(uint8_t words) {
    flexdatablock* blockp;
    uint32_t word;
    uint16_t fchecksum;

    bool firstfragment = true;
    if (fragmentid != 0x03) {
        firstfragment = false;
    } else {  // first fragment, get message number;
        messageno = flexstatus.curmessageno;
        flexstatus.curmessageno = (1 + flexstatus.curmessageno % 64);
    }

    // this updates how many words are required for the message, as the read pointer is incremented.
    wordcount = countWords(readp);
    if (words > wordcount) words = wordcount;

    // generate new flexdatablock
    blockp = new flexdatablock(words);
    blockp->type = flexvector::vectortype::VECTOR_ALPHA;

    // FIRST MESSAGE WORD
    word = addValue(11, 2, fragmentid);  // add fragment ID
    word |= addValue(13, 6, messageno);
    if (firstfragment == true) {                    // check if this is the first message fragment
        if (retrieval) word |= addValue(19, 1, 1);  // retrieval bool
        if (maildrop) word |= addValue(20, 1, 1);   // maildrop bool
    } else {
        // U/V fragment control goes here...
    }
    if (words < wordcount) {
        word |= addValue(10, 1, 1);  // add Continued flag
        fragmentid++;
        fragmentid %= 4;
        if (fragmentid == 0x03) fragmentid = 0x00;
    } else {
        // it'll fit in this block
    }

    blockp->vectorwordtwo = word;
    blockp->block[0] = word;

    // add first word checksums
    fchecksum = getValue(8, 8, word);
    fchecksum += getValue(16, 8, word);
    fchecksum &= 0x3FF;

    // WORD LOOP FOR ALL WORDS
    for (uint8_t c = 1; c < words; c++) {
        if (c == 1 && firstfragment == true) {  // second word of first fragment (add signature)
            word = addValue(0, 7, signature);
            word |= addValue(7, 7, *(readp++));
            word |= addValue(14, 7, *(readp++));
        } else {  // this could be cleaner I guess
            if ((*((uint8_t*)readp)) != 0x00) {
                word = addValue(0, 7, *(readp++));
            } else {
                word = addValue(0, 7, 0x03);
            }
            if ((*((uint8_t*)readp)) != 0x00) {
                word |= addValue(7, 7, *(readp++));
            } else {
                word |= addValue(7, 7, 0x03);
            }
            if ((*((uint8_t*)readp)) != 0x00) {
                word |= addValue(14, 7, *(readp++));
            } else {
                word |= addValue(14, 7, 0x03);
            }
        }
        fchecksum += getValue(0, 8, word);
        fchecksum += getValue(8, 8, word);
        fchecksum += getValue(16, 8, word);
        fchecksum &= 0x3FF;
        word = createcrc(word);
        blockp->block[c] = word;
    }

    // update words required (could be 0, could be what remains for this message). countWords alone doesn't work, as it always
    // assumes a minimum
    wordcount = countWords(readp);
    if (strlen(readp) == 0) {
        wordcount = 0;
    }

    // STORE FIRST WORD (WITH CHECKSUM) INTO POSITION
    fchecksum = 0xFFFF ^ fchecksum;
    word = blockp->vectorwordtwo | addValue(0, 10, fchecksum);
    word = createcrc(word);
    blockp->vectorwordtwo = word;
    blockp->block[0] = word;
    return blockp;
}
flexdatablock* flexmsg::getNumericBlock() {
    uint32_t word = 0;
    int8_t wordindex = 0;
    uint8_t curbyte = 0;
    uint8_t wordbitsleft = 21;
    uint8_t bytesleft = bytes;
    uint8_t orphanbyte = 0;
    uint8_t orphanbits = 2;
    char* datap = numericdata;
    uint8_t checksum = 0x00;

    flexdatablock* blockp = new flexdatablock(wordcount);
    blockp->type = flexvector::vectortype::VECTOR_NUMERIC;
    while (bytesleft) {
        word = 0;
        if (orphanbits) {
            orphanbyte &= ((1 << orphanbits) - 1);  // mask off the irrelevant part;
            word |= (uint32_t)orphanbyte;
            wordbitsleft -= orphanbits;
        }
        while (wordbitsleft >= 4) {  // we can shift-in an entire character (4 bits)
            if (bytes) {
                curbyte = encodeChar(*datap);
            } else {
                curbyte = encodeChar(0x20);
            }
            curbyte = (swap(curbyte)) >> 4;
            word <<= 4;
            word |= (uint32_t)curbyte;
            wordbitsleft -= 4;
            if (bytesleft) {
                datap++;
                bytesleft--;
            }
        }
        if (wordbitsleft) {  // room for a partial character
            if (bytes) {
                curbyte = encodeChar(*datap);
            } else {
                curbyte = 0;
            }
            curbyte = (swap(curbyte)) >> 4;
            orphanbits = 4 - wordbitsleft;
            word <<= wordbitsleft;
            word |= (uint32_t)(curbyte >> orphanbits);
            orphanbyte = curbyte;
            if (bytesleft) {
                bytesleft--;
                datap++;
            }
        } else {
            orphanbits = 0;
            orphanbyte = 0;
        }
        word <<= 11;
        checksum += swap(word >> 24);
        checksum += swap(word >> 16);
        checksum += swap((word >> 8) & 0xF8);

        // handling for double vector word?
        blockp->block[wordindex] = word;
        if (wordindex == 0) {
            blockp->vectorwordtwo = word;
        }
        wordindex++;
        wordbitsleft = 21;
    }
    checksum = (checksum >> 6) + (checksum & 0x3F);
    checksum = (0xFF ^ checksum) & 0x3F;
    blockp->numchecksum = checksum;
    checksum >>= 4;  // K4/K5 checksum bits in the message field;
                     // handling for double vector word
    blockp->block[0] |= addValue(0, 2, checksum);
    for (wordindex = 0; wordindex < wordcount; wordindex++) {
        blockp->block[wordindex] = createcrc((blockp->block[wordindex]) & 0b11111111111111111111100000000000);
    }
    wordcount = 0;
    return blockp;
};
flexdatablock* flexmsg::getShortInstructionBlock() {
    flexdatablock* blockp;
    blockp = new flexdatablock(0);
    blockp->type = type;
    blockp->vectorwordtwo = flexvector::getShortInstruction(messageno, vectorsize);
    return blockp;
}
void flexmsg::addAddress(uint32_t ric) {
    flexaddress* newaddr = new flexaddress();
    newaddr->decodeRIC(ric);
    addrarr.push_back(newaddr);
}
uint16_t flexmsg::countWords(char* start) {
    // checks how many words are needed to send the entire message.
    uint32_t words;
    bytesleft = strlen(start);
    if (fragmentid == 0x03) {  // check if this is the first message fragment
        // first message fragment uses an entire word for status info/length/etc, and the first part of the second word.
        words = (2 + ((bytesleft - 2) / 3));
        if ((bytesleft - 2) % 3) {
            words++;
        }
    } else {
        // all subsequent fragments use only the first word for info. Then, up to three characters per word are stored
        words = 1 + (bytesleft) / 3;
        if ((bytesleft) % 3) {
            words++;
        }
    }
    return words;
}
uint8_t flexmsg::addrBlockSize() {
    // check size of the address block for this message. Checks if address size is one or two words
    uint8_t ret = 0;
    for (uint8_t c = 0; c < addrarr.size(); c++) {
        ret += addrarr[c]->addresswords;
    }
    return ret;
}
//      RX
flexmsg* flexmsg::getIncomplete(uint32_t ric, uint8_t messageno) {
    flexmsg* tempmsg;
    for (uint8_t c = 0; c < flexmessages.size(); c++) {
        tempmsg = flexmessages.at(c);
        if (tempmsg->primaryAddress == ric && tempmsg->messageno == messageno) {
            flexmessages.erase(flexmessages.begin() + c);
            return tempmsg;
        }
    }
    return nullptr;
}
void flexmsg::addIncompleteMessage(flexmsg* temp) {
    flexmessages.push_back(temp);
    temp->timeout = 240;
}
uint8_t flexmsg::encodeChar(char c) {
    switch (c) {
        case '0':
            return 0;
        case '1':
            return 1;
        case '2':
            return 2;
        case '3':
            return 3;
        case '4':
            return 4;
        case '5':
            return 5;
        case '6':
            return 6;
        case '7':
            return 7;
        case '8':
            return 8;
        case '9':
            return 9;
        case '/':
            return 10;
        case 'U':
            return 11;
        case '-':
            return 13;
        case ']':
            return 14;
        case '[':
            return 15;
        case ' ':
        default:
            return 0x0C;
    }
};
void flexmsg::timeoutTrigger() {
    ets_printf("!!! Message expired...\n");
}
void flexmsg::removeStaleMessages() {
    flexmsg* tempmsg;
    for (uint8_t c = 0; c < flexmessages.size(); c++) {
        tempmsg = flexmessages.at(c);
        if (tempmsg->timeout) {
            tempmsg->timeout--;
        } else {
            flexmessages.erase(flexmessages.begin() + c);
            tempmsg->timeoutTrigger();
            delete tempmsg;
        }
    }
}
void flexmsg::addAlphaWords(flexphase* phase, uint8_t fragmentno, uint8_t blockstart, uint8_t blocksize) {
    uint16_t newsize = 0;
    bool isMissing = false;
    bool skipFirst = false;
    blocksize--;

    if (fragmentno == 3) {
        newsize = (blocksize * 3) - 1;
        fragmentid = 3;
        skipFirst = true;
    } else {
        newsize = bytes + (blocksize * 3);
        fragmentid++;
        fragmentid %= 4;
        if (fragmentid == 3) fragmentid = 0;
        if (fragmentid != fragmentno) {
            isMissing = true;
            fragmentid = fragmentno;
            newsize += 5;
        }
    }

    // calculate if we need additional bytes for storing error data in the text
    bool errorSet = false;
    uint8_t errorStateChange = 0;
    for (uint8_t c = 0; c < blocksize; c++) {
        if (phase->errorMarker[c] && !errorSet) {
            errorStateChange++;
            errorSet = true;
        } else if (!phase->errorMarker[c] && errorSet) {
            errorStateChange++;
            errorSet = false;
        }
    }
    if (errorSet) errorStateChange++;
    errorSet = false;
    if (errorStateChange) {
        ets_printf("%u errors found!!!\n", errorStateChange);
    }
    newsize = newsize + (4 * errorStateChange);
    // end calculate additional

    if (alphadata) {
        alphadata = (char*)realloc(alphadata, newsize + 3);
    } else {
        alphadata = (char*)calloc(1, newsize + 3);
    }
    if (isMissing) {
        strcpy(alphadata + bytes, "<!!!>\0");
        bytes += 5;
    }
    char* writep = alphadata + bytes;
    for (uint8_t c = 0; c < blocksize; c++) {
        if (phase->errorMarker[c] && !errorSet) {
            errorSet = true;
            *writep = 0x1B;
            writep++;
            *writep = 0x5B;
            writep++;
            *writep = 0x34;
            writep++;
            *writep = 0x6D;
            writep++;
        } else if (!phase->errorMarker[c] && errorSet) {
            errorSet = false;
            *writep = 0x1B;
            writep++;
            *writep = 0x5B;
            writep++;
            *writep = 0x30;
            writep++;
            *writep = 0x6D;
            writep++;
        }

        if (!skipFirst) {
            *writep = getValue(0, 7, phase->getWord(blockstart + c));
            if (*writep > 0x03) writep++;
        }
        *writep = getValue(7, 7, phase->getWord(blockstart + c));
        if (*writep > 0x03) writep++;
        *writep = getValue(14, 7, phase->getWord(blockstart + c));
        if (*writep > 0x03) writep++;

        skipFirst = false;
    }
    if (errorSet) {
        errorSet = false;
        *writep = 0x1B;
        writep++;
        *writep = 0x5B;
        writep++;
        *writep = 0x30;
        writep++;
        *writep = 0x6D;
        writep++;
    }
    *writep = 0;
    bytes = writep - alphadata;
}
void flexmsg::addNumericData(uint8_t* databuffer, uint8_t characters) {
    bytes = characters;
    if (!characters) return;
    numericdata = (char*)calloc(1, characters + 1);
    char* bufferp = numericdata;
    while (characters) {
        *bufferp = decodeChar(*databuffer);
        databuffer++;
        bufferp++;
        characters--;
    }
    *bufferp = 0x00;
}
char flexmsg::decodeChar(uint8_t c) {
    switch (c) {
        case 0:
            return '0';
        case 1:
            return '1';
        case 2:
            return '2';
        case 3:
            return '3';
        case 4:
            return '4';
        case 5:
            return '5';
        case 6:
            return '6';
        case 7:
            return '7';
        case 8:
            return '8';
        case 9:
            return '9';
        case 11:
            return '/';
        case 12:
            return 'U';
        case 13:
            return '-';
        case 14:
            return ']';
        case 15:
            return '[';
        default:
        case 10:
            return ' ';
    }
};
void flexmsg::addTempMappings(uint8_t curframe, flexaddress* addr) {
    flextempaddrmapping* mapping = flextempaddrmapping::getTempMapping(curframe, (addr->ric) - 0x1F7800, false);
    if (mapping) {
        for (uint8_t c = 0; c < mapping->addresslist.size(); c++) {
            flexaddress* tempaddr = mapping->addresslist.at(c);
            if (tempaddr) {
                addrarr.push_back(tempaddr);
            }
        }
        mapping->addresslist.clear();
    }
}
void flexmsg::addAddress(uint8_t curframe, flexaddress* addr) {
    if (addr->word[0].type == flexaddress::shortaddrtype::SHORT_TEMP) {
        addTempMappings(curframe, addr);
        delete addr;  // hmm.
    } else {
        addrarr.push_back(addr);
    }
}
char* flexmsg::getAlphaData() {
    return alphadata;
}
char* flexmsg::getNumericData() {
    return numericdata;
}
flexmsg::~flexmsg() {
    for (uint8_t c = 0; c < addrarr.size(); c++) {
        delete addrarr[c];
        addrarr[c] = NULL;
    }
    if (alphadata) {
        free(alphadata);
        alphadata = NULL;
    }

    if (numericdata) {
        free(numericdata);
        numericdata = NULL;
    }
}

// FLEX phase
flexphase::flexphase(idlestep step) {  // for TX. configured for an idle frame
    switch (step) {
        case idlestep::IDLE_1600:
        case idlestep::IDLE_3200_2:
        case idlestep::IDLE_3200_4_PHASE1:
        case idlestep::IDLE_6400_4_PHASE1:
        case idlestep::IDLE_6400_4_PHASE3:
            for (uint8_t c = 0; c < 11; c++) {
                block[c].word[0] = 0xFFFFFFFF;
                block[c].word[1] = 0x00000000;
                block[c].word[2] = 0xFFFFFFFF;
                block[c].word[3] = 0x00000000;
                block[c].word[4] = 0xFFFFFFFF;
                block[c].word[5] = 0x00000000;
                block[c].word[6] = 0xFFFFFFFF;
                block[c].word[7] = 0x00000000;
            }
            break;
        case idlestep::IDLE_3200_4_PHASE2:
        case idlestep::IDLE_6400_4_PHASE2:
        case idlestep::IDLE_6400_4_PHASE4:
            for (uint8_t c = 0; c < 11; c++) {
                block[c].word[0] = 0x00000000;
                block[c].word[1] = 0x00000000;
                block[c].word[2] = 0x00000000;
                block[c].word[3] = 0x00000000;
                block[c].word[4] = 0x00000000;
                block[c].word[5] = 0x00000000;
                block[c].word[6] = 0x00000000;
                block[c].word[7] = 0x00000000;
            }
            break;
    }
    biwone.biwtype = flexbiw::biwtype_e::BIW;
}
flexphase::flexphase() {
    for (uint8_t c = 0; c < 11; c++) {
        block[c].word[0] = 0x00000000;
        block[c].word[1] = 0x00000000;
        block[c].word[2] = 0x00000000;
        block[c].word[3] = 0x00000000;
        block[c].word[4] = 0x00000000;
        block[c].word[5] = 0x00000000;
        block[c].word[6] = 0x00000000;
        block[c].word[7] = 0x00000000;
    }
    biwone.biwtype = flexbiw::biwtype_e::BIW;
}
flexphase::~flexphase() {
    for (uint8_t c = 0; c < biwarr.size(); c++) {
        delete biwarr[c];
        biwarr[c] = nullptr;
    }
    for (uint8_t c = 0; c < blockarr.size(); c++) {
        delete blockarr[c];
        blockarr[c] = nullptr;
    }
}
// TX
flexbiw* flexphase::addBIW(flexbiw::biwtype_e type) {
    flexbiw* newbiw = new flexbiw();
    biwarr.push_back(newbiw);
    biwarr.back()->biwtype = type;
    return newbiw;
}
flexdatablock* flexphase::addBlock(flexdatablock* blockp) {
    blockarr.push_back(blockp);
    //ets_printf("Adding block... block size is now %u\n", blockarr.size());
    return blockp;
}
void flexphase::prepPhaseForTX() {
    // here, we prepare the phase data for transmission

    uint8_t index = 1;  // word 0 is reserved for the BIW, so we start at index = 1.
    uint8_t datablockstart = 0;
    uint32_t* word = &(block[0].word[0]);  // initialize current word pointer to the right location. We'll increment from here.

    // process BIW's from the array. There's a minimum of one BIW (the one that contains the vector pointer)
    biwone.blocksize = 1 + biwarr.size() % 4;
    for (uint8_t c = 0; c < biwarr.size() % 4; c++) {
        //ets_printf("DEBUG: Adding BIW to frame\n");
        word[index] = biwarr[c]->output();
        index++;
    }

    // We'll determine the start of the data block here. The start of the datablock is basically 1+(biwarray-size)+ addressess + vectors
    datablockstart = biwone.blocksize;
    for (uint8_t c = 0; c < blockarr.size(); c++) {
        datablockstart += 2 * (blockarr[c]->addrBlockSize());
        for (uint8_t d = 0; d < blockarr[c]->addrarr.size(); d++) {
            blockarr[c]->addrarr.at(d)->RICtoWord(&(word[index]));
            //ets_printf("DBG: +RIC %u to msg\n", blockarr[c]->addrarr.at(d)->ric);
            index += blockarr[c]->addrarr.at(d)->addresswords;
        }
    }

    // check if we managed to contain all addresses in the first block
    if (index < 8) {
        lowTraffic = true;
    }

    // we've collected enough data to generate BIW one
    biwone.collapse = flexstatus.collapse;
    biwone.vectorstart = index;
    word[0] = biwone.output();

    // iterate through all the blocks, generate vectors and copy relevant data to the word buffer
    for (uint8_t c = 0; c < blockarr.size(); c++) {
        // set vectors;
        switch (blockarr[c]->type) {
            case flexvector::vectortype::VECTOR_ALPHA:
                for (uint8_t d = 0; d < blockarr[c]->addrarr.size(); d++) {
                    // TODO: if only two-word addresses are used, we should probably skip the first word of the datablock, because it's not used
                    word[index] = flexvector::getAlphaVector(datablockstart + (blockarr[c]->addrarr.at(d)->addresswords) - 1, blockarr[c]->wordcount);
                    index++;
                    // if we have a two-word address, the first word after the vector is the first word of the message block
                    if (blockarr[c]->addrarr.at(d)->addresswords == 2) {
                        word[index] = blockarr[c]->vectorwordtwo;
                        index++;
                    } else {
                        // single-word address
                    }
                }
                break;
            case flexvector::vectortype::VECTOR_NUMERIC:
                for (uint8_t d = 0; d < blockarr[c]->addrarr.size(); d++) {
                    word[index] = flexvector::getNumericVector(datablockstart + (blockarr[c]->addrarr.at(d)->addresswords) - 1, blockarr[c]->wordcount, blockarr[c]->numchecksum);
                    index++;
                    if (blockarr[c]->addrarr.at(d)->addresswords == 2) {
                        word[index] = blockarr[c]->vectorwordtwo;
                        index++;
                        //ets_printf("DEBUG: 2-Word Numeric vector outputted\n");
                    } else {
                        //ets_printf("DEBUG: 1-Word Numeric vector outputted\n");
                    }
                }
                break;
            case flexvector::vectortype::VECTOR_SHORT_INSTRUCTION:
                for (uint8_t d = 0; d < blockarr[c]->addrarr.size(); d++) {
                    word[index] = blockarr[c]->vectorwordtwo;
                    index++;
                    if (blockarr[c]->addrarr.at(d)->addresswords == 2) {
                        word[index] = 0x00;
                        index++;
                        //ets_printf("DEBUG: 2-Word Short-instruction vector outputted\n");
                    } else {
                        //ets_printf("DEBUG: 1-Word Short-instruction vector outputted\n");
                    }
                }
                break;
            default:
                ets_printf("DEBUG: Unhandled message type in prepFrame\n");
                break;
        }

        // check if we have words to copy from current block
        if (blockarr[c]->wordcount) {
            //ets_printf("DEBUG: Copying %u bytes from %u to %u \n", blockarr[c]->wordcount * sizeof(uint32_t), &(blockarr[c]->block[0]), &(word[index]));
            if (datablockstart + blockarr[c]->wordcount >= 88) return;  // check for overflow
            memcpy(&(word[datablockstart]), &(blockarr[c]->block[0]), blockarr[c]->wordcount * sizeof(uint32_t));
            datablockstart += blockarr[c]->wordcount;
        } else {
            ets_printf("DEBUG: This vector doesn't have any block words\n");
        }
    }

    //ets_printf("DEBUG: Done with this frame\n");
}
uint8_t flexphase::usedWords() {
    uint8_t ret = 1;  // first word is always used by biw 1
    ret += biwarr.size();
    for (uint8_t c = 0; c < blockarr.size(); c++) {
        ret += blockarr[c]->blockSize();
    }
    return ret;
}

uint8_t flexphase::freeWords() {
    return 88 - usedWords();
}
// RX
void flexphase::decode(std::vector<flexmsg*>* messages) {
    CBCH3121 bch;
    uint16_t errors;
    for (uint8_t c = 0; c < 88; c++) {
        errorMarker[c] = !(bch.decode(block[c / 8].word[c % 8], errors));
    }
    biwone.input(getWord(0), 0);
    if (!biwone.isValid) {
        ets_printf("BIW failed to decode :( \n");
        return;
    }

    for (uint8_t c = 1; c < biwone.blocksize; c++) {
        flexbiw* secondbiw = new flexbiw();
        secondbiw->input(block[0].word[c % 8], c % 8);
        biwarr.push_back(secondbiw);
    }

    for (uint8_t c = biwone.blocksize; c < biwone.vectorstart;) {
        flexmsg* msg = flexdatablock::decode(this, c, frame);
        //add message to message array
        if (msg) {
            messages->push_back(msg);
            msg = nullptr;
        }
        // we need to determine how large the address words are in order to iterate through the entire block
        flexaddress* addr = new flexaddress();
        addr->decodeWord(getWordP(c));
        c += addr->addresswords;
        delete addr;
    }
}
uint32_t flexphase::getWord(uint8_t index) {
    return block[index / 8].word[index % 8];
}
uint32_t* flexphase::getWordP(uint8_t index) {
    return &(block[index / 8].word[index % 8]);
}
// FLEX Frame

void flexframe::addByte(uint8_t byte) {
    uint8_t column;
    uint8_t block;
    uint8_t byteoffset;
    switch (type) {
        case flexsynctype::SYNC_FLEX_1600:
            column = (bitcount / 8) % 32;
            block = (bitcount / 256) % 11;
            for (uint8_t c = 0; c < 8; c++) {
                phase[0]->block[block].word[c] |= (((byte << c) & 0x80) >> 7) << (31 - column);
            }
            bitcount += 8;
            break;
        case flexsynctype::SYNC_FLEX_3200_2:
        case flexsynctype::SYNC_FLEX_3200_4:
            column = (bitcount / 16) % 32;
            block = (bitcount / 512) % 11;
            byteoffset = ((bitcount / 8) % 2);
            phase[0]->block[block].word[0 + (byteoffset * 4)] |= (((byte)&0x80) >> 7) << (31 - column);
            phase[1]->block[block].word[0 + (byteoffset * 4)] |= (((byte << 1) & 0x80) >> 7) << (31 - column);
            phase[0]->block[block].word[1 + (byteoffset * 4)] |= (((byte << 2) & 0x80) >> 7) << (31 - column);
            phase[1]->block[block].word[1 + (byteoffset * 4)] |= (((byte << 3) & 0x80) >> 7) << (31 - column);
            phase[0]->block[block].word[2 + (byteoffset * 4)] |= (((byte << 4) & 0x80) >> 7) << (31 - column);
            phase[1]->block[block].word[2 + (byteoffset * 4)] |= (((byte << 5) & 0x80) >> 7) << (31 - column);
            phase[0]->block[block].word[3 + (byteoffset * 4)] |= (((byte << 6) & 0x80) >> 7) << (31 - column);
            phase[1]->block[block].word[3 + (byteoffset * 4)] |= (((byte << 7) & 0x80) >> 7) << (31 - column);
            bitcount += 8;
            break;
        case flexsynctype::SYNC_FLEX_6400:
            column = (bitcount / 32) % 32;
            block = (bitcount / 1024) % 11;
            byteoffset = ((bitcount / 8) % 4);
            phase[0]->block[block].word[byteoffset * 2] |= (((byte)&0x80) >> 7) << (31 - column);
            phase[1]->block[block].word[byteoffset * 2] |= (((byte << 1) & 0x80) >> 7) << (31 - column);
            phase[2]->block[block].word[byteoffset * 2] |= (((byte << 2) & 0x80) >> 7) << (31 - column);
            phase[3]->block[block].word[byteoffset * 2] |= (((byte << 3) & 0x80) >> 7) << (31 - column);
            phase[0]->block[block].word[(byteoffset * 2) + 1] |= (((byte << 4) & 0x80) >> 7) << (31 - column);
            phase[1]->block[block].word[(byteoffset * 2) + 1] |= (((byte << 5) & 0x80) >> 7) << (31 - column);
            phase[2]->block[block].word[(byteoffset * 2) + 1] |= (((byte << 6) & 0x80) >> 7) << (31 - column);
            phase[3]->block[block].word[(byteoffset * 2) + 1] |= (((byte << 7) & 0x80) >> 7) << (31 - column);
            bitcount += 8;
            break;
        default:
            break;
    }
}
uint8_t flexframe::getMultiplex(flexsynctype ftype) {
    switch (ftype) {
        case flexsynctype::SYNC_FLEX_1600:
            return 1;
            break;
        case flexsynctype::SYNC_FLEX_3200_2:
        case flexsynctype::SYNC_FLEX_3200_4:
            return 2;
            break;
        case flexsynctype::SYNC_FLEX_6400:
            return 4;
            break;
        default:
            return 1;
    }
}

flexframe::flexframe(flexsynctype ftype) {  // for TX
    type = ftype;
    uint8_t multiplexc = getMultiplex(ftype);
    multiplex = multiplexc;
    flexphase* ph;
    switch (ftype) {
        case flexsynctype::SYNC_FLEX_1600:
            ph = new flexphase(flexphase::idlestep::IDLE_1600);  //for TX, filled
            phase.push_back(ph);
            break;
        case flexsynctype::SYNC_FLEX_3200_2:
            ph = new flexphase(flexphase::IDLE_3200_2);  //for TX, filled
            phase.push_back(ph);
            ph = new flexphase(flexphase::IDLE_3200_2);  //for TX, filled
            phase.push_back(ph);
            break;
        case flexsynctype::SYNC_FLEX_3200_4:
            ph = new flexphase(flexphase::IDLE_3200_4_PHASE1);  //for TX, filled
            phase.push_back(ph);
            ph = new flexphase(flexphase::IDLE_3200_4_PHASE2);  //for TX, filled
            phase.push_back(ph);
            break;
        case flexsynctype::SYNC_FLEX_6400:
            ph = new flexphase(flexphase::IDLE_6400_4_PHASE1);  //for TX, filled
            phase.push_back(ph);
            ph = new flexphase(flexphase::IDLE_6400_4_PHASE2);  //for TX, filled
            phase.push_back(ph);
            ph = new flexphase(flexphase::IDLE_6400_4_PHASE3);  //for TX, filled
            phase.push_back(ph);
            ph = new flexphase(flexphase::IDLE_6400_4_PHASE4);  //for TX, filled
            phase.push_back(ph);
            break;
        default:
            ets_printf("!!! Tried setting up a frame for an unrecognized sync header\n");
            break;
    }
}
flexframe::flexframe(flexsynctype ftype, uint32_t fiwc) {  // for RX
    type = ftype;
    uint8_t multiplexc = getMultiplex(ftype);
    multiplex = multiplexc;
    flexphase* ph;
    while (multiplexc) {
        ph = new flexphase();  //for RX, unfilled
        //ph = new flexphase(false); //for TX, filled
        phase.push_back(ph);
        multiplexc--;
    }
    fiww.input(fiwc);
    switch (multiplex) {
        case 1:
            phase[0]->lowTraffic = fiww.traffic[0];
            phase[0]->frame = fiww.frame;
            break;
        case 2:
            phase[0]->lowTraffic = fiww.traffic[0];
            phase[0]->frame = fiww.frame;
            phase[1]->lowTraffic = fiww.traffic[0];
            phase[1]->frame = fiww.frame;
            break;
        case 4:
            phase[0]->lowTraffic = fiww.traffic[0];
            phase[1]->lowTraffic = fiww.traffic[1];
            phase[2]->lowTraffic = fiww.traffic[2];
            phase[3]->lowTraffic = fiww.traffic[3];
            phase[0]->frame = fiww.frame;
            phase[1]->frame = fiww.frame;
            phase[2]->frame = fiww.frame;
            phase[3]->frame = fiww.frame;
            break;
    }
}
flexframe::flexframe() {
}
flexframe::~flexframe() {
    for (uint8_t c = 0; c < phase.size(); c++) {
        if (phase[c]) delete phase[c];
    }
}
std::vector<flexmsg*>* flexframe::decode() {
    std::vector<flexmsg*>* messages = new std::vector<flexmsg*>;
    for (uint8_t d = 0; d < phase.size(); d++) {
        phase.at(d)->decode(messages);
        taskYIELD();
    }
    return messages;
}
uint32_t flexframe::getWord() {
    uint32_t temp = 0;
    uint8_t block;
    uint8_t column;
    switch (type) {
        case flexsynctype::SYNC_FLEX_1600:
            block = (bitcount / 256) % 11;
            for (uint8_t c = 0; c < 4; c++) {
                for (uint8_t d = 0; d < 8; d++) {
                    column = (31 - ((bitcount / 8) % 32));
                    temp <<= 1;
                    if (phase[0]->block[block].word[d] & (1 << column)) temp |= 1;
                }
                bitcount += 8;
            }
            break;
        case flexsynctype::SYNC_FLEX_3200_2:
        case flexsynctype::SYNC_FLEX_3200_4:
            block = (bitcount / 512) % 11;
            for (uint8_t c = 0; c < 2; c++) {
                for (uint8_t d = 0; d < 8; d++) {
                    column = (31 - ((bitcount / 16) % 32));
                    temp <<= 1;
                    if (phase[0]->block[block].word[d] & (1 << column)) temp |= 1;
                    temp <<= 1;
                    if (phase[1]->block[block].word[d] & (1 << column)) temp |= 1;
                }
                bitcount += 16;
            }
            column = (bitcount / 16) % 32;
            break;
        case flexsynctype::SYNC_FLEX_6400:
            block = (bitcount / 1024) % 11;
            column = (31 - ((bitcount / 32) % 32));
            for (uint8_t d = 0; d < 8; d++) {
                temp <<= 1;
                if (phase[0]->block[block].word[d] & (1 << column)) temp |= 1;
                temp <<= 1;
                if (phase[1]->block[block].word[d] & (1 << column)) temp |= 1;
                temp <<= 1;
                if (phase[2]->block[block].word[d] & (1 << column)) temp |= 1;
                temp <<= 1;
                if (phase[3]->block[block].word[d] & (1 << column)) temp |= 1;
            }
            bitcount += 32;
            break;
        default:
            break;
    }
    return temp;
}
void flexframe::queueTXBytes(uint8_t* buffer, uint8_t len, flexsynctype txtype) {
    uint8_t temp;
    uint8_t* outp = nullptr;
    switch (txtype) {
        case flexsynctype::SYNC_FLEX_1600:
            outp = (uint8_t*)calloc(4, len);
            for (uint8_t d = 0; d < len; d++) {
                for (uint8_t c = 0; c < 4; c++) {
                    temp = buffer[d] & (1 << (7 - (c * 2)));
                    if (temp) outp[(d * 4) + c] |= 0xA0;
                    temp = buffer[d] & (1 << (7 - ((c * 2) + 1)));
                    if (temp) outp[(d * 4) + c] |= 0x0A;
                }
            }
            for (uint8_t c = 0; c < len * 4; c++) {
                //ets_printf("byte %u = %u \n", c, outp[c]);
            }
            rf4463->writeTxFifo(outp, len * 4);
            break;
        case flexsynctype::SYNC_FLEX_3200_2:
            outp = (uint8_t*)calloc(2, len);
            for (uint8_t d = 0; d < len; d++) {
                if (buffer[d] & (1 << 7)) outp[d * 2] |= 0b10000000;
                if (buffer[d] & (1 << 6)) outp[d * 2] |= 0b00100000;
                if (buffer[d] & (1 << 5)) outp[d * 2] |= 0b00001000;
                if (buffer[d] & (1 << 4)) outp[d * 2] |= 0b00000010;
                if (buffer[d] & (1 << 3)) outp[(d * 2) + 1] |= 0b10000000;
                if (buffer[d] & (1 << 2)) outp[(d * 2) + 1] |= 0b00100000;
                if (buffer[d] & (1 << 1)) outp[(d * 2) + 1] |= 0b00001000;
                if (buffer[d] & 1) outp[(d * 2) + 1] |= 0b00000010;
            }
            rf4463->writeTxFifo(outp, len * 2);
            break;
        case flexsynctype::SYNC_FLEX_3200_4:
            outp = (uint8_t*)calloc(2, len);
            for (uint8_t d = 0; d < len; d++) {
                if (buffer[d] & (1 << 7)) outp[d * 2] |= 0b10100000;
                if (buffer[d] & (1 << 6)) outp[d * 2] |= 0b01010000;
                if (buffer[d] & (1 << 5)) outp[d * 2] |= 0b00001010;
                if (buffer[d] & (1 << 4)) outp[d * 2] |= 0b00000101;
                if (buffer[d] & (1 << 3)) outp[(d * 2) + 1] |= 0b10100000;
                if (buffer[d] & (1 << 2)) outp[(d * 2) + 1] |= 0b01010000;
                if (buffer[d] & (1 << 1)) outp[(d * 2) + 1] |= 0b00001010;
                if (buffer[d] & 1) outp[(d * 2) + 1] |= 0b00000101;
            }
            rf4463->writeTxFifo(outp, len * 2);
            break;
        case flexsynctype::SYNC_FLEX_6400:
            outp = (uint8_t*)calloc(1, len);
            for (uint8_t d = 0; d < len; d++) {
                outp[d] = buffer[d];
            }
            rf4463->writeTxFifo(outp, len);
            break;
        default:
            break;
    }
    if (outp) free(outp);
}
void flexframe::queueWord(uint32_t word, flexsynctype type) {
    uint8_t buffer[4];
    buffer[3] = (uint8_t)(word & 0xFF);
    buffer[2] = (uint8_t)(word >> 8) & 0xFF;
    buffer[1] = (uint8_t)(word >> 16) & 0xFF;
    buffer[0] = (uint8_t)(word >> 24) & 0xFF;
    queueTXBytes(buffer, 4, type);
}
void flexframe::startTX() {
    xTaskCreate(this->txTaskStart, "txFlexTask", 1500, this, configMAX_PRIORITIES - 1, NULL);
}
bool flexframe::txBytesWait(uint8_t bytes) {
    uint8_t free = rf4463->getTXFIFOAvail();
    if (free > bytes) return true;
    vTaskDelay((((bytes - free) * 10) / 8) / portTICK_PERIOD_MS);
    uint8_t timeout = 100;
    while (true) {
        int8_t avail = rf4463->getTXFIFOAvail();
        if (avail >= bytes) {
            return true;
        } else if (avail == -1) {
            return false;
        } else if (timeout == 0) {
            return false;
        } else {
            vTaskDelay(2 / portTICK_PERIOD_MS);
        }
        timeout--;
    }
}
void flexframe::txTaskStart(void* _this) {  // helper function
    bool txCompleted = static_cast<flexframe*>(_this)->txTask();
    if (!txCompleted) {
        ets_printf("FLEX TX incomplete!!!\n");
        static_cast<flexframe*>(_this)->rf4463->powerOnReset();
    }
    vTaskDelete(NULL);
}
bool flexframe::txTask() {
    // check if we're already sending data
    if ((!flexstatus.sendingflex) || (flexstatus.flexfrequency != frequency)) {
        // init RF4463 and restart transmission
        rf4463->init(frequency, RF4463::rftype::FLEX_6400_TX);
        rf4463->fifoReset();
        //rf4463->enterReadyMode();
        queueTXBytes((uint8_t*)(const uint8_t[]){0xAA, 0xAA, 0xAA}, 3, SYNC_FLEX_1600);  // 1 bytes shorter to account for changing frequency/init
        rf4463->startPacketTX(1496);
        flexstatus.sendingflex = true;
        flexstatus.flexfrequency = frequency;
    } else {
        // don't reinit
        //rf4463->enterReadyMode();
        rf4463->fifoReset();
        queueWord(0xAAAAAAAA, flexsynctype::SYNC_FLEX_1600);  // 16 bytes
        rf4463->startPacketTX(1500);
    }
    if (status.ledupdater) xTaskNotify(status.ledupdater, LED_RF_TX, eSetBits);
    bitcount = 0;
    vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 3);
    switch (type) {
        case flexsynctype::SYNC_FLEX_1600:
            queueWord(FLEX_A1, flexsynctype::SYNC_FLEX_1600);                          // 16
            queueTXBytes((uint8_t*)(const uint8_t[]){0x55, 0x55}, 2, SYNC_FLEX_1600);  // 8
            queueWord(FLEX_INV ^ FLEX_A1, flexsynctype::SYNC_FLEX_1600);               //16
            break;
        case flexsynctype::SYNC_FLEX_3200_2:
            queueWord(FLEX_A2, flexsynctype::SYNC_FLEX_1600);
            queueTXBytes((uint8_t*)(const uint8_t[]){0x55, 0x55}, 2, SYNC_FLEX_1600);
            queueWord(FLEX_INV ^ FLEX_A2, flexsynctype::SYNC_FLEX_1600);
            break;
        case flexsynctype::SYNC_FLEX_3200_4:
            queueWord(FLEX_A3, flexsynctype::SYNC_FLEX_1600);
            queueTXBytes((uint8_t*)(const uint8_t[]){0x55, 0x55}, 2, SYNC_FLEX_1600);
            queueWord(FLEX_INV ^ FLEX_A3, flexsynctype::SYNC_FLEX_1600);
            break;
        case flexsynctype::SYNC_FLEX_6400:
            queueWord(FLEX_A4, flexsynctype::SYNC_FLEX_1600);
            queueTXBytes((uint8_t*)(const uint8_t[]){0x55, 0x55}, 2, SYNC_FLEX_1600);
            queueWord(FLEX_INV ^ FLEX_A4, flexsynctype::SYNC_FLEX_1600);
            break;
        default:
            break;
    }
    if (!txBytesWait(32)) return false;  // set to a lower value if you encounter a FIFO low warning
    bitcount = 0;
    for (uint8_t c = 0; c < phase.size(); c++) {
        phase[c]->prepPhaseForTX();
        fiww.traffic[c] = phase[c]->lowTraffic;
    }
    if (multiplex == 2) {
        fiww.traffic[2] = fiww.traffic[1];
        fiww.traffic[3] = fiww.traffic[2];
        fiww.traffic[1] = fiww.traffic[0];
    }
    if (multiplex == 4) {
        fiww.traffic[1] = fiww.traffic[0];
        fiww.traffic[2] = fiww.traffic[0];
        fiww.traffic[3] = fiww.traffic[0];
    }

    uint8_t bavail = rf4463->getTXFIFOAvail();
    if (bavail > 40) {
        ets_printf("!!! only %u bytes remaining for TX in buffer!!!\n", 64 - bavail);
    }

    if (!txBytesWait(36)) return false;
    queueWord(fiww.output(), flexsynctype::SYNC_FLEX_1600);

    switch (type) {
        case flexsynctype::SYNC_FLEX_1600:
            queueTXBytes((uint8_t*)(const uint8_t[]){0xAE, 0xD8, 0x45, 0x12, 0x7B}, 5, SYNC_FLEX_1600);
            break;
        case flexsynctype::SYNC_FLEX_3200_2:
            queueTXBytes((uint8_t*)(const uint8_t[]){0xAA, 0xAA, 0xAA, 0xED, 0x84, 0x55, 0x55, 0x55, 0x12, 0x7B}, 10, SYNC_FLEX_3200_2);
            break;
        case flexsynctype::SYNC_FLEX_3200_4:
            queueTXBytes((uint8_t*)(const uint8_t[]){0x88, 0x88, 0x88, 0xED, 0x84, 0x22, 0x22, 0x22, 0x12, 0x7B}, 10, SYNC_FLEX_3200_4);
            break;
        case flexsynctype::SYNC_FLEX_6400:
            queueTXBytes((uint8_t*)(const uint8_t[]){0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0xED, 0x84, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x12, 0x7B}, 20, SYNC_FLEX_6400);
            break;
        default:
            break;
    }
    for (uint16_t c = 0; c < (88 * multiplex); c++) {
        if (!txBytesWait(4 * (4 / multiplex))) return false;
        queueWord(getWord(), type);
    }

    vTaskDelay(80 / portTICK_PERIOD_MS);

    // switch back to RX mode if in flex Synced mode on the RX frequency
    if ((settings.current.freq == frequency) && (settings.current.mode == MODE_FLEX_SYNCED)) {
        //vTaskDelay(3 / portTICK_PERIOD_MS);
        rf4463->enterReadyMode();
        restartDecoder();
        flexstatus.sendingflex = false;
    }
    return true;
}