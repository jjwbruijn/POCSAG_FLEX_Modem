#include "bitutils.h"

#include <Arduino.h>

char IRAM_ATTR bitswitch(char b) {
    // a really cool way to reverse bit order. Not sure if it's the fastest, but it works!
    b = (b * 0x0202020202ULL & 0x010884422010ULL) % 1023;
    return (b >> 1);
}

uint8_t IRAM_ATTR swap(uint8_t b) {
    // a really cool way to reverse bit order. Not sure if it's the fastest, but it works!
    return (b * 0x0202020202ULL & 0x010884422010ULL) % 1023;
}

uint16_t IRAM_ATTR swap16(uint16_t b) {
    uint8_t high = (uint8_t)(b >> 8);
    uint8_t low = (uint8_t)(b & 0x00FF);
    high = swap(high);
    low = swap(low);
    return ((uint16_t)low << 8) | ((uint16_t)high);
}

uint32_t swap32(uint32_t n) {
    n = ((n >> 1) & 0x55555555) | ((n << 1) & 0xaaaaaaaa);
    n = ((n >> 2) & 0x33333333) | ((n << 2) & 0xcccccccc);
    n = ((n >> 4) & 0x0f0f0f0f) | ((n << 4) & 0xf0f0f0f0);
    n = ((n >> 8) & 0x00ff00ff) | ((n << 8) & 0xff00ff00);
    n = ((n >> 16) & 0x0000ffff) | ((n << 16) & 0xffff0000);
    return n;
}

uint32_t addValue(uint8_t offset, uint8_t bsize, uint32_t value) {
    value = swap32(value & ((1 << bsize) - 1));
    return value >> offset;
}

uint32_t getValue(uint8_t offset, uint8_t bsize, uint32_t value) {
    value = swap32(value << offset);
    return value & ((1 << bsize) - 1);
}

uint8_t IRAM_ATTR countbits(uint32_t val) {
    uint8_t count = 0;
    for (uint8_t c = 0; c < 32; c++) {
        if (val & 0x00000001) count++;
        val >>= 1;
    }
    return count;
}

uint32_t motchecksum(uint32_t in) {
    uint8_t counter;
    uint32_t orig = in;
    in >>= 4;
    counter = swap((uint8_t)in) & 0x01;  // 1 bit
    in >>= 4;
    counter += swap((uint8_t)in) & 0x0F;  // 4 bits
    in >>= 4;
    counter += swap((uint8_t)in) & 0x0F;  // 4 bits
    in >>= 4;
    counter += swap((uint8_t)in) & 0x0F;  // 4 bits
    in >>= 4;
    counter += swap((uint8_t)in) & 0x0F;  // 4 bits
    counter = ~counter;
    counter = swap(counter) & 0xF0;
    orig |= ((uint32_t)counter) << 24;
    return orig;
}

bool validateMotChecksum(uint32_t word){
    uint32_t temp = word&0b11111111111111111111100000000000;;
    word &= 0b00001111111111111111100000000000;
    word = motchecksum(word);
    if(word == temp){
        return true;
    } else {
        return false;
    }
};

uint32_t IRAM_ATTR createcrc(uint32_t in) {
    // I borrowed this routine from Kristoff. Credit goes to him!
    // https://github.com/on1arf/rf22_pocsag
    // (c) 2014 Kristoff Bonne (ON1ARF)
    //
    // local vars
    uint32_t cw;  // codeword
    uint32_t local_cw = 0;
    uint32_t parity = 0;
    // init cw
    cw = in;
    // move up 11 bits to make place for crc + parity
    local_cw = in; /* bch */
    // calculate crc
    for (int bit = 1; bit <= 21; bit++, cw <<= 1) {
        if (cw & 0x80000000) {
            cw ^= 0xED200000;
        };  // end if
    };      // end for
    local_cw |= (cw >> 21);
    // parity
    cw = local_cw;
    for (int bit = 1; bit <= 32; bit++, cw <<= 1) {
        if (cw & 0x80000000) {
            parity++;
        };  // end if
    };      // end for
    // make even parity
    if (parity % 2) {
        local_cw++;
    };  // end if
    // done
    return (local_cw);
}