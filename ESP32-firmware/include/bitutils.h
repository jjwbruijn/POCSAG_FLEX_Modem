#include <Arduino.h>
char IRAM_ATTR bitswitch(char b);
uint32_t IRAM_ATTR createcrc(uint32_t in);
uint8_t IRAM_ATTR swap(uint8_t b);
uint16_t IRAM_ATTR swap16(uint16_t b);
uint32_t swap32(uint32_t n);
uint32_t addValue(uint8_t offset, uint8_t bsize, uint32_t value);
uint32_t getValue(uint8_t offset, uint8_t bsize, uint32_t value);
uint32_t motchecksum(uint32_t in);
bool validateMotChecksum(uint32_t word);
uint8_t IRAM_ATTR countbits(uint32_t val);