#include <Arduino.h>

#include <ESP32TimerInterrupt.h>

#define RX_WAIT_SYNC 0
#define RX_WAIT_ADDR 1
#define RX_IN_MESSAGE 2

extern SemaphoreHandle_t wsMutex;
extern volatile bool resetRX;
void timerInterrupt(void);
void pocsagReadBit(uint8_t val);
void IRAM_ATTR pocsagRXInterrupt();
void pocsagProcessing(void* parameter);
void IRAM_ATTR flexsynchelper();
char numericDecode(uint8_t c);
void pocsagTask(void* parameter);
void flexProcessing(void* parameter);
void flexTask(void *parameter);
void flexTest();