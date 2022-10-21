#include <Arduino.h>

extern bool HTTPrestartNeeded;
extern void httpdProcess(void *parameter);
extern void initHTTPD(void);

#define WS_SEND_MODE_STATUS 0x01
#define WS_SEND_SIGNAL 0x02
#define WS_SEND_MODMODE 0x04
#define WS_SEND_FRAME_STATUS 0x08