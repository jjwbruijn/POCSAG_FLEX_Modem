#include <Wire.h>

extern char macStr[13];
extern char shortmac[6];
extern volatile bool wifiConnected;
enum wifistatusenum {
    READY,
    DISCONNECTED,
    GOT_IP,
    CONNECTED
};

extern void wifiReset(void);
extern void wifiProcess(void* parameter);
extern void wifiResetConfig(void);
extern void wifiSaveConfigCallback(void);
extern void wifiInit(void);