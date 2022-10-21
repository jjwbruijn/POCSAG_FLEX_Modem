#ifndef POCSAGUTILS
#define POCSAGUTILS
#include <Arduino.h>

#define SYNCWORD 0x7CD215D8
#define IDLEWORD 0x7A89C197

class pocsagmsg;

class pocsagaddr {
   public:
    enum func { A = 'A',
                B = 'B',
                C = 'C',
                D = 'D' };
    enum func func;
    enum type { TONE,
                ALPHA,
                NUMERIC };
    enum type type;

    uint32_t ric;
    uint8_t frameno;
    pocsagaddr(uint32_t addrword, uint8_t word);
    pocsagaddr(uint32_t ric, enum type type, enum func function);
    pocsagaddr();
    uint32_t decodeAddress(uint32_t word, uint8_t wordcount);
    uint32_t getWord();
};

class pocsagdata {
   public:
    struct frame_s {
        uint32_t word[2];
    };
    struct batchs {
        struct frame_s frame[8];
    };
    struct batchs* batch = nullptr;

    uint16_t* errorflags;

    enum wordtype { TYPE_IDLE,
                    TYPE_ADDR,
                    TYPE_MSG };
    bool msgStarted = false;
    uint16_t batchcount = 0;
    uint16_t msgwordcount = 0;
    uint16_t baudrate = 512;
    uint32_t freq = 0;
    pocsagdata();
    pocsagdata(pocsagaddr ric, char* data);
    ~pocsagdata();
    void addMsg(pocsagaddr ric, char* data);
    void growBatches(uint16_t growby);
    uint16_t requiredSize(pocsagaddr ric, char* data);
    void addWord(uint32_t word, uint8_t wordcount, bool hasErrors);
    uint8_t getFrameSize();
    uint8_t getFrameSize(uint16_t batchcount);
    void decodeMessage(batchs* batchp, uint8_t batchcount, uint16_t wordcount, uint16_t* errorflagsp);
    uint8_t encodeNumber(char c);
    char decodeNumber(uint8_t c);
    enum wordtype getType(uint32_t word);
    bool isMsg(uint32_t word);
    bool isAddr(uint32_t word);
    bool isIdle(uint32_t word);
    static int getRating(char* alpha, char* numeric, pocsagaddr addr);
    void queueMessage(pocsagmsg* msg);
};

class pocsagmsg {
   public:
    pocsagaddr addr;
    char* alphadata;
    char* numdata;
    pocsagmsg(pocsagaddr addr, char* alphadata, char* numdata);
    ~pocsagmsg();
};
#endif