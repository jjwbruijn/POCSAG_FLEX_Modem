#ifndef flexutils
#define flexutils

#include <Arduino.h>
#include <vector>
#include "RF4463.h"

#define FLEX_A1 0b01111000111100110101100100111001
#define FLEX_A2 0b10000100111001110101100100111001
#define FLEX_A3 0b01001111100101110101100100111001
#define FLEX_A4 0b00100001010111110101100100111001
#define FLEX_AR 0b11001011001000000101100100111001
#define FLEX_INV 0b11111111111111111111111111111111

struct flexdatablock;
class flexframe;
class flexmsg;

/* 
FLEX pager protocol implementation

Structure:
    A flex frame consists of one or more 'phases'. These phases contain the message and instruction data. Within the 'phase' class, the
    information is stored in 'blocks' before being structured in phase words for transmission. This is done to allow for phase/frame
    optimization before transmission

    A message or instruction is created in a flexmsg object, and 'getBlock' is called to retrieve the block of phase data. This block is
    then assigned to a specific phase.

                                       ┌──────────────┐   ┌────────────┐
                                       │Message       │   │FLEX Address│
                                       │-type(instr)  │   │            │
                                       │-Alphadata*   │◄──┤-RIC        │
                                       │-Numericdata* │   │-Phase      │
                                       │-FLEX addr*[] │   │-Frame      │
                                       └──────┬───────┘   └────────────┘
                                              │
                                              ▼
  ┌──────────┐    ┌───────────────┐    ┌──────────────┐
  │FLEX Frame│    │FLEX Phase     │    │Datablocks    │
  ├──────────┤    ├───────────────┤    ├──────────────┤
  │-FIW      │    │-Data blocks*[]│◄───┤-FLEX addr*[] │
  │-Phases[] │◄───┤-BIW One       │    │-Data words * │
  └──────────┘    │-BIW*[]        │    └──────────────┘
       ▲          │-FLEX Words[88]│
       │          └───────────────┘
       │                 ▲
       │                 │
       │                 │
   ┌───┴────┐        ┌───┴────┐
   │FLEX FIW│        │FLEX BIW│
   └────────┘        └────────┘
*/

struct flexstatusstruct {
    uint8_t frame;
    uint8_t cycle;
    uint8_t state;
    uint8_t rxcollapse = 0;
    uint8_t collapse = 4;  // 0 = all bits masked;
    uint8_t curblock;
    uint8_t curbit;
    uint32_t flexfrequency = 0;
    uint8_t curmessageno = 0;
    bool sendingflex = false;
};

extern struct flexstatusstruct flexstatus;

enum flexsynctype { SYNC_NOSYNC,
                    SYNC_FLEX_1600,
                    SYNC_FLEX_3200_2,
                    SYNC_FLEX_3200_4,
                    SYNC_FLEX_6400,
                    SYNC_FLEX_RESYNC };

class flexsync {
   public:
    uint32_t FIWword;
    flexsynctype type;
    volatile bool isSynced;
    void IRAM_ATTR bitInterrupt(bool bit);
    void clearSync();
    TaskHandle_t flexRXTaskId;

   protected:
    uint16_t bs1;
    uint32_t Aword;
    uint16_t Bword;
    uint32_t notAword;
    bool IRAM_ATTR validate();
};

class flexfiw {
   public:
    uint8_t priority = 0;
    uint8_t cycle = 0;
    uint8_t frame = 0;
    uint8_t repeat = 0;
    uint8_t traffic[4];
    bool isValid = false;
    uint32_t output();
    void input(uint32_t word);
    flexfiw();
    flexfiw(uint32_t word);
};

class flexbiw {
   public:
    uint8_t priority = 0;     // 0 to 15
    uint8_t blocksize = 0;    // 0 to 3 (but really, 1 to 4)
    uint8_t vectorstart = 0;  // 0 to 63
    uint8_t carryon = 0;      // 0 to 3
    uint8_t collapse = 0;     // 0 to 7 (every 2^collapse frames);
    bool isValid = false;
    uint16_t localid = 0;
    uint8_t coveragezone = 0;
    struct time {
        uint8_t hours;
        uint8_t secs;
        uint8_t mins;
        uint8_t month;
        uint8_t day;
        uint16_t year;
    } time;
    enum biwtype_e {
        BIW,
        LOCAL_ID,
        DATE,
        TIME,
        COUNTRY,
        SYSTEM_INFO
    } biwtype;

    enum sysinfotype_e {
        MSG_ALL,
        MSG_HOME,
        MSG_ROAMING,
        MSG_SSID,
        MSG_TIME,
        SYS_TIME,
        CHANNEL_SETUP
    } sysinfo;
    flexbiw();
    flexbiw(uint32_t word);
    flexbiw(enum biwtype_e type);
    uint32_t output();
    void input(uint32_t in, uint8_t pos);
};

class flexaddress {
   public:
    uint8_t frame = 0;
    uint8_t phase = 0;
    uint32_t ric = 0;
    uint8_t addresswords = 0;

    enum shortaddrtype {
        SHORT_INVALID,
        SHORT_IDLE1,
        SHORT_LONG_1,
        SHORT,
        SHORT_LONG_3,
        SHORT_LONG_4,
        SHORT_RESERVED1,
        SHORT_INFO_SERVICE,
        SHORT_NETWORK,
        SHORT_TEMP,
        SHORT_OPERATOR,
        SHORT_RESERVED2,
        SHORT_LONG_2,
        SHORT_IDLE2
    };

    struct addrword {
        enum shortaddrtype type;
        uint32_t value;
    };

    struct addrword word[2];

    enum addrtype { ADDRT_INVALID,
                    ADDRT_SHORT,
                    ADDRT_1_2,
                    ADDRT_1_3,
                    ADDRT_1_4,
                    ADDRT_2_3 } addrtype;

    enum addruse { ADDRUSE_SHORT,
                   ADDRUSE_ILLEGAL,
                   ADDRUSE_RESERVED1,
                   ADDRUSE_INFORMATION_SVC_ADDR,
                   ADDRUSE_NETWORK_ADDR,
                   ADDRUSE_TEMPORARY,
                   ADDRUSE_OPERATOR_MSG,
                   ADDRUSE_RESERVED2,
                   ADDRUSE_UNUSED,
                   ADDRUSE_UNCOORDINATED,
                   ADDRUSE_BY_COUNTRY1,
                   ADDRUSE_GLOBAL1,
                   ADDRUSE_GLOBAL2,
                   ADDRUSE_GLOBAL3,
                   ADDRUSE_BY_COUNTRY2,
                   ADDRUSE_RESERVED3,
                   ADDRUSE_INFO_SVC_GLOBAL,
                   ADDRUSE_INFO_SVC_COUNTRY1,
                   ADDRUSE_INFO_SVC_COUNTRY2,
                   ADDRUSE_INFO_SVC_RESERVED } addruse;

    void decodeRIC(uint32_t address);
    void decodeRIC(uint32_t address, uint8_t offset);
    void decodeWord(uint32_t* wordp);
    void RICtoWord(uint32_t* wordp);

    static uint32_t temp(uint8_t tempaddr);
    static uint8_t getPhase(uint32_t address, char prefix);
    static uint8_t getFrame(uint32_t address, char prefix);
    static uint8_t getSubstract(char prefix);
    static uint8_t getNextFrame(uint32_t address, char prefix, uint8_t start, bool rxcollapse);
    flexaddress();
    flexaddress(uint32_t* wordp);

   protected:
    enum shortaddrtype getShortType(uint32_t address);
};

class flextempaddrmapping {
   public:
    std::vector<flexaddress*> addresslist;

    flextempaddrmapping(flexaddress addr_c, uint8_t frame_c, uint8_t tempaddr_c, uint8_t timeout_c);
    flextempaddrmapping();
    ~flextempaddrmapping();

    static flextempaddrmapping* getTempMapping(uint8_t frame, uint8_t tempaddr, bool allMappings);
    static void addTempMapping(uint8_t frame, uint8_t tempaddr, flexaddress* addr, uint8_t curframe);
    static void deleteTempMapping(flextempaddrmapping* mapping);
    static void removeStaleMappings();

   protected:
    uint8_t tempaddr;
    uint8_t frame;
    uint8_t timeout;
};

class flexvector {
   public:
    enum vectortype {
        VECTOR_INVALID,
        VECTOR_SECURE,
        VECTOR_SHORT_INSTRUCTION,
        VECTOR_SHORT_MESSAGE,
        VECTOR_NUMERIC,
        VECTOR_SPECIAL,
        VECTOR_ALPHA,
        VECTOR_HEX,
        VECTOR_NUMBEREDNUMERIC
    } type;

    // SHORT INSTRUCTION
    uint8_t tempAssignedFrame = 0;
    uint8_t tempAddress = 0;
    uint16_t systemEvent = 0;
    enum shorttype {
        VECTOR_SHORT_TEMP_ADDRESS,
        VECTOR_SYSTEM_EVENT
    } shortInstructionType;

    // DATA VECTOR
    uint8_t blockstart;
    uint8_t blocklength;
    uint8_t numchecksum;

    flexvector(uint32_t* wordp);
    static enum vectortype getVectorType(uint32_t* wordp);
    static enum vectortype getVectorType(uint32_t temp);
    static uint32_t getVector(uint8_t start, uint8_t blocks);
    static uint32_t getAlphaVector(uint8_t start, uint8_t blocks);
    static uint32_t getNumericVector(uint8_t start, uint8_t words, uint8_t checksum);
    static uint32_t getShortInstruction(uint8_t address, uint8_t frame);
    static uint32_t getShortInstruction(uint16_t event);

    //RX
    void decodeNumeric(uint32_t* wordp);
    void decodeShortInstruction(uint32_t* wordp);
    void decodeAlpha(uint32_t* wordp);
    void decodeShortMessage(uint32_t* wordp);
};

class flexphase {
   public:
    enum idlestep {
        IDLE_1600,
        IDLE_3200_2,
        IDLE_3200_4_PHASE1,
        IDLE_3200_4_PHASE2,
        IDLE_6400_4_PHASE1,
        IDLE_6400_4_PHASE2,
        IDLE_6400_4_PHASE3,
        IDLE_6400_4_PHASE4
    };
    struct flexblock {
        uint32_t word[8];
    } block[11];
    bool lowTraffic = false;
    bool errorMarker[88];
    flexbiw biwone;
    uint8_t frame = 0;

    flexphase(void);
    flexphase(idlestep step);
    ~flexphase(void);

    flexdatablock* addBlock(flexdatablock* blockp);
    uint32_t getWord(uint8_t index);
    uint32_t* getWordP(uint8_t index);
    void prepPhaseForTX(void);
    void decode(std::vector<flexmsg*>* messages);
    flexbiw* addBIW(flexbiw::biwtype_e type);
    uint8_t usedWords(void);
    uint8_t freeWords(void);

   protected:
    std::vector<flexbiw*> biwarr;
    std::vector<flexdatablock*> blockarr;
};

struct flexdatablock {
   public:
    uint8_t wordcount;
    uint32_t* block;
    std::vector<flexaddress*> addrarr;
    uint32_t vectorwordtwo;
    uint8_t numchecksum;
    flexvector::vectortype type;
    flexdatablock(uint8_t words);
    ~flexdatablock();
    static flexmsg* decode(flexphase* phase, uint8_t addrword, uint8_t curframe);
    uint8_t blockSize();
    uint8_t addrBlockSize();

   protected:
    static flexmsg* decodeAlpha(flexphase* phase, uint8_t addrword, uint8_t curframe, flexvector* vect);
    static flexmsg* decodeNumeric(flexphase* phase, uint8_t addrword, uint8_t curframe, flexvector* vect);
};

class flexmsg {
   public:
    flexvector::vectortype type;
    uint8_t signature = 0;
    bool maildrop = false;
    bool retrieval = false;
    uint8_t messageno = 0;  // doubles as temp address ID for short instruction vectors
    uint8_t fragmentid = 0x03;
    uint16_t wordcount = 0;

    // specifically RX
    uint32_t primaryAddress = 0;

    std::vector<flexaddress*> addrarr;

    void encode(flexvector::vectortype msgtype, char* datap);
    void encode(flexvector::vectortype msgtype, uint8_t tempaddress, uint8_t frame);
    flexdatablock* getBlock(uint8_t words);
    flexdatablock* getBlock();

    void addAddress(uint32_t ric);
    uint8_t addrBlockSize();

    ~flexmsg();

    //RX functions
    void timeoutTrigger();
    static void removeStaleMessages();
    static void addIncompleteMessage(flexmsg* temp);
    static flexmsg* getIncomplete(uint32_t ric, uint8_t messageno);
    void addAlphaWords(flexphase* phase, uint8_t fragmentno, uint8_t blockstart, uint8_t blocksize);
    void addNumericData(uint8_t* databuffer, uint8_t characters);
    void addAddress(uint8_t curframe, flexaddress* addr);
    char* getAlphaData();
    char* getNumericData();

   protected:
    char* alphadata = NULL;
    char* numericdata = NULL;
    uint8_t vectorsize;  // doubles as frame address for short instruction vectors
    uint16_t bytes = 0;
    uint16_t bytesleft = 0;

    char* readp = NULL;

    uint8_t timeout = 240;

    void encodeAlpha(char* datap);
    void encodeNumeric(char* datap);
    flexdatablock* getAlphaBlock(uint8_t words);
    flexdatablock* getNumericBlock();
    flexdatablock* getShortInstructionBlock();
    uint16_t countWords(char* start);
    void addTempMappings(uint8_t curframe, flexaddress* addr);  // RX
    uint8_t encodeChar(char c);
    char decodeChar(uint8_t);
};

class flexframe {
   public:
    RF4463* rf4463;
    uint32_t frequency;
    flexsynctype type;
    uint8_t multiplex;
    std::vector<flexphase*> phase;
    flexfiw fiww;
    bool shortPreamble;
    void addByte(uint8_t byte);
    std::vector<flexmsg*>* decode(void);
    void startTX(void);
    static uint8_t getMultiplex(flexsynctype ftype);

    flexframe(flexsynctype ftype, uint32_t fiwc);
    flexframe(flexsynctype ftype);
    flexframe();
    ~flexframe();

   protected:
    uint16_t bitcount = 0;
    uint32_t fiwword;
    uint32_t getWord();
    void queueTXBytes(const uint8_t* buffer, uint8_t len, flexsynctype txtype);
    void queueTXBytes(uint8_t* buffer, uint8_t len, flexsynctype txtype);
    void queueWord(uint32_t word, flexsynctype type);
    bool txBytesWait(uint8_t bytes);
    bool txTask(void);
    static void txTaskStart(void* _this);
};

#endif