#include "mesh.pb.h"
// CONFIGURATION:
#define REGION   RegionCode_EU865   // define your region here. For US, RegionCode_US, CN RegionCode_Cn etc.
char    MESHTASTIC_NAME[12] = {"Default"}; // Channel Name, but without "-Xy" suffix , e.g. use "Test" instead of "Test-A"
#define MESHTASTIC_SPEED  2     // 0 = short range, 1 = medium range, 2 = long range, 3 = very long range
#define TX_MAX_POWER     14     // max output power in dB, keep in mind the maximums set by law and the hardware
// :CONFIGURATION

#define RGB_GREEN                   0x000300    // receive mode  --- not longer used 
#define RGB_RED                     0x030000    // send mode

#define LORA_PREAMBLE_LENGTH        32          // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT         0           // Symbols
#define RX_TIMEOUT_VALUE            1000
#define MAX_PAYLOAD_LENGTH          0xFF        // max payload (see  \cores\asr650x\device\asr6501_lrwan\radio.c  --> MaxPayloadLength)

/* possible RegionCodes, from mesh.pb.h:
typedef enum _RegionCode {
    RegionCode_Unset = 0,
    RegionCode_US = 1,
    RegionCode_EU433 = 2,
    RegionCode_EU865 = 3,
    RegionCode_CN = 4,
    RegionCode_JP = 5,
    RegionCode_ANZ = 6,
    RegionCode_KR = 7,
    RegionCode_TW = 8
} RegionCode;
*/

// the PSK is not used for encryption/decryption, you can leave it as it is
#define MESHTASTIC_PSK      { 0x10, 0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59, 0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0xbf }
#define PSK_NOENCRYPTION    { 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

// sleep times @ speed   0    1   2     3
uint32_t sleepTime[] = { 77, 20, 1512, 2499 };

typedef struct {
    uint32_t to, from, id; 
    uint8_t flags;      // The bottom three bits of flags are used to store hop_limit, bit 4 is the WANT_ACK flag
} PacketHeader;

#define MSG(...)    Serial.printf(__VA_ARGS__)
//#define DUTY(symbTime) ( (uint32_t)( symbTime * LORA_PREAMBLE_LENGTH / 2 ) ) 

void onTxDone( void );
void onCadDone( bool ChannelActive );
void onRxTimeout( void );
void onTxTimeout( void );
void onRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void onCheckRadio(void);
void ConfigureRadio( ChannelSettings ChanSet );
unsigned long hash(char *str);


// from Meshtastic project: MeshRadio.h , RadioInterface.cpp
#define RDEF(name, freq, spacing, num_ch, power_limit)                                                                           \
    {                                                                                                                            \
        RegionCode_##name, num_ch, power_limit, freq, spacing, #name                                                             \
    }
struct RegionInfo {
    RegionCode code;
    uint8_t numChannels;
    uint8_t powerLimit; // Or zero for not set
    float freq;
    float spacing;
    const char *name; // EU433 etc
};

const RegionInfo regions[] = {
    RDEF(Unset, 903.08f, 2.16f, 13, 0), // I put it FIRST, so i can use regions[] with RegionCode as index (Unset == 0)
    RDEF(US, 903.08f, 2.16f, 13, 0), 
    RDEF(EU433, 433.175f, 0.2f, 8, 0), 
    RDEF(EU865, 865.2f, 0.3f, 10, 0),
    RDEF(CN, 470.0f, 2.0f, 20, 0),
    RDEF(JP, 920.0f, 0.5f, 10, 13),    // See https://github.com/meshtastic/Meshtastic-device/issues/346 power level 13
    RDEF(ANZ, 916.0f, 0.5f, 20, 0),    // AU/NZ channel settings 915-928MHz
    RDEF(KR, 921.9f, 0.2f, 8, 0),      // KR channel settings (KR920-923) Start from TTN download channel
                                       // freq. (921.9f is for download, others are for uplink)
    RDEF(TW, 923.0f, 0.2f, 10, 0)     // TW channel settings (AS2 bandplan 923-925MHz)
};
