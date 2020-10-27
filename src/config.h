// CONFIGURATION:
#define REGION          RegionCode_EU865   // define your region here. For US, RegionCode_US, CN RegionCode_Cn etc.
#define TX_MAX_POWER    14     // max output power in dB, keep in mind the maximums set by law and the hardware
char MeshtasticLink[] = "https://www.meshtastic.org/c/#GAMiENTxuzogKQdZ8Lz_q89Oab8qB0RlZmF1bHQ=" ;
// :CONFIGURATION

/*  RegionCodes:

    RegionCode_Unset
    RegionCode_US
    RegionCode_EU433
    RegionCode_EU865
    RegionCode_CN
    RegionCode_JP
    RegionCode_ANZ
    RegionCode_KR
    RegionCode_TW 
*/

#define RGB_GREEN                   0x000300    // receive mode  --- not longer used 
#define RGB_RED                     0x030000    // send mode

#define LORA_PREAMBLE_LENGTH        32          // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT         0           // Symbols
#define RX_TIMEOUT_VALUE            1000
#define MAX_PAYLOAD_LENGTH          0xFF        // max payload (see  \cores\asr650x\device\asr6501_lrwan\radio.c  --> MaxPayloadLength)

#include "mesh.pb.h"
#include "mesh-pb-constants.h"

typedef struct {
    uint32_t to, from, id; 
    uint8_t flags;      // The bottom three bits of flags are used to store hop_limit, bit 4 is the WANT_ACK flag
} PacketHeader;

#define MSG(...)    Serial.printf(__VA_ARGS__) 
#define LINE( count, c) { for (uint8_t i=0; i<count; i++ ) { Serial.print(c); } Serial.println();  } 

void onTxDone( void );
void onCadDone( bool ChannelActive );
void onRxTimeout( void );
void onTxTimeout( void );
void onRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void onCheckRadio( void );
void ConfigureRadio( ChannelSettings ChanSet );
void MCU_deepsleep( void );
unsigned long hash( char *str );

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

// Bandwidths array is specific to the Radio.c of the CubeCell boards
const uint32_t TheBandwidths[] = { 125E3, 250E3, 500E3, 62500, 41670, 31250, 20830, 15630, 10420, 7810 };


