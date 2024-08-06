#pragma Once
#include <Arduino.h>

//#define SILENT              // turn off serial output
#define CC_MY_NODE_NUM      0xC00BCE11
#define CC_MY_REGION        meshtastic_Config_LoRaConfig_RegionCode_EU_868     // see regions[] below
#define CC_MY_LORA_PRESET   meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST // LONG FAST is default preset
#define CC_LORA_USE_PRESET  true   // set true to use modem preset
#define CC_CHANNEL_NAME     "LongFast"
#define CC_CHANNEL_SLOT     0  // "0" for calculated slot. Any other number = chosen slot. Count starts with 1.

#define CC_MY_LORA_BW       125.0   // use these settings, if not using a modem preset
#define CC_MY_LORA_SF       9
#define CC_MY_LORA_CR       5
#define CC_MY_LORA_POWER    20      // 0 = max legal power for region
#define CC_MY_LORA_FREQ     0.0     // if you want to override frequency calculation: Freq in MHz (e.g. 869.4)

#define CC_MAX_POWER        22      // TX power setting. Absolute Max for CubeCell is 22, enforced by RadioLib.

#define CC_MONITOR_ONLY     false   // set true to suppress transmitting of packets (just monitor the traffic)

#define CC_SIGNAL_NEOPIXEL  // signal received packets with a green blink, signal transmits with red blink 
//#define CC_SIGNAL_GPIO13  // signal received packets with the green LED on HTCC-AB02A
                            // http://community.heltec.cn/t/htcc-ab02a-has-a-secret-green-led/3092/7

#define MAX_ID_LIST         64 // number of stored packet IDs to prevent unnecesary repeating
#define MAX_NODE_LIST       20 // number of stored known nodes
#define MAX_TX_QUEUE        8 // max number of packets which can be waiting for transmission

/// 16 bytes of random PSK for our _public_ default channel that all devices power up on (AES128)
/// Meshtastic default key (AQ==):
static const uint8_t mypsk[] = {0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
                                0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01};
// No Crypto = all zero

#ifndef SILENT
    #define MSG(...) Serial.printf(__VA_ARGS__)
    #define MSGFLOAT(a,b) Serial.print(a); Serial.print(b)
#else
    #define MSG(...)
    #define MSGFLOAT(a,b)
#endif

#define PACKET_FLAGS_HOP_LIMIT_MASK 0x07
#define PACKET_FLAGS_WANT_ACK_MASK 0x08
#define PACKET_FLAGS_VIA_MQTT_MASK 0x10
#define PACKET_FLAGS_HOP_START_MASK 0xE0
#define PACKET_FLAGS_HOP_START_SHIFT 5

#include <RadioLib.h>

/**************/
#ifdef CUBECELL

// Heltec borked the Arduino.h
#ifdef __cplusplus
#undef min
#undef max
#undef abs
#include <algorithm>
  using std::abs;
  using std::max;
  using std::min;
#endif /* __cplusplus */

#include "cyPm.c"  // for reliable sleep we use MCU_deepSleep()
extern  uint32_t systime;   // CubeCell global system time count, Millis
SX1262 radio = new Module(RADIOLIB_BUILTIN_MODULE);

#ifdef CC_SIGNAL_NEOPIXEL
#include "CubeCell_NeoPixel.h"
CubeCell_NeoPixel pixels(1, RGB, NEO_GRB + NEO_KHZ800);
#endif

#endif // CUBECELL
/****************/

//#include <assert.h>
#include <pb.h>
#include <MeshTypes.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <CryptoEngine.h>
#include <time.h>
/*
extern "C"
{ 
#include <mesh/compression/unishox2.h>
}
*/

#define MAX_RHPACKETLEN 256

// struct to store the raw packet data (buf, size) and the time of receiving
typedef struct {
    size_t   size;
    uint8_t  buf[MAX_RHPACKETLEN];
    uint32_t packetTime;
} Packet_t;

typedef struct {
    uint32_t to, from;
    uint32_t id;
    uint8_t flags;
    uint8_t channel;
    uint8_t next_hop;
    uint8_t relay_node;
} PacketHeader; // see Meshtastic RadioInterface.h

class PacketQueueClass {
private:
    Packet_t Queue[MAX_TX_QUEUE];
public:
    void clear(void);
    uint8_t Count = 0;
    void add(Packet_t* p);
    bool pop(void);
} txQueue;

class idStoreClass {
private:
    uint32_t storage[MAX_ID_LIST];
public:
    void clear(void);
    // add() returns false if the message id is already known
    bool add(uint32_t id);
} msgID;

class NodeStoreClass {
private:
    meshtastic_NodeInfo nodeDB[MAX_NODE_LIST];
    void add(meshtastic_NodeInfo* Node);
public:
    void clear(void);
    void update(meshtastic_NodeInfo* Node);
    // get the short name of the node. "" if unknown.
    char* get(uint32_t num);
} NodeDB;

CryptoKey psk;
meshtastic_MeshPacket mp;
meshtastic_NodeInfo theNode;
bool repeatPacket = false;
int err = RADIOLIB_ERR_NONE;
bool PacketReceived = false;
bool PacketSent = false;
bool isReceiving = false;

/* Packet we are currently wanting to send 
 * .packetTime is the time we started trying to send it.
 * packet will be dropped after a minute.
 */ 
Packet_t PacketToSend;
Packet_t* p = NULL;

// global dio1 flag - will be set to true, when dio1 is active
volatile bool dio1 = false;

// interrupt service routine for setting the dio1 flag
void ISR_dio1Action(void) {
     dio1 = true;
}

void MCU_deepsleep(void);
void startReceive(void);
bool perhapsSend(Packet_t* p);
bool perhapsDecode(Packet_t* p);
void printPacket(void);
void printVariants(void);

void signalizeRX_ON(void);
void signalizeTX_ON(void);
void signalizeLED_OFF(void);
void init_signalize(void);

/**************
 * Meshtastic *     https://github.com/meshtastic/firmware
 **************/
    float   bw = 0;
    uint8_t sf = 0;
    uint8_t cr = 0;
    int8_t power = CC_MY_LORA_POWER; // 0 = max legal power for region
    float   freq = 0;
    float    snr = 5.0;
    uint32_t activeReceiveStart = 0;

/** Slottime is the minimum time to wait, consisting of:
      - CAD duration (maximum of SX126x and SX127x);
      - roundtrip air propagation time (assuming max. 30km between nodes);
      - Tx/Rx turnaround time (maximum of SX126x and SX127x);
      - MAC processing time (measured on T-beam) */
    uint32_t slotTimeMsec; //= 8.5 * pow(2, sf) / bw + 0.2 + 0.4 + 7;  --> calculated at applyModemConfig()
    uint16_t preambleLength = 16;      // 8 is default, but we use longer to increase the amount of sleep time when receiving
    uint32_t preambleTimeMsec = 165;   // calculated on startup, this is the default for LongFast
    uint32_t maxPacketTimeMsec = 3246; // calculated on startup, this is the default for LongFast
    const uint32_t PROCESSING_TIME_MSEC =
        4500;                // time to construct, process and construct a packet again (empirically determined)
    const uint8_t CWmin = 2; // minimum CWsize
    const uint8_t CWmax = 8; // maximum CWsize

#define PACKET_FLAGS_HOP_MASK 0x07
#define PACKET_FLAGS_WANT_ACK_MASK 0x08
#define PACKET_FLAGS_VIA_MQTT_MASK 0x10

/// helper function for encoding a record as a protobuf, any failures to encode are fatal and we will panic
/// returns the encoded packet size
size_t pb_encode_to_bytes(uint8_t *destbuf, size_t destbufsize, const pb_msgdesc_t *fields, const void *src_struct)
{
    pb_ostream_t stream = pb_ostream_from_buffer(destbuf, destbufsize);
    if (!pb_encode(&stream, fields, src_struct)) {
        MSG("[ERROR]Panic: can't encode protobuf reason='%s'\n", PB_GET_ERROR(&stream));
        //assert(0); // If this assert fails it probably means you made a field too large for the max limits specified in mesh.options
    } else {
        return stream.bytes_written;
    }
}

/// helper function for decoding a record as a protobuf, we will return false if the decoding failed
bool pb_decode_from_bytes(const uint8_t *srcbuf, size_t srcbufsize, const pb_msgdesc_t *fields, void *dest_struct)
{
    pb_istream_t stream = pb_istream_from_buffer(srcbuf, srcbufsize);
    if (!pb_decode(&stream, fields, dest_struct)) {
        MSG("[ERROR]Can't decode protobuf reason='%s', pb_msgdesc %p\n", PB_GET_ERROR(&stream), fields);
        return false;
    } else {
        return true;
    }
}

#define RDEF(name, freq_start, freq_end, duty_cycle, spacing, power_limit, audio_permitted, frequency_switching, wide_lora)      \
    {                                                                                                                            \
        meshtastic_Config_LoRaConfig_RegionCode_##name, freq_start, freq_end, duty_cycle, spacing, power_limit, audio_permitted, \
            frequency_switching, wide_lora, #name                                                                                \
    }

struct RegionInfo {
    meshtastic_Config_LoRaConfig_RegionCode code;
    float freqStart;
    float freqEnd;
    float dutyCycle;
    float spacing;
    uint8_t powerLimit; // Or zero for not set
    bool audioPermitted;
    bool freqSwitching;
    bool wideLora;
    const char *name; // EU433 etc
};

const RegionInfo *myRegion;

const RegionInfo regions[] = {
    /*
        https://link.springer.com/content/pdf/bbm%3A978-1-4842-4357-2%2F1.pdf
        https://www.thethingsnetwork.org/docs/lorawan/regional-parameters/
    */
    RDEF(US, 902.0f, 928.0f, 100, 0, 30, true, false, false),

    /*
        https://lora-alliance.org/wp-content/uploads/2020/11/lorawan_regional_parameters_v1.0.3reva_0.pdf
     */
    RDEF(EU_433, 433.0f, 434.0f, 10, 0, 12, true, false, false),

    /*
        https://www.thethingsnetwork.org/docs/lorawan/duty-cycle/
        https://www.thethingsnetwork.org/docs/lorawan/regional-parameters/
        https://www.legislation.gov.uk/uksi/1999/930/schedule/6/part/III/made/data.xht?view=snippet&wrap=true

        audio_permitted = false per regulation

        Special Note:
        The link above describes LoRaWAN's band plan, stating a power limit of 16 dBm. This is their own suggested specification,
        we do not need to follow it. The European Union regulations clearly state that the power limit for this frequency range is
       500 mW, or 27 dBm. It also states that we can use interference avoidance and spectrum access techniques to avoid a duty
       cycle. (Please refer to section 4.21 in the following document)
        https://ec.europa.eu/growth/tools-databases/tris/index.cfm/ro/search/?trisaction=search.detail&year=2021&num=528&dLang=EN
     */
    RDEF(EU_868, 869.4f, 869.65f, 10, 0, 27, false, false, false),

    /*
        https://lora-alliance.org/wp-content/uploads/2020/11/lorawan_regional_parameters_v1.0.3reva_0.pdf
     */
    RDEF(CN, 470.0f, 510.0f, 100, 0, 19, true, false, false),

    /*
        https://lora-alliance.org/wp-content/uploads/2020/11/lorawan_regional_parameters_v1.0.3reva_0.pdf
     */
    RDEF(JP, 920.8f, 927.8f, 100, 0, 16, true, false, false),

    /*
        https://www.iot.org.au/wp/wp-content/uploads/2016/12/IoTSpectrumFactSheet.pdf
        https://iotalliance.org.nz/wp-content/uploads/sites/4/2019/05/IoT-Spectrum-in-NZ-Briefing-Paper.pdf
     */
    RDEF(ANZ, 915.0f, 928.0f, 100, 0, 30, true, false, false),

    /*
        https://digital.gov.ru/uploaded/files/prilozhenie-12-k-reshenyu-gkrch-18-46-03-1.pdf

        Note:
            - We do LBT, so 100% is allowed.
     */
    RDEF(RU, 868.7f, 869.2f, 100, 0, 20, true, false, false),

    /*
        ???
     */
    RDEF(KR, 920.0f, 923.0f, 100, 0, 0, true, false, false),

    /*
        ???
     */
    RDEF(TW, 920.0f, 925.0f, 100, 0, 0, true, false, false),

    /*
        https://lora-alliance.org/wp-content/uploads/2020/11/lorawan_regional_parameters_v1.0.3reva_0.pdf
     */
    RDEF(IN, 865.0f, 867.0f, 100, 0, 30, true, false, false),

    /*
         https://rrf.rsm.govt.nz/smart-web/smart/page/-smart/domain/licence/LicenceSummary.wdk?id=219752
         https://iotalliance.org.nz/wp-content/uploads/sites/4/2019/05/IoT-Spectrum-in-NZ-Briefing-Paper.pdf
      */
    RDEF(NZ_865, 864.0f, 868.0f, 100, 0, 36, true, false, false),

    /*
       https://lora-alliance.org/wp-content/uploads/2020/11/lorawan_regional_parameters_v1.0.3reva_0.pdf
    */
    RDEF(TH, 920.0f, 925.0f, 100, 0, 16, true, false, false),

    /*
        433,05-434,7 Mhz 10 mW
        https://nkrzi.gov.ua/images/upload/256/5810/PDF_UUZ_19_01_2016.pdf
    */
    RDEF(UA_433, 433.0f, 434.7f, 10, 0, 10, true, false, false),

    /*
        868,0-868,6 Mhz 25 mW
        https://nkrzi.gov.ua/images/upload/256/5810/PDF_UUZ_19_01_2016.pdf
    */
    RDEF(UA_868, 868.0f, 868.6f, 1, 0, 14, true, false, false),

    /*
        Malaysia
        433 - 435 MHz at 100mW, no restrictions.
        https://www.mcmc.gov.my/skmmgovmy/media/General/pdf/Short-Range-Devices-Specification.pdf
    */
    RDEF(MY_433, 433.0f, 435.0f, 100, 0, 20, true, false, false),

    /*
        Malaysia
        919 - 923 Mhz at 500mW, no restrictions.
        923 - 924 MHz at 500mW with 1% duty cycle OR frequency hopping.
        Frequency hopping is used for 919 - 923 MHz.
        https://www.mcmc.gov.my/skmmgovmy/media/General/pdf/Short-Range-Devices-Specification.pdf
    */
    RDEF(MY_919, 919.0f, 924.0f, 100, 0, 27, true, true, false),

    /*
       2.4 GHZ WLAN Band equivalent. Only for SX128x chips.
    */
    RDEF(LORA_24, 2400.0f, 2483.5f, 100, 0, 10, true, false, true),

    /*
        This needs to be last. Same as US.
    */
    RDEF(UNSET, 902.0f, 928.0f, 100, 0, 30, true, false, false)

};

void initRegion()
{
    MSG("[INF]Init region List ...");
    const RegionInfo *r = regions;
    for (; r->code != meshtastic_Config_LoRaConfig_RegionCode_UNSET && r->code != CC_MY_REGION; r++) ;
    myRegion = r;
    MSG(" done!\n");
}

/** hash a string into an integer
 *
 * djb2 by Dan Bernstein.
 * http://www.cse.yorku.ca/~oz/hash.html
 */
uint32_t hash(const char *str)
{
    uint32_t hash = 5381;
    int c;

    while ((c = *str++) != 0)
        hash = ((hash << 5) + hash) + (unsigned char)c; 

    return hash;
}

/** A channel number (index into the channel table)
 */
typedef uint8_t ChannelIndex;

/** A low quality hash of the channel PSK and the channel name.  created by generateHash(chIndex)
 * Used as a hint to limit which PSKs are considered for packet decoding.
 */
typedef uint8_t ChannelHash;

uint8_t xorHash(const uint8_t *p, size_t len)
{
    uint8_t code = 0;
    for (size_t i = 0; i < len; i++)
        code ^= p[i];
    return code;
}

/** Given a channel number, return the (0 to 255) hash for that channel.
 * The hash is just an xor of the channel name followed by the channel PSK being used for encryption
 * If no suitable channel could be found, return -1
 */

int16_t generateHash(const char *name)
{
   /* auto k = getKey(channelNum);
    if (k.length < 0)
        return -1; // invalid
    else {*/
        //const char *name = getName(channelNum);
        uint8_t h = xorHash((const uint8_t *)name, strlen(name));
        h ^= xorHash(psk.bytes, psk.length);
        return h;
    //}
}

uint32_t getPacketTime(uint32_t pl)
{
    float bandwidthHz = bw * 1000.0f;
    bool  headDisable = false; // we currently always use the header
    float tSym = (1 << sf) / bandwidthHz;

    bool lowDataOptEn = tSym > 16e-3 ? true : false; // Needed if symbol time is >16ms

    float tPreamble = (preambleLength + 4.25f) * tSym;
    float numPayloadSym =
        8 + max(ceilf(((8.0f * pl - 4 * sf + 28 + 16 - 20 * headDisable) / (4 * (sf - 2 * lowDataOptEn))) * cr), 0.0f);
    float tPayload = numPayloadSym * tSym;
    float tPacket = tPreamble + tPayload;
  
    return (uint32_t)(tPacket * 1000);
}

void applyModemConfig()
{
    if (CC_LORA_USE_PRESET) {

        switch (CC_MY_LORA_PRESET) {
        case meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST:
            bw = (myRegion->wideLora) ? 812.5 : 250;
            cr = 5;
            sf = 7;
            break;
        case meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW:
            bw = (myRegion->wideLora) ? 812.5 : 250;
            cr = 5;
            sf = 8;
            break;
        case meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST:
            bw = (myRegion->wideLora) ? 812.5 : 250;
            cr = 5;
            sf = 9;
            break;
        case meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW:
            bw = (myRegion->wideLora) ? 812.5 : 250;
            cr = 5;
            sf = 10;
            break;
        default: // Config_LoRaConfig_ModemPreset_LONG_FAST is default.
            bw = (myRegion->wideLora) ? 812.5 : 250;
            cr = 5;
            sf = 11;
            break;
        case meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE:
            bw = (myRegion->wideLora) ? 406.25 : 125;
            cr = 8;
            sf = 11;
            break;
        case meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW:
            bw = (myRegion->wideLora) ? 406.25 : 125;
            cr = 8;
            sf = 12;
            break;
        case meshtastic_Config_LoRaConfig_ModemPreset_VERY_LONG_SLOW:
            bw = (myRegion->wideLora) ? 203.125 : 62.5;
            cr = 8;
            sf = 12;
            break;
        }
    } else {
        // Custom Settings:
        sf = CC_MY_LORA_SF;
        cr = CC_MY_LORA_CR;
        bw = CC_MY_LORA_BW;
        power = CC_MY_LORA_POWER;
        // sanity check:
        if (bw == 31) // This parameter is not an integer
            bw = 31.25;
        if (bw == 62) // Fix for 62.5Khz bandwidth
            bw = 62.5;
        if (bw == 200)
            bw = 203.125;
        if (bw == 400)
            bw = 406.25;
        if (bw == 800)
            bw = 812.5;
        if (bw == 1600)
            bw = 1625.0;
    }

    if (power == 0) 
        power = (int8_t) myRegion->powerLimit;

    if (power == 0)
        power = 17; // Default to this power level if we don't have a valid regional power limit (powerLimit of myRegion defaults
                    // to 0, currently no region has an actual power limit of 0 [dBm] so we can assume regions which have this
                    // variable set to 0 don't have a valid power limit)
    
    power = (power > CC_MAX_POWER)? CC_MAX_POWER : power;

    // Calculate the number of channels
    uint32_t numChannels = floor((myRegion->freqEnd - myRegion->freqStart) / (myRegion->spacing + (bw / 1000)));

    const char *channelName = CC_CHANNEL_NAME; 
    int channel_num;

    if (CC_CHANNEL_SLOT == 0) {
        channel_num = hash(channelName) % numChannels;
    } else {
        channel_num = (CC_CHANNEL_SLOT-1) % numChannels;
    }

    MSG("[INF]Channel name is \"%s\"\n\r", channelName);
    MSG("[INF]Channel slot is %i (%i available frequency slots).\n\r", channel_num + 1, numChannels);
    freq = myRegion->freqStart + (bw / 2000) + (channel_num * (bw / 1000));

    // override if we have a verbatim frequency
    if (CC_MY_LORA_FREQ > 0.0) {
        freq = CC_MY_LORA_FREQ;
        channel_num = -1;
    }
    
    MSG("[INF]Using Region %s : freq=%d bw=%i sf=%i cr=%i power=%i ... ",myRegion->name, lround(freq*1E6), lround(bw*1000), sf, cr, power);
    
    // Syncword is 0x2b, see RadioLibInterface.h
    // preamble length is 16, see RadioInterface.h
    
    err = radio.begin(freq, bw, sf, cr, 0x2b, power, 16); 

    if (err == RADIOLIB_ERR_NONE) {
        MSG("success!\n");
    } else {
        MSG("\n[ERROR] [SX1262} Init failed, code: %i\n\n\r ** Full Stop **", err);
        while (true);
    }

    // used to calculate wait time before repeating a packet
    slotTimeMsec = 8.5 * pow(2, sf) / bw + 0.2 + 0.4 + 7;
    preambleTimeMsec = getPacketTime((uint32_t)0);
    maxPacketTimeMsec = getPacketTime(meshtastic_Constants_DATA_PAYLOAD_LEN + sizeof(PacketHeader));
    MSG("[INF]SlotTime=%ims PreambleTime=%ims maxPacketTime=%ims\n\r", slotTimeMsec, preambleTimeMsec, maxPacketTimeMsec);
}

/** The delay to use when we want to flood a message */
uint32_t getTxDelayMsecWeighted(float snr) // RadioInterface.cpp
{
    // The minimum value for a LoRa SNR
    const uint32_t SNR_MIN = -20;

    // The maximum value for a LoRa SNR
    const uint32_t SNR_MAX = 15;

    //  high SNR = large CW size (Long Delay)
    //  low SNR = small CW size (Short Delay)
    uint32_t delay = 0;
    uint8_t CWsize = map(snr, SNR_MIN, SNR_MAX, CWmin, CWmax);

    // our role is meshtastic_Config_DeviceConfig_Role_REPEATER
    delay = random(0, 2 * CWsize) * slotTimeMsec;
    return delay;
}

bool isActivelyReceiving()
{
    // The IRQ status will be cleared when we start our read operation. Check if we've started a header, but haven't yet
    // received and handled the interrupt for reading the packet/handling errors.

    uint16_t irq = radio.getIrqStatus();
    bool detected = (irq & (RADIOLIB_SX126X_IRQ_HEADER_VALID | RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED));
    // Handle false detections
    if (detected) {
        uint32_t now = millis();
        if (!activeReceiveStart) {
            activeReceiveStart = now;
        } else if ((now - activeReceiveStart > 2 * preambleTimeMsec) && !(irq & RADIOLIB_SX126X_IRQ_HEADER_VALID)) {
            // The HEADER_VALID flag should be set by now if it was really a packet, so ignore PREAMBLE_DETECTED flag
            activeReceiveStart = 0;
            MSG("Ignore false preamble detection.\n\r");
            return false;
        } else if (now - activeReceiveStart > maxPacketTimeMsec) {
            // We should have gotten an RX_DONE IRQ by now if it was really a packet, so ignore HEADER_VALID flag
            activeReceiveStart = 0;
            MSG("Ignore false header detection.\n\r");
            return false;
        }
    }
    return detected;
}
