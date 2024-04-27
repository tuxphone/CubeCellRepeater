#include <Arduino.h>
#include <RadioLib.h>

// #define SILENT           // turn off serial output

#define CC_MY_REGION        meshtastic_Config_LoRaConfig_RegionCode_EU_868     // see regions[] below
#define CC_MY_LORA_PRESET   meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST // LONG FAST is default preset
#define CC_LORA_USE_PRESET  true   // set true to use modem preset
#define CC_CHANNEL_NAME     "Primary Channel"

#define CC_MY_LORA_BW       125.0   // use these settings, if not using a modem preset
#define CC_MY_LORA_SF       10
#define CC_MY_LORA_CR       5
#define CC_MY_LORA_POWER    0       // 0 = max legal power for region
#define CC_MY_LORA_FREQ     0.0     // if you want to override frequency calculation, in MHz (e.g. 869.4)

#define CC_MAX_POWER        22      // TX power setting. Maximum for CubeCell is 22, enforced by RadioLib.

#ifdef CUBECELL
#include "cyPm.c"  // for reliable sleep we use MCU_deepSleep()
    extern  uint32_t systime;   // CubeCell global system time count, Millis
    SX1262 radio = new Module(RADIOLIB_BUILTIN_MODULE);
#endif //CUBECELL

uint8_t radiobuf[256];
uint32_t lastPacketID = 0;
int state = RADIOLIB_ERR_NONE;

void MCU_deepsleep(void);   
void clearInterrupts(void);
void startReceive(void);

// Flag and ISR for "Received packet" - events
volatile bool PacketReceived = false;

void ISR_setReceived(void) {
    PacketReceived = true;
}

// Flag and ISR for "Packet sent" - events
volatile bool PacketSent = false;

void ISR_setPacketSent(void) {
    PacketSent = true;  
}

#ifndef SILENT
    #define MSG(...) Serial.printf(__VA_ARGS__)
    #define MSGFLOAT(a,b) Serial.print(a); Serial.print(b)
#else
    #define MSG(...)
    #define MSGFLOAT(a,b)
#endif


/**************
 * Meshtastic * 
 **************/
#define PACKET_FLAGS_HOP_MASK 0x07
#define PACKET_FLAGS_WANT_ACK_MASK 0x08
#define PACKET_FLAGS_VIA_MQTT_MASK 0x10

typedef enum _meshtastic_Config_LoRaConfig_RegionCode {
    /* Region is not set */
    meshtastic_Config_LoRaConfig_RegionCode_UNSET = 0,
    /* United States */
    meshtastic_Config_LoRaConfig_RegionCode_US = 1,
    /* European Union 433mhz */
    meshtastic_Config_LoRaConfig_RegionCode_EU_433 = 2,
    /* European Union 868mhz */
    meshtastic_Config_LoRaConfig_RegionCode_EU_868 = 3,
    /* China */
    meshtastic_Config_LoRaConfig_RegionCode_CN = 4,
    /* Japan */
    meshtastic_Config_LoRaConfig_RegionCode_JP = 5,
    /* Australia / New Zealand */
    meshtastic_Config_LoRaConfig_RegionCode_ANZ = 6,
    /* Korea */
    meshtastic_Config_LoRaConfig_RegionCode_KR = 7,
    /* Taiwan */
    meshtastic_Config_LoRaConfig_RegionCode_TW = 8,
    /* Russia */
    meshtastic_Config_LoRaConfig_RegionCode_RU = 9,
    /* India */
    meshtastic_Config_LoRaConfig_RegionCode_IN = 10,
    /* New Zealand 865mhz */
    meshtastic_Config_LoRaConfig_RegionCode_NZ_865 = 11,
    /* Thailand */
    meshtastic_Config_LoRaConfig_RegionCode_TH = 12,
    /* WLAN Band */
    meshtastic_Config_LoRaConfig_RegionCode_LORA_24 = 13,
    /* Ukraine 433mhz */
    meshtastic_Config_LoRaConfig_RegionCode_UA_433 = 14,
    /* Ukraine 868mhz */
    meshtastic_Config_LoRaConfig_RegionCode_UA_868 = 15,
    /* Malaysia 433mhz */
    meshtastic_Config_LoRaConfig_RegionCode_MY_433 = 16,
    /* Malaysia 919mhz */
    meshtastic_Config_LoRaConfig_RegionCode_MY_919 = 17,
    /* Singapore 923mhz */
    meshtastic_Config_LoRaConfig_RegionCode_SG_923 = 18
} meshtastic_Config_LoRaConfig_RegionCode;

/* Standard predefined channel settings
 Note: these mappings must match ModemPreset Choice in the device code. */
typedef enum _meshtastic_Config_LoRaConfig_ModemPreset {
    /* Long Range - Fast */
    meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST = 0,
    /* Long Range - Slow */
    meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW = 1,
    /* Very Long Range - Slow */
    meshtastic_Config_LoRaConfig_ModemPreset_VERY_LONG_SLOW = 2,
    /* Medium Range - Slow */
    meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW = 3,
    /* Medium Range - Fast */
    meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST = 4,
    /* Short Range - Slow */
    meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW = 5,
    /* Short Range - Fast */
    meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST = 6,
    /* Long Range - Moderately Fast */
    meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE = 7
} meshtastic_Config_LoRaConfig_ModemPreset;


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
    const RegionInfo *r = regions;
    for (; r->code != meshtastic_Config_LoRaConfig_RegionCode_UNSET && r->code != CC_MY_REGION; r++)
        ;
    myRegion = r;
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

void applyModemConfig()
{
    float bw=0;
    uint8_t sf = 0;
    uint8_t cr = 0;
    int8_t power = 0; // 0 = max legal power for region
    float freq = 0;

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

    int channel_num = hash(channelName) % numChannels;

    freq = myRegion->freqStart + (bw / 2000) + (channel_num * (bw / 1000));

    // override if we have a verbatim frequency
    if (CC_MY_LORA_FREQ > 0.0) {
        freq = CC_MY_LORA_FREQ;
        channel_num = -1;
    }

    MSG("\nRegion %s (Power Limit is %idb) ",myRegion->name, myRegion->powerLimit);
    MSG(" freq %d bw %i sf %i cr %i power %i  ... ", lround(freq*1E6), lround(bw*1000), sf, cr, power);
    
    // Syncword is 0x2b, see RadioLibInterface.h
    // preamble length is 16, see RadioInterface.h

    int state = radio.begin(freq, bw, sf, cr, 0x2b, power, 16); 

    if (state == RADIOLIB_ERR_NONE) {
        MSG("success!\n");
    } else {
        MSG("\n[ERROR] [SX1262} Init failed, code: %i\n\n ** Full Stop **", state);
        while (true);
    }
    
}