/*
COPYRIGHT NOTICE

This work is based on the Factory_test examples, found here: https://github.com/HelTecAutomation/ASR650x-Arduino/tree/master/libraries/Basics/examples
so the copyright for this code belongs to Heltec Automation.

Any additional changes are free-to-use, but at your own risk.
*/

#include <Arduino.h>    // needed for platform.io
#include "mesh.pb.h"

// ToRadio Incoming;  // not used right now
// Radio settings for meshtastic channel "Default" @ "very long range (but slow)". Will not work with other settings or channels.
// #define RF_FREQUENCY                865200000   // Hz  -- see "CH0" in Meshradio.h
#define TX_OUTPUT_POWER             22          // dBm
#define LORA_BANDWIDTH              0           // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: 62.5kHz, 4: 41.67kHz, 5: 31.25kHz, 6: 20.83kHz, 7: 15.63kHz, 8: 10.42kHz, 9: 7.81kHz]
// FYI:
// const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500, LORA_BW_062,LORA_BW_041, LORA_BW_031, LORA_BW_020, LORA_BW_015, LORA_BW_010, LORA_BW_007 };


#define LORA_SPREADING_FACTOR       12          // [SF5..SF12]
#define LORA_CODINGRATE             4           // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH        32          // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT         0           // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false
#define LORA_CHAN_NUM               0           // Channel Number (meshtastic specific)
#define RX_TIMEOUT_VALUE            1000
#define BUFFER_SIZE                 0xFF        // max payload (see  \cores\asr650x\device\asr6501_lrwan\radio.c  --> MaxPayloadLength)

#define ID_BUFFER_SIZE              32

char mPacket[BUFFER_SIZE];

#define HW_VERSION_EU865 // define your region _before_ including MeshRadio.h !
#include "MeshRadio.h"
// initializing ChannelSettings with valid data... You can modify it to your settings here. Do not use ModemConfig. Change bw/sf/cr instead!
// "Bandwidth" is the index of the array Bandwidths (see above), not the actual bandwidth! Index is starting with 0.
ChannelSettings_psk_t mPSK;   // not used
ChannelSettings ChanSet={ TX_OUTPUT_POWER, ChannelSettings_ModemConfig_Bw125Cr48Sf4096, mPSK, "Default", 
                        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_CHAN_NUM };

uint32_t receivedID[ID_BUFFER_SIZE];
int receivedCount = -1;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void ConfigureRadio( ChannelSettings ChanSet );

typedef enum
{
    LOWPOWER,
    RX,
    TX
}States_t;

States_t state;
int16_t rxSize;
uint32_t x;             // re-used variable of type uint32_t, used in onRxDone()
bool found;             // used in onRxDone()


#define NOTSILENT         // delete or change to VERBOSE if serial output is needed

void setup() {
    #ifndef SILENT
    Serial.begin(115200);
    Serial.print("\nSetting up Radio..");
    #endif
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    Radio.Init( &RadioEvents );
    ConfigureRadio( ChanSet );
    #ifndef SILENT
    Serial.println(".done!\n");
    #endif
    state=RX;   // initial mode = receive
}

void loop()
{
	switch(state)
	{
		case TX:
            #ifndef SILENT
            Serial.print("Sending packet..");
            Serial.printf("(Size: %i)..", rxSize);
            #endif
            Radio.Send( (uint8_t *)mPacket, rxSize );
		    state=LOWPOWER;
		    break;
		case RX:
		    Radio.Rx( 0 );  // receive mode with no time-out
		    state=LOWPOWER; 
		    break;
		case LOWPOWER:
			lowPowerHandler(); // put SoC to sleep, wake-up at LoRa receive
		    break;
        default:
            break;
	}
    Radio.IrqProcess( );
}

void OnTxDone( void )
{
	#ifndef SILENT
    Serial.println(".done!");
    #endif
	state=RX; // switch to receive mode
}

void OnTxTimeout( void )
{
    #ifndef SILENT
    Serial.println(".failed! (TX Timeout)");
    #endif
    Radio.Sleep();
    state=RX; // switch to receive mode
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    ( size > BUFFER_SIZE ) ?  rxSize = BUFFER_SIZE : rxSize = size;
    memcpy( mPacket, payload, size );
    Radio.Sleep();
#ifndef SILENT
    Serial.printf("\nReceived packet! (Size %i bytes, RSSI %i, SNR %i)\n", size, rssi, snr);
    
    Serial.print("TO: ");
    for(int i=3; i>-1; i--){
        Serial.print(mPacket[i], HEX);
    }

    Serial.print("  FROM: ");
    for(int i=7; i>3; i--){
        Serial.print(mPacket[i], HEX);
    }

    Serial.print("  PacketID: ");
    for(int i=11; i>7; i--){
        Serial.print(mPacket[i], HEX);
    }

    Serial.print("  Flags: ");
    x = mPacket[12];
    Serial.print("WANT_ACK=");
    if (x > 8) {
        Serial.print("YES ");
        x -= 8;
    }
    else Serial.print("NO ");

    Serial.printf("HOP_LIMIT=%i", x);
    Serial.println();

    Serial.print("Payload: ");
    for(int i=13; i<size; i++){        
        Serial.print(mPacket[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
#endif 
    // uint32_t packetID 
    x = (uint32_t)mPacket[11]<<24 | (uint32_t)mPacket[10]<<16 | (uint32_t)mPacket[9]<<8 | (uint32_t)mPacket[8];
    found = false;
    for(int i = 0; i < ID_BUFFER_SIZE; i++){
        if (receivedID[i] == x) found = true;
    }
    if (!found){ 
        state=TX; // will repeat package
        (receivedCount < (ID_BUFFER_SIZE - 1) ) ?  receivedCount++ : receivedCount = 0; // if counter = max, overwrite first entry in list, reset counter
        receivedID[receivedCount] = x; // add ID to received list
     }
    else{
        #ifndef SILENT
        Serial.println("PacketID known, will not repeat again.");
        #endif
        state = RX; // wait for new package
    } 
}

void ConfigureRadio( ChannelSettings ChanSet )
{
    uint32_t freq = (CH0 + CH_SPACING * ChanSet.channel_num)*1E6;
   
    Serial.printf("\nSetting frequency to: %i ..",freq );

    Radio.SetChannel( freq );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, ChanSet.bandwidth, ChanSet.spread_factor, ChanSet.coding_rate,
                                   LORA_PREAMBLE_LENGTH, false, true, false, 0, false, 20000 );

    Radio.SetRxConfig( MODEM_LORA, ChanSet.bandwidth, ChanSet.spread_factor, ChanSet.coding_rate, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, false , 0, true, false, 0, false, true );

}
