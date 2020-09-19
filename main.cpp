/*
COPYRIGHT NOTICE

This work is based on the Factory_test examples, found here: https://github.com/HelTecAutomation/ASR650x-Arduino/tree/master/libraries/Basics/examples
so the copyright for this code belongs to Heltec Automation.

Any additional changes are free-to-use, but at your own risk.
*/

// CONFIGURATION of Meshtastic channel name and speed: change values in MeshRadio.h !

#include <Arduino.h>    // needed for platform.io
#include "mesh.pb.h"
#include "MeshRadio.h"

#define LORA_PREAMBLE_LENGTH        32          // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT         0           // Symbols
#define RX_TIMEOUT_VALUE            1000
#define BUFFER_SIZE                 0xFF        // max payload (see  \cores\asr650x\device\asr6501_lrwan\radio.c  --> MaxPayloadLength)
#define ID_BUFFER_SIZE              32

char mPacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void ConfigureRadio( ChannelSettings ChanSet );
unsigned long hash(char *str);

ChannelSettings ChanSet;

uint32_t receivedID[ID_BUFFER_SIZE];
int receivedCount = -1;

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


#define VERBOSE         // delete or change to VERBOSE if serial output is needed

void setup() {
    #ifndef SILENT
    Serial.begin(115200);
    Serial.println("\nSetting up Radio:");
    #endif
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    Radio.Init( &RadioEvents );
    
    ChanSet.name[12] = MESHTASTIC_NAME[12];
    ChanSet.channel_num = hash( MESHTASTIC_NAME ) % NUM_CHANNELS;  // see MeshRadio.h
    ChanSet.tx_power    = TX_OUTPUT_POWER;
    /* FYI: 
    "bandwidth":
    [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: 62.5kHz, 4: 41.67kHz, 5: 31.25kHz, 6: 20.83kHz, 7: 15.63kHz, 8: 10.42kHz, 9: 7.81kHz]
    "speed":
        0: ChannelSettings_ModemConfig_Bw125Cr45Sf128 aka short range
        1: ChannelSettings_ModemConfig_Bw500Cr45Sf128 aka medium range
        2: ChannelSettings_ModemConfig_Bw31_25Cr48Sf512 aka long range
        3: ChannelSettings_ModemConfig_Bw125Cr48Sf4096  aka very long range
    "coding rate":
        [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    */
    switch ( MESHTASTIC_SPEED ){
        case 0: {  // short range 
            ChanSet.bandwidth = 0;      // 125 kHz
            ChanSet.coding_rate = 1;    // = 4/5
            ChanSet.spread_factor = 7;
            break;
        }
        case 1: {  // medium range 
            ChanSet.bandwidth = 2;      // 500 kHz
            ChanSet.coding_rate = 1;    // = 4/5
            ChanSet.spread_factor = 7;
            break;
        }
        case 2: {  // long range 
            ChanSet.bandwidth = 5;      // 31.25 kHz
            ChanSet.coding_rate = 4;    // = 4/5
            ChanSet.spread_factor = 9;
            break;
        }
        case 3: {  // very long range 
            ChanSet.bandwidth = 0;      // 125 kHz
            ChanSet.coding_rate = 4;    // = 4/8
            ChanSet.spread_factor = 12;
            break;
        }
        default:{  // default setting is medium range
            ChanSet.bandwidth = 2;      // 500 kHz
            ChanSet.coding_rate = 1;    // = 4/5
            ChanSet.spread_factor = 7;
        }
    }
    ConfigureRadio( ChanSet );
    #ifndef SILENT
    Serial.println("..done!\n");
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
    memcpy( mPacket, payload, rxSize );
    Radio.Sleep();
    uint32_t packetID = (uint32_t)mPacket[11]<<24 | (uint32_t)mPacket[10]<<16 | (uint32_t)mPacket[9]<<8 | (uint32_t)mPacket[8];

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
/*
    Serial.print("  PacketID: ");
    for(int i=11; i>7; i--){
        Serial.print(mPacket[i], HEX);
    }
*/
    Serial.print("  Packet ID: ");
    Serial.println(packetID, HEX);

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
    found = false;
    for(int i = 0; i < ID_BUFFER_SIZE; i++){
        if (receivedID[i] == packetID) found = true;
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

// hash a string into an integer - djb2 by Dan Bernstein. -
// http://www.cse.yorku.ca/~oz/hash.html
unsigned long hash(char *str) 
{
    unsigned long hash = 5381;
    int c;
    while ((c = *str++) != 0)
      hash = ((hash << 5) + hash) + (unsigned char) c;
    return hash;
}

void ConfigureRadio( ChannelSettings ChanSet )
{
    uint32_t freq = (CH0 + CH_SPACING * ChanSet.channel_num)*1E6;
    #ifndef SILENT
    Serial.printf("\nSetting frequency to %i Hz (meshtastic channel %i) .. \n",freq,ChanSet.channel_num );
    Serial.printf("Setting bandwidth to index %i ..\n",ChanSet.bandwidth);
    Serial.printf("Setting CodeRate to index %i .. \n", ChanSet.coding_rate);
    Serial.printf("Setting SpreadingFactor to %i ..\n",ChanSet.spread_factor);
    #endif
    Radio.SetChannel( freq );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, ChanSet.bandwidth, ChanSet.spread_factor, ChanSet.coding_rate,
                                   LORA_PREAMBLE_LENGTH, false, true, false, 0, false, 20000 );

    Radio.SetRxConfig( MODEM_LORA, ChanSet.bandwidth, ChanSet.spread_factor, ChanSet.coding_rate, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, false , 0, true, false, 0, false, true );

}
