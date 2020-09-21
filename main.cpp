#include <Arduino.h>    // needed for platform.io
#include "mesh.pb.h"
#include "MeshRadio.h"
// CONFIGURATION of Meshtastic channel name and speed: change values in MeshRadio.h !

#define LORA_PREAMBLE_LENGTH        32          // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT         0           // Symbols
#define RX_TIMEOUT_VALUE            1000
#define MAX_PAYLOAD_LENGTH          0xFF        // max payload (see  \cores\asr650x\device\asr6501_lrwan\radio.c  --> MaxPayloadLength)
#define ID_BUFFER_SIZE              32

#define VERBOSE                                // define to SILENT to turn off serial messages

typedef struct {
    uint32_t to, from, id; 
    uint8_t flags;       // The bottom three bits of flags are use to store hop_limit, bit 4 is the WANT_ACK flag
} PacketHeader;

void TxDone( void );
void TxTimeout( void );
void RxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void ConfigureRadio( ChannelSettings ChanSet );
unsigned long hash(char *str);

MeshPacket thePacket;
ChannelSettings ChanSet;
static RadioEvents_t RadioEvents;
uint32_t receivedID[ID_BUFFER_SIZE];
int receivedCount = -1;

void setup() {
    #ifndef SILENT
    Serial.begin(115200);
    Serial.println("\nSetting up Radio:");
    #endif
    RadioEvents.TxDone = TxDone;
    RadioEvents.TxTimeout = TxTimeout;
    RadioEvents.RxDone = RxDone;
    Radio.Init( &RadioEvents );
    Radio.Sleep();
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
    Serial.println("..done! Switch to Receive Mode.\n");
    #endif
    Radio.Rx(0);  // initial mode = receive
}

void loop( )
{
    lowPowerHandler( ); 
    Radio.IrqProcess( );
}

void TxDone( void )
{
	#ifndef SILENT
    Serial.println(".done! Switch to Receive Mode.");
    #endif
    Radio.Sleep( );
	Radio.Rx( 0 ); // switch to receive mode
}

void TxTimeout( void )
{
    #ifndef SILENT
    Serial.println(".failed (TX Timeout)! Switch to Receive Mode.");
    #endif
    Radio.Sleep( );
    Radio.Rx( 0 ); // switch to receive mode
}

void RxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    if ( size > MAX_PAYLOAD_LENGTH ) size = MAX_PAYLOAD_LENGTH;
    if ( !(size > sizeof(PacketHeader)) ) {
        #ifndef SILENT
            Serial.printf("\nReceived packet! (Size %i bytes, RSSI %i, SNR %i)\n", size, rssi, snr);
            Serial.println("Packet to small (No MeshPacket), discard. Switch to Receive Mode.");
        #endif
        Radio.Rx( 0 );
        return;
    }
    PacketHeader * h = (PacketHeader *)payload;
    MeshPacket   * p = &thePacket;
    p->to   = h->to;
    p->from = h->from;
    p->id   = h->id;
    p->hop_limit = h->flags && 0b00000111;
    p->want_ack  = h->flags && 0b00001000;
    p->which_payload = MeshPacket_encrypted_tag;
    p->encrypted.size= size - sizeof(PacketHeader);
    memcpy(p->encrypted.bytes, payload + sizeof(PacketHeader), p->encrypted.size);
    
#ifndef SILENT
    Serial.printf("\nReceived packet! (Size %i bytes, RSSI %i, SNR %i)\n", size, rssi, snr);
    
    Serial.print("TO: ");
    (thePacket.to == UINT32_MAX) ? Serial.print( "BROADCAST" ) : Serial.print( thePacket.to, HEX );

    Serial.print("  FROM: ");
    Serial.print(thePacket.from, HEX);

    Serial.print("  Packet ID: ");
    Serial.print(thePacket.id, HEX);

    Serial.print("  Flags: WANT_ACK=");
    Serial.print( (thePacket.want_ack) ? "YES " : "NO ");
    
    Serial.printf("HOP_LIMIT=%i\n", thePacket.hop_limit);
    
    Serial.print("Payload: ");
    for(int i=0; i < p->encrypted.size; i++){        
        Serial.print(p->encrypted.bytes[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
#endif 
    bool found = false;
    for(int i = 0; i < ID_BUFFER_SIZE; i++){
        if (receivedID[i] == thePacket.id) found = true;
    }
    if (!found){ 
        // will repeat package
        
        #ifndef SILENT
            Serial.print("Sending packet..");
            Serial.printf("(Size: %i)..", size);
        #endif 
        (receivedCount < (ID_BUFFER_SIZE - 1) ) ?  receivedCount++ : receivedCount = 0; // if counter = max, overwrite first entry in list, reset counter
        receivedID[receivedCount] = thePacket.id; // add ID to received list
        Radio.Send( payload, size );
     }
    else{
        #ifndef SILENT
            Serial.println("PacketID known, will not repeat again.");
        #endif
        Radio.Rx( 0 ); // wait for new package
    } 
}

// hash a string into an integer - djb2 by Dan Bernstein. -
// http://www.cse.yorku.ca/~oz/hash.html
unsigned long hash(char *str) 
{
    unsigned long hash = 5381;
    int c;
    while ((c = *str++) != 0)
      hash = ((hash << 5) + hash) + (unsigned char) c;    // hash * 33 + c //
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
