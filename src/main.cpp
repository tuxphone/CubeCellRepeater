#include <Arduino.h>
#include "CubeCell_NeoPixel.h"
#include "config.h"

// CONFIGURATION: change values in config.h !
#define VERBOSE        // define to SILENT to turn off serial messages
#define BLINK        // define to NOBLINK to turn off LED signaling

MeshPacket thePacket;
ChannelSettings ChanSet;
static RadioEvents_t RadioEvents;
uint32_t lastreceivedID = 0x00000000;

#ifndef NOBLINK
CubeCell_NeoPixel LED(1, RGB, NEO_GRB + NEO_KHZ800);
#endif

void setup() {
#ifndef NOBLINK
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext,LOW); //SET POWER
    LED.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    LED.clear( );
    LED.show();
#endif
#ifndef SILENT
    Serial.begin(115200);
    MSG("\nSetting up Radio:\n");
#endif
    RadioEvents.TxDone = TxDone;
    RadioEvents.TxTimeout = TxTimeout;
    RadioEvents.RxDone = RxDone;
    Radio.Init( &RadioEvents );
    Radio.Sleep();
    memcpy(ChanSet.name, MESHTASTIC_NAME, 12);
    //myRegion = &regions[REGION];    
    ChanSet.channel_num = hash( MESHTASTIC_NAME ) % regions[REGION].numChannels; // see config.h
    ChanSet.tx_power    = (regions[REGION].powerLimit == 0) ? TX_MAX_POWER : MIN(regions[REGION].powerLimit, TX_MAX_POWER) ;
    ChanSet.psk         = MESHTASTIC_PSK;
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
            ChanSet.coding_rate = 4;    // = 4/8
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
    MSG("..done! Switch to Receive Mode.\n");
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
    Radio.Sleep( );
#ifndef NOBLINK
    LED.clear( );
    LED.show( );
#endif
#ifndef SILENT
    MSG(".done! Switch to Receive Mode.\n");
#endif
	Radio.Rx( 0 ); // switch to receive mode
}

void TxTimeout( void )
{
    Radio.Sleep( );
#ifndef NOBLINK
   LED.clear( );
   LED.show( );
#endif
#ifndef SILENT
    MSG(".failed (TX Timeout)! Switch to Receive Mode.\n");
#endif
    Radio.Rx( 0 ); // switch to receive mode
}

void RxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    if ( size > MAX_PAYLOAD_LENGTH ) size = MAX_PAYLOAD_LENGTH;
    if ( !(size > sizeof(PacketHeader)) ) {
        #ifndef SILENT
            MSG("\nReceived packet! (Size %i bytes, RSSI %i, SNR %i)\n", size, rssi, snr);
            MSG("Packet to small (No MeshPacket), discard. Switch to Receive Mode.\n");
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
    p->decoded.data.payload.size = p->encrypted.size;
    memcpy(p->encrypted.bytes, payload + sizeof(PacketHeader), p->encrypted.size);
#ifndef SILENT
    MSG("\nReceived packet! (Size %i bytes, RSSI %i, SNR %i)\n", size, rssi, snr);
    MSG("TO: ");                HEXMSG(thePacket.to);
    MSG("  FROM: ");            HEXMSG(thePacket.from);
    MSG("  Packet ID: ");       HEXMSG(thePacket.id);
    MSG("  Flags: WANT_ACK=");  MSG((thePacket.want_ack) ? "YES " : "NO ");
    MSG(" HOP_LIMIT=%i\n",      thePacket.hop_limit);
#endif 
    if ( !(lastreceivedID == thePacket.id) ){ 
        // will repeat package
        #ifndef SILENT
            MSG("Sending packet.. (Size: %i)..", size);
        #endif 
        lastreceivedID = thePacket.id;
        #ifndef NOBLINK
            LED.setPixelColor( 0, RGB_RED );    // send mode
            LED.show();
        #endif
        Radio.Send( payload, size );
     }
    else{
        #ifndef SILENT
            MSG("PacketID = last PacketID, will not repeat again.\n");
        #endif
        Radio.Rx( 0 ); // switch to Receive Mode
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
    //uint32_t freq = (myRegion->freq + myRegion->spacing * ChanSet.channel_num)*1E6;
    uint32_t freq = (regions[REGION].freq + regions[REGION].spacing * ChanSet.channel_num)*1E6;
    
    #ifndef SILENT
    MSG("\nRegion is: %s", regions[REGION].name);
    MSG("  TX power: %i\n", ChanSet.tx_power);
    MSG("Setting frequency to %i Hz (meshtastic channel %i) .. \n",freq,ChanSet.channel_num );
    MSG("Channel name is: %s .. \n", ChanSet.name );
    MSG("Setting bandwidth to index %i ..\n",ChanSet.bandwidth);
    MSG("Setting CodeRate to index %i .. \n", ChanSet.coding_rate);
    MSG("Setting SpreadingFactor to %i ..\n",ChanSet.spread_factor);
    #endif
    Radio.SetChannel( freq );
    Radio.SetTxConfig( MODEM_LORA, ChanSet.tx_power ,0 , ChanSet.bandwidth, ChanSet.spread_factor, ChanSet.coding_rate,
                                   LORA_PREAMBLE_LENGTH, false, true, false, 0, false, 20000 );
    Radio.SetRxConfig( MODEM_LORA, ChanSet.bandwidth, ChanSet.spread_factor, ChanSet.coding_rate, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, false , 0, true, false, 0, false, true );
}
