#include <Arduino.h>
#include "config.h"
#include <Base64.h>
#include "cyPm.c"

// CONFIGURATION: 
// Change RegionCode, Frequency, Speed in config.h !
#define VERBOSE         // define to SILENT to turn off serial messages
#define NOBLINK        // define to NOBLINK to turn off LED signaling
#define NO_OLED        // define to NO_OLED to turn off display
                       // OLED supported for cubecell_board_Plus (HTCC-AB02) and cubecell_gps (HTCC-AB02S)
// :CONFIGURATION

static MeshPacket       thePacket;
static ChannelSettings  ChanSet;
static RadioEvents_t    RadioEvents;
static TimerEvent_t     CheckRadio;
static uint32_t         lastreceivedID = 0;
static uint32_t         symbolTime;
static uint32_t         sendTime;
extern uint32_t         systime;   // global system time count --> millis()

#ifndef NOBLINK
#include "CubeCell_NeoPixel.h"
CubeCell_NeoPixel LED(1, RGB, NEO_GRB + NEO_KHZ800);
#endif

#ifndef NO_OLED
    #ifdef CubeCell_BoardPlus
        #include "cubecell_SH1107Wire.h"
        SH1107Wire  display(0x3c, 500000, I2C_NUM_0,GEOMETRY_128_64,GPIO10 ); 
    #endif
    #ifdef CubeCell_GPS
        #include "cubecell_SSD1306Wire.h"
        SSD1306Wire  display(0x3c, 500000, I2C_NUM_0,GEOMETRY_128_64,GPIO10 ); 
    #endif
char str[32];
#endif

void setup() {
    TimerInit( &CheckRadio, onCheckRadio );
#ifndef NO_OLED
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext,LOW);
#endif
#ifndef NOBLINK
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext,LOW); 
    delay(100);
    LED.begin();
    LED.clear( );
    LED.show();
#endif
#ifndef NO_OLED
    display.init();
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0,0,"CC Repeater");
    display.display();
#endif
#ifndef SILENT
    Serial.begin(115200);
    MSG("\n");
    LINE(60, "*");
    MSG("Setting up Radio:\n");
#endif
    RadioEvents.TxDone      = onTxDone;
    RadioEvents.TxTimeout   = onTxTimeout;
    RadioEvents.RxDone      = onRxDone;
    RadioEvents.RxTimeout   = onRxTimeout;
    RadioEvents.CadDone     = onCadDone;
    Radio.Init( &RadioEvents );
    Radio.Sleep();
    // import Channel Settings from Meshtastic Link (channel name, modem config):
    char decoded[Base64.decodedLength(&MeshtasticLink[30], sizeof(MeshtasticLink)-30)];
    Base64.decode(decoded, &MeshtasticLink[30], sizeof(MeshtasticLink)-30);
    pb_decode_from_bytes((uint8_t *)&decoded[0], sizeof(decoded), ChannelSettings_fields, &ChanSet);
    // done, populate remaining settings according to channel name and provided modem config
    ChanSet.channel_num = hash( ChanSet.name ) % regions[REGION].numChannels; // see config.h
    ChanSet.tx_power    = (regions[REGION].powerLimit == 0) ? TX_MAX_POWER : MIN(regions[REGION].powerLimit, TX_MAX_POWER) ;
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
    switch ( (uint8_t)ChanSet.modem_config ){
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
        default:{  // default setting is very long range
            ChanSet.bandwidth = 0;      // 125 kHz
            ChanSet.coding_rate = 4;    // = 4/8
            ChanSet.spread_factor = 12;
        }
    }
    symbolTime =  ((1<< ChanSet.spread_factor)*1000 +32) *1000 / TheBandwidths[ChanSet.bandwidth]; // in micro seconds!
    ConfigureRadio( ChanSet );
#ifndef SILENT
    MSG("\n..done!\n");
    LINE(60,"*");
#endif
#ifndef NO_OLED
    display.drawString(0,32, "Receive Mode..");
    display.display();
#endif
    TimerSetValue( &CheckRadio, symbolTime / 100 ); // MCU sleeps 10 LoRa symbols (in milli seconds)
    TimerStart( &CheckRadio );                      // onCheckRadio() will break deep sleep mode
    Radio.StartCad( 3 ); // length in symbols
}

void onCheckRadio(void)
{ 
    TimerReset(&CheckRadio);
}

// Cycle starts @ 0 symbols. LoRA: CAD for x symbols, then (implicitly) Standby  MCU: sleep
// After 10 LoRa symbols, wake up MCU and check for IRQs from LoRa (including CADdone)
// If channel activiy detected, switch to RX mode for 500 symbols, to capture very long packages.
// If the package is shorter, the onRXDone handler will put the LoRa to sleep, so no excess power consumption
// Radio.Send() is non-blocking, so if a TX is running we cannot immediatly start a new CAD. We wait (sleep) until LoRa is idle.
 
void loop( )
{
    
    MCU_deepsleep( );
    Radio.IrqProcess(); // handle events from LoRa, if CAD, set SX1262 to receive (onCadDone)
    if ( Radio.GetStatus() == RF_IDLE ) Radio.StartCad( 3 ); // (in symbols)
    
}

void onCadDone( bool ChannelActive ){
    // Rx Time = 500 * symbol time (in ms) should be longer than receive time for max. packet length 
    (ChannelActive) ? Radio.Rx( symbolTime >> 1 ) : Radio.Sleep();
}

void onRxTimeout( void ){
    Radio.Sleep();
}

void onTxDone( void )
{
#ifndef NOBLINK
    LED.clear( );
    LED.show( );
#endif
#ifndef SILENT
MSG(".done (%ims)! Switch to Receive Mode.\n", millis() - sendTime );
#endif
#ifndef NO_OLED
            sprintf(str,"..done. RX Mode..");
            display.drawString(42,53,str);
            display.display();
#endif
    Radio.Sleep();
}

void onTxTimeout( void )
{
#ifndef NOBLINK
   LED.clear( );
   LED.show( );
#endif
#ifndef SILENT
    MSG(".failed (TX Timeout)! Switch to Receive Mode.\n");
#endif
#ifndef NO_OLED
            sprintf(str,"..timeout!");
            display.drawString(42,53,str);
            display.display();
#endif
    Radio.Sleep();
}

void onRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep();
    if ( size > MAX_PAYLOAD_LENGTH ) size = MAX_PAYLOAD_LENGTH;
    if ( !(size > sizeof(PacketHeader)) ) {
        #ifndef SILENT
            MSG("\nReceived packet! (Size %i bytes, RSSI %i, SNR %i)\n", size, rssi, snr);
            MSG("Packet to small (No MeshPacket).\n");
        #endif
        #ifndef NOBLINK
            LED.setPixelColor( 0, RGB_RED );    // send mode
            LED.show();
        #endif
        Radio.Send( payload, size );
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
    MSG("TO: %.2X ",                p->to);
    MSG("  FROM: %.2X ",            p->from);
    MSG("  Packet ID: %.2X ",       p->id);
    MSG("  Flags: WANT_ACK="); MSG((p->want_ack) ? "YES " : "NO ");
    MSG(" HOP_LIMIT=%i\n",          p->hop_limit);
    MSG("Payload:"); for ( int i=0; i < p->encrypted.size; i++ ) MSG(" %.2X", p->encrypted.bytes[i]);
    MSG("\n");
#endif 
#ifndef NO_OLED
    display.clear();
    display.display();
    display.setFont(ArialMT_Plain_10);
    sprintf(str,"Size: %i RSSI %i SNR %i", size, rssi, snr);
    display.drawString(0,0,str);
    sprintf(str,"%X", p->to);
    display.drawString(0,11,"TO:");
    display.drawString(40,11,str);
    sprintf(str,"%X", p->from);
    display.drawString(0,22,"FROM:");
    display.drawString(40,22,str);
    sprintf(str,"%X", p->id);
    display.drawString(0,32,"ID:");
    display.drawString(40,32,str);
    display.display();
#endif
    if ( !(lastreceivedID == thePacket.id) ){ 
        // will repeat package
        lastreceivedID = thePacket.id;
        #ifndef NO_OLED
            sprintf(str,"Sending..");
            display.drawString(0,53,str);
            display.display();
        #endif
        #ifndef NOBLINK
            LED.setPixelColor( 0, RGB_RED );    // send mode
            LED.show();
        #endif
        #ifndef SILENT
            MSG("Sending packet..");
            sendTime = millis();
        #endif 
        Radio.Send( payload, size );
     }
    else{
        #ifndef SILENT
            MSG("PacketID = last PacketID, will not repeat again.\n");
        #endif
        #ifndef NO_OLED
            sprintf(str,"ID known!");
            display.drawString(0,53,str);
            display.display();
        #endif
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
    uint32_t freq = (regions[REGION].freq + regions[REGION].spacing * ChanSet.channel_num)*1E6;
    #ifndef SILENT
    MSG("\nRegion is: %s", regions[REGION].name);
    MSG("  TX power: %i\n", ChanSet.tx_power);
    MSG("Channel name is: '%s' .. \n", ChanSet.name );
    MSG("Setting frequency to %i Hz (meshtastic channel %i) .. \n",freq,ChanSet.channel_num );
    MSG("Setting bandwidth to index  %i (%ikHz)..\n", ChanSet.bandwidth, TheBandwidths[ChanSet.bandwidth]  );
    MSG("Setting CodeRate  to index  %i (4/%i).. \n",  ChanSet.coding_rate, ChanSet.coding_rate + 4 );
    MSG("Setting SpreadingFactor to %i ..\n",ChanSet.spread_factor);
    MSG("Symboltime is %i micro s ..\n", symbolTime );
    #endif
    Radio.SetChannel( freq );
    Radio.SetTxConfig( MODEM_LORA, ChanSet.tx_power ,0 , ChanSet.bandwidth, ChanSet.spread_factor, ChanSet.coding_rate,
                                   LORA_PREAMBLE_LENGTH, false, true, false, 0, false, 20000 );
    Radio.SetRxConfig( MODEM_LORA, ChanSet.bandwidth, ChanSet.spread_factor, ChanSet.coding_rate, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, false , 0, true, false, 0, false, true );
}

void MCU_deepsleep(void)
{
    #ifndef SILENT
    UART_1_Sleep;
    #endif
    pinMode(P4_1, ANALOG); // SPI0 MISO
    CySysPmDeepSleep(); // deep sleep mode    
    systime = (uint32_t)TimerGetCurrentTime();
    pinMode(P4_1, INPUT);
    #ifndef SILENT
    UART_1_Wakeup;
    #endif
}
