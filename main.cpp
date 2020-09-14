/*
COPYRIGHT NOTICE

This work is based on the Factory_test examples, found here: https://github.com/HelTecAutomation/ASR650x-Arduino/tree/master/libraries/Basics/examples
so the copyright for this code belongs to Heltec Automation.

Any additional changes are free-to-use, but at your own risk.
*/
#include <Arduino.h>    // for platform.io, remove when using Arduino IDE    
#include <radio.h>      // not needed when using platform.io

// Radio settings for meshtastic channel "Default" @ "very long range (but slow)". Will not work with other settings or channels.
#define RF_FREQUENCY                865200000   // Hz
#define TX_OUTPUT_POWER             22          // dBm
#define LORA_BANDWIDTH              0           // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR       12          // [SF7..SF12]
#define LORA_CODINGRATE             4           // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH        32          // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT         0           // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false
#define RX_TIMEOUT_VALUE            1000
#define BUFFER_SIZE                 512         // max payload = maximum of the SX12xx

#define ID_BUFFER_SIZE              32

char mPacket[BUFFER_SIZE];

uint32_t receivedID[ID_BUFFER_SIZE];
int receivedCount = 0;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

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


#define SILENT         // delete or change to VERBOSE if serial output is needed

void setup() {
    #ifndef SILENT
    Serial.begin(115200);
    Serial.print("\nSetting up Radio..");
    #endif
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, 0, true, 0, 0, 0, 20000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, 0 , 0, true, 0, 0, 0, true );

    state=RX;   // initial mode = receive
    #ifndef SILENT
    Serial.println(".done!\n");
    #endif
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
        receivedID[receivedCount] = x; // add ID to received list
        (receivedCount < (ID_BUFFER_SIZE - 1) ) ?  receivedCount++ : receivedCount = 0; // if counter = max, reset counter. New entries will overwrite.
    }
    else{
        #ifndef SILENT
        Serial.println("PacketID known, will not repeat again.");
        #endif
        state = RX; // wait for new package
    } 
}
