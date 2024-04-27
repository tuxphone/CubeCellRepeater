#include <main.h>

typedef struct {
    uint32_t to, from, id;
    // The bottom three bits of flags are used to store hop_limit
    uint8_t flags;
    uint8_t channel;
} PacketHeader;

void setup() {
#ifndef SILENT
  Serial.begin(115200);
#endif
  initRegion();       // create regions[] and load myRegion
  applyModemConfig(); // apply lora settings
  radio.standby();
  radio.setPacketReceivedAction(ISR_setReceived); // Interrupt Handler -> set "PacketReceived" flag at RX
  MSG("[INFO][SX1262] Starting to listen ... ");
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    MSG("success!\n\n");
  } else {
    MSG("\n[ERROR][SX1262] startReceive() failed, code: %i\n\n ** Full Stop **", state);
    while (true);
  }

}

void clearInterrupts(void) {
  radio.clearDio1Action();
  radio.finishTransmit();
}

void loop() {
  if (PacketReceived) {
    PacketReceived = false;
    const size_t length = radio.getPacketLength();
    state = radio.readData(radiobuf, length);
    MSG("[RX]");
    if (state == RADIOLIB_ERR_NONE) {
      const int32_t payloadLen = length - sizeof(PacketHeader);
      if (payloadLen < 0) {
        MSG("\n[WARN]Not a Meshtastic packet, too short!\n");
        return; // will not repeat
      }
      //const uint8_t *payload = radiobuf + sizeof(PacketHeader);
      PacketHeader *h = (PacketHeader *)radiobuf;

      const uint8_t hop_limit = h->flags & PACKET_FLAGS_HOP_MASK;
      
      MSG(" (id=0x%08X) HopLim=%i from=0x%08X to=0x%08X WantAck=%s viaMQTT=%s", h->id, 
         hop_limit, h->from, h->to, (h->flags & PACKET_FLAGS_WANT_ACK_MASK)? "YES":"NO", (h->flags & PACKET_FLAGS_VIA_MQTT_MASK)? "YES":"NO");
      
      MSGFLOAT(" SNR=",radio.getSNR() );
      MSGFLOAT(" RSSI=", radio.getRSSI() );
      MSG(" re-send=%s\n", ( (lastPacketID != h->id) && (hop_limit!=0) )?"YES":"NO");

      if ( (lastPacketID != h->id) && (hop_limit!=0) ) {
        h->flags -= 1; // decrease HopLim by 1
        clearInterrupts();
        while ( radio.scanChannel() == RADIOLIB_LORA_DETECTED ) delay(100);
        MSG("[TX] (id=0x%08X) HopLim=%i ... ", h->id, (h->flags & PACKET_FLAGS_HOP_MASK));
        lastPacketID = h->id;
        clearInterrupts();
        radio.setPacketSentAction(ISR_setPacketSent);
        state=radio.startTransmit((uint8_t*)&radiobuf, length);
        if (state == RADIOLIB_ERR_NONE) {
          MSG("OK\n");
        }
        else {
          MSG("failed, ERR = %i - resume RX", state);
          clearInterrupts();
          radio.setPacketReceivedAction(ISR_setReceived);
          radio.startReceive(); 
        }
      }
      
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      MSG(" [ERROR]CRC error!\n");
    } else {
      MSG(" [ERROR]Receive failed, code: %i!\n", state);
    }
  }

  if (PacketSent) {
    PacketSent = false;
    clearInterrupts();
    radio.setPacketReceivedAction(ISR_setReceived);
    radio.startReceive(); 
  }
  
  delay(100); // wait for Serial
  MCU_deepsleep();

}

void MCU_deepsleep(void) {
#ifdef CUBECELL  
#ifndef SILENT
    UART_1_Sleep;  // aka USB, if you use UART2 for communication, add UART_2_Sleep / Wakeup
#endif
    pinMode(P4_1, ANALOG); // SPI0 MISO, save power
    CySysPmDeepSleep(); // deep sleep mode
    // after sleep, set global time counter, set MISO to input, reactivate UART :    
    systime = (uint32_t)RtcGetTimerValue();
    pinMode(P4_1, INPUT);
#ifndef SILENT
    UART_1_Wakeup;
#endif
#endif //CUBECELL

}
