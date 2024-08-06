#include <main.h>

void setup() {
  init_signalize();
  msgID.clear();
  txQueue.clear();
  memcpy(psk.bytes, mypsk, sizeof(mypsk));
  psk.length = sizeof(mypsk);
  crypto->setKey(psk);

  initRegion();       // create regions[] and load myRegion
  applyModemConfig(); // apply lora settings
  radio.setDio1Action(ISR_dio1Action);
  startReceive();
}

void loop() {

  if (dio1) {
    dio1 = false;
    static uint16_t irqStatus = radio.getIrqStatus();
    if (irqStatus & RADIOLIB_SX126X_IRQ_RX_DONE) {
      PacketReceived = true;
    }
    if (irqStatus & RADIOLIB_SX126X_IRQ_TX_DONE) {
      p = NULL;
      PacketSent = false;
      startReceive();
    }
  }

  if (PacketReceived) {
    PacketReceived = false;
    signalizeRX_ON();
    Packet_t pck;
    pck.packetTime = millis();
    pck.size = radio.getPacketLength();
    if (pck.size == 0) {
      signalizeLED_OFF();
      return; // we are still in receive mode
    }
    err = radio.readData(pck.buf, pck.size);
    PacketHeader* h = (PacketHeader *)pck.buf;
    if (err == RADIOLIB_ERR_NONE) {
      const int32_t payloadLen = pck.size - sizeof(PacketHeader);
      if (payloadLen < 0) {
        signalizeLED_OFF();
        return; // will not repeat, continue receiving
      }
      const uint8_t hop_limit = h->flags & PACKET_FLAGS_HOP_MASK;
      repeatPacket = msgID.add(h->id);

      // do not repeat if id is known or hop limit is zero
      if (hop_limit == 0) repeatPacket = false;
      if ( repeatPacket ){
        h->flags -= 1; // decrease hop limit by 1
        txQueue.add(&pck);
      }
      
    } else if (err == RADIOLIB_ERR_CRC_MISMATCH) {
      // CRC Error
    } else {
      // Receive failed
    }
  }

  if ( (txQueue.Count > 0) || !(p==NULL) ) { 
    uint32_t now = millis();
    // we have packets in queue and currently no packet to transmit
    if (p == NULL) {
      if ( txQueue.pop() ) {
        p = &PacketToSend;
      }
    }
    // resume receiving in case we popped an empty packet
    if (p == NULL) {
      PacketSent = true; 
      return;
    } 
      
    if (p->size > 0) {  // size == 0 means "deleted", we don't send deleted packets
      uint32_t wait = getTxDelayMsecWeighted(snr);
      activeReceiveStart = 0;
      while ( ((now + wait) > millis() ) || ( isActivelyReceiving() ) ) {
        // while waiting, we still are in receive mode
        if (dio1) return; // new packet arrived, return to handle it
        delay(5);
      }
      // drop packet if we could not send it in 1 minute
      if ( (p->packetTime + 60*1000) < millis() ) {
        p = NULL;
        PacketSent = true;
        // could not send packet (time out is 1 minute)
        signalizeLED_OFF();
        return;
      }
      if (perhapsSend(p) ) {
        // packet successfully sent
        PacketSent = true;
        p = NULL; 
      } else {      
        // resume receiving if we could not send
        PacketSent = true;
      }
    } else {
      // tried to send empty packet (should not happen)
      PacketSent = true;
    }
  }

  if (PacketSent) {
      PacketSent = false;
      startReceive();
  }

  if (txQueue.Count == 0) {
    signalizeLED_OFF();
    MCU_deepsleep(); // sleep until IRQ
  }
}


void MCU_deepsleep(void) {
#ifdef CUBECELL  
#ifndef SILENT
    UART_1_Sleep;  // aka USB, if you use UART2 for communication, add UART_2_Sleep / Wakeup
#endif
    pinMode(P4_1, ANALOG); // SPI0 MISO, save power
    CySysPmDeepSleep();
    // after sleep, set global time counter, set MISO to input, reactivate UART :    
    systime = (uint32_t)RtcGetTimerValue();
    pinMode(P4_1, INPUT);
#ifndef SILENT
    UART_1_Wakeup;
#endif
#endif //CUBECELL
}

void startReceive(){
  // clear irq status, standby()
  radio.finishTransmit();
  dio1 = false;
  err = radio.startReceiveDutyCycleAuto(preambleLength, 8,
    RADIOLIB_SX126X_IRQ_RX_DEFAULT | RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED | RADIOLIB_SX126X_IRQ_HEADER_VALID);
  if (err != RADIOLIB_ERR_NONE){  // we got an error
    radio.reset();
    delay(500);
    applyModemConfig();
    startReceive();
  } 
  isReceiving = true;
}

// send packet (blocking), MCU sleeps while waiting for TX DONE
bool perhapsSend(Packet_t* p) {
  
    if (p->size > RADIOLIB_SX126X_MAX_PACKET_LENGTH) {
      p->size = RADIOLIB_SX126X_MAX_PACKET_LENGTH;
    }
    // clear irq status, standby()
    radio.finishTransmit();
    dio1 = false;
    PacketHeader* h = (PacketHeader *)p->buf;
    err=radio.startTransmit(p->buf, p->size);
    isReceiving = false;
    if (err == RADIOLIB_ERR_NONE) {
      // success
      signalizeTX_ON();
    } else {
      // failed
      return false;
    }
    delay(10);
    MCU_deepsleep(); // wait for TX to complete, will wake on any DIO1
    err = radio.getIrqStatus();
    if ( err & RADIOLIB_SX126X_IRQ_TX_DONE ) {
      // done
      dio1=true;
    } else {
      // failed
      dio1 = false;
    }
    radio.finishTransmit(); 
    signalizeLED_OFF();
    return ( err & RADIOLIB_SX126X_IRQ_TX_DONE );
}

// PacketQueueClass Definitions

void PacketQueueClass::add(Packet_t* p) {
  uint8_t idx = MAX_TX_QUEUE;
  for (uint8_t i=0; i<(MAX_TX_QUEUE-1); i++) {
    if (Queue[i].size == 0) {  // search for a free slot
      idx = i;
      break;
    }
  }
  if (idx == MAX_TX_QUEUE) {  // no free slot, overwrite oldest packet
    for (uint8_t i=1; i<(MAX_TX_QUEUE-1); i++) {
      if (Queue[idx].packetTime < Queue[i].packetTime) idx = i;
    }
  }
  Queue[idx].size = p->size;
  Queue[idx].packetTime = p->packetTime;
  memcpy(Queue[idx].buf, p->buf, p->size);
  this->Count += 1;
}

bool PacketQueueClass::pop(void) {
  if (this->Count == 0) return false;
  uint8_t idx = MAX_TX_QUEUE;
  for (uint8_t i=0 ;i < (MAX_TX_QUEUE -1); i++ ){
    if (this->Queue[i].size != 0) { // first not empty entry
      idx = i;
      break;
    }
  }
  if (idx == MAX_TX_QUEUE) { // empty Queue
    this->Count = 0;
    return false;
  }
  for (uint8_t i=idx; i<(MAX_TX_QUEUE -1); i++) {
    if ( (this->Queue[i].size != 0) && (this->Queue[idx].packetTime < this->Queue[i].packetTime) ) {
      idx = i; // find oldest packet
    }
  } 
  PacketToSend.packetTime = millis(); // start timer. after 1 minute, drop packet
  PacketToSend.size = this->Queue[idx].size;
  memcpy(PacketToSend.buf, this->Queue[idx].buf, this->Queue[idx].size);
  this->Queue[idx].size = 0;
  this->Count -= 1;
  return true;
}

void PacketQueueClass::clear(void) {
  for (uint8_t i = 0; i<(MAX_TX_QUEUE - 1); i++) {
    this->Queue[i].size = 0; // mark as "deleted"
    this->Queue[i].packetTime = 0;
    memset(this->Queue[i].buf, 0, sizeof(this->Queue[i].buf));
  }
  this->Count = 0;
}

// idStoreClass Definitions

bool idStoreClass::add(uint32_t id) {
  uint8_t idx = 0;
  for (int i = MAX_ID_LIST-1; i >= 0; i--) {
    if (storage[i] == id){
     return false; // packet ID is known, no update
    }
    if (storage[i] == 0) idx = i;
  }
  storage[idx] = id;
  if (++idx <= MAX_ID_LIST) { 
    storage[idx] = 0; 
  } else {
    storage[0] = 0;
  }
  return true; // new packet ID was added
}

void idStoreClass::clear(void) {
  memset(this->storage, 0, sizeof(this->storage));
}


void init_signalize(void) {
#ifdef CC_SIGNAL_NEOPIXEL
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext,LOW);
    pixels.begin();
    pixels.clear();
#endif
#ifdef CC_SIGNAL_GPIO13
#ifndef GPIO13
#define GPIO13 P0_6
#endif
    pinMode(GPIO13, OUTPUT);
#endif   
}

void signalizeRX_ON(void){
#ifdef CC_SIGNAL_NEOPIXEL
    pixels.setPixelColor(0, pixels.Color(0, 24, 0));
    pixels.show();
#endif
#ifdef CC_SIGNAL_GPIO13
    digitalWrite(GPIO13, HIGH);
#endif
}

void signalizeTX_ON(void){
#ifdef CC_SIGNAL_NEOPIXEL
    pixels.setPixelColor(0, pixels.Color(24, 0, 0));
    pixels.show();
#endif
#ifdef CC_SIGNAL_GPIO13
    digitalWrite(GPIO13, HIGH);
#endif
}

void signalizeLED_OFF(void){
#ifdef CC_SIGNAL_NEOPIXEL
    pixels.clear();
    pixels.show();
#endif
#ifdef CC_SIGNAL_GPIO13
    digitalWrite(GPIO13, LOW);
#endif
}