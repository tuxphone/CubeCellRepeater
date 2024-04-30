#include <main.h>
#include <assert.h>

typedef struct {
    uint32_t to, from, id;
    // The bottom three bits of flags are used to store hop_limit
    uint8_t flags;
    uint8_t channel;
} PacketHeader;

void setup() {
  UART_1_RxWakeDisableInt(); // no wake on UART activity
  memset(idList, 0, MAX_ID_LIST *4);
  for (uint8_t i = 0; i<(MAX_TX_QUEUE - 1); i++) {
    txQueue[i].size = 0;
  }
  #ifndef SILENT
  Serial.begin(115200);
  #endif

  MSG("[INFO][CryptoEngine] Initializing ... ");
  memcpy(psk.bytes, mypsk, sizeof(mypsk));
  psk.length = sizeof(psk.bytes);
  crypto->setKey(psk);

  initRegion();       // create regions[] and load myRegion
  applyModemConfig(); // apply lora settings
  MSG("[INFO][SX1262] Starting to listen ... ");
  startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    MSG("success!\n\n");
  } else {
    MSG("\n[ERROR][SX1262] startReceive() failed, code: %i\n\n ** Full Stop **", state);
    while (true);
  }
}

void loop() {
  if (PacketReceived) {
    PacketReceived = false;
    const size_t length = radio.getPacketLength();
    state = radio.readData(radiobuf, length);
    PacketHeader* h = (PacketHeader *)radiobuf;

    //MSG("[RX]");
    if (state == RADIOLIB_ERR_NONE) {
      const int32_t payloadLen = length - sizeof(PacketHeader);
      if (payloadLen < 0) {
        MSG("[WARN]Not a Meshtastic packet, too short!\n");
        return; // will not repeat
      }
      const uint8_t hop_limit = h->flags & PACKET_FLAGS_HOP_MASK;
      /*
      MSG(" (id=0x%08X) HopLim=%i from=0x%08X to=0x%08X WantAck=%s viaMQTT=%s", h->id, 
         hop_limit, h->from, h->to, (h->flags & PACKET_FLAGS_WANT_ACK_MASK)? "YES":"NO", (h->flags & PACKET_FLAGS_VIA_MQTT_MASK)? "YES":"NO");
      
      MSGFLOAT(" SNR=",radio.getSNR() );
      MSGFLOAT(" RSSI=", radio.getRSSI() );
      */
      repeatPacket =  update_idList(h->id); 
      if ( hop_limit == 0) repeatPacket = false; // do not repeat if id is known or hop limit is Zero
#ifndef SILENT // print packet:
      const uint8_t *payload = radiobuf + sizeof(PacketHeader);
      mp.from = h->from;
      mp.to = h->to;
      mp.id = h->id;
      mp.channel = h->channel;
      mp.hop_limit = h->flags   & PACKET_FLAGS_HOP_MASK;
      mp.want_ack = h->flags & PACKET_FLAGS_WANT_ACK_MASK;
      mp.via_mqtt = h->flags & PACKET_FLAGS_VIA_MQTT_MASK;
      mp.rx_snr = radio.getSNR();
      mp.rx_rssi = lround(radio.getRSSI());
      mp.which_payload_variant = meshtastic_MeshPacket_encrypted_tag; // Mark that the payload is still encrypted at this point
      assert(((uint32_t)payloadLen) <= sizeof(mp.encrypted.bytes));
      memcpy(mp.encrypted.bytes, payload, payloadLen);
      mp.encrypted.size = payloadLen;
      MSG("\n[NEW]");
      if ( !repeatPacket ){
        MSG("(id=0x%08X) no repeat! (HopLim %d)\n", mp.id, mp.hop_limit);
      } else {
        perhapsDecode(&mp); // try to decode and print the result
      }
#endif //SILENT     
      
/*    MSG(" repeat=%s\n", (repeatPacket)?"YES":"NO");
      MSGFLOAT(" Freq Error=", radio.getFrequencyError());
      MSG("\n");
*/
      if ( repeatPacket ) {
        h->flags -= 1; // decrease HopLim by 1
        enqueueTX(radiobuf, length);
      }
      
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      MSG(" [ERROR]CRC error!\n");
    } else {
      MSG(" [ERROR]Receive failed, code: %i!\n", state);
    }
  }

  if (txQueueHasPacket) { // try to repeat packets
    //const long wait = lround( abs( radio.getSNR() * radio.getRSSI() ));
    long wait = 1000;
    radio.startChannelScan(3, 24, 10);
    for (int i = 1; i < wait; i++) {
      if (radio.getChannelScanResult() == RADIOLIB_LORA_DETECTED) {
        clearInterrupts();
        radio.setPacketReceivedAction(ISR_setReceived);
        radio.startReceive(20000); 
        return; // new packet arrived while waiting!
      }
      delay(2);
    }

    uint8_t idx = poptxQueue();
    if (txQueue[idx].size == 0){
      txQueueHasPacket = false; // empty Queue, do nothing
      //MSG("[INFO] TX Queue is empty\n");
    }
    else {
      perhapsSend(&txQueue[idx].buf[0], txQueue[idx].size);
      txQueue[idx].size = 0;
    }    
  }

  if (PacketSent) {
    PacketSent = false;
    startReceive();
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

void clearInterrupts(void) {
  radio.clearDio1Action();
  radio.finishTransmit();
}

void startReceive(){
  clearInterrupts();
  radio.setPacketReceivedAction(ISR_setReceived);
  state=radio.startReceive(); 
}

bool update_idList(uint32_t id){
  for (uint8_t i=0; i < (MAX_ID_LIST - 1); i++) {
     //MSG("\nstored: 0x%08X  delivered: 0x%08X",idQueue[i-1], id);
    if (idList[i] == id){
     return false; // packet ID is known, no Queue update
    }
  }

  // store new ID:
  uint8_t idx = MAX_ID_LIST;
  for (uint8_t i=0; i < (MAX_ID_LIST -1); i++){
    if (idList[i] == 0) {
      idx = i;
      break;
    } 
  }
  if (idx == MAX_ID_LIST) {
    idList[0] = id;
    idList[1] = 0;
  }
  else {
    idList[idx] = id;
    if (++idx < MAX_ID_LIST) { idList[idx] = 0; }
    else { idList[0] = 0; }
  }
  return true; // packet ID was not known, Queue was updated
}

void enqueueTX(uint8_t* buf, size_t size){
  PacketHeader* h = (PacketHeader *)buf;
  uint8_t idx = MAX_TX_QUEUE;
  for (uint8_t i=0; i<(MAX_TX_QUEUE -1); i++) {
    if (txQueue[i].size == 0) {  // search for a free slot
      idx = i;
      break;
    }
  }
  if (idx == MAX_TX_QUEUE) {  // no free slot, overwrite oldest packet
    idx = 0;
    for (uint8_t i=1; i<(MAX_TX_QUEUE -1); i++) {
      if (txQueue[idx].packetTime < txQueue[i].packetTime) idx = i;
    }
  }
  txQueue[idx].size = size;
  MSG("[INF](id=0x%08X) enQueue Index=%i size=%i", h->id, idx, size);
  txQueue[idx].packetTime = (uint32_t)RtcGetTimerValue();
  MSG(" Time=%i\n",txQueue[idx].packetTime); //MSG("\n");
  memcpy(&txQueue[idx].buf[0], buf, size);
  //MSG(" Copy done!\n");
  txQueueHasPacket = true;
}

uint8_t poptxQueue(void) {
  uint8_t idx = MAX_TX_QUEUE;
  for (uint8_t i=0 ;i < (MAX_TX_QUEUE -1); i++ ){
    if (txQueue[i].size != 0) {
      idx = i;
      break;
    }
  }
  if (idx == MAX_TX_QUEUE) { // empty Queue
    return 0; 
  }
  for (uint8_t i=idx; i<(MAX_TX_QUEUE -1); i++) {
    if ( (txQueue[i].size != 0) && (txQueue[idx].packetTime < txQueue[i].packetTime) ) idx = i; // find oldest packet
  }
  //MSG("[INFO]POP index=%i\n", idx);
  return idx;
}

void perhapsSend(uint8_t* buf, size_t size) {
  PacketHeader* h = (PacketHeader *)buf;
  //clearInterrupts();
  //while ( radio.scanChannel() == RADIOLIB_LORA_DETECTED ) delay( (uint32_t)lround( abs(radio.getSNR() + radio.getRSSI() ) ) );
  MSG("[TX] (id=0x%08X) HopLim=%i ... ", h->id, (h->flags & PACKET_FLAGS_HOP_MASK));
  clearInterrupts();
  radio.setPacketSentAction(ISR_setPacketSent);
  state=radio.startTransmit(buf, size);
  if (state == RADIOLIB_ERR_NONE) {
    MSG("OK\n");
  }
  else {
    MSG("failed, ERR = %i - resume RX", state);
    PacketSent=true; // do not halt on error
  }
  
}

static uint8_t bytes[MAX_RHPACKETLEN];

bool perhapsDecode(meshtastic_MeshPacket *p)
{
/*  concurrency::LockGuard g(cryptLock);

    if (config.device.role == meshtastic_Config_DeviceConfig_Role_REPEATER &&
        config.device.rebroadcast_mode == meshtastic_Config_DeviceConfig_RebroadcastMode_ALL_SKIP_DECODING)
        return false;

    if (config.device.rebroadcast_mode == meshtastic_Config_DeviceConfig_RebroadcastMode_KNOWN_ONLY &&
        !nodeDB.getMeshNode(p->from)->has_user) {
        LOG_DEBUG("Node 0x%x not in NodeDB. Rebroadcast mode KNOWN_ONLY will ignore packet\n", p->from);
        return false;
    }
*/
    if (p->which_payload_variant == meshtastic_MeshPacket_decoded_tag)
        return true; // If packet was already decoded just return

    // assert(p->which_payloadVariant == MeshPacket_encrypted_tag);

    // Try to find a channel that works with this hash
/*    for (ChannelIndex chIndex = 0; chIndex < channels.getNumChannels(); chIndex++) {
        // Try to use this hash/channel pair
        if (channels.decryptForHash(chIndex, p->channel)) { */
            // Try to decrypt the packet if we can
            size_t rawSize = p->encrypted.size;
            assert(rawSize <= sizeof(bytes));
            memcpy(bytes, p->encrypted.bytes,
                   rawSize); // we have to copy into a scratch buffer, because these bytes are a union with the decoded protobuf
            crypto->decrypt(p->from, p->id, rawSize, bytes);

            // printBytes("plaintext", bytes, p->encrypted.size);

            // Take those raw bytes and convert them back into a well structured protobuf we can understand
            memset(&p->decoded, 0, sizeof(p->decoded));
            if (!pb_decode_from_bytes(bytes, rawSize, &meshtastic_Data_msg, &p->decoded)) {
                MSG("[ERROR]Invalid protobufs in received mesh packet (bad psk?)!\n");
            } else if (p->decoded.portnum == meshtastic_PortNum_UNKNOWN_APP) {
                MSG("[ERROR]Invalid portnum (bad psk?)!\n");
            } else {
                // parsing was successful
                p->which_payload_variant = meshtastic_MeshPacket_decoded_tag; // change type to decoded
                p->channel = generateHash(0);                                         // change to store the index instead of the hash

                // Decompress if needed. jm
                if (p->decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_COMPRESSED_APP) {
                    // Decompress the payload
                    char compressed_in[meshtastic_Constants_DATA_PAYLOAD_LEN] = {};
                    char decompressed_out[meshtastic_Constants_DATA_PAYLOAD_LEN] = {};
                    int decompressed_len;

                    memcpy(compressed_in, p->decoded.payload.bytes, p->decoded.payload.size);

                    decompressed_len = unishox2_decompress_simple(compressed_in, p->decoded.payload.size, decompressed_out);

                    // LOG_DEBUG("\n\n**\n\nDecompressed length - %d \n", decompressed_len);

                    memcpy(p->decoded.payload.bytes, decompressed_out, decompressed_len);

                    // Switch the port from PortNum_TEXT_MESSAGE_COMPRESSED_APP to PortNum_TEXT_MESSAGE_APP
                    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
                }

                printPacket("", p);
                return true;
            }
        //}
    //}

    MSG("[ERROR]No suitable channel found for decoding, hash was 0x%x!\n", p->channel);
    return false;
}

void printPacket(const char *prefix, const meshtastic_MeshPacket *p) {

  MSG("%s(id=0x%08X fr=0x%.2X to=0x%.2X, WantAck=%s, HopLim=%d Ch=0x%X", prefix, p->id,
                                            p->from, p->to, (p->want_ack)? "YES":"NO", p->hop_limit, p->channel);
  if (p->which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
        auto &s = p->decoded;
        MSG(" Portnum=%d", s.portnum);
        if (s.want_response)
            MSG(" WANTRESP");
        if (s.source != 0)
            MSG(" source=%08x", s.source);
        if (s.dest != 0)
            MSG(" dest=%08x", s.dest);
        if (s.request_id)
            MSG(" requestId=%0x", s.request_id);
        if (p->rx_time != 0)
        MSG(" rxtime=%u", p->rx_time);
        if (p->rx_snr != 0.0)
            MSGFLOAT(" rxSNR=", p->rx_snr);
        if (p->rx_rssi != 0)
            MSG(" rxRSSI=%i", p->rx_rssi);
        if (p->via_mqtt != 0)
            MSG(" via MQTT");
        if (p->priority != 0)
            MSG(" priority=%d", p->priority);

        MSG("\nPayload: "); printVariants(p); 
    } else {
        MSG(" encrypted!\n");
    }
    //MSG("\n");    
}

void printVariants(const meshtastic_MeshPacket *p){

  // Make sure we have a decoded packet
  if (p->which_payload_variant !=  meshtastic_MeshPacket_decoded_tag) 
    return;

  auto &mp = p->decoded;

  // TEXT MESSAGE
  // /modules/TextMessageModule.cpp
  if (mp.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP){
    MSG("\"%.*s\"\n", mp.payload.size, mp.payload.bytes);
    return;
  }

  // POSITION MESSAGE
  // /modules/PositionModule.cpp
  if (mp.portnum == meshtastic_PortNum_POSITION_APP){
    MSG("Position ");
    meshtastic_Position pos;
    if (!pb_decode_from_bytes(mp.payload.bytes, mp.payload.size, &meshtastic_Position_msg, &pos)) {
      MSG("*** Error ***\n");
      return;
    }
    // Log packet size and data fields
    MSG("node=%08x l=%d latI=%d lonI=%d msl=%d hae=%d geo=%d pdop=%d hdop=%d vdop=%d siv=%d fxq=%d fxt=%d pts=%d "
             "time=%d\n",
             p->from, mp.payload.size, pos.latitude_i, pos.longitude_i, pos.altitude, pos.altitude_hae,
             pos.altitude_geoidal_separation, pos.PDOP, pos.HDOP, pos.VDOP, pos.sats_in_view, pos.fix_quality, pos.fix_type, pos.timestamp,
             pos.time);
    return;
  }
  
  // NODEINFO MESSAGE
  // /modules/NodeInfoModule.cpp
  if (mp.portnum == meshtastic_PortNum_NODEINFO_APP){
    MSG("Node Info: ");
    meshtastic_User user;
    if (!pb_decode_from_bytes(mp.payload.bytes, mp.payload.size, &meshtastic_User_msg, &user)) {
      MSG("*** Error ***\n");
      return;
    }
    MSG("id:%s short: %s long: %s licensed: %s ",user.id, user.short_name, user.long_name, (user.is_licensed)?"YES":"NO");
    MSG("MAC:");
    for (uint8_t i=0; i<sizeof(user.macaddr); i++) { 
      MSG("%0X",user.macaddr[i]);
    }
    MSG(" HW model: %i role: %i\n", (uint8_t)user.hw_model, (uint8_t)user.role);
    return;
  }
    
  

  // TELEMETRY
  if (mp.portnum == meshtastic_PortNum_TELEMETRY_APP){
    MSG("Telemetry ");
    meshtastic_Telemetry telemetry;
    meshtastic_Telemetry *t = &telemetry;
    if (!pb_decode_from_bytes(mp.payload.bytes, mp.payload.size, &meshtastic_Telemetry_msg, &telemetry)) {
      MSG("*** Error ***\n");
      return;
    }
    // /modules/Telemetry/PowerTelemetry.cpp
    if (t->which_variant == meshtastic_Telemetry_power_metrics_tag) {
        MSG("(Power Metrics): ");
        MSGFLOAT(  "ch1_voltage=", t->variant.power_metrics.ch1_voltage);
        MSGFLOAT(", ch1_current=", t->variant.power_metrics.ch1_current);
        MSGFLOAT(", ch2_voltage=", t->variant.power_metrics.ch2_voltage);
        MSGFLOAT(", ch2_current=", t->variant.power_metrics.ch2_current);
        MSGFLOAT(", ch3_voltage=", t->variant.power_metrics.ch3_voltage);
        MSGFLOAT(", ch3_current=", t->variant.power_metrics.ch3_current);
        MSG("\n" ); 
        return;
    }
    // /modules/Telemetry/DeviceTelemetry.cpp
    if (t->which_variant == meshtastic_Telemetry_device_metrics_tag) {
        MSG("(Device Metrics): ");
        MSGFLOAT(  "air_util_tx=",         t->variant.device_metrics.air_util_tx);
        MSGFLOAT(", channel_utilization=", t->variant.device_metrics.channel_utilization);
        MSGFLOAT(", battery_level=",       t->variant.device_metrics.battery_level);
        MSGFLOAT(", voltage=",             t->variant.device_metrics.voltage);
        MSG("\n" );
        return;
    }
    // /modules/Telemetry/EnvironmentTelemetry.cpp
    if (t->which_variant == meshtastic_Telemetry_environment_metrics_tag) {
        MSG("(Environment Metrics): ");
        MSGFLOAT("barometric_pressure=", t->variant.environment_metrics.barometric_pressure);
        MSGFLOAT(", current=",   t->variant.environment_metrics.current);
        MSGFLOAT(", gas_resistance=",t->variant.environment_metrics.gas_resistance);
        MSGFLOAT(", relative_humidity=",t->variant.environment_metrics.relative_humidity);
        MSGFLOAT(", temperature=", t->variant.environment_metrics.temperature);
        MSGFLOAT(", voltage=", t->variant.environment_metrics.voltage);
        MSG("\n");
        return;
    }
    // /modules/Telemetry/AirQualityTelemetry.cpp
    if (t->which_variant == meshtastic_Telemetry_air_quality_metrics_tag) {
        MSG("Air Quality Metrics:\n");
        MSG( "pm10_standard=%i, pm25_standard=%i, pm100_standard=%i\n", 
                 t->variant.air_quality_metrics.pm10_standard, t->variant.air_quality_metrics.pm25_standard,
                 t->variant.air_quality_metrics.pm100_standard);
        MSG("PM1.0(Environmental)=%i, PM2.5(Environmental)=%i, PM10.0(Environmental)=%i\n",
                 t->variant.air_quality_metrics.pm10_environmental, t->variant.air_quality_metrics.pm25_environmental,
                 t->variant.air_quality_metrics.pm100_environmental);
        return;
    }
  }

  // No known PortNum:
  MSG("\"");
  for (uint32_t i=0; i < mp.payload.size; i++){
    MSG("%X", mp.payload.bytes[i]);
  }
  MSG("\"\n");
  return;
}
