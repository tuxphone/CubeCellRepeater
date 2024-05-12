#include <main.h>

void setup() {
  msgID.clear();
  txQueue.clear();
  #ifndef SILENT
  Serial.begin(115200);
  #endif

  MSG("[INF][CryptoEngine]Initializing ... ");
  memcpy(psk.bytes, mypsk, sizeof(mypsk));
  psk.length = sizeof(psk.bytes);
  crypto->setKey(psk);

  initRegion();       // create regions[] and load myRegion
  applyModemConfig(); // apply lora settings
  radio.setDio1Action(ISR_dio1Action);
  MSG("[INF][SX1262]Starting to listen ...\n");
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
      PacketSent = true;
    }
  }

  if (PacketReceived) {
    PacketReceived = false;
    const size_t length = radio.getPacketLength();
    state = radio.readData(radiobuf, length);
    PacketHeader* h = (PacketHeader *)radiobuf;

    if (state == RADIOLIB_ERR_NONE) {
      const int32_t payloadLen = length - sizeof(PacketHeader);
      if (payloadLen < 0) {
        MSG("[WARN]Not a Meshtastic packet, too short!\n");
        return; // will not repeat
      }
      const uint8_t hop_limit = h->flags & PACKET_FLAGS_HOP_MASK;
      // do not repeat if id is known or hop limit is zero
      repeatPacket =  msgID.add(h->id); 
      if (hop_limit == 0) repeatPacket = false;
      MSG("\n[NEW](id=0x%08X) (HopLim %d) ", h->id, hop_limit);
      if ( !repeatPacket ){
        MSG("no repeat!\n");
      } 
      else {
        h->flags -= 1; // decrease hop limit by 1
        txQueue.add(radiobuf, length);
      }
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      MSG(" [ERROR]CRC error!\n");
    } else {
      MSG(" [ERROR]Receive failed, code: %i!\n", state);
    }
  }

  while (txQueue.hasPackets) {
    p = txQueue.pop();
    // if we pop a Nullpointer (empty queue), hasPackets will be false now 
    if (txQueue.hasPackets) {
      TimerTime_t now = systime; //RtcGetTimerValue();
      //try to decode the packet and print it
      uint32_t wait = maxPacketTimeMsec + abs( radio.getRSSI() * radio.getSNR() ) + random(0, 3*slotTimeMsec);
      MSG("[INF]wait %i ms before TX\n", wait);
      while ( (now + wait) > systime ) {
        // while waiting, we still are in receive mode
        if (dio1) {
          MSG("[INF]New packet, no TX\n");
          return; // new packet arrived, return to handle it
        }
        delay(5);
      }
      // last check before actually sending the packet
      radio.startChannelScan(); // RadioLib 6.5 overrides all user params
      // CAD will activate dio1 at activity or timeout
      MCU_deepsleep();
      if (radio.getChannelScanResult() == RADIOLIB_LORA_DETECTED) {
        dio1 = false;
        startReceive();
        MSG("[INF]Lora activity, no TX\n");
        return;
      }
      dio1 = false;
      if (perhapsSend(&p->buf[0], p->size) ) {
        perhapsDecode(&p->buf[0], p->size); 
        PacketSent = true;
        // mark packet as "deleted"
        p->size = 0;
      } else {
        // Could not send, resume receiving
        PacketSent = true;
      }
    }
  
    if (PacketSent) {
      PacketSent = false;
      radio.finishTransmit();
      dio1 = false;
      startReceive();
    }
  }

  #ifndef SILENT
  // wait for serial output to conplete
  delay(10);
  #endif

  MCU_deepsleep();
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
  MSG("[RX]Start receiving ...\n");
  while (RADIOLIB_ERR_NONE != radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF, RADIOLIB_SX126X_IRQ_RX_DEFAULT , RADIOLIB_SX126X_IRQ_RX_DONE, 0) )
  {
   MSG("\n[ERROR][SX1262] startReceive() failed, code: %i\n", state);
   delay(1000); 
  }
}

bool perhapsSend(uint8_t* buf, size_t size) {
  PacketHeader* h = (PacketHeader *)buf;
  MSG("\n[TX](id=0x%08X) HopLim=%i  ", h->id, (h->flags & PACKET_FLAGS_HOP_MASK));
  state=radio.startTransmit(buf, size);
  if (state == RADIOLIB_ERR_NONE) {
    MSG("starting ... ");
  }
  else {
    MSG("failed, ERR = %i - resume RX\n", state);
    return false;
  }
  delay(10);
  MCU_deepsleep(); // wait for TX to complete, will wake on any DIO1
  state = radio.getIrqStatus();
  (state & RADIOLIB_SX126X_IRQ_TX_DONE) ? MSG("done!\n") : MSG("failed. Returned IRQ=%i\n", state);
  dio1 = false;
  return ( state & RADIOLIB_SX126X_IRQ_TX_DONE );  
}

bool perhapsDecode(uint8_t* buf, size_t size) {
// modified code, (c) Meshtastic https://github.com/meshtastic/firmware
  PacketHeader* h = (PacketHeader *)buf;
  const int32_t len = size - sizeof(PacketHeader);
  const uint8_t *payload = radiobuf + sizeof(PacketHeader);
  mp.from = h->from;
  mp.to = h->to;
  mp.id = h->id;
  mp.channel = h->channel;
  mp.hop_limit = h->flags & PACKET_FLAGS_HOP_MASK;
  mp.hop_limit += 1;
  mp.want_ack  = h->flags & PACKET_FLAGS_WANT_ACK_MASK;
  mp.via_mqtt  = h->flags & PACKET_FLAGS_VIA_MQTT_MASK;
  mp.rx_snr  = radio.getSNR();
  mp.rx_rssi = lround(radio.getRSSI());
  mp.which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
  mp.encrypted.size = 0;
  static uint8_t scratchbuf[MAX_RHPACKETLEN];
  assert(len <= sizeof(scratchbuf));
  // we have to copy into a scratch buffer, because mp.encrypted is a union with the decoded protobuf
  memcpy(scratchbuf, buf + sizeof(PacketHeader), len); 
  crypto->decrypt(mp.from, mp.id, len, scratchbuf);
  memset(&mp.decoded, 0, sizeof(mp.decoded));
  if (!pb_decode_from_bytes((const uint8_t*)scratchbuf, len, &meshtastic_Data_msg, &mp.decoded)) {
    MSG("[ERROR]Invalid protobufs in received mesh packet (bad psk?)!\n");
  } else if (mp.decoded.portnum == meshtastic_PortNum_UNKNOWN_APP) {
    MSG("[ERROR]Invalid portnum (bad psk?)!\n");
  } else {
    mp.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    mp.channel = generateHash(0);
    /*
    if (mp.decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_COMPRESSED_APP) {
      char compressed_in[meshtastic_Constants_DATA_PAYLOAD_LEN] = {};
      char decompressed_out[meshtastic_Constants_DATA_PAYLOAD_LEN] = {};
      int decompressed_len;
      memcpy(compressed_in, mp.decoded.payload.bytes, mp.decoded.payload.size);
      decompressed_len = unishox2_decompress_simple(compressed_in, mp.decoded.payload.size, decompressed_out);
      memcpy(mp.decoded.payload.bytes, decompressed_out, decompressed_len);
      mp.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    }
    */
    printPacket();
    return true;
  }
  MSG("[ERROR]No suitable channel found for decoding, hash was 0x%x!\n", mp.channel);
  return false;
}

void printPacket(void) {
  MSG("[INF](id=0x%08X) from=0x%.2X to=0x%.2X, WantAck=%s, HopLim=%d Ch=0x%X", 
        mp.id, mp.from, mp.to, (mp.want_ack)? "YES":"NO", mp.hop_limit, mp.channel);
  if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag) {
    auto &s = mp.decoded;
    MSG(" Portnum=%d", s.portnum);
    if (s.want_response)  MSG(" WANTRESP");
    if (s.source != 0)    MSG(" source=%08x", s.source);
    if (s.dest != 0)      MSG(" dest=%08x", s.dest);
    if (s.request_id)     MSG(" requestId=%0x", s.request_id);
    if (mp.rx_time != 0)  MSG(" rxtime=%u", mp.rx_time);
    if (mp.rx_snr != 0.0) MSGFLOAT(" rxSNR=", mp.rx_snr);
    if (mp.rx_rssi != 0)  MSG(" rxRSSI=%i", mp.rx_rssi);
    if (mp.via_mqtt != 0) MSG(" via MQTT");
    if (mp.priority != 0) MSG(" priority=%d", mp.priority);
    MSG("\nPayload: "); 
    printVariants(); 
  } else {
    MSG(" encrypted!\n");
  }  
}

void printVariants(void){
  // Make sure we have a decoded packet
  if (mp.which_payload_variant !=  meshtastic_MeshPacket_decoded_tag) 
    return;
  auto &d = mp.decoded;

  // TEXT MESSAGE
  // /modules/TextMessageModule.cpp
  if (d.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP){
    MSG("TEXT ");
    MSG("\"%.*s\"\n", d.payload.size, d.payload.bytes);
    return;
  }

  // HARDWARE MESSAGE
  // /modules/RemoteHardwareModule.cpp
  if (d.portnum == meshtastic_PortNum_REMOTE_HARDWARE_APP){
    MSG("GPIO ");
    meshtastic_NodeRemoteHardwarePin pin;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_NodeRemoteHardwarePin_msg, &pin)) {
      MSG("*** Error ***\n");
      return;
    }
    
    return;
  }

  // POSITION MESSAGE
  // /modules/PositionModule.cpp
  if (d.portnum == meshtastic_PortNum_POSITION_APP){
    MSG("POSITION ");
    meshtastic_Position pos;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_Position_msg, &pos)) {
      MSG("*** Error ***\n");
      return;
    }
    // Log packet size and data fields
    MSG("Node=%08X l=%d latI=%d lonI=%d msl=%d hae=%d geo=%d pdop=%d hdop=%d vdop=%d siv=%d fxq=%d fxt=%d pts=%d "
             "time=%d\n",
             mp.from, d.payload.size, pos.latitude_i, pos.longitude_i, pos.altitude, pos.altitude_hae,
             pos.altitude_geoidal_separation, pos.PDOP, pos.HDOP, pos.VDOP, pos.sats_in_view, pos.fix_quality, pos.fix_type, pos.timestamp,
             pos.time);
    return;
  }
  
  // NODEINFO MESSAGE
  // /modules/NodeInfoModule.cpp
  if (d.portnum == meshtastic_PortNum_NODEINFO_APP){
    MSG("NODE INFO ");
    meshtastic_User user;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_User_msg, &user)) {
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

  // ROUTING MESSAGE
  // /modules/RoutingModule.cpp
   if (d.portnum == meshtastic_PortNum_ROUTING_APP){
    MSG("ROUTING \n");
    /*
    meshtastic_Routing r;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_Routing_msg, &r)) {
      MSG("*** Error ***\n");
      return;
    }
    if (r.which_variant == sizeof(meshtastic_Routing_Error) ) {
      MSG("RoutingError=%i\n", r.error_reason);
    } else {
      MSG("RouteRequest ["); 
      for (uint8_t i=0; i < r.route_request.route_count; i++) MSG("0x%X ", r.route_request.route[i]);
      MSG("] ");
      MSG("RouteReply ["); 
      for (uint8_t i=0; i < r.route_reply.route_count; i++) MSG("0x%X ", r.route_reply.route[i]);
      MSG("]\n");

    }
    */
    return;
  }

  // TELEMETRY MESSAGE
  if (d.portnum == meshtastic_PortNum_TELEMETRY_APP){
    MSG("TELEMETRY");
    meshtastic_Telemetry telemetry;
    meshtastic_Telemetry *t = &telemetry;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_Telemetry_msg, &telemetry)) {
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
        MSGFLOAT(", rel_humidity=",t->variant.environment_metrics.relative_humidity);
        MSGFLOAT(", temp=", t->variant.environment_metrics.temperature);
        MSGFLOAT(", volt=", t->variant.environment_metrics.voltage);
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

  // TRACEROUTE MESSAGE
  // /modules/TraceRouteModule.cpp
  if (d.portnum == meshtastic_PortNum_TRACEROUTE_APP){
    MSG("TRACEROUTE");
    meshtastic_RouteDiscovery route;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_RouteDiscovery_msg, &route)) {
      MSG("*** Error ***\n");
      return;
    }
    MSG("(seen by %i Nodes", route.route_count);
    if (route.route_count > 0) {
      for (uint8_t i=0; i < route.route_count; i++) {
        MSG(" %08X", route.route[i]);
      }
    }
    MSG(")\n");
    return;
  }

  // NEIGHBORINFO MESSAGE
  // /modules/NeighborInfoModule.cpp
  if (d.portnum == meshtastic_PortNum_NEIGHBORINFO_APP){
    MSG("NEIGHBORINFO ");
    meshtastic_NeighborInfo np;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_NeighborInfo_msg, &np)) {
      MSG("*** Error ***\n");
      return;
    }
    MSG("(last sent by 0x%x) Number of neighbors=%d\n", np.last_sent_by_id, np.neighbors_count);
    for (uint8_t i = 0; i < np.neighbors_count; i++) {
        MSG("[0x%X, ", np.neighbors[i].node_id);
        MSGFLOAT("snr=%.2f]\n", np.neighbors[i].snr);
    }
    return;
  }

  // ATAK PLUGIN MESSAGE
  // /modules/AtakPluginModule.cpp
  if (d.portnum == meshtastic_PortNum_ATAK_PLUGIN){
    MSG("ATAK \n");    
    return;
  }

  // No known PortNum:
  MSG("\"");
  for (uint32_t i=0; i < d.payload.size; i++){
    MSG("%X", d.payload.bytes[i]);
  }
  MSG("\"\n");
  return;
}

// PacketQueueClass Definitions

void PacketQueueClass::add(uint8_t* buf, size_t size) {
  uint8_t idx = MAX_TX_QUEUE;
  for (uint8_t i=0; i<(MAX_TX_QUEUE -1); i++) {
    if (Queue[i].size == 0) {  // search for a free slot
      idx = i;
      break;
    }
  }
  if (idx == MAX_TX_QUEUE) {  // no free slot, overwrite oldest packet
    for (uint8_t i=1; i<(MAX_TX_QUEUE -1); i++) {
      if (Queue[idx].packetTime < Queue[i].packetTime) idx = i;
    }
  }
  Queue[idx].size = size;
  Queue[idx].packetTime = (uint32_t)RtcGetTimerValue();
  MSG("enQueue Index=%i Size=%i\n", idx, Queue[idx].size);
  memcpy(Queue[idx].buf, buf, size);
  hasPackets = true;
}

Packet_t* PacketQueueClass::pop(void) {
  uint8_t idx = MAX_TX_QUEUE;
  for (uint8_t i=0 ;i < (MAX_TX_QUEUE -1); i++ ){
    if (Queue[i].size != 0) { // first not empty entry
      idx = i;
      break;
    }
  }
  if (idx == MAX_TX_QUEUE) { // empty Queue
    hasPackets = false;
    return (Packet_t*)NULL;
  }
  for (uint8_t i=idx; i<(MAX_TX_QUEUE -1); i++) {
    if ( (Queue[i].size != 0) && (Queue[idx].packetTime < Queue[i].packetTime) ) idx = i; // find oldest packet
  }
  return &Queue[idx];
}

void PacketQueueClass::clear(void) {
  for (uint8_t i = 0; i<(MAX_TX_QUEUE - 1); i++) {
    Queue[i].size = 0; // mark as "deleted"
  }
}

// idStoreClass Definitions

bool idStoreClass::add(uint32_t id) {
  uint8_t idx = 0;
  for (uint8_t i = MAX_ID_LIST; i > 0; i--) {
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