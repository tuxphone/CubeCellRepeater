#include <main.h>

void setup() {
  #ifndef SILENT
  Serial.begin(115200);
  delay(5000);
  #endif
  init_signalize();
  msgID.clear();
  txQueue.clear();
  NodeDB.clear();
  
  MSG("[INF][CryptoEngine]Initializing ... ");
  memcpy(psk.bytes, mypsk, sizeof(mypsk));
  psk.length = sizeof(mypsk);
  crypto->setKey(psk);

  initRegion();       // create regions[] and load myRegion
  applyModemConfig(); // apply lora settings
  radio.setDio1Action(ISR_dio1Action);
  MSG("[INF][SX1262]Starting to listen ...\n\r");
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
      MSG("[ERR]Received packet length = 0!\n\r");
      return; // we are still in receive mode
    }
    err = radio.readData(pck.buf, pck.size);
    PacketHeader* h = (PacketHeader *)pck.buf;
    snr = radio.getSNR();
    if (err == RADIOLIB_ERR_NONE) {
      const int32_t payloadLen = pck.size - sizeof(PacketHeader);
      if (payloadLen < 0) {
        MSG("[WARN]Not a Meshtastic packet, too short!\n\r");
        return; // will not repeat, continue receiving
      }
      const uint8_t hop_limit = h->flags & PACKET_FLAGS_HOP_MASK;
      MSG("[NEW](id=0x%08X) (HopLim %d) ", h->id, hop_limit);
      repeatPacket = msgID.add(h->id);
      // print new packets only not repeated due to HopLim 0
      if ((repeatPacket) && (hop_limit==0)) {
        MSG("\n\r");
        #ifndef SILENT      
        perhapsDecode(&pck);
        MSG("\n\r");
        #endif
      }
      if (hop_limit == 0) repeatPacket = false;
      // do not repeat if id is known or hop limit is zero
      if ( !repeatPacket ){
        MSG(" !!! no repeat !!!\n\r");
      } 
      else {
        h->flags -= 1; // decrease hop limit by 1
        txQueue.add(&pck);
      }
    } else if (err == RADIOLIB_ERR_CRC_MISMATCH) {
      MSG(" [ERROR]CRC error!\n\r");
    } else {
      MSG(" [ERROR]Receive failed, code: %i!\n\r", err);
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

      MSG("[INF]wait %i ms before TX\n\r", wait);
      activeReceiveStart = 0;
      while ( ((now + wait) > millis() ) || ( isActivelyReceiving() ) ) {
        // while waiting, we still are in receive mode
        if (dio1) {
          MSG("[INF]New packet, no TX\n\r");
          return; // new packet arrived, return to handle it
        }
        delay(5);
      }
      // drop packet if we could not send it in 1 minute
      if ( (p->packetTime + 60*1000) < millis() ) {
        p = NULL;
        PacketSent = true;
        MSG("[INF] TX aborted, could not send packet in 1 minute\n\r");
        return;
      }
      if (perhapsSend(p) ) {
        // packet successfully sent
        // try to decode the packet and print it
        #ifndef SILENT
        perhapsDecode(p);
        MSG("\n\r");
        #endif
        PacketSent = true;
        p = NULL; 
      } else {      
        // resume receiving if we could not send
        PacketSent = true;
      }
    } else {
      MSG("[ERR]Tried to send empty package! TxQueue count=%i\n\r", txQueue.Count);
      PacketSent = true;
    }
  }

  if (PacketSent) {
      PacketSent = false;
      startReceive();
  }

  if (txQueue.Count == 0) {
    #ifndef SILENT
    // wait for serial output to conplete
    delay(10);
    #endif
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
  if (err != RADIOLIB_ERR_NONE) MSG("[ERROR]Radiolib error %d when attempting SX126X startReceiveDutyCycleAuto!\n\r", err);
  assert(err == RADIOLIB_ERR_NONE);
  //radio.setDio1Action(ISR_dio1Action);
  isReceiving = true;
  MSG("[RX] ... \n\r");
}

// send packet (blocking), MCU sleeps while waiting for TX DONE
bool perhapsSend(Packet_t* p) {
  if (!CC_MONITOR_ONLY) {
    if (p->size > RADIOLIB_SX126X_MAX_PACKET_LENGTH) {
      MSG("\n\r[INF]Packet size is %i! Reducing to %i. Sending ...", p->size, RADIOLIB_SX126X_MAX_PACKET_LENGTH);
      p->size = RADIOLIB_SX126X_MAX_PACKET_LENGTH;
    }
    // clear irq status, standby()
    radio.finishTransmit();
    dio1 = false;
    PacketHeader* h = (PacketHeader *)p->buf;
    MSG("[TX] (id=0x%08X) HopLim=%i  ", h->id, (h->flags & PACKET_FLAGS_HOP_MASK));
    err=radio.startTransmit(p->buf, p->size);
    isReceiving = false;
    if (err == RADIOLIB_ERR_NONE) {
      MSG("starting ... ");
    } else {
      MSG("failed, ERR = %i - resume RX\n\r", err);
      return false;
    }
    signalizeTX_ON();
    delay(10);
    MCU_deepsleep(); // wait for TX to complete, will wake on any DIO1
    err = radio.getIrqStatus();
    (err & RADIOLIB_SX126X_IRQ_TX_DONE) ? MSG("done!\n\r") : MSG("failed. Returned IRQ=%i\n\r", err);
    dio1 = false;
    radio.finishTransmit(); 
    return ( err & RADIOLIB_SX126X_IRQ_TX_DONE );
  } else {
    MSG("[TX]**Monitor only, no TX**\n\r");
    return true;
  }
}

bool perhapsDecode(Packet_t* p) {
// modified code, (c) Meshtastic https://github.com/meshtastic/firmware
  PacketHeader* h = (PacketHeader *)p->buf;
  const int32_t len = p->size - sizeof(PacketHeader);
  const uint8_t *payload = p->buf + sizeof(PacketHeader);
  mp.from = h->from;
  mp.to = h->to;
  mp.id = h->id;
  mp.channel = h->channel;
  mp.hop_limit = h->flags & PACKET_FLAGS_HOP_MASK;
  mp.hop_limit += 1;
  mp.want_ack  = h->flags & PACKET_FLAGS_WANT_ACK_MASK;
  mp.via_mqtt  = h->flags & PACKET_FLAGS_VIA_MQTT_MASK;
  mp.rx_snr  = snr;
  mp.rx_rssi = lround(radio.getRSSI());
  mp.which_payload_variant = meshtastic_MeshPacket_encrypted_tag;
  mp.encrypted.size = 0;
  static uint8_t scratchbuf[MAX_RHPACKETLEN];
  assert(len <= sizeof(scratchbuf));
  // we have to copy into a scratch buffer, because mp.encrypted is a union with the decoded protobuf
  memcpy(scratchbuf, p->buf + sizeof(PacketHeader), len); 
  crypto->decrypt(mp.from, mp.id, len, scratchbuf);
  memset(&mp.decoded, 0, sizeof(mp.decoded));
  if (!pb_decode_from_bytes((const uint8_t*)scratchbuf, len, &meshtastic_Data_msg, &mp.decoded)) {
    MSG("[ERROR]Invalid protobufs in received mesh packet (bad psk?)!\n\r");
  } else if (mp.decoded.portnum == meshtastic_PortNum_UNKNOWN_APP) {
    MSG("[ERROR]Invalid portnum (bad psk?)!\n\r");
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
    memset(theNode.user.short_name, 0, sizeof(theNode.user.short_name));
    memset(theNode.user.long_name, 0, sizeof(theNode.user.long_name));
    theNode.num = mp.from;
    theNode.has_user = false;
    theNode.has_position = false;
    theNode.has_device_metrics = false;
    printPacket();
    return true;
  }
  MSG("[ERROR]No suitable channel found for decoding, hash was 0x%x!\n\r", mp.channel);
  return false;
}

void printPacket(void) {
  MSG("[INF](id=0x%08X) from=0x%.2X(%s) to=0x%.2X(%s), WantAck=%s, HopLim=%d Ch=0x%X", 
        mp.id, mp.from, NodeDB.get(mp.from), mp.to, NodeDB.get(mp.to), (mp.want_ack)? "YES":"NO", mp.hop_limit, mp.channel);
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
    MSG("\n\rPayload: "); 
    printVariants(); 
  } else {
    MSG(" encrypted!\n\r");
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
    MSG("\"%.*s\"\n\r", d.payload.size, d.payload.bytes);
    NodeDB.update(&theNode); // update last heard
    return;
  }

  // HARDWARE MESSAGE
  // /modules/RemoteHardwareModule.cpp
  if (d.portnum == meshtastic_PortNum_REMOTE_HARDWARE_APP){
    MSG("GPIO ");
    meshtastic_NodeRemoteHardwarePin pin;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_NodeRemoteHardwarePin_msg, &pin)) {
      MSG("*** Error ***\n\r");
      return;
    }
    NodeDB.update(&theNode); // update last seen
    return;
  }

  // POSITION MESSAGE
  // /modules/PositionModule.cpp
  if (d.portnum == meshtastic_PortNum_POSITION_APP){
    MSG("POSITION ");
    meshtastic_Position pos;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_Position_msg, &pos)) {
      MSG("*** Error ***\n\r");
      return;
    }
    // Log packet size and data fields
    MSG("Node=%08X(%s) l=%d latI=%d lonI=%d msl=%d hae=%d geo=%d pdop=%d hdop=%d vdop=%d siv=%d fxq=%d fxt=%d pts=%d ",
             mp.from, NodeDB.get(mp.from), d.payload.size, pos.latitude_i, pos.longitude_i, pos.altitude, pos.altitude_hae,
             pos.altitude_geoidal_separation, pos.PDOP, pos.HDOP, pos.VDOP, pos.sats_in_view, pos.fix_quality, pos.fix_type, pos.timestamp
             );
    MSG("time=%04d-%02d-%02d %02d:%02d:%02d\n\r",
           year(pos.time), month(pos.time), day(pos.time),
           hour(pos.time), minute(pos.time), second(pos.time));
    NodeDB.update(&theNode); // update last heard
    return;
  }
  
  // NODEINFO MESSAGE
  // /modules/NodeInfoModule.cpp
  if (d.portnum == meshtastic_PortNum_NODEINFO_APP){
    MSG("NODE INFO ");
    meshtastic_User user;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_User_msg, &user)) {
      MSG("*** Error ***\n\r");
      return;
    }
    MSG("id:%s short: %s long: %s licensed: %s ",user.id, user.short_name, user.long_name, (user.is_licensed)?"YES":"NO");
    MSG("MAC:");
    for (uint8_t i=0; i<sizeof(user.macaddr); i++) { 
      MSG("%0X",user.macaddr[i]);
    }
    MSG(" HW model: %i role: %i\n\r", (uint8_t)user.hw_model, (uint8_t)user.role);
    theNode.has_user = true;
    strncpy(theNode.user.short_name, user.short_name, sizeof(theNode.user.short_name) - 1);
    strncpy(theNode.user.long_name, user.long_name, sizeof(theNode.user.long_name) - 1);
    NodeDB.update(&theNode); // update user info
    return;
  }

  // ROUTING MESSAGE
  // /modules/RoutingModule.cpp
   if (d.portnum == meshtastic_PortNum_ROUTING_APP){
    MSG("ROUTING \n\r");
    meshtastic_Routing r;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_Routing_msg, &r)) {
      MSG("*** Error ***\n\r");
      return;
    }
    if (r.which_variant == sizeof(meshtastic_Routing_Error) ) {
      MSG("RoutingError=%i\n\r", r.error_reason);
    /*  
    } else {
      MSG("RouteRequest ["); 
      for (uint8_t i=0; i < r.route_request.route_count; i++) MSG("0x%X ", r.route_request.route[i]);
      MSG("] ");
      MSG("RouteReply ["); 
      for (uint8_t i=0; i < r.route_reply.route_count; i++) MSG("0x%X ", r.route_reply.route[i]);
      MSG("]\n\r");
    */
    }
    NodeDB.update(&theNode); // update last heard
    return;
  }

  // TELEMETRY MESSAGE
  if (d.portnum == meshtastic_PortNum_TELEMETRY_APP){
    NodeDB.update(&theNode); // update last heard
    MSG("TELEMETRY");
    meshtastic_Telemetry telemetry;
    meshtastic_Telemetry *t = &telemetry;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_Telemetry_msg, &telemetry)) {
      MSG("*** Error ***\n\r");
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
        MSG("\n\r" ); 
        return;
    }
    // /modules/Telemetry/DeviceTelemetry.cpp
    if (t->which_variant == meshtastic_Telemetry_device_metrics_tag) {
        MSG("(Device Metrics): ");
        MSGFLOAT(  "air_util_tx=",         t->variant.device_metrics.air_util_tx);
        MSGFLOAT(", channel_utilization=", t->variant.device_metrics.channel_utilization);
        MSGFLOAT(", battery_level=",       t->variant.device_metrics.battery_level);
        MSGFLOAT(", voltage=",             t->variant.device_metrics.voltage);
        MSG("\n\r" );
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
        MSG("\n\r");
        return;
    }
    // /modules/Telemetry/AirQualityTelemetry.cpp
    if (t->which_variant == meshtastic_Telemetry_air_quality_metrics_tag) {
        MSG("Air Quality Metrics:\n\r");
        MSG( "pm10_standard=%i, pm25_standard=%i, pm100_standard=%i\n\r", 
                 t->variant.air_quality_metrics.pm10_standard, t->variant.air_quality_metrics.pm25_standard,
                 t->variant.air_quality_metrics.pm100_standard);
        MSG("PM1.0(Environmental)=%i, PM2.5(Environmental)=%i, PM10.0(Environmental)=%i\n\r",
                 t->variant.air_quality_metrics.pm10_environmental, t->variant.air_quality_metrics.pm25_environmental,
                 t->variant.air_quality_metrics.pm100_environmental);
        return;
    }
  }

  // TRACEROUTE MESSAGE
  // /modules/TraceRouteModule.cpp
  if (d.portnum == meshtastic_PortNum_TRACEROUTE_APP){
    NodeDB.update(&theNode); // update last heard
    MSG("TRACEROUTE");
    meshtastic_RouteDiscovery route;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_RouteDiscovery_msg, &route)) {
      MSG("*** Error ***\n\r");
      return;
    }
    MSG("(seen by %i Nodes", route.route_count);
    if (route.route_count > 0) {
      for (uint8_t i=0; i < route.route_count; i++) {
        MSG(" %08X", route.route[i]);
      }
    }
    MSG(")\n\r");
    return;
  }

  // NEIGHBORINFO MESSAGE
  // /modules/NeighborInfoModule.cpp
  if (d.portnum == meshtastic_PortNum_NEIGHBORINFO_APP){
    NodeDB.update(&theNode); // update last heard
    MSG("NEIGHBORINFO ");
    meshtastic_NeighborInfo np;
    if (!pb_decode_from_bytes(d.payload.bytes, d.payload.size, &meshtastic_NeighborInfo_msg, &np)) {
      MSG("*** Error ***\n\r");
      return;
    }
    MSG("(last sent by %8X) Number of neighbors=%d\n\r", np.last_sent_by_id, np.neighbors_count);
    for (uint8_t i = 0; i < np.neighbors_count; i++) {
        MSG("[%8X, ", np.neighbors[i].node_id);
        MSGFLOAT("snr=", np.neighbors[i].snr);
        MSG("]\n\r");
    }
    return;
  }

  // ATAK PLUGIN MESSAGE
  // /modules/AtakPluginModule.cpp
  if (d.portnum == meshtastic_PortNum_ATAK_PLUGIN){
    NodeDB.update(&theNode); // update last heard
    MSG("ATAK \n\r");    
    return;
  }

  // No known PortNum:
  MSG("UNKNOWN #%i \"", d.portnum);
  NodeDB.update(&theNode); // update last heard
  for (uint32_t i=0; i < d.payload.size; i++){
    MSG("%X", d.payload.bytes[i]);
  }
  MSG("\"\n\r");
  return;
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
  MSG("enQueue Index=%i Size=%i\n\r", idx, Queue[idx].size);
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

// NodeStoreClass Definitions

void NodeStoreClass::clear(void) {
  for (uint8_t i=1; i < (MAX_NODE_LIST - 1); i++) {
    this->nodeDB[i].has_user = false;
    memset(this->nodeDB[i].user.short_name, 0, 5);
    this->nodeDB[i].num = 0;
  }
  this->nodeDB[0].has_user = true;
  this->nodeDB[0].num = 0xFFFFFFFF;
  this->nodeDB[0].last_heard = 0xFFFFFFFF;
  strncpy(this->nodeDB[0].user.short_name, "ALL", 3);
  strncpy(this->nodeDB[0].user.long_name, "Channel Broadcast", 17);
}
    
// returns false if the Node is already known
void NodeStoreClass::update(meshtastic_NodeInfo* Node) {
  uint8_t idx = MAX_NODE_LIST;
  // check if it is a known Node Number
  for (uint8_t i=0; i < (MAX_NODE_LIST - 1); i++) {
    if (this->nodeDB[i].has_user) {
      if (this->nodeDB[i].num == Node->num) {
        // known Node
        idx = i;
        break;
      }
    }
  }
  if (idx < MAX_NODE_LIST) {
    this->nodeDB[idx].last_heard = millis();
    if (Node->has_user) {
      strncpy(this->nodeDB[idx].user.short_name, Node->user.short_name, sizeof(this->nodeDB[idx].user.short_name) - 1);
      strncpy(this->nodeDB[idx].user.long_name,  Node->user.long_name,  sizeof(this->nodeDB[idx].user.long_name)  - 1);
    }
  } else { // new entry
    this->add(Node);
  }
}

// get the short name of the node. "" if unknown.
char* NodeStoreClass::get(uint32_t num) {
  for (uint8_t i=0; i < MAX_NODE_LIST; i++) {
    if (this->nodeDB[i].has_user) {
      if (this->nodeDB[i].num == num) {
        return this->nodeDB[i].user.short_name;
      }
    }
  }
  return "";
}

void NodeStoreClass::add(meshtastic_NodeInfo* Node) {
  Node->last_heard = millis();
  uint8_t idx = 0;
  for (uint8_t i=1; i < (MAX_NODE_LIST - 1); i++) {
    if (!this->nodeDB[i].has_user) {
      idx = i;
      break; // found unused slot
    } else {
      // find oldest node (longest time not heard from), overwrite if necessary
      if (this->nodeDB[i].last_heard < this->nodeDB[idx].last_heard) {
        idx = i;
      }
    }
  }
  //assert(idx != 0);
  this->nodeDB[idx].num = Node->num;;
  this->nodeDB[idx].has_user = true;
  this->nodeDB[idx].last_heard = Node->last_heard;
  if (Node->has_user) {
    strncpy(this->nodeDB[idx].user.short_name, Node->user.short_name, sizeof(this->nodeDB[idx].user.short_name) - 1);
    strncpy(this->nodeDB[idx].user.long_name,  Node->user.long_name,  sizeof(this->nodeDB[idx].user.long_name)  - 1);
  }
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
    pixels.setPixelColor(0, pixels.Color(0, 32, 0));
    pixels.show();
#endif
#ifdef CC_SIGNAL_GPIO13
    digitalWrite(GPIO13, HIGH);
#endif
}

void signalizeTX_ON(void){
#ifdef CC_SIGNAL_NEOPIXEL
    pixels.setPixelColor(0, pixels.Color(32, 0, 0));
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