/*
  MotorControllerBridge.ino
  -------------------------
  ESP32 that:
  - receives text commands over Serial (USB)
  - sends binary protocol packets to MotorNode over ESP-NOW
  - receives ACK/STATUS/CONFIG from MotorNode and prints JSON lines over Serial

  Works with ESP32 Arduino core 3.x (new esp_now recv callback signature).

  NEW in this version:
  - CUR <irun> <ihold> [FLAGS|SAVE]
      Sends CMD_SET_CONFIG with TWO TLVs:
        P_TMC_IRUN  (0x0013, VT_U8)
        P_TMC_IHOLD (0x0014, VT_U8)
      Examples:
        CUR 20 8
        CUR 20 8 SAVE
        CUR 20 8 5        (flags=5 -> APPLY_NOW(1) | SAVE_AFTER_APPLY(4))
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ----------------- Protocol (match your MotorNode) -----------------
namespace Proto {
  static constexpr uint16_t MAGIC = 0xA55A;
  static constexpr uint8_t  VERSION = 1;

  enum MsgType : uint8_t {
    CMD_PING          = 0x01,
    CMD_ENABLE        = 0x02,
    CMD_STOP          = 0x08,
    CMD_MOVE_ABS_DEG  = 0x20,
    CMD_VELOCITY_DPS  = 0x21,
    CMD_HOME_START    = 0x30,

    CMD_GET_CONFIG    = 0x41,
    RSP_CONFIG        = 0xC1,
    CMD_SET_CONFIG    = 0x42,
    CMD_SAVE_CONFIG   = 0x43,

    RSP_ACK           = 0x80,
    RSP_STATUS        = 0x81,
  };

  enum Flags : uint8_t {
    FLAG_ACK_REQ  = 1 << 0,
    FLAG_IS_ACK   = 1 << 1,
    FLAG_IS_EVENT = 1 << 2,
  };

  enum SetFlags : uint8_t {
    SET_APPLY_NOW        = 1 << 0,
    SET_APPLY_WHEN_IDLE  = 1 << 1,
    SET_SAVE_AFTER_APPLY = 1 << 2,
    SET_STRICT           = 1 << 3
  };

  enum ValueType : uint8_t {
    VT_U8   = 1,
    VT_U16  = 2,
    VT_U32  = 3,
    VT_I16  = 4,
    VT_I32  = 5,
    VT_BOOL = 6,
  };

  // These must match your MotorNode ConfigDefs.h IDs
  static constexpr uint16_t P_TMC_IRUN  = 0x0013; // u8 (0..31)
  static constexpr uint16_t P_TMC_IHOLD = 0x0014; // u8 (0..31)

  #pragma pack(push, 1)
  struct Header {
    uint16_t magic;
    uint8_t  version;
    uint8_t  msg_type;
    uint8_t  flags;
    uint8_t  seq;
    uint8_t  node_id;
    uint8_t  payload_len;
    uint16_t crc16;
  };

  struct AckPayload {
    uint8_t ack_seq;
    uint8_t result;
  };

  struct CmdEnablePayload { uint8_t enable; };
  struct CmdMoveAbsDegPayload { int32_t target_deg_q100; };
  struct CmdVelocityDpsPayload { int32_t vel_dps_q100; };

  struct CmdGetConfigPayload {
    uint8_t list_len; // if 0 -> all, else followed by uint16 ids
  };

  struct StatusPayload {
    uint32_t ms;
    uint8_t  state;
    uint8_t  motorEnabled;
    uint8_t  moving;
    uint8_t  lossActive;

    int32_t  pos_steps;
    int32_t  target_steps;
    int32_t  home_steps;

    int32_t  enc_deg_q100;
    int32_t  enc_abs_deg_q100;
    int32_t  error_steps;

    uint16_t faultCode;
    int16_t  temp_c_q10;
    uint32_t config_revision;
  };

  struct TLV {
    uint16_t param_id;
    uint8_t  type;
    uint8_t  len;
    // value[len] follows
  };
  #pragma pack(pop)

  uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t seed = 0xFFFF) {
    uint16_t crc = seed;
    for (size_t i = 0; i < len; i++) {
      crc ^= (uint16_t)data[i] << 8;
      for (int b = 0; b < 8; b++) {
        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
        else crc <<= 1;
      }
    }
    return crc;
  }
} // namespace Proto

// ----------------- Bridge state -----------------
static uint8_t gPeerMac[6] = {0};
static bool    gHasPeer = false;
static uint8_t gChannel = 6;
static uint8_t gSeqTx = 1;

// Small helpers
static bool parseMac(const String& s, uint8_t out[6]) {
  int vals[6];
  if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x",
             &vals[0], &vals[1], &vals[2], &vals[3], &vals[4], &vals[5]) != 6) return false;
  for (int i=0;i<6;i++) out[i] = (uint8_t)vals[i];
  return true;
}

static String macToString(const uint8_t mac[6]) {
  char b[24];
  snprintf(b, sizeof(b), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(b);
}

static bool ensurePeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = gChannel;   // explicit channel
  p.encrypt = false;
  return esp_now_add_peer(&p) == ESP_OK;
}

static bool buildAndSend(const uint8_t mac[6], uint8_t msgType, uint8_t flags,
                         const uint8_t* payload, uint8_t payloadLen) {
  Proto::Header h{};
  h.magic = Proto::MAGIC;
  h.version = Proto::VERSION;
  h.msg_type = msgType;
  h.flags = flags;
  h.seq = gSeqTx++;
  h.node_id = 0;
  h.payload_len = payloadLen;
  h.crc16 = 0;

  static uint8_t buf[300];
  if (sizeof(Proto::Header) + payloadLen > sizeof(buf)) return false;

  memcpy(buf, &h, sizeof(h));
  if (payloadLen) memcpy(buf + sizeof(h), payload, payloadLen);

  Proto::Header* ph = (Proto::Header*)buf;
  Proto::Header tmp = *ph;
  tmp.crc16 = 0;

  uint16_t crc = Proto::crc16_ccitt((uint8_t*)&tmp, sizeof(tmp));
  if (payloadLen) crc = Proto::crc16_ccitt(buf + sizeof(Proto::Header), payloadLen, crc);
  ph->crc16 = crc;

  if (!ensurePeer(mac)) return false;
  return esp_now_send(mac, buf, sizeof(Proto::Header) + payloadLen) == ESP_OK;
}

// Build CMD_SET_CONFIG payload with 1 or more TLVs.
// Payload layout: [setFlags u8] [TLV][value] [TLV][value] ...
static uint8_t buildSetCfg2_U8(uint8_t* out, size_t outMax,
                              uint8_t setFlags,
                              uint16_t id1, uint8_t v1,
                              uint16_t id2, uint8_t v2) {
  if (outMax < 1 + (sizeof(Proto::TLV) + 1) * 2) return 0;
  size_t w = 0;
  out[w++] = setFlags;

  Proto::TLV t1{ id1, (uint8_t)Proto::VT_U8, 1 };
  memcpy(out + w, &t1, sizeof(t1)); w += sizeof(t1);
  out[w++] = v1;

  Proto::TLV t2{ id2, (uint8_t)Proto::VT_U8, 1 };
  memcpy(out + w, &t2, sizeof(t2)); w += sizeof(t2);
  out[w++] = v2;

  return (uint8_t)w;
}

// ----------------- RX handling from MotorNode -----------------
static void printJsonEscaped(const char* s) {
  while (*s) {
    char c = *s++;
    if (c == '\"') Serial.print("\\\"");
    else if (c == '\\') Serial.print("\\\\");
    else if ((uint8_t)c < 0x20) Serial.print(' ');
    else Serial.print(c);
  }
}

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || len < (int)sizeof(Proto::Header)) return;
  const uint8_t* src = info->src_addr;

  Proto::Header hdr{};
  memcpy(&hdr, data, sizeof(hdr));
  if (hdr.magic != Proto::MAGIC || hdr.version != Proto::VERSION) return;
  if ((int)(sizeof(Proto::Header) + hdr.payload_len) != len) return;

  Proto::Header tmp = hdr;
  tmp.crc16 = 0;
  uint16_t crc = Proto::crc16_ccitt((const uint8_t*)&tmp, sizeof(tmp));
  if (hdr.payload_len) crc = Proto::crc16_ccitt(data + sizeof(Proto::Header), hdr.payload_len, crc);
  if (crc != hdr.crc16) return;

  const uint8_t* payload = data + sizeof(Proto::Header);

  // Print JSON lines for Python GUI
  Serial.print("{\"from\":\""); Serial.print(macToString(src)); Serial.print("\",");
  Serial.print("\"msg\":"); Serial.print(hdr.msg_type); Serial.print(",");
  Serial.print("\"seq\":"); Serial.print(hdr.seq); Serial.print(",");

  if (hdr.msg_type == Proto::RSP_ACK && hdr.payload_len == sizeof(Proto::AckPayload)) {
    auto* a = (const Proto::AckPayload*)payload;
    Serial.print("\"type\":\"ack\",");
    Serial.print("\"ack_seq\":"); Serial.print(a->ack_seq); Serial.print(",");
    Serial.print("\"result\":"); Serial.print(a->result);
    Serial.println("}");
    return;
  }

  if (hdr.msg_type == Proto::RSP_STATUS && hdr.payload_len == sizeof(Proto::StatusPayload)) {
    auto* s = (const Proto::StatusPayload*)payload;
    Serial.print("\"type\":\"status\",");
    Serial.print("\"ms\":"); Serial.print(s->ms); Serial.print(",");
    Serial.print("\"state\":"); Serial.print(s->state); Serial.print(",");
    Serial.print("\"motorEnabled\":"); Serial.print(s->motorEnabled); Serial.print(",");
    Serial.print("\"moving\":"); Serial.print(s->moving); Serial.print(",");
    Serial.print("\"lossActive\":"); Serial.print(s->lossActive); Serial.print(",");
    Serial.print("\"pos_steps\":"); Serial.print(s->pos_steps); Serial.print(",");
    Serial.print("\"target_steps\":"); Serial.print(s->target_steps); Serial.print(",");
    Serial.print("\"home_steps\":"); Serial.print(s->home_steps); Serial.print(",");
    Serial.print("\"enc_deg_q100\":"); Serial.print(s->enc_deg_q100); Serial.print(",");
    Serial.print("\"enc_abs_deg_q100\":"); Serial.print(s->enc_abs_deg_q100); Serial.print(",");
    Serial.print("\"error_steps\":"); Serial.print(s->error_steps); Serial.print(",");
    Serial.print("\"faultCode\":"); Serial.print(s->faultCode); Serial.print(",");
    Serial.print("\"temp_c_q10\":"); Serial.print(s->temp_c_q10); Serial.print(",");
    Serial.print("\"config_revision\":"); Serial.print(s->config_revision);
    Serial.println("}");
    return;
  }

  if (hdr.msg_type == Proto::RSP_CONFIG) {
    Serial.print("\"type\":\"config\",");
    Serial.print("\"tlv_hex\":\"");
    for (int i=0; i<hdr.payload_len; i++) {
      char b[3];
      snprintf(b, sizeof(b), "%02X", payload[i]);
      Serial.print(b);
    }
    Serial.print("\"");
    Serial.println("}");
    return;
  }

  Serial.print("\"type\":\"other\",\"len\":"); Serial.print(hdr.payload_len);
  Serial.println("}");
}

// ----------------- ESP-NOW init -----------------
static bool espnowBegin(uint8_t channel) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.disconnect(true, true);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  return true;
}

// ----------------- Serial command parser -----------------
// Commands (one per line):
//   MAC aa:bb:cc:dd:ee:ff
//   CHAN n
//   PING
//   ENABLE 0|1
//   STOP
//   MOVE_DEG <float_degrees>
//   VEL_DPS <float_deg_per_sec>
//   HOME
//   GETCFG ALL
//   GETCFG IDS 0x0001,0x0010,0x0400
//   SETCFG <flags_int> <id_hex> <type_int> <value>
//   CUR <irun> <ihold> [FLAGS|SAVE]
//   SAVE
//
// flags_int uses Proto::SetFlags bits.

static void replyOk(const char* msg) {
  Serial.print("{\"type\":\"bridge\",\"ok\":true,\"msg\":\"");
  printJsonEscaped(msg);
  Serial.println("\"}");
}
static void replyErr(const char* msg) {
  Serial.print("{\"type\":\"bridge\",\"ok\":false,\"msg\":\"");
  printJsonEscaped(msg);
  Serial.println("\"}");
}

static bool requirePeer() {
  if (!gHasPeer) { replyErr("No peer MAC set. Use MAC aa:bb:..."); return false; }
  return true;
}

static void handleLine(String line) {
  line.trim();
  if (!line.length()) return;

  int sp = line.indexOf(' ');
  String cmd = (sp < 0) ? line : line.substring(0, sp);
  cmd.toUpperCase();
  String rest = (sp < 0) ? "" : line.substring(sp + 1);
  rest.trim();

  if (cmd == "MAC") {
    uint8_t mac[6];
    if (!parseMac(rest, mac)) { replyErr("Bad MAC format"); return; }
    memcpy(gPeerMac, mac, 6);
    gHasPeer = true;
    ensurePeer(gPeerMac);
    replyOk(("Peer set to " + macToString(gPeerMac)).c_str());
    return;
  }

  if (cmd == "CHAN") {
    int ch = rest.toInt();
    if (ch < 1 || ch > 14) { replyErr("Bad channel"); return; }
    gChannel = (uint8_t)ch;
    esp_now_deinit();
    if (!espnowBegin(gChannel)) { replyErr("ESP-NOW reinit failed"); return; }
    replyOk("Channel set");
    return;
  }

  if (cmd == "PING") {
    if (!requirePeer()) return;
    if (!buildAndSend(gPeerMac, Proto::CMD_PING, Proto::FLAG_ACK_REQ, nullptr, 0)) replyErr("send failed");
    else replyOk("sent ping");
    return;
  }

  if (cmd == "ENABLE") {
    if (!requirePeer()) return;
    int en = rest.toInt();
    Proto::CmdEnablePayload p{ (uint8_t)(en ? 1 : 0) };
    if (!buildAndSend(gPeerMac, Proto::CMD_ENABLE, Proto::FLAG_ACK_REQ, (uint8_t*)&p, sizeof(p))) replyErr("send failed");
    else replyOk("sent enable");
    return;
  }

  if (cmd == "STOP") {
    if (!requirePeer()) return;
    if (!buildAndSend(gPeerMac, Proto::CMD_STOP, Proto::FLAG_ACK_REQ, nullptr, 0)) replyErr("send failed");
    else replyOk("sent stop");
    return;
  }

  if (cmd == "MOVE_DEG") {
    if (!requirePeer()) return;
    double deg = rest.toFloat();
    int32_t q100 = (int32_t)llround(deg * 100.0);
    Proto::CmdMoveAbsDegPayload p{ q100 };
    if (!buildAndSend(gPeerMac, Proto::CMD_MOVE_ABS_DEG, Proto::FLAG_ACK_REQ, (uint8_t*)&p, sizeof(p))) replyErr("send failed");
    else replyOk("sent move");
    return;
  }

  if (cmd == "VEL_DPS") {
    if (!requirePeer()) return;
    double dps = rest.toFloat();
    int32_t q100 = (int32_t)llround(dps * 100.0);
    Proto::CmdVelocityDpsPayload p{ q100 };
    if (!buildAndSend(gPeerMac, Proto::CMD_VELOCITY_DPS, Proto::FLAG_ACK_REQ, (uint8_t*)&p, sizeof(p))) replyErr("send failed");
    else replyOk("sent velocity");
    return;
  }

  if (cmd == "HOME") {
    if (!requirePeer()) return;
    if (!buildAndSend(gPeerMac, Proto::CMD_HOME_START, Proto::FLAG_ACK_REQ, nullptr, 0)) replyErr("send failed");
    else replyOk("sent home start");
    return;
  }

  if (cmd == "SAVE") {
    if (!requirePeer()) return;
    if (!buildAndSend(gPeerMac, Proto::CMD_SAVE_CONFIG, Proto::FLAG_ACK_REQ, nullptr, 0)) replyErr("send failed");
    else replyOk("sent save");
    return;
  }

  // NEW: CUR <irun> <ihold> [FLAGS|SAVE]
  if (cmd == "CUR") {
    if (!requirePeer()) return;

    // Parse: irun ihold [opt]
    int a = rest.indexOf(' ');
    if (a < 0) { replyErr("Usage: CUR <irun> <ihold> [FLAGS|SAVE]"); return; }
    String sIrun = rest.substring(0, a); sIrun.trim();
    String rem1 = rest.substring(a + 1); rem1.trim();

    int b = rem1.indexOf(' ');
    String sIhold = (b < 0) ? rem1 : rem1.substring(0, b);
    sIhold.trim();
    String opt = (b < 0) ? "" : rem1.substring(b + 1);
    opt.trim();
    opt.toUpperCase();

    int irun = sIrun.toInt();
    int ihold = sIhold.toInt();
    if (irun < 0) irun = 0;
    if (ihold < 0) ihold = 0;
    if (irun > 31) irun = 31;
    if (ihold > 31) ihold = 31;

    uint8_t flags = Proto::SET_APPLY_NOW;
    if (opt == "SAVE") {
      flags = (uint8_t)(Proto::SET_APPLY_NOW | Proto::SET_SAVE_AFTER_APPLY);
    } else if (opt.length()) {
      // allow numeric flags
      flags = (uint8_t)opt.toInt();
    }

    uint8_t payload[1 + (sizeof(Proto::TLV) + 1) * 2];
    uint8_t plen = buildSetCfg2_U8(payload, sizeof(payload), flags,
                                   Proto::P_TMC_IRUN,  (uint8_t)irun,
                                   Proto::P_TMC_IHOLD, (uint8_t)ihold);
    if (!plen) { replyErr("Internal payload build failed"); return; }

    if (!buildAndSend(gPeerMac, Proto::CMD_SET_CONFIG, Proto::FLAG_ACK_REQ, payload, plen)) {
      replyErr("send failed");
    } else {
      String m = "sent CUR irun=" + String(irun) + " ihold=" + String(ihold) + " flags=" + String(flags);
      replyOk(m.c_str());
    }
    return;
  }

  if (cmd == "GETCFG") {
    if (!requirePeer()) return;
    rest.toUpperCase();
    if (rest == "ALL") {
      Proto::CmdGetConfigPayload p{0};
      if (!buildAndSend(gPeerMac, Proto::CMD_GET_CONFIG, Proto::FLAG_ACK_REQ, (uint8_t*)&p, sizeof(p))) replyErr("send failed");
      else replyOk("sent getcfg all");
      return;
    }
    if (rest.startsWith("IDS")) {
      String idsStr = rest.substring(3);
      idsStr.trim();
      if (idsStr.length() == 0) { replyErr("No IDs"); return; }

      uint16_t ids[40];
      size_t n = 0;

      while (idsStr.length() && n < 40) {
        int comma = idsStr.indexOf(',');
        String one = (comma < 0) ? idsStr : idsStr.substring(0, comma);
        one.trim();
        if (comma >= 0) idsStr = idsStr.substring(comma + 1);
        else idsStr = "";

        unsigned int v = 0;
        if (sscanf(one.c_str(), "0x%x", &v) != 1 && sscanf(one.c_str(), "%u", &v) != 1) {
          replyErr("Bad ID");
          return;
        }
        ids[n++] = (uint16_t)v;
      }

      uint8_t payload[1 + 2*40];
      Proto::CmdGetConfigPayload p{ (uint8_t)n };
      memcpy(payload, &p, sizeof(p));
      memcpy(payload + sizeof(p), ids, n*2);

      if (!buildAndSend(gPeerMac, Proto::CMD_GET_CONFIG, Proto::FLAG_ACK_REQ, payload, (uint8_t)(sizeof(p) + n*2))) replyErr("send failed");
      else replyOk("sent getcfg ids");
      return;
    }

    replyErr("Usage: GETCFG ALL | GETCFG IDS 0x0001,0x0010");
    return;
  }

  if (cmd == "SETCFG") {
    if (!requirePeer()) return;
    // SETCFG <flags> <id_hex> <type_int> <value>
    int flags = 0, type = 0;
    unsigned int id = 0;

    int a = rest.indexOf(' ');
    if (a < 0) { replyErr("Bad SETCFG"); return; }
    String t1 = rest.substring(0, a); t1.trim();
    String rem1 = rest.substring(a+1); rem1.trim();

    int b = rem1.indexOf(' ');
    if (b < 0) { replyErr("Bad SETCFG"); return; }
    String t2 = rem1.substring(0, b); t2.trim();
    String rem2 = rem1.substring(b+1); rem2.trim();

    int c = rem2.indexOf(' ');
    if (c < 0) { replyErr("Bad SETCFG"); return; }
    String t3 = rem2.substring(0, c); t3.trim();
    String valStr = rem2.substring(c+1); valStr.trim();

    flags = t1.toInt();
    if (sscanf(t2.c_str(), "0x%x", &id) != 1 && sscanf(t2.c_str(), "%u", &id) != 1) { replyErr("Bad id"); return; }
    type = t3.toInt();

    uint8_t valueBytes[8];
    uint8_t vlen = 0;

    auto putU8  = [&](uint8_t v){ valueBytes[0]=v; vlen=1; };
    auto putU16 = [&](uint16_t v){ memcpy(valueBytes,&v,2); vlen=2; };
    auto putU32 = [&](uint32_t v){ memcpy(valueBytes,&v,4); vlen=4; };
    auto putI16 = [&](int16_t v){ memcpy(valueBytes,&v,2); vlen=2; };
    auto putI32 = [&](int32_t v){ memcpy(valueBytes,&v,4); vlen=4; };

    if (type == Proto::VT_BOOL) {
      bool bv = (valStr == "1" || valStr.equalsIgnoreCase("true") || valStr.equalsIgnoreCase("on"));
      putU8(bv ? 1 : 0);
    } else if (type == Proto::VT_U8) {
      putU8((uint8_t)valStr.toInt());
    } else if (type == Proto::VT_U16) {
      putU16((uint16_t)valStr.toInt());
    } else if (type == Proto::VT_U32) {
      putU32((uint32_t)strtoul(valStr.c_str(), nullptr, 0));
    } else if (type == Proto::VT_I16) {
      putI16((int16_t)valStr.toInt());
    } else if (type == Proto::VT_I32) {
      putI32((int32_t)strtol(valStr.c_str(), nullptr, 0));
    } else {
      replyErr("Unknown type");
      return;
    }

    uint8_t out[1 + sizeof(Proto::TLV) + 8];
    out[0] = (uint8_t)flags;

    Proto::TLV tlv{ (uint16_t)id, (uint8_t)type, vlen };
    memcpy(out + 1, &tlv, sizeof(tlv));
    memcpy(out + 1 + sizeof(tlv), valueBytes, vlen);

    uint8_t outLen = (uint8_t)(1 + sizeof(tlv) + vlen);
    if (!buildAndSend(gPeerMac, Proto::CMD_SET_CONFIG, Proto::FLAG_ACK_REQ, out, outLen)) replyErr("send failed");
    else replyOk("sent setcfg");
    return;
  }

  replyErr("Unknown command");
}

static String gLine;

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!espnowBegin(gChannel)) {
    Serial.println("{\"type\":\"bridge\",\"ok\":false,\"msg\":\"espnow init failed\"}");
  } else {
    Serial.print("{\"type\":\"bridge\",\"ok\":true,\"msg\":\"ready\",\"channel\":");
    Serial.print(gChannel);
    Serial.print(",\"controller_mac\":\"");
    Serial.print(WiFi.macAddress());
    Serial.println("\"}");
  }

  // Optional: preset peer here (or set from GUI)
  // parseMac("AA:BB:CC:DD:EE:FF", gPeerMac); gHasPeer = true; ensurePeer(gPeerMac);
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleLine(gLine);
      gLine = "";
    } else {
      if (gLine.length() < 200) gLine += c;
    }
  }
}
