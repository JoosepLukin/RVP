#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// =====================
// ESP-NOW protocol (MUST match MotorNode)
// =====================
static constexpr uint16_t MN_MAGIC   = 0x4D4E; // 'MN'
static constexpr uint8_t  MN_VERSION = 1;


enum MsgType : uint8_t { MSG_CMD = 1, MSG_STATUS = 2, MSG_ACK = 3 };

enum CmdId : uint16_t {
  CMD_PING                     = 0x0001,

  CMD_SET_ENABLE               = 0x0010,
  CMD_SET_KEEP_ENABLED         = 0x0011,
  CMD_SET_CL_MODE              = 0x0012,

  CMD_APPLY_CONFIG_NOW         = 0x0013,

  CMD_SET_MICROSTEPS           = 0x0020,
  CMD_SET_INTPOL               = 0x0021,
  CMD_SET_DEDGE                = 0x0022,
  CMD_SET_CURRENTS             = 0x0023,

  CMD_SET_SPEED_SPS            = 0x0030,
  CMD_SET_ACCEL_SPS2           = 0x0031,

  CMD_MOVE_BY                  = 0x0040,
  CMD_MOVE_TO                  = 0x0041,
  CMD_VELOCITY                 = 0x0042,
  CMD_STOP_DECEL               = 0x0043,
  CMD_FORCE_STOP               = 0x0044,

  CMD_SET_ENCODER_POLL_US      = 0x0050,
  CMD_ENC_SET_TO_MOTOR         = 0x0051,
  CMD_MOTOR_SET_TO_ENC         = 0x0052,
  CMD_MOTOR_LOGICAL_ZERO       = 0x0053,
  CMD_ENC_OFFSET_ZERO          = 0x0054,

  CMD_SET_MISMATCH_BASE_FULL_Q = 0x0060, // fullsteps*256 (QFULL)
  CMD_SET_MISMATCH_GAIN_FULL_Q = 0x0061, // fullsteps/rps*256 (QFULL)
  CMD_SET_MISMATCH_CHECK_US    = 0x0062,

  CMD_SET_THERMISTOR_PARAMS    = 0x0070,

  CMD_REQUEST_STATUS_NOW       = 0x00F0
};

struct __attribute__((packed)) MsgCommand {
  uint16_t magic;
  uint8_t  version;
  uint8_t  type;     // MSG_CMD
  uint16_t cmd;
  uint16_t seq;
  uint8_t  payload[32];
};

struct __attribute__((packed)) MsgAck {
  uint16_t magic;
  uint8_t  version;
  uint8_t  type;     // MSG_ACK
  uint16_t cmd;
  uint16_t seq;
  uint8_t  code;
  uint8_t  rsv[3];
};

struct __attribute__((packed)) MsgStatus {
  uint16_t magic;
  uint8_t  version;
  uint8_t  type;     // MSG_STATUS
  uint16_t seq;
  uint32_t uptime_ms;

  int32_t motor_pos_user;
  int32_t enc_pos_user;
  int32_t err_user;
  int32_t thr_user;

  uint32_t missed_events;

  int16_t  temp_c_x10;   // INT16_MIN if invalid
  uint8_t  moving;
  uint8_t  outputs_enabled;
  uint8_t  cl_mode;
  uint8_t  keep_enabled;

  uint32_t speed_sps;
  uint32_t accel_sps2;

  uint16_t usteps;
  uint8_t  irun;
  uint8_t  ihold;
  uint8_t  iholddelay;
  uint8_t  rsv0;

  uint32_t drv_status;
  uint32_t ioin;
  uint8_t  ifcnt;
  uint8_t  rsv1[3];
};

static_assert(sizeof(MsgCommand) == 40, "MsgCommand must be 40 bytes");
static_assert(sizeof(MsgAck)     == 12, "MsgAck must be 12 bytes");
static_assert(sizeof(MsgStatus)  == 62, "MsgStatus must be 62 bytes");

// =====================
// WiFi / ESP-NOW
// =====================
// IMPORTANT:
// - Keep master + motor on same channel.
// - If your MotorNode is not forcing a channel, both usually end up on channel 1 when not connected to AP.
// If you get no comms, set the same WIFI_CHANNEL in BOTH motor+master.
static constexpr int WIFI_CHANNEL = 1;

static const uint8_t BROADCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static uint16_t g_seq = 1;

static bool     g_haveMotor = false;
static uint8_t  g_motorMac[6] = {0};
static char     g_motorMacStr[18] = "??:??:??:??:??:??";

static bool     g_haveLastStatus = false;
static MsgStatus g_lastStatus{};
static char      g_selfMacStr[18] = {0};

// RX queue so callbacks stay tiny
struct RxItem {
  uint8_t mac[6];
  uint8_t len;
  uint8_t data[80];
};

static QueueHandle_t g_rxQ = nullptr;

// =====================
// Helpers
// =====================
static void macToStr(const uint8_t* mac, char* out18) {
  snprintf(out18, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void ensureBroadcastPeer() {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (!esp_now_is_peer_exist(BROADCAST_MAC)) {
    esp_now_add_peer(&peer);
  }
}

static void sendCmd(uint16_t cmd, const void* payload, size_t payloadLen) {
  MsgCommand m{};
  m.magic = MN_MAGIC;
  m.version = MN_VERSION;
  m.type = MSG_CMD;
  m.cmd = cmd;
  m.seq = g_seq++;

  if (payload && payloadLen > 0) {
    if (payloadLen > sizeof(m.payload)) payloadLen = sizeof(m.payload);
    memcpy(m.payload, payload, payloadLen);
  }

  // Send as broadcast so you don't need motor MAC hardcoded.
  // MotorNode will learn your MAC and then unicast ACK/STATUS back.
  esp_now_send(BROADCAST_MAC, (uint8_t*)&m, sizeof(m));
}

static void sendCmdU8(uint16_t cmd, uint8_t v) { sendCmd(cmd, &v, 1); }
static void sendCmdU16(uint16_t cmd, uint16_t v) { sendCmd(cmd, &v, 2); }
static void sendCmdU32(uint16_t cmd, uint32_t v) { sendCmd(cmd, &v, 4); }
static void sendCmdI32(uint16_t cmd, int32_t v) { sendCmd(cmd, &v, 4); }

static int32_t fullStepsFloatToQ(float fs) {
  // QFULL=256 format
  double q = (double)fs * 256.0;
  if (q > (double)INT32_MAX) q = (double)INT32_MAX;
  if (q < (double)INT32_MIN) q = (double)INT32_MIN;
  return (int32_t)llround(q);
}

// =====================
// ESP-NOW callbacks (IDF5)
// =====================
static void onRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  if (!info || !data || len <= 0) return;

  RxItem item{};
  memcpy(item.mac, info->src_addr, 6);

  if (len > (int)sizeof(item.data)) len = sizeof(item.data);
  item.len = (uint8_t)len;
  memcpy(item.data, data, len);

  if (g_rxQ) {
    xQueueSend(g_rxQ, &item, 0);
  }
}

static void onSent(const wifi_tx_info_t* /*tx_info*/, esp_now_send_status_t /*status*/) {
  // not used
}

// =====================
// Serial input
// =====================
static bool readLine(String &out) {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { out = buf; buf = ""; return true; }
    buf += c;
    if (buf.length() > 220) { out = buf; buf = ""; return true; }
  }
  return false;
}

static void printHelp() {
  Serial.println("{\"type\":\"info\",\"msg\":\"Commands: HELP, PING, ENABLE 0|1, KEEP 0|1, CL 0|1, APPLY, USTEPS n, INTPOL 0|1, DEDGE 0|1, CUR irun ihold ihd, SPEED sps, ACCEL sps2, MOVE_TO x, MOVE_BY d, VEL v, STOP, FSTOP, ENC_POLL us, ENC_TO_MOTOR, MOTOR_TO_ENC, MOTOR_ZERO, ENC_ZERO, THR_BASE fs, THR_GAIN fsPerRps, THR_US us, THERM rFixed r0 beta t0C samples, GET_STATUS\"}");
}

static void emitLastStatusJson() {
  if (!g_haveLastStatus) {
    Serial.println("{\"type\":\"info\",\"msg\":\"No status received yet.\"}");
    return;
  }

  const MsgStatus &s = g_lastStatus;

  float temp = NAN;
  if (s.temp_c_x10 != INT16_MIN) temp = (float)s.temp_c_x10 / 10.0f;

  char drvHex[12], ioinHex[12];
  snprintf(drvHex, sizeof(drvHex), "0x%08lX", (unsigned long)s.drv_status);
  snprintf(ioinHex, sizeof(ioinHex), "0x%08lX", (unsigned long)s.ioin);

  // JSON line for Python GUI
  Serial.printf(
    "{\"type\":\"status\",\"from\":\"%s\",\"seq\":%u,\"uptime_ms\":%lu,"
    "\"motor_pos\":%ld,\"enc_pos\":%ld,\"err\":%ld,\"thr\":%ld,"
    "\"missed\":%lu,"
    "\"temp_c\":%s,"
    "\"moving\":%u,\"en\":%u,\"cl\":%u,\"keep\":%u,"
    "\"speed\":%lu,\"accel\":%lu,"
    "\"usteps\":%u,\"irun\":%u,\"ihold\":%u,\"ihd\":%u,"
    "\"drv\":\"%s\",\"ioin\":\"%s\",\"ifcnt\":%u}\n",
    g_motorMacStr,
    (unsigned)s.seq,
    (unsigned long)s.uptime_ms,
    (long)s.motor_pos_user,
    (long)s.enc_pos_user,
    (long)s.err_user,
    (long)s.thr_user,
    (unsigned long)s.missed_events,
    (isnan(temp) ? "null" : String(temp, 1).c_str()),
    (unsigned)s.moving,
    (unsigned)s.outputs_enabled,
    (unsigned)s.cl_mode,
    (unsigned)s.keep_enabled,
    (unsigned long)s.speed_sps,
    (unsigned long)s.accel_sps2,
    (unsigned)s.usteps,
    (unsigned)s.irun,
    (unsigned)s.ihold,
    (unsigned)s.iholddelay,
    drvHex,
    ioinHex,
    (unsigned)s.ifcnt
  );
}

static void handleSerialCommand(const String &lineIn) {
  String line = lineIn;
  line.trim();
  if (!line.length()) return;

  // Tokenize
  char buf[256];
  size_t n = line.length();
  if (n >= sizeof(buf)) n = sizeof(buf) - 1;
  memcpy(buf, line.c_str(), n);
  buf[n] = 0;

  char* tok = strtok(buf, " ");
  if (!tok) return;

  auto eq = [](const char* a, const char* b){ return strcasecmp(a,b)==0; };

  if (eq(tok, "HELP")) { printHelp(); return; }
  if (eq(tok, "GET_STATUS")) { emitLastStatusJson(); return; }

  if (eq(tok, "PING")) { sendCmd(CMD_PING, nullptr, 0); return; }

  if (eq(tok, "ENABLE")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdU8(CMD_SET_ENABLE, (uint8_t)atoi(t));
    return;
  }

  if (eq(tok, "KEEP")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdU8(CMD_SET_KEEP_ENABLED, (uint8_t)atoi(t));
    return;
  }

  if (eq(tok, "CL")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdU8(CMD_SET_CL_MODE, (uint8_t)atoi(t));
    return;
  }

  if (eq(tok, "APPLY")) { sendCmd(CMD_APPLY_CONFIG_NOW, nullptr, 0); return; }

  if (eq(tok, "USTEPS")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdU16(CMD_SET_MICROSTEPS, (uint16_t)atoi(t));
    return;
  }

  if (eq(tok, "INTPOL")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdU8(CMD_SET_INTPOL, (uint8_t)atoi(t));
    return;
  }

  if (eq(tok, "DEDGE")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdU8(CMD_SET_DEDGE, (uint8_t)atoi(t));
    return;
  }

  if (eq(tok, "CUR")) {
    char* a = strtok(nullptr, " ");
    char* b = strtok(nullptr, " ");
    char* c = strtok(nullptr, " ");
    if (!a || !b || !c) return;
    uint8_t p[3] = {(uint8_t)atoi(a), (uint8_t)atoi(b), (uint8_t)atoi(c)};
    sendCmd(CMD_SET_CURRENTS, p, sizeof(p));
    return;
  }

  if (eq(tok, "SPEED")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdU32(CMD_SET_SPEED_SPS, (uint32_t)strtoul(t, nullptr, 10));
    return;
  }

  if (eq(tok, "ACCEL")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdU32(CMD_SET_ACCEL_SPS2, (uint32_t)strtoul(t, nullptr, 10));
    return;
  }

  if (eq(tok, "MOVE_TO")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdI32(CMD_MOVE_TO, (int32_t)strtol(t, nullptr, 10));
    return;
  }

  if (eq(tok, "MOVE_BY")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdI32(CMD_MOVE_BY, (int32_t)strtol(t, nullptr, 10));
    return;
  }

  if (eq(tok, "VEL")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdI32(CMD_VELOCITY, (int32_t)strtol(t, nullptr, 10));
    return;
  }

  if (eq(tok, "STOP")) { sendCmd(CMD_STOP_DECEL, nullptr, 0); return; }
  if (eq(tok, "FSTOP")) { sendCmd(CMD_FORCE_STOP, nullptr, 0); return; }

  if (eq(tok, "ENC_POLL")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdU32(CMD_SET_ENCODER_POLL_US, (uint32_t)strtoul(t, nullptr, 10));
    return;
  }

  if (eq(tok, "ENC_TO_MOTOR")) { sendCmd(CMD_ENC_SET_TO_MOTOR, nullptr, 0); return; }
  if (eq(tok, "MOTOR_TO_ENC")) { sendCmd(CMD_MOTOR_SET_TO_ENC, nullptr, 0); return; }
  if (eq(tok, "MOTOR_ZERO")) { sendCmd(CMD_MOTOR_LOGICAL_ZERO, nullptr, 0); return; }
  if (eq(tok, "ENC_ZERO")) { sendCmd(CMD_ENC_OFFSET_ZERO, nullptr, 0); return; }

  if (eq(tok, "THR_BASE")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    float fs = (float)strtod(t, nullptr);
    int32_t q = fullStepsFloatToQ(fs);
    sendCmdI32(CMD_SET_MISMATCH_BASE_FULL_Q, q);
    return;
  }

  if (eq(tok, "THR_GAIN")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    float fs = (float)strtod(t, nullptr);
    int32_t q = fullStepsFloatToQ(fs);
    sendCmdI32(CMD_SET_MISMATCH_GAIN_FULL_Q, q);
    return;
  }

  if (eq(tok, "THR_US")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    sendCmdU32(CMD_SET_MISMATCH_CHECK_US, (uint32_t)strtoul(t, nullptr, 10));
    return;
  }

  if (eq(tok, "THERM")) {
    // THERM rFixed r0 beta t0C samples
    char* a = strtok(nullptr, " ");
    char* b = strtok(nullptr, " ");
    char* c = strtok(nullptr, " ");
    char* d = strtok(nullptr, " ");
    char* e = strtok(nullptr, " ");
    if (!a || !b || !c || !d || !e) return;

    uint32_t rFixed = (uint32_t)strtoul(a, nullptr, 10);
    uint32_t r0     = (uint32_t)strtoul(b, nullptr, 10);
    uint16_t beta   = (uint16_t)strtoul(c, nullptr, 10);
    float t0C       = (float)strtod(d, nullptr);
    int16_t t0x10   = (int16_t)lroundf(t0C * 10.0f);
    uint8_t samples = (uint8_t)atoi(e);

    uint8_t p[13] = {0};
    memcpy(p + 0,  &rFixed, 4);
    memcpy(p + 4,  &r0,     4);
    memcpy(p + 8,  &beta,   2);
    memcpy(p + 10, &t0x10,  2);
    p[12] = samples;

    sendCmd(CMD_SET_THERMISTOR_PARAMS, p, sizeof(p));
    return;
  }

  if (eq(tok, "REQ_STATUS")) {
    // motor doesn't immediately reply, but it will ACK and continue periodic status
    sendCmd(CMD_REQUEST_STATUS_NOW, nullptr, 0);
    return;
  }

  Serial.println("{\"type\":\"info\",\"msg\":\"Unknown command. Send HELP\"}");
}

// =====================
// Arduino
// =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(50);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  macToStr(mac, g_selfMacStr);

  // Force a fixed channel (recommended)
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("{\"type\":\"info\",\"msg\":\"esp_now_init FAILED\"}");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  ensureBroadcastPeer();

  g_rxQ = xQueueCreate(24, sizeof(RxItem));
  if (!g_rxQ) {
    Serial.println("{\"type\":\"info\",\"msg\":\"RX queue alloc FAILED\"}");
    while (true) delay(1000);
  }

  Serial.printf("{\"type\":\"info\",\"msg\":\"MasterNode ready\",\"mac\":\"%s\",\"channel\":%d}\n", g_selfMacStr, WIFI_CHANNEL);
  printHelp();
}

void loop() {
  // Process incoming ESP-NOW packets
  RxItem item;
  while (xQueueReceive(g_rxQ, &item, 0) == pdTRUE) {
    if (item.len < 4) continue;

    // Peek common header
    uint16_t magic;
    uint8_t version, type;
    memcpy(&magic, item.data + 0, 2);
    version = item.data[2];
    type    = item.data[3];

    if (magic != MN_MAGIC || version != MN_VERSION) continue;

    // Remember motor MAC from any valid packet
    if (!g_haveMotor) {
      memcpy(g_motorMac, item.mac, 6);
      macToStr(g_motorMac, g_motorMacStr);
      g_haveMotor = true;
      Serial.printf("{\"type\":\"info\",\"msg\":\"Motor detected\",\"motor\":\"%s\"}\n", g_motorMacStr);
    }

    if (type == MSG_ACK && item.len == sizeof(MsgAck)) {
      MsgAck a;
      memcpy(&a, item.data, sizeof(a));
      Serial.printf("{\"type\":\"ack\",\"from\":\"%s\",\"cmd\":\"0x%04X\",\"seq\":%u,\"code\":%u}\n",
                    g_motorMacStr, (unsigned)a.cmd, (unsigned)a.seq, (unsigned)a.code);
    } else if (type == MSG_STATUS && item.len == sizeof(MsgStatus)) {
      memcpy(&g_lastStatus, item.data, sizeof(MsgStatus));
      g_haveLastStatus = true;
      emitLastStatusJson();
    }
  }

  // Process serial commands from PC
  String line;
  if (readLine(line)) {
    handleSerialCommand(line);
  }
}
