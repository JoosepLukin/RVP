#include "SketchPreamble.h"
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

  CMD_SET_NODE_ID              = 0x00E0, // u16 (0..32; 0=unassigned)
  CMD_SAVE_CONFIG_NVS          = 0x00E1, // no payload

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
  uint16_t node_id;
  uint8_t  rsv0;
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
  uint16_t node_id;
  uint8_t  rsv1;
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

static char      g_selfMacStr[18] = {0};

static constexpr uint8_t MAX_MOTORS = 32;

struct MotorRecord {
  bool     used = false;
  uint8_t  mac[6] = {0};
  char     macStr[18] = "??:??:??:??:??:??";
  uint16_t node_id = 0; // 0=unassigned
  uint32_t last_seen_ms = 0;
  bool     have_status = false;
  MsgStatus last_status{};
};

static MotorRecord g_motors[MAX_MOTORS];

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

static void ensurePeer(const uint8_t* mac) {
  if (!mac) return;
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  (void)esp_now_add_peer(&peer);
}

static int findMotorIndexByMac(const uint8_t* mac) {
  for (int i = 0; i < MAX_MOTORS; i++) {
    if (!g_motors[i].used) continue;
    if (memcmp(g_motors[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

static MotorRecord* upsertMotor(const uint8_t* mac) {
  int idx = findMotorIndexByMac(mac);
  if (idx >= 0) {
    g_motors[idx].last_seen_ms = millis();
    return &g_motors[idx];
  }
  for (int i = 0; i < MAX_MOTORS; i++) {
    if (g_motors[i].used) continue;
    g_motors[i] = MotorRecord{};
    g_motors[i].used = true;
    memcpy(g_motors[i].mac, mac, 6);
    macToStr(mac, g_motors[i].macStr);
    g_motors[i].last_seen_ms = millis();
    ensurePeer(mac);
    Serial.printf("{\"type\":\"info\",\"msg\":\"Motor detected\",\"motor\":\"%s\"}\n", g_motors[i].macStr);
    return &g_motors[i];
  }
  Serial.println("{\"type\":\"info\",\"msg\":\"Motor table full\"}");
  return nullptr;
}

static MotorRecord* getOnlyMotorOrNull() {
  MotorRecord* found = nullptr;
  for (int i = 0; i < MAX_MOTORS; i++) {
    if (!g_motors[i].used) continue;
    if (found) return nullptr;
    found = &g_motors[i];
  }
  return found;
}

// =====================
// Reliable TX (queue + retries)
// =====================
static constexpr uint16_t TX_ACK_TIMEOUT_MS = 120;
static constexpr uint8_t  TX_RETRY_COUNT    = 2;   // total sends = 1 + retries
static constexpr uint8_t  TXQ_LEN           = 64;

struct TxQueued {
  bool     byMask = false;
  uint32_t mask = 0;
  bool     byMac = false;
  uint8_t  mac[6] = {0};
  uint16_t cmd = 0;
  uint8_t  payloadLen = 0;
  uint8_t  payload[32] = {0};
};

struct PendingTx {
  bool     in_use = false;
  uint8_t  mac[6] = {0};
  char     macStr[18] = "??:??:??:??:??:??";
  uint16_t node_id = 0;
  MsgCommand msg{};
  uint8_t  retries_left = 0;
  uint32_t next_retry_ms = 0;
};

static TxQueued  g_txq[TXQ_LEN];
static uint8_t   g_txq_head = 0, g_txq_tail = 0, g_txq_count = 0;

static bool      g_tx_active = false;
static PendingTx g_pending[MAX_MOTORS];

static bool txqPush(const TxQueued& it) {
  if (g_txq_count >= TXQ_LEN) return false;
  g_txq[g_txq_tail] = it;
  g_txq_tail = (uint8_t)((g_txq_tail + 1) % TXQ_LEN);
  g_txq_count++;
  return true;
}

static bool txqPop(TxQueued& out) {
  if (g_txq_count == 0) return false;
  out = g_txq[g_txq_head];
  g_txq_head = (uint8_t)((g_txq_head + 1) % TXQ_LEN);
  g_txq_count--;
  return true;
}

static void buildCmd(MsgCommand& m, uint16_t cmd, uint16_t seq, const void* payload, size_t payloadLen) {
  memset(&m, 0, sizeof(m));
  m.magic = MN_MAGIC;
  m.version = MN_VERSION;
  m.type = MSG_CMD;
  m.cmd = cmd;
  m.seq = seq;
  if (payload && payloadLen > 0) {
    if (payloadLen > sizeof(m.payload)) payloadLen = sizeof(m.payload);
    memcpy(m.payload, payload, payloadLen);
  }
}

static void clearPendingMatching(const uint8_t* mac, uint16_t cmd, uint16_t seq) {
  for (int i = 0; i < MAX_MOTORS; i++) {
    if (!g_pending[i].in_use) continue;
    if (g_pending[i].msg.cmd != cmd) continue;
    if (g_pending[i].msg.seq != seq) continue;
    if (memcmp(g_pending[i].mac, mac, 6) != 0) continue;
    g_pending[i].in_use = false;
  }
}

static void beginBatch(const TxQueued& q) {
  memset(g_pending, 0, sizeof(g_pending));

  uint16_t seq = g_seq++;
  MsgCommand msg{};
  buildCmd(msg, q.cmd, seq, q.payload, q.payloadLen);

  uint32_t now = millis();
  uint8_t targets = 0;

  if (q.byMask) {
    for (int i = 0; i < MAX_MOTORS; i++) {
      if (!g_motors[i].used) continue;
      uint16_t id = g_motors[i].node_id;
      if (id < 1 || id > 32) continue;
      if ((q.mask & (1UL << (id - 1))) == 0) continue;

      g_pending[targets] = PendingTx{};
      g_pending[targets].in_use = true;
      memcpy(g_pending[targets].mac, g_motors[i].mac, 6);
      memcpy(g_pending[targets].macStr, g_motors[i].macStr, sizeof(g_pending[targets].macStr));
      g_pending[targets].node_id = id;
      g_pending[targets].msg = msg;
      g_pending[targets].retries_left = TX_RETRY_COUNT;
      g_pending[targets].next_retry_ms = now + TX_ACK_TIMEOUT_MS;
      ensurePeer(g_pending[targets].mac);
      (void)esp_now_send(g_pending[targets].mac, (uint8_t*)&g_pending[targets].msg, sizeof(MsgCommand));
      targets++;
      if (targets >= MAX_MOTORS) break;
    }
  } else if (q.byMac) {
    MotorRecord* m = upsertMotor(q.mac);
    if (m) {
      g_pending[0] = PendingTx{};
      g_pending[0].in_use = true;
      memcpy(g_pending[0].mac, m->mac, 6);
      memcpy(g_pending[0].macStr, m->macStr, sizeof(g_pending[0].macStr));
      g_pending[0].node_id = m->node_id;
      g_pending[0].msg = msg;
      g_pending[0].retries_left = TX_RETRY_COUNT;
      g_pending[0].next_retry_ms = now + TX_ACK_TIMEOUT_MS;
      ensurePeer(g_pending[0].mac);
      (void)esp_now_send(g_pending[0].mac, (uint8_t*)&g_pending[0].msg, sizeof(MsgCommand));
      targets = 1;
    }
  }

  if (targets == 0) {
    Serial.println("{\"type\":\"info\",\"msg\":\"No targets (unknown IDs or no motors)\"}");
    g_tx_active = false;
    return;
  }

  g_tx_active = true;
}

static void serviceTx() {
  uint32_t now = millis();

  if (!g_tx_active) {
    TxQueued next;
    if (txqPop(next)) beginBatch(next);
    return;
  }

  bool any = false;
  for (int i = 0; i < MAX_MOTORS; i++) {
    if (!g_pending[i].in_use) continue;
    any = true;

    if ((int32_t)(now - g_pending[i].next_retry_ms) < 0) continue;

    if (g_pending[i].retries_left > 0) {
      g_pending[i].retries_left--;
      g_pending[i].next_retry_ms = now + TX_ACK_TIMEOUT_MS;
      ensurePeer(g_pending[i].mac);
      (void)esp_now_send(g_pending[i].mac, (uint8_t*)&g_pending[i].msg, sizeof(MsgCommand));
      continue;
    }

    Serial.printf(
      "{\"type\":\"info\",\"msg\":\"TX timeout\",\"id\":%u,\"mac\":\"%s\",\"cmd\":\"0x%04X\",\"seq\":%u}\n",
      (unsigned)g_pending[i].node_id,
      g_pending[i].macStr,
      (unsigned)g_pending[i].msg.cmd,
      (unsigned)g_pending[i].msg.seq
    );
    g_pending[i].in_use = false;
  }

  if (!any) g_tx_active = false;
}

static void sendBroadcastCmd(uint16_t cmd) {
  MsgCommand m{};
  buildCmd(m, cmd, g_seq++, nullptr, 0);
  ensureBroadcastPeer();
  (void)esp_now_send(BROADCAST_MAC, (uint8_t*)&m, sizeof(m));
}

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
  Serial.println(
    "{\"type\":\"info\",\"msg\":\"Commands: HELP, GET_STATUS, PING, IDS <mask> <cmd...>, SET_ID <mac> <id>, SAVE_CFG. "
    "Use IDS for motor-targeted commands (ENABLE/KEEP/CL/APPLY/USTEPS/INTPOL/DEDGE/CUR/SPEED/ACCEL/MOVE_TO/MOVE_BY/VEL/STOP/FSTOP/ENC_POLL/ENC_TO_MOTOR/MOTOR_TO_ENC/MOTOR_ZERO/ENC_ZERO/THR_BASE/THR_GAIN/THR_US/THERM/REQ_STATUS).\"}"
  );
}

static void emitStatusJson(const char* macStr, uint16_t node_id, const MsgStatus &s) {
  float temp = NAN;
  if (s.temp_c_x10 != INT16_MIN) temp = (float)s.temp_c_x10 / 10.0f;

  char drvHex[12], ioinHex[12];
  snprintf(drvHex, sizeof(drvHex), "0x%08lX", (unsigned long)s.drv_status);
  snprintf(ioinHex, sizeof(ioinHex), "0x%08lX", (unsigned long)s.ioin);

  // JSON line for Python GUI
  Serial.printf(
    "{\"type\":\"status\",\"from\":\"%s\",\"id\":%u,\"seq\":%u,\"uptime_ms\":%lu,"
    "\"motor_pos\":%ld,\"enc_pos\":%ld,\"err\":%ld,\"thr\":%ld,"
    "\"missed\":%lu,"
    "\"temp_c\":%s,"
    "\"moving\":%u,\"en\":%u,\"cl\":%u,\"keep\":%u,"
    "\"speed\":%lu,\"accel\":%lu,"
    "\"usteps\":%u,\"irun\":%u,\"ihold\":%u,\"ihd\":%u,"
    "\"drv\":\"%s\",\"ioin\":\"%s\",\"ifcnt\":%u}\n",
    macStr,
    (unsigned)node_id,
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

static void emitAckJson(const char* macStr, uint16_t node_id, const MsgAck &a) {
  Serial.printf("{\"type\":\"ack\",\"from\":\"%s\",\"id\":%u,\"cmd\":\"0x%04X\",\"seq\":%u,\"code\":%u}\n",
                macStr, (unsigned)node_id, (unsigned)a.cmd, (unsigned)a.seq, (unsigned)a.code);
}

static void emitAllStatusJson() {
  bool any = false;
  for (int i = 0; i < MAX_MOTORS; i++) {
    if (!g_motors[i].used || !g_motors[i].have_status) continue;
    any = true;
    emitStatusJson(g_motors[i].macStr, g_motors[i].node_id, g_motors[i].last_status);
  }
  if (!any) Serial.println("{\"type\":\"info\",\"msg\":\"No status received yet.\"}");
}

static bool parseMacStr(const char* s, uint8_t out[6]) {
  if (!s || !out) return false;
  int v[6] = {0};
  if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
  for (int i = 0; i < 6; i++) {
    if (v[i] < 0 || v[i] > 255) return false;
    out[i] = (uint8_t)v[i];
  }
  return true;
}

static bool enqueueCmdForMask(uint32_t mask, uint16_t cmd, const void* payload, size_t payloadLen) {
  TxQueued q{};
  q.byMask = true;
  q.mask = mask;
  q.cmd = cmd;
  if (payload && payloadLen > 0) {
    if (payloadLen > sizeof(q.payload)) payloadLen = sizeof(q.payload);
    q.payloadLen = (uint8_t)payloadLen;
    memcpy(q.payload, payload, payloadLen);
  }
  if (!txqPush(q)) {
    Serial.println("{\"type\":\"info\",\"msg\":\"TX queue full\"}");
    return false;
  }
  return true;
}

static bool enqueueCmdForMac(const uint8_t mac[6], uint16_t cmd, const void* payload, size_t payloadLen) {
  TxQueued q{};
  q.byMac = true;
  memcpy(q.mac, mac, 6);
  q.cmd = cmd;
  if (payload && payloadLen > 0) {
    if (payloadLen > sizeof(q.payload)) payloadLen = sizeof(q.payload);
    q.payloadLen = (uint8_t)payloadLen;
    memcpy(q.payload, payload, payloadLen);
  }
  if (!txqPush(q)) {
    Serial.println("{\"type\":\"info\",\"msg\":\"TX queue full\"}");
    return false;
  }
  return true;
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

  bool hasMask = false;
  uint32_t mask = 0;
  if (eq(tok, "IDS")) {
    char* m = strtok(nullptr, " ");
    if (!m) return;
    mask = (uint32_t)strtoul(m, nullptr, 0);
    hasMask = true;
    tok = strtok(nullptr, " ");
    if (!tok) return;
  }

  if (eq(tok, "HELP")) { printHelp(); return; }
  if (eq(tok, "GET_STATUS")) { emitAllStatusJson(); return; }
  if (eq(tok, "PING")) { sendBroadcastCmd(CMD_PING); return; }

  if (eq(tok, "SET_ID")) {
    char* m = strtok(nullptr, " ");
    char* t = strtok(nullptr, " ");
    if (!m || !t) return;
    uint8_t mac[6];
    if (!parseMacStr(m, mac)) {
      Serial.println("{\"type\":\"info\",\"msg\":\"SET_ID bad MAC\"}");
      return;
    }
    uint16_t id = (uint16_t)strtoul(t, nullptr, 10);
    uint8_t p[2] = {0};
    memcpy(p, &id, 2);
    (void)enqueueCmdForMac(mac, CMD_SET_NODE_ID, p, sizeof(p));
    return;
  }

  if (eq(tok, "SAVE_CFG")) {
    if (!hasMask) {
      Serial.println("{\"type\":\"info\",\"msg\":\"SAVE_CFG requires IDS <mask>\"}");
      return;
    }
    (void)enqueueCmdForMask(mask, CMD_SAVE_CONFIG_NVS, nullptr, 0);
    return;
  }

  // If no IDS prefix, allow legacy single-motor mode.
  bool byMac = false;
  uint8_t mac[6] = {0};
  if (!hasMask) {
    MotorRecord* only = getOnlyMotorOrNull();
    if (!only) {
      Serial.println("{\"type\":\"info\",\"msg\":\"Multiple or zero motors known. Use: IDS <mask> <cmd...>\"}");
      return;
    }
    byMac = true;
    memcpy(mac, only->mac, 6);
  }

  auto enqueueTargeted = [&](uint16_t cmd, const void* payload, size_t payloadLen) {
    if (hasMask) (void)enqueueCmdForMask(mask, cmd, payload, payloadLen);
    else         (void)enqueueCmdForMac(mac, cmd, payload, payloadLen);
  };

  if (eq(tok, "ENABLE")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    uint8_t v = (uint8_t)atoi(t);
    enqueueTargeted(CMD_SET_ENABLE, &v, 1);
    return;
  }

  if (eq(tok, "KEEP")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    uint8_t v = (uint8_t)atoi(t);
    enqueueTargeted(CMD_SET_KEEP_ENABLED, &v, 1);
    return;
  }

  if (eq(tok, "CL")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    uint8_t v = (uint8_t)atoi(t);
    enqueueTargeted(CMD_SET_CL_MODE, &v, 1);
    return;
  }

  if (eq(tok, "APPLY")) { enqueueTargeted(CMD_APPLY_CONFIG_NOW, nullptr, 0); return; }

  if (eq(tok, "USTEPS")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    uint16_t v = (uint16_t)atoi(t);
    enqueueTargeted(CMD_SET_MICROSTEPS, &v, 2);
    return;
  }

  if (eq(tok, "INTPOL")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    uint8_t v = (uint8_t)atoi(t);
    enqueueTargeted(CMD_SET_INTPOL, &v, 1);
    return;
  }

  if (eq(tok, "DEDGE")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    uint8_t v = (uint8_t)atoi(t);
    enqueueTargeted(CMD_SET_DEDGE, &v, 1);
    return;
  }

  if (eq(tok, "CUR")) {
    char* a = strtok(nullptr, " ");
    char* b = strtok(nullptr, " ");
    char* c = strtok(nullptr, " ");
    if (!a || !b || !c) return;
    uint8_t p[3] = {(uint8_t)atoi(a), (uint8_t)atoi(b), (uint8_t)atoi(c)};
    enqueueTargeted(CMD_SET_CURRENTS, p, sizeof(p));
    return;
  }

  if (eq(tok, "SPEED")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    uint32_t v = (uint32_t)strtoul(t, nullptr, 10);
    enqueueTargeted(CMD_SET_SPEED_SPS, &v, 4);
    return;
  }

  if (eq(tok, "ACCEL")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    uint32_t v = (uint32_t)strtoul(t, nullptr, 10);
    enqueueTargeted(CMD_SET_ACCEL_SPS2, &v, 4);
    return;
  }

  if (eq(tok, "MOVE_TO")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    int32_t v = (int32_t)strtol(t, nullptr, 10);
    enqueueTargeted(CMD_MOVE_TO, &v, 4);
    return;
  }

  if (eq(tok, "MOVE_BY")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    int32_t v = (int32_t)strtol(t, nullptr, 10);
    enqueueTargeted(CMD_MOVE_BY, &v, 4);
    return;
  }

  if (eq(tok, "VEL")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    int32_t v = (int32_t)strtol(t, nullptr, 10);
    enqueueTargeted(CMD_VELOCITY, &v, 4);
    return;
  }

  if (eq(tok, "STOP")) { enqueueTargeted(CMD_STOP_DECEL, nullptr, 0); return; }
  if (eq(tok, "FSTOP")) { enqueueTargeted(CMD_FORCE_STOP, nullptr, 0); return; }

  if (eq(tok, "ENC_POLL")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    uint32_t v = (uint32_t)strtoul(t, nullptr, 10);
    enqueueTargeted(CMD_SET_ENCODER_POLL_US, &v, 4);
    return;
  }

  if (eq(tok, "ENC_TO_MOTOR")) { enqueueTargeted(CMD_ENC_SET_TO_MOTOR, nullptr, 0); return; }
  if (eq(tok, "MOTOR_TO_ENC")) { enqueueTargeted(CMD_MOTOR_SET_TO_ENC, nullptr, 0); return; }
  if (eq(tok, "MOTOR_ZERO")) { enqueueTargeted(CMD_MOTOR_LOGICAL_ZERO, nullptr, 0); return; }
  if (eq(tok, "ENC_ZERO")) { enqueueTargeted(CMD_ENC_OFFSET_ZERO, nullptr, 0); return; }

  if (eq(tok, "THR_BASE")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    float fs = (float)strtod(t, nullptr);
    int32_t q = fullStepsFloatToQ(fs);
    enqueueTargeted(CMD_SET_MISMATCH_BASE_FULL_Q, &q, 4);
    return;
  }

  if (eq(tok, "THR_GAIN")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    float fs = (float)strtod(t, nullptr);
    int32_t q = fullStepsFloatToQ(fs);
    enqueueTargeted(CMD_SET_MISMATCH_GAIN_FULL_Q, &q, 4);
    return;
  }

  if (eq(tok, "THR_US")) {
    char* t = strtok(nullptr, " ");
    if (!t) return;
    uint32_t v = (uint32_t)strtoul(t, nullptr, 10);
    enqueueTargeted(CMD_SET_MISMATCH_CHECK_US, &v, 4);
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

    enqueueTargeted(CMD_SET_THERMISTOR_PARAMS, p, sizeof(p));
    return;
  }

  if (eq(tok, "REQ_STATUS")) { enqueueTargeted(CMD_REQUEST_STATUS_NOW, nullptr, 0); return; }

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

    MotorRecord* motor = upsertMotor(item.mac);
    if (!motor) continue;
    motor->last_seen_ms = millis();

    if (type == MSG_ACK && item.len == sizeof(MsgAck)) {
      MsgAck a;
      memcpy(&a, item.data, sizeof(a));
      motor->node_id = a.node_id;
      emitAckJson(motor->macStr, motor->node_id, a);
      clearPendingMatching(item.mac, a.cmd, a.seq);
    } else if (type == MSG_STATUS && item.len == sizeof(MsgStatus)) {
      memcpy(&motor->last_status, item.data, sizeof(MsgStatus));
      motor->have_status = true;
      motor->node_id = motor->last_status.node_id;
      emitStatusJson(motor->macStr, motor->node_id, motor->last_status);
    }
  }

  // Process serial commands from PC
  String line;
  if (readLine(line)) {
    handleSerialCommand(line);
  }

  serviceTx();
}
