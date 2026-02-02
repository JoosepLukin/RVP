#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "MotionControl.h"
#include "Sensors.h"

#ifndef MN_DEBUG
#define MN_DEBUG 0
#endif

static constexpr int WIFI_CHANNEL = 1;  // MUST match MasterNode


namespace EspNowComms {

static constexpr uint16_t MN_MAGIC   = 0x4D4E; // 'MN'
static constexpr uint8_t  MN_VERSION = 1;

enum MsgType : uint8_t {
  MSG_CMD    = 1,
  MSG_STATUS = 2,
  MSG_ACK    = 3
};

enum AckCode : uint8_t {
  ACK_OK          = 0,
  ACK_BAD_ARG     = 1,
  ACK_UNKNOWN_CMD = 4
};

enum CmdId : uint16_t {
  CMD_PING                     = 0x0001,

  CMD_SET_ENABLE               = 0x0010, // u8
  CMD_SET_KEEP_ENABLED         = 0x0011, // u8
  CMD_SET_CL_MODE              = 0x0012, // u8 (0/1)

  CMD_APPLY_CONFIG_NOW         = 0x0013,

  CMD_SET_MICROSTEPS           = 0x0020, // u16
  CMD_SET_INTPOL               = 0x0021, // u8
  CMD_SET_DEDGE                = 0x0022, // u8
  CMD_SET_CURRENTS             = 0x0023, // irun, ihold, ihd

  CMD_SET_SPEED_SPS            = 0x0030, // u32
  CMD_SET_ACCEL_SPS2           = 0x0031, // u32

  CMD_MOVE_BY                  = 0x0040, // i32
  CMD_MOVE_TO                  = 0x0041, // i32
  CMD_VELOCITY                 = 0x0042, // i32
  CMD_STOP_DECEL               = 0x0043,
  CMD_FORCE_STOP               = 0x0044,

  CMD_SET_ENCODER_POLL_US      = 0x0050, // u32
  CMD_ENC_SET_TO_MOTOR         = 0x0051,
  CMD_MOTOR_SET_TO_ENC         = 0x0052,
  CMD_MOTOR_LOGICAL_ZERO       = 0x0053,
  CMD_ENC_OFFSET_ZERO          = 0x0054,

  CMD_SET_MISMATCH_BASE_FULL_Q = 0x0060, // i32 (fullsteps * 256)
  CMD_SET_MISMATCH_GAIN_FULL_Q = 0x0061, // i32 (fullsteps/rps * 256)
  CMD_SET_MISMATCH_CHECK_US    = 0x0062, // u32

  CMD_SET_THERMISTOR_PARAMS    = 0x0070, // rFixed,u32 r0,u32 beta,u16 t0_x10,i16 samples,u8

  CMD_SET_NODE_ID              = 0x00E0, // u16 (0..32; 0=unassigned)
  CMD_SAVE_CONFIG_NVS          = 0x00E1, // no payload

  CMD_REQUEST_STATUS_NOW       = 0x00F0
};

// Fixed-size command message (40 bytes)
struct __attribute__((packed)) MsgCommand {
  uint16_t magic;
  uint8_t  version;
  uint8_t  type;     // MSG_CMD
  uint16_t cmd;
  uint16_t seq;
  uint8_t  payload[32];
};

// ACK (small)
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

// Status (62 bytes)
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

static_assert(sizeof(MsgCommand) == 40, "MsgCommand size mismatch");
static_assert(sizeof(MsgAck)     == 12, "MsgAck size mismatch");
static_assert(sizeof(MsgStatus)  == 62, "MsgStatus size mismatch");

// -------- master tracking ----------
static bool g_hasMaster = false;
static uint8_t g_masterMac[6] = {0};
static uint16_t g_statusSeq = 0;

// -------- node ID + status request ----------
static Preferences g_prefs;
static uint16_t g_node_id = 0; // 0=unassigned
static bool g_status_request_pending = false;

static char g_selfMacStr[18] = {0};

static inline const char* selfMacStr() { return g_selfMacStr; }
static inline bool hasMaster() { return g_hasMaster; }
static inline uint16_t nodeId() { return g_node_id; }

static inline bool consumeStatusRequest() {
  bool v = g_status_request_pending;
  g_status_request_pending = false;
  return v;
}

// -------- command queue ----------
struct CmdItem {
  uint8_t mac[6];
  MsgCommand msg;
};

static QueueHandle_t g_cmdQ = nullptr;

// -------- utils ----------
static inline void macToStr(const uint8_t* mac, char* out18) {
  snprintf(out18, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static inline void ensurePeer(const uint8_t* mac) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;     // same channel
  peer.encrypt = false; // no encryption

  if (!esp_now_is_peer_exist(mac)) {
    (void)esp_now_add_peer(&peer);
  }
}

static inline void setMasterFrom(const uint8_t* mac) {
  if (g_hasMaster && memcmp(g_masterMac, mac, 6) == 0) return;
  memcpy(g_masterMac, mac, 6);
  g_hasMaster = true;
  ensurePeer(mac);

  char s[18]; macToStr(mac, s);
  Serial.print("[MN] Master set to "); Serial.println(s);
}

static inline void sendAckTo(const uint8_t* mac, uint16_t cmd, uint16_t seq, uint8_t code) {
  MsgAck a{};
  a.magic = MN_MAGIC;
  a.version = MN_VERSION;
  a.type = MSG_ACK;
  a.cmd = cmd;
  a.seq = seq;
  a.code = code;
  a.node_id = g_node_id;
  a.rsv0 = 0;
  (void)esp_now_send(mac, (uint8_t*)&a, sizeof(a));
}

static inline void fillStatus(MsgStatus& st) {
  st.magic = MN_MAGIC;
  st.version = MN_VERSION;
  st.type = MSG_STATUS;
  st.seq = g_statusSeq++;
  st.uptime_ms = millis();

  st.motor_pos_user = MotionControl::motorUserPos();
  st.enc_pos_user   = MotionControl::encoderUserPos();
  st.err_user       = st.motor_pos_user - st.enc_pos_user;
  st.thr_user       = MotionControl::thresholdUserSteps();

  st.missed_events  = MotionControl::missedStepEvents();

  st.temp_c_x10     = INT16_MIN; // filled in MotorNode.ino

  st.moving          = MotionControl::isMoving() ? 1 : 0;
  st.outputs_enabled = MotionControl::outputsEnabled() ? 1 : 0;
  st.cl_mode         = MotionControl::closedLoopMode();
  st.keep_enabled    = MotionControl::keepEnabled() ? 1 : 0;

  st.speed_sps       = MotionControl::speedSps();
  st.accel_sps2      = MotionControl::accelSps2();

  st.usteps          = MotionControl::microsteps();
  st.irun            = MotionControl::irun();
  st.ihold           = MotionControl::ihold();
  st.iholddelay      = MotionControl::iholddelay();
  st.rsv0            = 0;

  st.drv_status      = MotionControl::drvStatus();
  st.ioin            = MotionControl::ioin();
  st.ifcnt           = MotionControl::ifcnt();
  st.node_id         = g_node_id;
  st.rsv1            = 0;
}

static inline void sendStatus(const MsgStatus& st) {
  if (!g_hasMaster) return;
  (void)esp_now_send(g_masterMac, (const uint8_t*)&st, sizeof(st));
}

// -------- handler ----------
static inline uint8_t handleCommand(const MsgCommand& m) {
  switch (m.cmd) {
    case CMD_PING: return ACK_OK;

    case CMD_SET_ENABLE: {
      MotionControl::setOutputsEnabled(m.payload[0] != 0);
      return ACK_OK;
    }
    case CMD_SET_KEEP_ENABLED: {
      MotionControl::setKeepEnabled(m.payload[0] != 0);
      return ACK_OK;
    }
    case CMD_SET_CL_MODE: {
      MotionControl::setClosedLoopMode(m.payload[0]);
      return ACK_OK;
    }

    case CMD_APPLY_CONFIG_NOW:
      MotionControl::applyDriverConfigNow();
      return ACK_OK;

    case CMD_SET_MICROSTEPS: {
      uint16_t u;
      memcpy(&u, m.payload, sizeof(u));
      MotionControl::setMicrosteps(u);
      return ACK_OK;
    }
    case CMD_SET_INTPOL:
      MotionControl::setInterpolation(m.payload[0] != 0);
      return ACK_OK;

    case CMD_SET_DEDGE:
      MotionControl::setDedge(m.payload[0] != 0);
      return ACK_OK;

    case CMD_SET_CURRENTS: {
      MotionControl::setCurrents(m.payload[0], m.payload[1], m.payload[2]);
      return ACK_OK;
    }

    case CMD_SET_SPEED_SPS: {
      uint32_t s;
      memcpy(&s, m.payload, sizeof(s));
      MotionControl::setSpeed(s);
      return ACK_OK;
    }

    case CMD_SET_ACCEL_SPS2: {
      uint32_t a;
      memcpy(&a, m.payload, sizeof(a));
      MotionControl::setAccel(a);
      return ACK_OK;
    }

    case CMD_MOVE_BY: {
      int32_t d;
      memcpy(&d, m.payload, sizeof(d));
      MotionControl::moveBy(d);
      return ACK_OK;
    }

    case CMD_MOVE_TO: {
      int32_t t;
      memcpy(&t, m.payload, sizeof(t));
      MotionControl::moveTo(t);
      return ACK_OK;
    }

    case CMD_VELOCITY: {
      int32_t v;
      memcpy(&v, m.payload, sizeof(v));
      MotionControl::velocity(v);
      return ACK_OK;
    }

    case CMD_STOP_DECEL:
      MotionControl::stopDecel();
      return ACK_OK;

    case CMD_FORCE_STOP:
      MotionControl::forceStop();
      return ACK_OK;

    case CMD_SET_ENCODER_POLL_US: {
      uint32_t us;
      memcpy(&us, m.payload, sizeof(us));
      MotionControl::setEncoderPollUs(us);
      return ACK_OK;
    }

    case CMD_ENC_SET_TO_MOTOR:
      MotionControl::encoderSetToMotorPosition();
      return ACK_OK;

    case CMD_MOTOR_SET_TO_ENC:
      MotionControl::motorSetToEncoderPosition();
      return ACK_OK;

    case CMD_MOTOR_LOGICAL_ZERO:
      MotionControl::motorLogicalZero();
      return ACK_OK;

    case CMD_ENC_OFFSET_ZERO:
      MotionControl::encoderOffsetZero();
      return ACK_OK;

    case CMD_SET_MISMATCH_BASE_FULL_Q: {
      int32_t q;
      memcpy(&q, m.payload, sizeof(q));
      MotionControl::setMismatchBaseFullQ(q);
      return ACK_OK;
    }

    case CMD_SET_MISMATCH_GAIN_FULL_Q: {
      int32_t q;
      memcpy(&q, m.payload, sizeof(q));
      MotionControl::setMismatchGainFullPerRpsQ(q);
      return ACK_OK;
    }

    case CMD_SET_MISMATCH_CHECK_US: {
      uint32_t us;
      memcpy(&us, m.payload, sizeof(us));
      MotionControl::setMismatchCheckUs(us);
      return ACK_OK;
    }

    case CMD_SET_THERMISTOR_PARAMS: {
      uint32_t rFixed, r0;
      uint16_t beta;
      int16_t t0x10;
      uint8_t samples;

      memcpy(&rFixed, m.payload + 0, 4);
      memcpy(&r0,     m.payload + 4, 4);
      memcpy(&beta,   m.payload + 8, 2);
      memcpy(&t0x10,  m.payload + 10, 2);
      samples = m.payload[12];

      Sensors::setThermistorParams(rFixed, r0, beta, t0x10, samples);
      return ACK_OK;
    }

    case CMD_SET_NODE_ID: {
      uint16_t id;
      memcpy(&id, m.payload, sizeof(id));
      if (id > 32) return ACK_BAD_ARG;
      g_node_id = id;
      (void)g_prefs.putUShort("id", id);
      return ACK_OK;
    }

    case CMD_SAVE_CONFIG_NVS:
      MotionControl::saveConfigToNvs();
      Sensors::saveConfigToNvs();
      return ACK_OK;

    case CMD_REQUEST_STATUS_NOW:
      g_status_request_pending = true;
      return ACK_OK;

    default:
      return ACK_UNKNOWN_CMD;
  }
}

// -------- callbacks (ESP-IDF 5.x / Arduino-ESP32 v3.x) ----------
static void onRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  if (!info || !data) return;

  const uint8_t* mac = info->src_addr;

  if (len != (int)sizeof(MsgCommand)) return;

  MsgCommand m;
  memcpy(&m, data, sizeof(m));

  if (m.magic != MN_MAGIC || m.version != MN_VERSION || m.type != MSG_CMD) return;

  setMasterFrom(mac);

  if (g_cmdQ) {
    CmdItem item{};
    memcpy(item.mac, mac, 6);
    item.msg = m;
    (void)xQueueSend(g_cmdQ, &item, 0);
  }
}

static void onSent(const wifi_tx_info_t* /*tx_info*/, esp_now_send_status_t /*status*/) {
  // empty (LED handled in main loop after send)
}

// -------- public API ----------
static inline bool begin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(50);

  // Force a fixed channel for ESP-NOW (MUST match MasterNode)
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  macToStr(mac, g_selfMacStr);

  (void)g_prefs.begin("mn", false);
  g_node_id = (uint16_t)g_prefs.getUShort("id", 0);

  if (esp_now_init() != ESP_OK) return false;

  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  g_cmdQ = xQueueCreate(12, sizeof(CmdItem));
  if (!g_cmdQ) return false;

#if MN_DEBUG
  Serial.println("[MN] ESP-NOW init OK");
#endif
  return true;
}


static inline void service() {
  if (!g_cmdQ) return;

  // Dedup ring so retries never re-execute motor actions.
  static constexpr uint8_t DEDUP_SLOTS = 64;
  struct DedupEntry { uint16_t seq; uint16_t cmd; uint8_t code; bool valid; };
  static DedupEntry dedup[DEDUP_SLOTS] = {};
  static uint8_t dedup_head = 0;

  CmdItem item;
  while (xQueueReceive(g_cmdQ, &item, 0) == pdTRUE) {
    bool is_dup = false;
    uint8_t code = ACK_OK;

    for (uint8_t i = 0; i < DEDUP_SLOTS; i++) {
      if (!dedup[i].valid) continue;
      if (dedup[i].seq != item.msg.seq) continue;
      if (dedup[i].cmd != item.msg.cmd) continue;
      code = dedup[i].code;
      is_dup = true;
      break;
    }

    if (!is_dup) {
      code = handleCommand(item.msg);
      dedup[dedup_head] = DedupEntry{item.msg.seq, item.msg.cmd, code, true};
      dedup_head = (uint8_t)((dedup_head + 1) % DEDUP_SLOTS);
    }

    sendAckTo(item.mac, item.msg.cmd, item.msg.seq, code);
  }
}

} // namespace EspNowComms
