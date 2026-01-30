/*
  Protocol.h
  ----------
  ESP-NOW binary protocol (versioned) + TLV config messages.
  This drop adds payload structs for:
  - CMD_ENABLE / CMD_MOVE_ABS_DEG / CMD_VELOCITY_DPS / CMD_HOME_START
  - CMD_GET_CONFIG -> RSP_CONFIG (TLV list)
*/

#pragma once
#include <Arduino.h>
#include "system/Types.h"


namespace Proto {

static constexpr uint16_t MAGIC = 0xA55A;
static constexpr uint8_t  VERSION = 1;

enum MsgType : uint8_t {
  CMD_PING          = 0x01,
  CMD_ENABLE        = 0x02,
  CMD_STOP          = 0x08,

  CMD_MOVE_ABS_DEG  = 0x20,   // target in degrees (q100)
  CMD_VELOCITY_DPS  = 0x21,   // velocity in deg/s (q100)
  CMD_ZERO_POS      = 0x22,   // zero motor position (current = 0)

  CMD_HOME_START    = 0x30,

  CMD_GET_CAPS      = 0x40,
  RSP_CAPS          = 0xC0,

  CMD_GET_CONFIG    = 0x41,
  RSP_CONFIG        = 0xC1,
  CMD_SET_CONFIG    = 0x42,
  RSP_SET_RESULT    = 0xC2,
  CMD_SAVE_CONFIG   = 0x43,

  RSP_ACK           = 0x80,
  RSP_STATUS        = 0x81,
  RSP_EVENT         = 0x82
};

enum Flags : uint8_t {
  FLAG_ACK_REQ  = 1 << 0,
  FLAG_IS_ACK   = 1 << 1,
  FLAG_IS_EVENT = 1 << 2,
};

#pragma pack(push, 1)
struct Header {
  uint16_t magic;
  uint8_t  version;
  uint8_t  msg_type;
  uint8_t  flags;
  uint8_t  seq;
  uint8_t  node_id;      // 255 = broadcast
  uint8_t  payload_len;
  uint16_t crc16;        // CRC over header (crc16=0) + payload
};

// Generic ACK
struct AckPayload {
  uint8_t  ack_seq;
  uint8_t  result; // 0=OK, nonzero=error
};

struct CmdEnablePayload { uint8_t enable; };     // 0/1

struct CmdMoveAbsDegPayload { int32_t target_deg_q100; };

struct CmdVelocityDpsPayload { int32_t vel_dps_q100; };

struct CmdGetConfigPayload {
  // If list_len == 0 -> return "all params"
  // Else param_ids[list_len] follow (uint16 each)
  uint8_t list_len;
  // uint16_t param_ids[list_len]
};

// Status telemetry payload (compact)
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

// Config TLV
struct TLV {
  uint16_t param_id;
  uint8_t  type;
  uint8_t  len;
  // value[len] follows
};
#pragma pack(pop)

// Config SET flags
enum SetFlags : uint8_t {
  SET_APPLY_NOW        = 1 << 0,
  SET_APPLY_WHEN_IDLE  = 1 << 1,
  SET_SAVE_AFTER_APPLY = 1 << 2,
  SET_STRICT           = 1 << 3
};

// Param value types
enum ValueType : uint8_t {
  VT_U8  = 1,
  VT_U16 = 2,
  VT_U32 = 3,
  VT_I16 = 4,
  VT_I32 = 5,
  VT_BOOL = 6,
};

// CRC16
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t seed = 0xFFFF);

} // namespace Proto
