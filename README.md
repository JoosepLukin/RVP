## 1) Radio / ESP-NOW requirements

- **WiFi mode:** `WIFI_STA` (station)
- **Fixed channel:** `1` on both Master and MotorNodes
  - MotorNode hard-codes `WIFI_CHANNEL = 1` and calls `esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE)`.
- **Encryption:** none
- **Peers:**
  - MotorNode learns "the master" from the **source MAC** of the first valid command it receives.
  - After that, the MotorNode sends **all Status** frames and **all ACKs** to that master MAC.
  - Master can use **broadcast** (`FF:FF:FF:FF:FF:FF`) `CMD_PING` to discover nodes; MotorNodes will still unicast the ACK back to the master MAC.

Important note:
- MotorNode is effectively **single-master**: the most recent sender of a valid command becomes the master target for Status/ACK.

---

## 2) Message types (binary, fixed-size)

All packets are **little-endian** and **packed** (no padding). The MotorNode will ignore any incoming command that is **not exactly 40 bytes**.

Constants:
- `MN_MAGIC = 0x4D4E` (`'M''N'`)
- `MN_VERSION = 1`
- `WIFI_CHANNEL = 1`

Message types (`type` field):
- `MSG_CMD    = 1`
- `MSG_STATUS = 2`
- `MSG_ACK    = 3`

### 2.1 Command: `MsgCommand` (40 bytes)

```
struct __attribute__((packed)) MsgCommand {
  uint16_t magic;        // 0x4D4E
  uint8_t  version;      // 1
  uint8_t  type;         // 1 = MSG_CMD
  uint16_t cmd;          // CmdId (see below)
  uint16_t seq;          // sequence number chosen by master
  uint8_t  payload[32];  // command-specific, unused bytes = 0
};
```

Field layout (byte offsets):
- `0..1`   `magic`
- `2`      `version`
- `3`      `type`
- `4..5`   `cmd`
- `6..7`   `seq`
- `8..39`  `payload[32]`

### 2.2 ACK: `MsgAck` (12 bytes)

```
struct __attribute__((packed)) MsgAck {
  uint16_t magic;     // 0x4D4E
  uint8_t  version;   // 1
  uint8_t  type;      // 3 = MSG_ACK
  uint16_t cmd;       // echoed command id
  uint16_t seq;       // echoed sequence
  uint8_t  code;      // AckCode
  uint16_t node_id;   // 0..32 (0 = unassigned)
  uint8_t  rsv0;      // 0
};
```

ACK codes (`code` field):
- `0` = `ACK_OK`
- `1` = `ACK_BAD_ARG`
- `4` = `ACK_UNKNOWN_CMD`

### 2.3 Status: `MsgStatus` (62 bytes)

MotorNode streams Status every ~500ms **after** it has learned a master (i.e., after receiving at least one valid command).

```
struct __attribute__((packed)) MsgStatus {
  uint16_t magic;        // 0x4D4E
  uint8_t  version;      // 1
  uint8_t  type;         // 2 = MSG_STATUS
  uint16_t seq;          // status sequence (MotorNode internal)
  uint32_t uptime_ms;

  int32_t motor_pos_user;  // logical motor position (user steps)
  int32_t enc_pos_user;    // encoder position (user steps)
  int32_t err_user;        // motor_pos_user - enc_pos_user
  int32_t thr_user;        // current mismatch threshold (user steps)

  uint32_t missed_events;  // latched mismatch events count

  int16_t  temp_c_x10;     // degC*10, or INT16_MIN if invalid
  uint8_t  moving;         // 1 while stepper is running
  uint8_t  outputs_enabled;// 1 if driver outputs are enabled
  uint8_t  cl_mode;        // 0/1 closed-loop mode
  uint8_t  keep_enabled;   // 1 keeps outputs enabled after moves

  uint32_t speed_sps;      // configured speed (user steps/s)
  uint32_t accel_sps2;     // configured accel (user steps/s^2)

  uint16_t usteps;         // microsteps per fullstep (1..256)
  uint8_t  irun;           // 0..31
  uint8_t  ihold;          // 0..31
  uint8_t  iholddelay;     // 0..15
  uint8_t  rsv0;           // 0

  uint32_t drv_status;     // TMC2209 DRV_STATUS
  uint32_t ioin;           // TMC2209 IOIN
  uint8_t  ifcnt;          // TMC2209 IFCNT
  uint16_t node_id;        // 0..32 (0 = unassigned)
  uint8_t  rsv1;           // 0
};
```

Units:
- **User steps** = `fullsteps * microsteps` (e.g., 200-step motor with `usteps=32` -> `6400` user steps per revolution).

---

## 3) Reliability model (ACK + safe retries)

MotorNode sends an **ACK for every command** it processes.

To make ESP-NOW reliable, your master should:
1. Assign a `seq` number to each command (uint16).
2. Send the 40-byte `MsgCommand`.
3. Wait for a matching `MsgAck` with the same `(cmd, seq)`.
4. If no ACK within a timeout (typical: **80-150ms**), **re-send the exact same bytes** (same `cmd`, same `seq`, same payload).

MotorNode includes **duplicate-command protection**:
- Duplicates are detected by `(seq, cmd)` and will **not re-execute** the action.
- The MotorNode will re-send the **same ACK code** for duplicates.
- The dedup history is a small ring buffer (recent commands), so retries should happen promptly.

Practical recommendation:
- Don't blast commands faster than the node can service them. Wait for ACKs on configuration/motion commands.

---

## 4) Command list (CmdId + payload formats)

All payload integers are **little-endian**.

### Basics / modes

- `0x0001 CMD_PING`
  - Payload: none
  - Use: discovery / pairing (starts status streaming once received)

- `0x0010 CMD_SET_ENABLE`
  - Payload: `u8 enable` (`0`=disable outputs, `1`=enable outputs)

- `0x0011 CMD_SET_KEEP_ENABLED`
  - Payload: `u8 keep` (`0`=auto-disable after moves, `1`=keep outputs enabled)
  - Notes: enabling keep will also enable outputs immediately.

- `0x0012 CMD_SET_CL_MODE`
  - Payload: `u8 mode` (`0`=off, `1`=on)
  - Notes: does **not** force outputs on.

### Driver config

- `0x0013 CMD_APPLY_CONFIG_NOW`
  - Payload: none
  - Use: forces a rewrite of TMC2209 config registers.

- `0x0020 CMD_SET_MICROSTEPS`
  - Payload: `u16 usteps` (allowed: `1,2,4,8,16,32,64,128,256`)

- `0x0021 CMD_SET_INTPOL`
  - Payload: `u8 intpol` (`0`/`1`)

- `0x0022 CMD_SET_DEDGE`
  - Payload: `u8 dedge` (`0`/`1`)

- `0x0023 CMD_SET_CURRENTS`
  - Payload:
    - `u8 irun` (0..31)
    - `u8 ihold` (0..31)
    - `u8 iholddelay` (0..15)

### Motion config

- `0x0030 CMD_SET_SPEED_SPS`
  - Payload: `u32 speed_sps` (user steps/s, min 1, max clamped)

- `0x0031 CMD_SET_ACCEL_SPS2`
  - Payload: `u32 accel_sps2` (user steps/s^2, min 1)

### Motion commands (non-blocking)

- `0x0040 CMD_MOVE_BY`
  - Payload: `i32 delta_user_steps`

- `0x0041 CMD_MOVE_TO`
  - Payload: `i32 abs_user_steps` (logical position)

- `0x0042 CMD_VELOCITY`
  - Payload: `i32 signed_speed_sps` (`0` stops, `+` forward, `-` backward)

- `0x0043 CMD_STOP_DECEL`
  - Payload: none
  - Use: smooth decelerating stop.

- `0x0044 CMD_FORCE_STOP`
  - Payload: none
  - Use: immediate hard stop.

### Encoder / alignment

- `0x0050 CMD_SET_ENCODER_POLL_US`
  - Payload: `u32 poll_us` (clamped to `200..20000`)

- `0x0051 CMD_ENC_SET_TO_MOTOR`
  - Payload: none
  - Use: adjust encoder offset so `enc_pos_user == motor_pos_user` now.

- `0x0052 CMD_MOTOR_SET_TO_ENC`
  - Payload: none
  - Use: adjust motor logical bias so `motor_pos_user == enc_pos_user` now.

- `0x0053 CMD_MOTOR_LOGICAL_ZERO`
  - Payload: none
  - Use: set current motor logical position to 0 (does not move).

- `0x0054 CMD_ENC_OFFSET_ZERO`
  - Payload: none
  - Use: set current encoder position to 0 (offsets encoder without moving).

### Mismatch / closed-loop tuning

These values are in "QFULL" fixed point where `QFULL = 256`:
- Example: `2 fullsteps` -> `2 * 256 = 512`.

- `0x0060 CMD_SET_MISMATCH_BASE_FULL_Q`
  - Payload: `i32 base_fullsteps_q256`

- `0x0061 CMD_SET_MISMATCH_GAIN_FULL_Q`
  - Payload: `i32 gain_fullsteps_per_rps_q256`

- `0x0062 CMD_SET_MISMATCH_CHECK_US`
  - Payload: `u32 check_us` (clamped to `500..50000`)

### Thermistor (optional)

- `0x0070 CMD_SET_THERMISTOR_PARAMS`
  - Payload (packed):
    - `u32 rFixed_ohm`   @ offset `0`
    - `u32 r0_ohm`       @ offset `4`
    - `u16 beta`         @ offset `8`
    - `i16 t0_c_x10`     @ offset `10` (degC*10)
    - `u8 samples`       @ offset `12` (1..64)

### Node identity + persistence

- `0x00E0 CMD_SET_NODE_ID`
  - Payload: `u16 id` (valid `0..32`; `0` = unassigned)
  - Notes: persisted in NVS; returned in ACK/Status.

- `0x00E1 CMD_SAVE_CONFIG_NVS`
  - Payload: none
  - Notes: persists MotionControl + Sensors configuration to flash.

- `0x00F0 CMD_REQUEST_STATUS_NOW`
  - Payload: none
  - Notes: requests an immediate Status send (in addition to periodic streaming).

---

## 5) Example packets (hex)

All multi-byte values are little-endian.

### Example: `CMD_MOVE_BY` (+1000 user steps), `seq = 1`

Header (8 bytes):
- `magic=0x4D4E` -> `4E 4D`
- `version=1` -> `01`
- `type=MSG_CMD(1)` -> `01`
- `cmd=0x0040` -> `40 00`
- `seq=0x0001` -> `01 00`

Payload (32 bytes):
- `delta=i32(1000)` -> `E8 03 00 00` then 28x`00`

Full 40-byte command:
```
4E 4D 01 01 40 00 01 00  E8 03 00 00 00 00 00 00
00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00
```

### Example ACK (OK, node_id=2) for that command
```
4E 4D 01 03 40 00 01 00  00 02 00 00
```

---

## 6) Minimal MasterNode implementation sketch (Arduino / ESP32)

This is a small skeleton showing:
- fixed channel setup
- receive callback that parses ACK/Status
- sending a command

```cpp
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static constexpr int WIFI_CHANNEL = 1;
static constexpr uint16_t MN_MAGIC = 0x4D4E;
static constexpr uint8_t MN_VERSION = 1;

enum MsgType : uint8_t { MSG_CMD=1, MSG_STATUS=2, MSG_ACK=3 };
enum CmdId : uint16_t { CMD_PING=0x0001, CMD_MOVE_BY=0x0040 /* ... */ };

struct __attribute__((packed)) MsgCommand {
  uint16_t magic;
  uint8_t  version;
  uint8_t  type;
  uint16_t cmd;
  uint16_t seq;
  uint8_t  payload[32];
};

struct __attribute__((packed)) MsgAck {
  uint16_t magic;
  uint8_t  version;
  uint8_t  type;
  uint16_t cmd;
  uint16_t seq;
  uint8_t  code;
  uint16_t node_id;
  uint8_t  rsv0;
};

struct __attribute__((packed)) MsgStatus {
  uint16_t magic;
  uint8_t  version;
  uint8_t  type;
  uint16_t seq;
  uint32_t uptime_ms;
  int32_t motor_pos_user;
  int32_t enc_pos_user;
  int32_t err_user;
  int32_t thr_user;
  uint32_t missed_events;
  int16_t  temp_c_x10;
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

static uint16_t g_seq = 1;

static void onRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  if (!info || !data) return;
  if (len < 4) return;

  uint16_t magic = *(const uint16_t*)data;
  uint8_t version = data[2];
  uint8_t type = data[3];
  if (magic != MN_MAGIC || version != MN_VERSION) return;

  if (type == MSG_ACK && len == (int)sizeof(MsgAck)) {
    MsgAck a; memcpy(&a, data, sizeof(a));
    // TODO: match (a.cmd, a.seq) to your pending TX table
  } else if (type == MSG_STATUS && len == (int)sizeof(MsgStatus)) {
    MsgStatus s; memcpy(&s, data, sizeof(s));
    // TODO: store latest status by MAC or s.node_id
  }
}

static bool sendCommand(const uint8_t mac[6], uint16_t cmd, const void* payload, size_t payload_len) {
  MsgCommand m{};
  m.magic = MN_MAGIC;
  m.version = MN_VERSION;
  m.type = MSG_CMD;
  m.cmd = cmd;
  m.seq = g_seq++;
  if (payload && payload_len) memcpy(m.payload, payload, payload_len > 32 ? 32 : payload_len);
  return esp_now_send(mac, (const uint8_t*)&m, sizeof(m)) == ESP_OK;
}

void setup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) while(true) delay(1000);
  esp_now_register_recv_cb(onRecv);
}

void loop() {
  // Example: broadcast ping (no peer needed for broadcast)
  // uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  // sendCommand(bcast, CMD_PING, nullptr, 0);
  delay(1000);
}
```

To control multiple nodes:
- Use broadcast `CMD_PING` to discover MACs (you'll receive ACKs/Status from each).
- Track `node_id` from ACK/Status to build an `ID -> MAC` table.
- Unicast commands to the selected MAC(s) and use ACK timeouts + retries for reliability.

---

## 7) Save/restore behavior (flash)

MotorNode supports saving configuration to internal flash:
- Send your config commands (currents, usteps, speed/accel, keep/cl, thresholds, thermistor params, etc.).
- Send `CMD_SAVE_CONFIG_NVS` (`0x00E1`).

On boot, MotorNode restores:
- Motion config (speed/accel, thresholds, microsteps, currents, keep_enabled, cl_mode, encoder poll, etc.)
- Sensor config (thermistor params)
- Node ID (`CMD_SET_NODE_ID` stored separately)
