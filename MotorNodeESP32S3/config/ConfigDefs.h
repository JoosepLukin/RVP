/*
  ConfigDefs.h
  ------------
  Configuration registry:
  - Param IDs (stable)
  - Defaults + min/max
  - Units (documented)
*/

#pragma once
#include <Arduino.h>

namespace Cfg {

// Param IDs (uint16) — keep stable forever
enum ParamId : uint16_t {
  // Motor/driver
  P_NODE_ID                 = 0x0001, // u8
  P_MICROSTEPS              = 0x0010, // u16 (min 16)
  P_MAX_SPEED_REV_S_Q100    = 0x0011, // u32 (rev/s *100)
  P_ACCEL_REV_S2_Q100       = 0x0012, // u32 (rev/s^2 *100)

	//TMC2209 current scale (0..31)
	P_TMC_IRUN                = 0x0013, // u8
	P_TMC_IHOLD               = 0x0014, // u8

  // Homing
  P_HOME_ANGLE_DEG_Q100     = 0x0100, // i32 (deg*100, wrapped 0..360*100)
  P_HOME_WINDOW_DEG_Q100    = 0x0101, // u16 (deg*100)
  P_HOME_SETTLE_MS          = 0x0102, // u16
  P_HOME_TIMEOUT_MS         = 0x0103, // u32
  P_HOME_SPEED_REV_S_Q100   = 0x0104, // u32
  P_HOME_ACCEL_REV_S2_Q100  = 0x0105, // u32

  // Step-loss (thresholds in FULL STEPS)
  P_LOSS_THR_HOLD_FS        = 0x0200, // u16
  P_LOSS_THR_RUN_FS         = 0x0201, // u16
  P_LOSS_THR_FAST_FS        = 0x0202, // u16
  P_LOSS_SPEED_HOLD_SPS     = 0x0203, // u32 (steps/s)
  P_LOSS_SPEED_FAST_SPS     = 0x0204, // u32
  P_LOSS_SPEED_HYST_SPS     = 0x0205, // u32
  P_LOSS_CONFIRM_HOLD_MS    = 0x0206, // u16
  P_LOSS_CONFIRM_RUN_MS     = 0x0207, // u16
  P_LOSS_CONFIRM_FAST_MS    = 0x0208, // u16
  P_RECOVER_TIME_MS         = 0x0209, // u16
  P_DISABLE_TIME_MS         = 0x020A, // u16

  // Thermistor
  P_NTC_R25_OHM             = 0x0300, // u32 (47000)
  P_R_FIXED_OHM             = 0x0301, // u32 (10000)
  P_NTC_BETA                = 0x0302, // u32 (TODO if known; default 3950)
  P_THERM_INTERVAL_MS       = 0x0303, // u16 (1000)

  // Telemetry
  P_TELEM_INTERVAL_MS       = 0x0400, // u16 (500..1000)

  // Encoder
  P_ENCODER_INVERT          = 0x0500, // bool (0/1)
};

struct ActiveConfig {
  uint8_t  node_id = 1;

  uint16_t microsteps = 16;
  uint32_t max_speed_rev_s_q100 = 500;   // 5.00 rev/s
  uint32_t accel_rev_s2_q100 = 200;      // 2.00 rev/s^2 default

  uint8_t  tmc_irun  = 5;  // 0..31
  uint8_t  tmc_ihold = 0;   // 0..31


  int32_t  home_angle_deg_q100 = 0;
  uint16_t home_window_deg_q100 = 50;    // 0.50 deg
  uint16_t home_settle_ms = 200;
  uint32_t home_timeout_ms = 8000;
  uint32_t home_speed_rev_s_q100 = 100;  // 1.00 rev/s
  uint32_t home_accel_rev_s2_q100 = 200;

  uint16_t loss_thr_hold_fs = 1;
  uint16_t loss_thr_run_fs  = 3;
  uint16_t loss_thr_fast_fs = 8;
  uint32_t loss_speed_hold_sps = 200;    // steps/s
  uint32_t loss_speed_fast_sps = 4000;   // steps/s
  uint32_t loss_speed_hyst_sps = 200;
  uint16_t loss_confirm_hold_ms = 300;
  uint16_t loss_confirm_run_ms  = 100;
  uint16_t loss_confirm_fast_ms = 80;
  uint16_t recover_time_ms = 1500;
  uint16_t disable_time_ms = 2000;

  uint32_t ntc_r25_ohm = 47000;
  uint32_t r_fixed_ohm = 10000;
  uint32_t ntc_beta = 3950;              // TODO: replace with real beta when known
  uint16_t therm_interval_ms = 1000;

  uint16_t telem_interval_ms = 500;

  uint8_t encoder_invert = 0;

  uint32_t revision = 1;
};

} // namespace Cfg
