#pragma once
#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <SPI.h>
#include <math.h>
#include <type_traits>
#include <utility>

#ifndef MN_DEBUG
#define MN_DEBUG 0
#endif

namespace MotionControl {

// =====================
// Pins
// =====================
static const int PIN_STEP = 10;
static const int PIN_DIR  = 11;
static const int PIN_ENN  = 47;   // ENN active-low

static const int PIN_TMC_RX = 7;
static const int PIN_TMC_TX = 16;

static const int PIN_ENC_SCK  = 13;
static const int PIN_ENC_MOSI = 14;
static const int PIN_ENC_MISO = 12;
static const int PIN_ENC_CS   = 21;

// =====================
// Driver
// =====================
static const uint8_t TMC_ADDR = 0;
static const float RSENSE_DUMMY = 0.11f;

static HardwareSerial TMCSerial(1);
static TMC2209Stepper driver(&TMCSerial, RSENSE_DUMMY, TMC_ADDR);

// ==========================
// Defaults
// ==========================
static uint8_t  g_irun       = 20;
static uint8_t  g_ihold      = 0;
static uint8_t  g_iholddelay = 1;

static uint16_t g_usteps     = 32;
static bool     g_intpol     = true;
static bool     g_dedge      = true;

static const uint16_t FULLSTEPS_PER_REV = 200;

// =====================
// FastAccelStepper
// =====================
static FastAccelStepperEngine g_engine;
static FastAccelStepper*      g_stepper = nullptr;

enum class Mode : uint8_t { IDLE, MOVING, VELOCITY, STOPPING, RECOVERING };
static Mode g_mode = Mode::IDLE;

static int8_t  g_vel_dir = +1;
static bool    g_vel_restart_pending = false;

static bool g_outputs_enabled = false;
static bool g_keep_enabled    = false;

// CL mode: 0=off, 1=active recovery when idle + correction during moves
static uint8_t g_cl_mode = 0;

// =====================
// Guards / limits
// =====================
static const uint32_t ENABLE_GUARD_US  = 300;
static const uint32_t DISABLE_GUARD_US = 0;

static const uint32_t FAS_MAX_PULSE_HZ_ESP32 = 200000;

static uint32_t g_speed_sps_user  = 20000;
static uint32_t g_accel_sps2_user = 200000;

// logical bias
static volatile int32_t g_pos_bias_lib = 0;

// =====================
// Encoder state (AS5047P)
// =====================
static volatile uint16_t g_enc_raw14 = 0;
static volatile int64_t  g_enc_unwrap_counts = 0;
static volatile int64_t  g_enc_offset_counts = 0;
static volatile int32_t  g_enc_pos_steps = 0;

static uint16_t g_enc_prev_raw14 = 0;
static bool     g_enc_has_prev = false;

static uint32_t g_enc_poll_us = 2000;
static uint32_t g_enc_next_us = 0;

static SPISettings g_enc_spi_settings(8000000, MSBFIRST, SPI_MODE1);

static const uint16_t AS5047P_REG_ANGLECOM = 0x3FFF;
static const uint16_t AS5047P_REG_NOP      = 0x0000;

// =====================
// Mismatch detection
// =====================
static const int32_t QFULL = 256;

static volatile int32_t g_mismatch_base_full_q = 2 * QFULL;
static volatile int32_t g_mismatch_gain_full_per_rps_q = 0;

static uint32_t g_mismatch_check_us = 2000;
static uint32_t g_mismatch_next_us  = 0;

static bool     g_mismatch_active   = false;
static bool     g_mismatch_latched  = false;
static uint32_t g_missed_events     = 0;

// Closed-loop correction timing
static uint32_t g_cl_next_us = 0;

// raw target for MOVING corrections
static volatile int32_t g_raw_target_lib = 0;
static volatile bool    g_has_raw_target = false;

// =====================
// Compile-time detection helpers
// =====================
template<typename T> using forceStop_expr = decltype(std::declval<T>().forceStop());
template<typename T>
using forceStopAndNewPosition_expr = decltype(std::declval<T>().forceStopAndNewPosition(int32_t{}));
template<typename T> using applySpeedAcceleration_expr = decltype(std::declval<T>().applySpeedAcceleration());

template<typename, template<typename> class, typename = void>
struct has_method : std::false_type {};
template<typename T, template<typename> class Expr>
struct has_method<T, Expr, std::void_t<Expr<T>>> : std::true_type {};

// =====================
// Helpers
// =====================
static inline uint32_t edgeFactor() { return g_dedge ? 2u : 1u; }
static inline uint32_t stepsPerRevUser() { return (uint32_t)FULLSTEPS_PER_REV * (uint32_t)g_usteps; }
static inline uint32_t maxUserSpeedSps() { return FAS_MAX_PULSE_HZ_ESP32 * edgeFactor(); }
static inline uint32_t ceilDivU32(uint32_t a, uint32_t b) { return (a + b - 1) / b; }
static inline uint32_t userSpsToPulseHz(uint32_t user_sps) { return ceilDivU32(user_sps, edgeFactor()); }
static inline uint32_t userAccelToPulseHz2(uint32_t user_accel) { return ceilDivU32(user_accel, edgeFactor()); }

static inline int32_t userStepsToLibSteps(int32_t user_steps) {
  int64_t s = user_steps;
  uint32_t ef = edgeFactor();
  if (s >= 0) return (int32_t)((s + (ef - 1)) / (int64_t)ef);
  else        return (int32_t)(-(((-s) + (ef - 1)) / (int64_t)ef));
}
static inline int32_t libStepsToUserSteps(int32_t lib_steps) {
  return (int32_t)((int64_t)lib_steps * (int64_t)edgeFactor());
}

static inline int32_t getRawLibPos() { return g_stepper ? g_stepper->getCurrentPosition() : 0; }

static inline int32_t getLogicalLibPos() {
  int32_t b; noInterrupts(); b = g_pos_bias_lib; interrupts();
  return getRawLibPos() + b;
}
static inline int32_t getLogicalUserPos() { return libStepsToUserSteps(getLogicalLibPos()); }

static inline void setBiasLib(int32_t new_bias) { noInterrupts(); g_pos_bias_lib = new_bias; interrupts(); }
static inline void addBiasLib(int32_t delta) { noInterrupts(); g_pos_bias_lib += delta; interrupts(); }

static inline void clearMoveTargetTracking() { noInterrupts(); g_has_raw_target = false; interrupts(); }

static void enableOutputsGuarded() {
  if (!g_stepper) return;
  g_stepper->enableOutputs();
  g_outputs_enabled = true;
  if (ENABLE_GUARD_US) delayMicroseconds(ENABLE_GUARD_US);
}
static void disableOutputsGuarded() {
  if (!g_stepper) return;
  if (DISABLE_GUARD_US) delayMicroseconds(DISABLE_GUARD_US);
  g_stepper->disableOutputs();
  g_outputs_enabled = false;
}

static void applyConfig() {
  driver.pdn_disable(true);
  driver.internal_Rsense(true);

  driver.mstep_reg_select(true);
  driver.multistep_filt(true);

  driver.en_spreadCycle(true);

  driver.ihold(g_ihold);
  driver.irun(g_irun);
  driver.iholddelay(g_iholddelay);
  driver.TPOWERDOWN(10);

  driver.microsteps(g_usteps);
  driver.intpol(g_intpol);
  driver.dedge(g_dedge);

  driver.toff(3);
  driver.tbl(2);
  driver.hstrt(4);
  driver.hend(0);
}

static void applySpeedAccelNowIfSupported() {
  if (!g_stepper) return;
  if constexpr (has_method<FastAccelStepper, applySpeedAcceleration_expr>::value) {
    g_stepper->applySpeedAcceleration();
  } else {
    if (g_mode == Mode::VELOCITY) {
      if (g_vel_dir > 0) g_stepper->runForward();
      else               g_stepper->runBackward();
    }
  }
}

static bool setMotorSpeedSps(uint32_t speed_sps_user) {
  if (!g_stepper) return false;
  uint32_t max_sps = maxUserSpeedSps();
  if (speed_sps_user > max_sps) speed_sps_user = max_sps;
  if (speed_sps_user == 0) speed_sps_user = 1;

  g_speed_sps_user = speed_sps_user;

  uint32_t hz = userSpsToPulseHz(speed_sps_user);
  if (hz < 1) hz = 1;
  if (hz > FAS_MAX_PULSE_HZ_ESP32) hz = FAS_MAX_PULSE_HZ_ESP32;

  int rc = g_stepper->setSpeedInHz(hz);
  if (g_mode == Mode::VELOCITY && g_stepper->isRunning()) applySpeedAccelNowIfSupported();
  return (rc == 0);
}

static bool setMotorAccelSps2(uint32_t accel_sps2_user) {
  if (!g_stepper) return false;
  if (accel_sps2_user == 0) accel_sps2_user = 1;

  g_accel_sps2_user = accel_sps2_user;

  uint32_t a_hz2 = userAccelToPulseHz2(accel_sps2_user);
  if (a_hz2 < 1) a_hz2 = 1;

  int rc = g_stepper->setAcceleration(a_hz2);
  if (g_mode == Mode::VELOCITY && g_stepper->isRunning()) applySpeedAccelNowIfSupported();
  return (rc == 0);
}

static void applySpeedAccelToStepper() {
  (void)setMotorSpeedSps(g_speed_sps_user);
  (void)setMotorAccelSps2(g_accel_sps2_user);
}

// =====================
// AS5047P SPI read
// =====================
static inline uint8_t evenParity15(uint16_t v15) {
  uint32_t ones = __builtin_popcount((uint32_t)(v15 & 0x7FFF));
  return (ones & 1) ? 1 : 0;
}
static inline uint16_t makeReadCmd(uint16_t addr14) {
  uint16_t frame = (uint16_t)(0x4000 | (addr14 & 0x3FFF));
  uint8_t par = evenParity15(frame);
  if (par) frame |= 0x8000; else frame &= 0x7FFF;
  return frame;
}
static inline uint16_t makeNopCmd() {
  uint16_t frame = (uint16_t)(0x4000 | (AS5047P_REG_NOP & 0x3FFF));
  uint8_t par = evenParity15(frame);
  if (par) frame |= 0x8000; else frame &= 0x7FFF;
  return frame;
}
static uint16_t as5047pReadReg14(uint16_t addr14) {
  SPI.beginTransaction(g_enc_spi_settings);

  digitalWrite(PIN_ENC_CS, LOW);
  (void)SPI.transfer16(makeReadCmd(addr14));
  digitalWrite(PIN_ENC_CS, HIGH);

  digitalWrite(PIN_ENC_CS, LOW);
  uint16_t rx = SPI.transfer16(makeNopCmd());
  digitalWrite(PIN_ENC_CS, HIGH);

  SPI.endTransaction();
  return (uint16_t)(rx & 0x3FFF);
}

// counts <-> steps
static inline int32_t countsToUserSteps(int64_t counts_unwrapped_plus_offset) {
  const int64_t spr = (int64_t)stepsPerRevUser();
  int64_t num = counts_unwrapped_plus_offset * spr;
  if (num >= 0) num += 8192;
  else          num -= 8192;
  return (spr == 0) ? 0 : (int32_t)(num / 16384);
}
static inline int64_t userStepsToCounts(int32_t user_steps) {
  const int64_t spr = (int64_t)stepsPerRevUser();
  if (spr == 0) return 0;
  int64_t num = (int64_t)user_steps * 16384;
  if (num >= 0) num += (spr / 2);
  else          num -= (spr / 2);
  return (num / spr);
}

static void encoderPollService() {
  uint32_t now = micros();
  if ((int32_t)(now - g_enc_next_us) < 0) return;
  g_enc_next_us = now + g_enc_poll_us;

  uint16_t raw = as5047pReadReg14(AS5047P_REG_ANGLECOM);
  g_enc_raw14 = raw;

  if (!g_enc_has_prev) {
    g_enc_prev_raw14 = raw;
    g_enc_has_prev = true;
    g_enc_unwrap_counts = (int64_t)raw;
  } else {
    int32_t delta = (int32_t)raw - (int32_t)g_enc_prev_raw14;
    if (delta > 8192)  delta -= 16384;
    if (delta < -8192) delta += 16384;
    g_enc_prev_raw14 = raw;
    g_enc_unwrap_counts += (int64_t)delta;
  }

  int64_t off;
  noInterrupts(); off = g_enc_offset_counts; interrupts();
  g_enc_pos_steps = countsToUserSteps(g_enc_unwrap_counts + off);
}

// =====================
// Threshold + mismatch
// =====================
static inline uint32_t estimateSpeedSpsMag() {
  return (g_stepper && g_stepper->isRunning()) ? g_speed_sps_user : 0;
}

static inline int32_t dynamicThresholdUserSteps() {
  int32_t base_full_q, gain_full_q;
  noInterrupts();
  base_full_q = g_mismatch_base_full_q;
  gain_full_q = g_mismatch_gain_full_per_rps_q;
  interrupts();

  uint32_t speed_sps = estimateSpeedSpsMag();
  uint32_t spr = stepsPerRevUser();

  int64_t add_full_q = 0;
  if (spr > 0 && gain_full_q > 0 && speed_sps > 0) {
    add_full_q = ((int64_t)speed_sps * (int64_t)gain_full_q) / (int64_t)spr;
  }

  int64_t dyn_full_q = (int64_t)base_full_q + add_full_q;
  if (dyn_full_q < 0) dyn_full_q = 0;
  if (dyn_full_q > INT32_MAX) dyn_full_q = INT32_MAX;

  int64_t thr_user = (dyn_full_q * (int64_t)g_usteps + (QFULL / 2)) / QFULL;
  if (thr_user < 0) thr_user = 0;
  if (thr_user > INT32_MAX) thr_user = INT32_MAX;
  return (int32_t)thr_user;
}

static void mismatchService() {
  uint32_t now = micros();
  if ((int32_t)(now - g_mismatch_next_us) < 0) return;
  g_mismatch_next_us = now + g_mismatch_check_us;

  int32_t motor_user = getLogicalUserPos();
  int32_t enc_user; noInterrupts(); enc_user = g_enc_pos_steps; interrupts();

  int32_t err = motor_user - enc_user;
  int32_t err_abs = (err >= 0) ? err : -err;
  int32_t thr_user = dynamicThresholdUserSteps();

  bool mism = (err_abs > thr_user);
  g_mismatch_active = mism;

  if (mism && !g_mismatch_latched) {
    g_mismatch_latched = true;
    g_missed_events++;
  } else if (!mism) {
    g_mismatch_latched = false;
  }
}

// =====================
// Closed-loop correction during MOVING
// =====================
static void closedLoopCorrectService() {
  if (g_cl_mode == 0 || !g_stepper) return;

  uint32_t now = micros();
  if ((int32_t)(now - g_cl_next_us) < 0) return;
  g_cl_next_us = now + g_mismatch_check_us;

  if (g_mode != Mode::MOVING) return;

  bool has_target;
  int32_t raw_target;
  noInterrupts();
  has_target = g_has_raw_target;
  raw_target = g_raw_target_lib;
  interrupts();
  if (!has_target) return;

  int32_t motor_user = getLogicalUserPos();
  int32_t enc_user; noInterrupts(); enc_user = g_enc_pos_steps; interrupts();

  int32_t err = motor_user - enc_user;
  int32_t err_abs = (err >= 0) ? err : -err;

  int32_t thr_user = dynamicThresholdUserSteps();
  if (err_abs <= thr_user) return;

  int32_t corr_user = err;
  int32_t corr_lib  = userStepsToLibSteps(corr_user);
  if (corr_lib == 0) return;

  raw_target += corr_lib;
  addBiasLib(-corr_lib);

  noInterrupts();
  g_raw_target_lib = raw_target;
  interrupts();

  g_stepper->moveTo(raw_target);
}

// =====================
// Active recovery while IDLE (CL mode 1)
// enable -> correct -> disable
// =====================
static void closedLoopIdleRecoveryService() {
  if (g_cl_mode != 1 || !g_stepper) return;
  if (g_mode != Mode::IDLE) return;
  if (g_outputs_enabled) return;
  if (!g_mismatch_active) return;

  int32_t motor_user = getLogicalUserPos();
  int32_t enc_user; noInterrupts(); enc_user = g_enc_pos_steps; interrupts();
  int32_t err = motor_user - enc_user;

  int32_t thr_user = dynamicThresholdUserSteps();
  int32_t err_abs = (err >= 0) ? err : -err;
  if (err_abs <= thr_user) return;

  int32_t corr_lib = userStepsToLibSteps(err);
  if (corr_lib == 0) return;

  applyConfig();
  applySpeedAccelToStepper();
  enableOutputsGuarded();

  // Keep logical position unchanged while physically moving corr_lib
  addBiasLib(-corr_lib);

  int32_t raw_target = getRawLibPos() + corr_lib;
  g_stepper->moveTo(raw_target);

  clearMoveTargetTracking();
  g_mode = Mode::RECOVERING;
}

// =====================
// Commands
// =====================
static void cmdStopDecel() {
  if (!g_stepper) return;

  g_vel_restart_pending = false;
  clearMoveTargetTracking();

  if (g_stepper->isRunning()) {
    g_stepper->stopMove();
    g_mode = Mode::STOPPING;
  } else {
    if (!g_keep_enabled) disableOutputsGuarded();
    g_mode = Mode::IDLE;
  }
}

static void cmdForceStopNow() {
  if (!g_stepper) return;

  g_vel_restart_pending = false;
  clearMoveTargetTracking();

  if constexpr (has_method<FastAccelStepper, forceStopAndNewPosition_expr>::value) {
    int32_t pos = g_stepper->getCurrentPosition();
    g_stepper->forceStopAndNewPosition(pos);
  } else if constexpr (has_method<FastAccelStepper, forceStop_expr>::value) {
    g_stepper->forceStop();
  } else {
    g_stepper->stopMove();
  }

  disableOutputsGuarded();
  g_mode = Mode::IDLE;
}

static void cmdMoveBy(int32_t delta_steps_user) {
  if (!g_stepper) return;
  g_vel_restart_pending = false;

  applyConfig();
  applySpeedAccelToStepper();
  if (!g_outputs_enabled) enableOutputsGuarded();

  int32_t delta_lib  = userStepsToLibSteps(delta_steps_user);

  int32_t raw_start  = getRawLibPos();
  int32_t raw_target = raw_start + delta_lib;

  noInterrupts();
  g_raw_target_lib = raw_target;
  g_has_raw_target = true;
  interrupts();

  g_stepper->moveTo(raw_target);
  g_mode = Mode::MOVING;
}

static void cmdMoveTo(int32_t abs_steps_user) {
  if (!g_stepper) return;
  g_vel_restart_pending = false;

  applyConfig();
  applySpeedAccelToStepper();
  if (!g_outputs_enabled) enableOutputsGuarded();

  int32_t target_logical_lib = userStepsToLibSteps(abs_steps_user);
  int32_t bias; noInterrupts(); bias = g_pos_bias_lib; interrupts();
  int32_t raw_target = target_logical_lib - bias;

  noInterrupts();
  g_raw_target_lib = raw_target;
  g_has_raw_target = true;
  interrupts();

  g_stepper->moveTo(raw_target);
  g_mode = Mode::MOVING;
}

static void cmdVelocity(int32_t speed_sps_user_signed) {
  if (!g_stepper) return;

  clearMoveTargetTracking();

  if (speed_sps_user_signed == 0) {
    cmdStopDecel();
    return;
  }

  int8_t new_dir = (speed_sps_user_signed > 0) ? +1 : -1;
  uint32_t new_speed_user = (uint32_t)abs(speed_sps_user_signed);

  setMotorSpeedSps(new_speed_user);
  setMotorAccelSps2(g_accel_sps2_user);

  if (g_mode == Mode::VELOCITY && g_stepper->isRunning()) {
    if (new_dir == g_vel_dir) {
      applySpeedAccelNowIfSupported();
      return;
    }
    g_vel_dir = new_dir;
    g_vel_restart_pending = true;
    g_stepper->stopMove();
    g_mode = Mode::STOPPING;
    return;
  }

  if (g_mode == Mode::STOPPING) {
    g_vel_dir = new_dir;
    g_vel_restart_pending = true;
    return;
  }

  g_vel_dir = new_dir;
  g_vel_restart_pending = false;

  applyConfig();
  applySpeedAccelToStepper();
  if (!g_outputs_enabled) enableOutputsGuarded();

  if (g_vel_dir > 0) g_stepper->runForward();
  else               g_stepper->runBackward();

  g_mode = Mode::VELOCITY;
}

// =====================
// serviceMotion (non-blocking)
// =====================
static void serviceMotion() {
  if (!g_stepper) return;

  if (g_mode == Mode::MOVING) {
    if (!g_stepper->isRunning()) {
      clearMoveTargetTracking();
      g_mode = Mode::IDLE;
      if (!g_keep_enabled) disableOutputsGuarded();
    }
    return;
  }

  if (g_mode == Mode::RECOVERING) {
    if (!g_stepper->isRunning()) {
      g_mode = Mode::IDLE;
      disableOutputsGuarded(); // must disable after CL1 recovery
    }
    return;
  }

  if (g_mode == Mode::STOPPING) {
    if (!g_stepper->isRunning()) {
      if (g_vel_restart_pending) {
        applyConfig();
        applySpeedAccelToStepper();
        if (!g_outputs_enabled) enableOutputsGuarded();

        if (g_vel_dir > 0) g_stepper->runForward();
        else               g_stepper->runBackward();

        g_vel_restart_pending = false;
        g_mode = Mode::VELOCITY;
        return;
      }

      g_mode = Mode::IDLE;
      if (!g_keep_enabled) disableOutputsGuarded();
    }
    return;
  }
}

// =====================
// Public API
// =====================
static inline bool begin() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENN, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_ENN, HIGH);

  TMCSerial.begin(115200, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);

  driver.begin();
  driver.toff(0);
  applyConfig();

  pinMode(PIN_ENC_CS, OUTPUT);
  digitalWrite(PIN_ENC_CS, HIGH);
  SPI.begin(PIN_ENC_SCK, PIN_ENC_MISO, PIN_ENC_MOSI, PIN_ENC_CS);

  (void)as5047pReadReg14(AS5047P_REG_ANGLECOM);
  g_enc_next_us = micros() + g_enc_poll_us;

  g_engine.init();
  g_stepper = g_engine.stepperConnectToPin(PIN_STEP);
  if (!g_stepper) return false;

  g_stepper->setDirectionPin(PIN_DIR, true);
  g_stepper->setEnablePin(PIN_ENN, true);
  g_stepper->setAutoEnable(false);
  g_stepper->disableOutputs();
  g_outputs_enabled = false;

  g_stepper->setCurrentPosition(0);
  setBiasLib(0);
  clearMoveTargetTracking();

  (void)setMotorSpeedSps(g_speed_sps_user);
  (void)setMotorAccelSps2(g_accel_sps2_user);

  g_mismatch_next_us = micros() + g_mismatch_check_us;
  g_cl_next_us       = micros() + g_mismatch_check_us;

  return true;
}

static inline void service() {
  encoderPollService();
  serviceMotion();
  mismatchService();
  closedLoopCorrectService();
  closedLoopIdleRecoveryService();
}

static inline void setOutputsEnabled(bool en) { if (en) enableOutputsGuarded(); else disableOutputsGuarded(); }
static inline bool outputsEnabled() { return g_outputs_enabled; }

static inline void setKeepEnabled(bool en) { g_keep_enabled = en; }
static inline bool keepEnabled() { return g_keep_enabled; }

static inline void setClosedLoopMode(uint8_t m) { g_cl_mode = (m == 0) ? 0 : 1; }
static inline uint8_t closedLoopMode() { return g_cl_mode; }

static inline void applyDriverConfigNow() { applyConfig(); }

static inline void setMicrosteps(uint16_t u) {
  if (u==1||u==2||u==4||u==8||u==16||u==32||u==64||u==128||u==256) g_usteps = u;
  applyConfig();
}
static inline uint16_t microsteps() { return g_usteps; }

static inline void setInterpolation(bool en) { g_intpol = en; applyConfig(); }
static inline void setDedge(bool en) { g_dedge = en; applyConfig(); applySpeedAccelToStepper(); }

static inline void setCurrents(uint8_t irun, uint8_t ihold, uint8_t ihd) {
  if (irun <= 31) g_irun = irun;
  if (ihold <= 31) g_ihold = ihold;
  if (ihd <= 15) g_iholddelay = ihd;
  applyConfig();
}

static inline void setSpeed(uint32_t sps) { (void)setMotorSpeedSps(sps); }
static inline void setAccel(uint32_t sps2) { (void)setMotorAccelSps2(sps2); }

static inline void moveBy(int32_t delta_user) { cmdMoveBy(delta_user); }
static inline void moveTo(int32_t abs_user) { cmdMoveTo(abs_user); }
static inline void velocity(int32_t signed_sps) { cmdVelocity(signed_sps); }
static inline void stopDecel() { cmdStopDecel(); }
static inline void forceStop() { cmdForceStopNow(); }

static inline void setEncoderPollUs(uint32_t us) {
  if (us < 200) us = 200;
  if (us > 20000) us = 20000;
  g_enc_poll_us = us;
  g_enc_next_us = micros() + g_enc_poll_us;
}

static inline void encoderSetToMotorPosition() {
  int32_t motor_user = getLogicalUserPos();
  int32_t enc_user; noInterrupts(); enc_user = g_enc_pos_steps; interrupts();
  int32_t delta_user = motor_user - enc_user;
  if (delta_user == 0) return;
  int64_t delta_counts = userStepsToCounts(delta_user);
  noInterrupts(); g_enc_offset_counts += delta_counts; interrupts();
}
static inline void motorSetToEncoderPosition() {
  if (!g_stepper) return;
  int32_t enc_user; noInterrupts(); enc_user = g_enc_pos_steps; interrupts();
  int32_t desired_logical_lib = userStepsToLibSteps(enc_user);
  int32_t raw_lib = getRawLibPos();
  int32_t new_bias = desired_logical_lib - raw_lib;
  setBiasLib(new_bias);
}

static inline void motorLogicalZero() {
  int32_t raw = getRawLibPos();
  setBiasLib(-raw);
}
static inline void encoderOffsetZero() { noInterrupts(); g_enc_offset_counts = 0; interrupts(); }

static inline void setMismatchBaseFullQ(int32_t full_q) { noInterrupts(); g_mismatch_base_full_q = full_q; interrupts(); }
static inline void setMismatchGainFullPerRpsQ(int32_t full_q) { noInterrupts(); g_mismatch_gain_full_per_rps_q = full_q; interrupts(); }
static inline void setMismatchCheckUs(uint32_t us) {
  if (us < 500) us = 500;
  if (us > 50000) us = 50000;
  g_mismatch_check_us = us;
  g_mismatch_next_us = micros() + g_mismatch_check_us;
  g_cl_next_us = micros() + g_mismatch_check_us;
}

static inline int32_t motorUserPos() { return getLogicalUserPos(); }
static inline int32_t encoderUserPos() { int32_t e; noInterrupts(); e = g_enc_pos_steps; interrupts(); return e; }

static inline int32_t thresholdUserSteps() { return dynamicThresholdUserSteps(); }

static inline bool isMoving() { return g_stepper && g_stepper->isRunning(); }
static inline bool mismatchActive() { return g_mismatch_active; }
static inline uint32_t missedStepEvents() { return g_missed_events; }

static inline uint32_t drvStatus() { return driver.DRV_STATUS(); }
static inline uint32_t ioin() { return driver.IOIN(); }
static inline uint8_t ifcnt() { return driver.IFCNT(); }

static inline uint32_t speedSps() { return g_speed_sps_user; }
static inline uint32_t accelSps2() { return g_accel_sps2_user; }
static inline uint8_t irun() { return g_irun; }
static inline uint8_t ihold() { return g_ihold; }
static inline uint8_t iholddelay() { return g_iholddelay; }

static inline uint16_t encoderRaw14() { return g_enc_raw14; }

} // namespace MotionControl
