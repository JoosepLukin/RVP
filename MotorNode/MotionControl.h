#pragma once
#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <SPI.h>
#include <Preferences.h>
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
// NVS persistence
// =====================
static Preferences g_prefs;
static bool g_prefs_inited = false;
// ensurePrefs: Lazy-init Preferences storage for motion config.
static inline void ensurePrefs() {
  if (g_prefs_inited) return;
  (void)g_prefs.begin("mn_mc", false);
  g_prefs_inited = true;
}

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
// edgeFactor: Returns 2 when dedge is enabled (double-edge stepping), otherwise 1.
static inline uint32_t edgeFactor() { return g_dedge ? 2u : 1u; }
// isValidMicrosteps: True for supported TMC2209 microstep values.
static inline bool isValidMicrosteps(uint16_t u /* microsteps per fullstep */) {
  return u==1||u==2||u==4||u==8||u==16||u==32||u==64||u==128||u==256;
}
// stepsPerRevUser: User-space steps per revolution (FULLSTEPS_PER_REV * microsteps).
static inline uint32_t stepsPerRevUser() { return (uint32_t)FULLSTEPS_PER_REV * (uint32_t)g_usteps; }
// maxUserSpeedSps: Max user-space speed in steps/s given the pulse rate limit and edgeFactor().
static inline uint32_t maxUserSpeedSps() { return FAS_MAX_PULSE_HZ_ESP32 * edgeFactor(); }
// ceilDivU32: Unsigned ceil(a / b). b must be non-zero.
static inline uint32_t ceilDivU32(uint32_t a /* numerator */, uint32_t b /* denominator (non-zero) */) { return (a + b - 1) / b; }
// userSpsToPulseHz: Convert user steps/s into step pulse Hz (accounts for dedge).
static inline uint32_t userSpsToPulseHz(uint32_t user_sps /* speed in user steps/s */) { return ceilDivU32(user_sps, edgeFactor()); }
// userAccelToPulseHz2: Convert user accel (steps/s^2) into step pulse Hz^2 (accounts for dedge).
static inline uint32_t userAccelToPulseHz2(uint32_t user_accel /* accel in user steps/s^2 */) { return ceilDivU32(user_accel, edgeFactor()); }

// userStepsToLibSteps: Convert user-space steps into FastAccelStepper "steps" (accounts for dedge).
static inline int32_t userStepsToLibSteps(int32_t user_steps /* signed user steps */) {
  int64_t s = user_steps;
  uint32_t ef = edgeFactor();
  if (s >= 0) return (int32_t)((s + (ef - 1)) / (int64_t)ef);
  else        return (int32_t)(-(((-s) + (ef - 1)) / (int64_t)ef));
}
// libStepsToUserSteps: Convert FastAccelStepper "steps" into user-space steps (accounts for dedge).
static inline int32_t libStepsToUserSteps(int32_t lib_steps /* signed library steps */) {
  return (int32_t)((int64_t)lib_steps * (int64_t)edgeFactor());
}

// getRawLibPos: Physical stepper position (FastAccelStepper units, no bias).
static inline int32_t getRawLibPos() { return g_stepper ? g_stepper->getCurrentPosition() : 0; }

// getLogicalLibPos: Logical position = raw position + bias (used for user-facing position).
static inline int32_t getLogicalLibPos() {
  int32_t b; noInterrupts(); b = g_pos_bias_lib; interrupts();
  return getRawLibPos() + b;
}
// getLogicalUserPos: Logical position in user-space steps.
static inline int32_t getLogicalUserPos() { return libStepsToUserSteps(getLogicalLibPos()); }

// setBiasLib: Set logical bias (FastAccelStepper units).
static inline void setBiasLib(int32_t new_bias /* new bias in library steps */) { noInterrupts(); g_pos_bias_lib = new_bias; interrupts(); }
// addBiasLib: Adjust logical bias by delta (FastAccelStepper units).
static inline void addBiasLib(int32_t delta /* delta bias in library steps */) { noInterrupts(); g_pos_bias_lib += delta; interrupts(); }

// clearMoveTargetTracking: Clear stored raw target used by closed-loop corrections.
static inline void clearMoveTargetTracking() { noInterrupts(); g_has_raw_target = false; interrupts(); }

// enableOutputsGuarded: Enable stepper outputs and apply a guard delay (if configured).
static void enableOutputsGuarded() {
  if (!g_stepper) return;
  g_stepper->enableOutputs();
  g_outputs_enabled = true;
  if (ENABLE_GUARD_US) delayMicroseconds(ENABLE_GUARD_US);
}
// disableOutputsGuarded: Disable stepper outputs and apply a guard delay (if configured).
static void disableOutputsGuarded() {
  if (!g_stepper) return;
  if (DISABLE_GUARD_US) delayMicroseconds(DISABLE_GUARD_US);
  g_stepper->disableOutputs();
  g_outputs_enabled = false;
}

// applyConfig: Write current TMC2209 settings (microsteps, currents, flags) to the driver.
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

// applySpeedAccelNowIfSupported: Apply speed/accel immediately (library-version dependent).
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

// setMotorSpeedSps: Set speed in user steps/s (clamped), updating the stepper driver.
static bool setMotorSpeedSps(uint32_t speed_sps_user /* absolute speed in user steps/s */) {
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

// setMotorAccelSps2: Set acceleration in user steps/s^2 (min 1), updating the stepper driver.
static bool setMotorAccelSps2(uint32_t accel_sps2_user /* absolute accel in user steps/s^2 */) {
  if (!g_stepper) return false;
  if (accel_sps2_user == 0) accel_sps2_user = 1;

  g_accel_sps2_user = accel_sps2_user;

  uint32_t a_hz2 = userAccelToPulseHz2(accel_sps2_user);
  if (a_hz2 < 1) a_hz2 = 1;

  int rc = g_stepper->setAcceleration(a_hz2);
  if (g_mode == Mode::VELOCITY && g_stepper->isRunning()) applySpeedAccelNowIfSupported();
  return (rc == 0);
}

// applySpeedAccelToStepper: Re-apply current speed/accel settings to the stepper driver.
static void applySpeedAccelToStepper() {
  (void)setMotorSpeedSps(g_speed_sps_user);
  (void)setMotorAccelSps2(g_accel_sps2_user);
}

// =====================
// AS5047P SPI read
// =====================
// evenParity15: Compute even parity over the low 15 bits (AS5047P frame format).
static inline uint8_t evenParity15(uint16_t v15 /* 15-bit value */) {
  uint32_t ones = __builtin_popcount((uint32_t)(v15 & 0x7FFF));
  return (ones & 1) ? 1 : 0;
}
// makeReadCmd: Build a 16-bit read frame for a 14-bit register address.
static inline uint16_t makeReadCmd(uint16_t addr14 /* 14-bit register address */) {
  uint16_t frame = (uint16_t)(0x4000 | (addr14 & 0x3FFF));
  uint8_t par = evenParity15(frame);
  if (par) frame |= 0x8000; else frame &= 0x7FFF;
  return frame;
}
// makeNopCmd: Build a NOP frame (used to clock out the response after a read).
static inline uint16_t makeNopCmd() {
  uint16_t frame = (uint16_t)(0x4000 | (AS5047P_REG_NOP & 0x3FFF));
  uint8_t par = evenParity15(frame);
  if (par) frame |= 0x8000; else frame &= 0x7FFF;
  return frame;
}
// as5047pReadReg14: Read a 14-bit register value from AS5047P over SPI.
static uint16_t as5047pReadReg14(uint16_t addr14 /* 14-bit register address */) {
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
// countsToUserSteps: Convert encoder counts (unwrapped+offset) into user-space steps.
static inline int32_t countsToUserSteps(int64_t counts_unwrapped_plus_offset /* encoder counts (unwrapped + offset) */) {
  const int64_t spr = (int64_t)stepsPerRevUser();
  int64_t num = counts_unwrapped_plus_offset * spr;
  if (num >= 0) num += 8192;
  else          num -= 8192;
  return (spr == 0) ? 0 : (int32_t)(num / 16384);
}
// userStepsToCounts: Convert user-space steps into encoder counts (for offset math).
static inline int64_t userStepsToCounts(int32_t user_steps /* signed user steps */) {
  const int64_t spr = (int64_t)stepsPerRevUser();
  if (spr == 0) return 0;
  int64_t num = (int64_t)user_steps * 16384;
  if (num >= 0) num += (spr / 2);
  else          num -= (spr / 2);
  return (num / spr);
}

// encoderPollService: Poll AS5047P periodically, update unwrapped counts and encoderUserPos().
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
// estimateSpeedSpsMag: Approx current speed magnitude for dynamic threshold scaling.
static inline uint32_t estimateSpeedSpsMag() {
  return (g_stepper && g_stepper->isRunning()) ? g_speed_sps_user : 0;
}

// dynamicThresholdUserSteps: Current mismatch threshold in user-space steps.
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

// mismatchService: Periodically compute motor-vs-encoder error and latch mismatch events.
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
// closedLoopCorrectService: While moving, nudge the raw target if motor-vs-encoder error exceeds threshold.
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
// closedLoopIdleRecoveryService: When idle in CL mode 1, run a short corrective move if mismatch is active.
static void closedLoopIdleRecoveryService() {
  if (g_cl_mode != 1 || !g_stepper) return;
  if (g_mode != Mode::IDLE) return;
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
  if (!g_outputs_enabled) enableOutputsGuarded();

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
// cmdStopDecel: Request a smooth stop (decelerate) if moving; otherwise go idle.
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

// cmdForceStopNow: Hard-stop immediately (and reset internal state).
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

// cmdMoveBy: Move by delta user-steps (signed), enabling outputs as needed.
static void cmdMoveBy(int32_t delta_steps_user /* signed delta in user steps */) {
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

// cmdMoveTo: Move to an absolute logical position in user-steps (enables outputs as needed).
static void cmdMoveTo(int32_t abs_steps_user /* absolute logical position in user steps */) {
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

// cmdVelocity: Run continuously at a signed user speed (0=stop).
static void cmdVelocity(int32_t speed_sps_user_signed /* signed speed in user steps/s */) {
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
// serviceMotion: Advance the motion state machine and auto-disable outputs when allowed.
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
      if (!g_keep_enabled) disableOutputsGuarded();
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
static constexpr uint32_t MC_CFG_MAGIC   = 0x4D4E4346; // 'MNCF'
static constexpr uint16_t MC_CFG_VERSION = 1;

struct MotionConfigBlob {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved0;

  uint32_t speed_sps;
  uint32_t accel_sps2;
  uint32_t mismatch_check_us;
  uint32_t enc_poll_us;

  int32_t mismatch_base_full_q;
  int32_t mismatch_gain_full_q;

  uint16_t usteps;
  uint8_t  irun;
  uint8_t  ihold;
  uint8_t  iholddelay;
  uint8_t  intpol;
  uint8_t  dedge;
  uint8_t  keep_enabled;
  uint8_t  cl_mode;
  uint8_t  rsv[3];
};

static_assert(sizeof(MotionConfigBlob) == 44, "MotionConfigBlob size mismatch");

// loadConfigFromNvs: Restore MotionControl config from NVS (if present).
static inline void loadConfigFromNvs() {
  ensurePrefs();

  MotionConfigBlob blob{};
  if (g_prefs.getBytesLength("cfg") != sizeof(blob)) return;
  if (g_prefs.getBytes("cfg", &blob, sizeof(blob)) != sizeof(blob)) return;
  if (blob.magic != MC_CFG_MAGIC || blob.version != MC_CFG_VERSION) return;

  // Apply with validation/clamping
  g_keep_enabled = (blob.keep_enabled != 0);
  g_cl_mode = (blob.cl_mode == 0) ? 0 : 1;

  if (isValidMicrosteps(blob.usteps)) {
    g_usteps = blob.usteps;
  }

  if (blob.irun <= 31) g_irun = blob.irun;
  if (blob.ihold <= 31) g_ihold = blob.ihold;
  if (blob.iholddelay <= 15) g_iholddelay = blob.iholddelay;

  g_intpol = (blob.intpol != 0);
  g_dedge  = (blob.dedge  != 0);

  if (blob.speed_sps != 0) g_speed_sps_user = blob.speed_sps;
  if (blob.accel_sps2 != 0) g_accel_sps2_user = blob.accel_sps2;

  noInterrupts();
  g_mismatch_base_full_q = blob.mismatch_base_full_q;
  g_mismatch_gain_full_per_rps_q = blob.mismatch_gain_full_q;
  interrupts();

  uint32_t mc_us = blob.mismatch_check_us;
  if (mc_us < 500) mc_us = 500;
  if (mc_us > 50000) mc_us = 50000;
  g_mismatch_check_us = mc_us;
  g_mismatch_next_us = micros() + g_mismatch_check_us;
  g_cl_next_us = micros() + g_mismatch_check_us;

  uint32_t ep_us = blob.enc_poll_us;
  if (ep_us < 200) ep_us = 200;
  if (ep_us > 20000) ep_us = 20000;
  g_enc_poll_us = ep_us;
  g_enc_next_us = micros() + g_enc_poll_us;
}

// saveConfigToNvs: Persist current MotionControl config to NVS.
static inline void saveConfigToNvs() {
  ensurePrefs();

  MotionConfigBlob blob{};
  blob.magic = MC_CFG_MAGIC;
  blob.version = MC_CFG_VERSION;
  blob.reserved0 = 0;

  blob.speed_sps = g_speed_sps_user;
  blob.accel_sps2 = g_accel_sps2_user;
  blob.mismatch_check_us = g_mismatch_check_us;
  blob.enc_poll_us = g_enc_poll_us;

  noInterrupts();
  blob.mismatch_base_full_q = g_mismatch_base_full_q;
  blob.mismatch_gain_full_q = g_mismatch_gain_full_per_rps_q;
  interrupts();

  blob.usteps = g_usteps;
  blob.irun = g_irun;
  blob.ihold = g_ihold;
  blob.iholddelay = g_iholddelay;
  blob.intpol = g_intpol ? 1 : 0;
  blob.dedge = g_dedge ? 1 : 0;
  blob.keep_enabled = g_keep_enabled ? 1 : 0;
  blob.cl_mode = g_cl_mode;
  memset(blob.rsv, 0, sizeof(blob.rsv));

  (void)g_prefs.putBytes("cfg", &blob, sizeof(blob));
}

// begin: Initialize driver, stepper engine, encoder SPI, and restore saved config.
static inline bool begin() {
  loadConfigFromNvs();

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

  // Prime encoder state so ENC_ZERO works immediately after boot.
  uint16_t raw = as5047pReadReg14(AS5047P_REG_ANGLECOM);
  g_enc_raw14 = raw;
  g_enc_prev_raw14 = raw;
  g_enc_has_prev = true;
  g_enc_unwrap_counts = (int64_t)raw;
  g_enc_pos_steps = countsToUserSteps(g_enc_unwrap_counts);
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

  if (g_keep_enabled) enableOutputsGuarded();

  return true;
}

// service: Periodic service (encoder poll, motion state machine, mismatch + CL).
static inline void service() {
  encoderPollService();
  serviceMotion();
  mismatchService();
  closedLoopCorrectService();
  closedLoopIdleRecoveryService();
}

// setOutputsEnabled: Immediately enable/disable driver outputs (independent of keep-enabled).
static inline void setOutputsEnabled(bool en /* true to enable outputs */) {
  if (en) {
    applyConfig();
    applySpeedAccelToStepper();
    if (!g_outputs_enabled) enableOutputsGuarded();
  } else {
    disableOutputsGuarded();
  }
}
// outputsEnabled: Current driver outputs state.
static inline bool outputsEnabled() { return g_outputs_enabled; }

// setKeepEnabled: Keep outputs enabled while idle/moving (when false, auto-disable after moves).
static inline void setKeepEnabled(bool en /* true to keep outputs enabled */) {
  g_keep_enabled = en;
  if (en) {
    applyConfig();
    applySpeedAccelToStepper();
    if (!g_outputs_enabled) enableOutputsGuarded();
    return;
  }
  // When disabling Keep while idle, drop outputs immediately so the UI feels "instant".
  if (g_mode == Mode::IDLE && g_outputs_enabled) {
    disableOutputsGuarded();
  }
}
// keepEnabled: Current keep-outputs flag.
static inline bool keepEnabled() { return g_keep_enabled; }

// setClosedLoopMode: Enable/disable closed-loop correction (0=off, 1=on).
static inline void setClosedLoopMode(uint8_t m /* 0=off; non-zero=on */) {
  uint8_t new_mode = (m == 0) ? 0 : 1;
  if (new_mode == g_cl_mode) return;
  g_cl_mode = new_mode;

  if (g_cl_mode == 0) {
    // If we're in a CL-initiated recovery move, stop it when CL is turned off.
    if (g_mode == Mode::RECOVERING && g_stepper && g_stepper->isRunning()) {
      g_vel_restart_pending = false;
      clearMoveTargetTracking();
      g_stepper->stopMove();
      g_mode = Mode::STOPPING;
    }
    return;
  }

  // Make CL feel immediate: run mismatch/correction logic ASAP (outputs are NOT forced on).
  uint32_t now = micros();
  g_mismatch_next_us = now;
  g_cl_next_us = now;
}
// closedLoopMode: Current closed-loop mode (0/1).
static inline uint8_t closedLoopMode() { return g_cl_mode; }

// applyDriverConfigNow: Force a re-write of TMC2209 configuration registers.
static inline void applyDriverConfigNow() { applyConfig(); }

// setMicrosteps: Set microsteps per fullstep (only supported values are accepted).
static inline void setMicrosteps(uint16_t u /* microsteps per fullstep */) {
  if (isValidMicrosteps(u)) g_usteps = u;
  applyConfig();
}
// microsteps: Current microsteps per fullstep.
static inline uint16_t microsteps() { return g_usteps; }

// setInterpolation: Enable/disable microstep interpolation in the driver.
static inline void setInterpolation(bool en /* true to enable interpolation */) { g_intpol = en; applyConfig(); }
// setDedge: Enable/disable double-edge stepping (affects pulse rate conversions).
static inline void setDedge(bool en /* true to enable double-edge stepping */) { g_dedge = en; applyConfig(); applySpeedAccelToStepper(); }

// setCurrents: Set TMC2209 currents (range-checked) and apply immediately.
static inline void setCurrents(uint8_t irun /* run current (0..31) */,
                               uint8_t ihold /* hold current (0..31) */,
                               uint8_t ihd /* hold delay (0..15) */) {
  if (irun <= 31) g_irun = irun;
  if (ihold <= 31) g_ihold = ihold;
  if (ihd <= 15) g_iholddelay = ihd;
  applyConfig();
}

// setSpeed: Set move/velocity speed (user steps/s).
static inline void setSpeed(uint32_t sps /* speed in user steps/s */) { (void)setMotorSpeedSps(sps); }
// setAccel: Set move/velocity acceleration (user steps/s^2).
static inline void setAccel(uint32_t sps2 /* acceleration in user steps/s^2 */) { (void)setMotorAccelSps2(sps2); }

// moveBy: Start a relative move (non-blocking).
static inline void moveBy(int32_t delta_user /* signed delta in user steps */) { cmdMoveBy(delta_user); }
// moveTo: Start an absolute move in logical user-steps (non-blocking).
static inline void moveTo(int32_t abs_user /* absolute logical position in user steps */) { cmdMoveTo(abs_user); }
// velocity: Start/stop velocity mode (signed speed; 0 stops).
static inline void velocity(int32_t signed_sps /* signed speed in user steps/s */) { cmdVelocity(signed_sps); }
// stopDecel: Request a smooth stop (decelerate) if moving.
static inline void stopDecel() { cmdStopDecel(); }
// forceStop: Hard-stop immediately.
static inline void forceStop() { cmdForceStopNow(); }

// setEncoderPollUs: Set encoder polling period in microseconds (clamped).
static inline void setEncoderPollUs(uint32_t us /* poll interval in microseconds */) {
  if (us < 200) us = 200;
  if (us > 20000) us = 20000;
  g_enc_poll_us = us;
  g_enc_next_us = micros() + g_enc_poll_us;
}

// encoderSetToMotorPosition: Adjust encoder offset so encoderUserPos() matches motorUserPos().
static inline void encoderSetToMotorPosition() {
  int32_t motor_user = getLogicalUserPos();
  int32_t enc_user; noInterrupts(); enc_user = g_enc_pos_steps; interrupts();
  int32_t delta_user = motor_user - enc_user;
  if (delta_user == 0) return;
  int64_t delta_counts = userStepsToCounts(delta_user);
  noInterrupts(); g_enc_offset_counts += delta_counts; interrupts();
}
// motorSetToEncoderPosition: Adjust logical bias so motorUserPos() matches encoderUserPos().
static inline void motorSetToEncoderPosition() {
  if (!g_stepper) return;
  int32_t enc_user; noInterrupts(); enc_user = g_enc_pos_steps; interrupts();
  int32_t desired_logical_lib = userStepsToLibSteps(enc_user);
  int32_t raw_lib = getRawLibPos();
  int32_t new_bias = desired_logical_lib - raw_lib;
  setBiasLib(new_bias);
}

// motorLogicalZero: Set the current raw motor position as logical zero.
static inline void motorLogicalZero() {
  int32_t raw = getRawLibPos();
  setBiasLib(-raw);
}
// encoderOffsetZero: Set the current encoder physical position as encoderUserPos() == 0.
static inline void encoderOffsetZero() {
  // Set the current physical encoder position as the "zero" reference for encoderUserPos().
  noInterrupts();
  g_enc_offset_counts = -g_enc_unwrap_counts;
  g_enc_pos_steps = 0;
  interrupts();
}

// setMismatchBaseFullQ: Set base mismatch threshold in QFULL fullsteps.
static inline void setMismatchBaseFullQ(int32_t full_q /* base threshold fullsteps*QFULL */) { noInterrupts(); g_mismatch_base_full_q = full_q; interrupts(); }
// setMismatchGainFullPerRpsQ: Set speed-dependent mismatch gain in QFULL fullsteps/rps.
static inline void setMismatchGainFullPerRpsQ(int32_t full_q /* gain (fullsteps/rps)*QFULL */) { noInterrupts(); g_mismatch_gain_full_per_rps_q = full_q; interrupts(); }
// setMismatchCheckUs: Set mismatch/CL check period in microseconds (clamped).
static inline void setMismatchCheckUs(uint32_t us /* check interval in microseconds */) {
  if (us < 500) us = 500;
  if (us > 50000) us = 50000;
  g_mismatch_check_us = us;
  g_mismatch_next_us = micros() + g_mismatch_check_us;
  g_cl_next_us = micros() + g_mismatch_check_us;
}

// motorUserPos: Current logical motor position in user steps.
static inline int32_t motorUserPos() { return getLogicalUserPos(); }
// encoderUserPos: Current encoder position in user steps.
static inline int32_t encoderUserPos() { int32_t e; noInterrupts(); e = g_enc_pos_steps; interrupts(); return e; }

// thresholdUserSteps: Current mismatch threshold in user steps (dynamic).
static inline int32_t thresholdUserSteps() { return dynamicThresholdUserSteps(); }

// isMoving: True while the stepper is running (move or velocity).
static inline bool isMoving() { return g_stepper && g_stepper->isRunning(); }
// mismatchActive: True when motor-vs-encoder error exceeds the threshold.
static inline bool mismatchActive() { return g_mismatch_active; }
// missedStepEvents: Latched count of mismatch events.
static inline uint32_t missedStepEvents() { return g_missed_events; }

// drvStatus: Read TMC2209 DRV_STATUS register.
static inline uint32_t drvStatus() { return driver.DRV_STATUS(); }
// ioin: Read TMC2209 IOIN register.
static inline uint32_t ioin() { return driver.IOIN(); }
// ifcnt: Read TMC2209 IFCNT (UART write counter).
static inline uint8_t ifcnt() { return driver.IFCNT(); }

// speedSps: Current configured speed (user steps/s).
static inline uint32_t speedSps() { return g_speed_sps_user; }
// accelSps2: Current configured acceleration (user steps/s^2).
static inline uint32_t accelSps2() { return g_accel_sps2_user; }
// irun: Current configured run current (0..31).
static inline uint8_t irun() { return g_irun; }
// ihold: Current configured hold current (0..31).
static inline uint8_t ihold() { return g_ihold; }
// iholddelay: Current configured hold delay (0..15).
static inline uint8_t iholddelay() { return g_iholddelay; }

// encoderRaw14: Latest raw AS5047P angle reading (0..16383).
static inline uint16_t encoderRaw14() { return g_enc_raw14; }

} // namespace MotionControl
