/*
  StepLossMonitor.cpp
  -------------------
  Implements speed-band thresholding and timed confirmation.
  Uses encoder accuracy realistically:
  - compares in FULL STEPS (1.8deg per full step for 1.8deg motor)
  - converts final error back to microsteps for telemetry and correction
*/

#include "motion/StepLossMonitor.h"
#include <math.h>

void StepLossMonitor::reset() {
  _errorSteps = 0;
  _lossActive = false;
  _recovering = false;
  _faultRequested = false;
  _overThreshSinceMs = 0;
  _recoverStartMs = 0;
}

StepLossMonitor::SpeedBand StepLossMonitor::bandForSpeed(const Cfg::ActiveConfig& cfg, int32_t absSpeedSps) const {
  // absSpeedSps is microsteps/s. Compare to configured band thresholds (also in steps/s).
  // cfg.loss_speed_* are in "steps/s" (microsteps/s), so consistent.
  if ((uint32_t)absSpeedSps <= cfg.loss_speed_hold_sps) return BAND_HOLD;
  if ((uint32_t)absSpeedSps >= cfg.loss_speed_fast_sps) return BAND_FAST;
  return BAND_RUN;
}

static inline uint16_t thrForBand(const Cfg::ActiveConfig& cfg, StepLossMonitor::SpeedBand b) {
  if (b == StepLossMonitor::BAND_HOLD) return cfg.loss_thr_hold_fs;
  if (b == StepLossMonitor::BAND_FAST) return cfg.loss_thr_fast_fs;
  return cfg.loss_thr_run_fs;
}

static inline uint16_t confirmForBand(const Cfg::ActiveConfig& cfg, StepLossMonitor::SpeedBand b) {
  if (b == StepLossMonitor::BAND_HOLD) return cfg.loss_confirm_hold_ms;
  if (b == StepLossMonitor::BAND_FAST) return cfg.loss_confirm_fast_ms;
  return cfg.loss_confirm_run_ms;
}

void StepLossMonitor::update(const Cfg::ActiveConfig& cfg,
                            uint32_t nowMs,
                            int32_t posSteps,
                            int32_t speedSpsEst,
                            int32_t encAbsDegQ100) {
  // Convert encoder abs degrees -> FULL STEPS (1.8deg = 180 q100)
  // fullsteps = encAbsDegQ100 / 180
  int32_t encFullSteps = (int32_t)llround((double)encAbsDegQ100 / 180.0);

  // Commanded full steps = posSteps / microsteps
  int32_t ms = (int32_t)cfg.microsteps;
  if (ms < 16) ms = 16;
  int32_t cmdFullSteps = (int32_t)llround((double)posSteps / (double)ms);

  int32_t errFullSteps = encFullSteps - cmdFullSteps;

  // Store error as microsteps for telemetry
  _errorSteps = errFullSteps * ms;

  // Choose threshold band based on speed magnitude
  int32_t absSpeed = abs(speedSpsEst);
  SpeedBand b = bandForSpeed(cfg, absSpeed);

  uint16_t thrFS = thrForBand(cfg, b);
  uint16_t confirmMs = confirmForBand(cfg, b);

  bool over = (abs(errFullSteps) > (int32_t)thrFS);

  if (over) {
    if (_overThreshSinceMs == 0) _overThreshSinceMs = nowMs;

    if (!_lossActive && (nowMs - _overThreshSinceMs) >= confirmMs) {
      _lossActive = true;
      _recovering = true;
      _recoverStartMs = nowMs;
    }
  } else {
    _overThreshSinceMs = 0;
    // If we were recovering and now back within threshold -> clear
    if (_lossActive) {
      _lossActive = false;
      _recovering = false;
      _recoverStartMs = 0;
    }
  }

  // Recovery timeout -> request fault
  if (_recovering) {
    if ((nowMs - _recoverStartMs) >= cfg.recover_time_ms) {
      _faultRequested = true;
      _recovering = false;
    }
  }
}

int32_t StepLossMonitor::suggestedCorrectionSteps(const Cfg::ActiveConfig& cfg) const {
  // Suggest a nudge that tries to reduce error.
  // Clamp to a small-ish correction chunk to avoid oscillation.
  int32_t ms = (int32_t)cfg.microsteps;
  if (ms < 16) ms = 16;

  // Use microstep error, but correct in FULL STEPS increments (encoder-limited)
  int32_t errFullSteps = (int32_t)llround((double)_errorSteps / (double)ms);

  // Clamp correction to +/- 4 full steps per tick
  if (errFullSteps > 4) errFullSteps = 4;
  if (errFullSteps < -4) errFullSteps = -4;

  // We want to push motor command toward encoder, so move by -err (reduce error)
  return -errFullSteps * ms;
}
