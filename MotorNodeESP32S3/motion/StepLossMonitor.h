/*
  StepLossMonitor.h
  -----------------
  Step-loss detection and basic recovery controller.

  Compares:
  - commanded motor position (steps, from MotionEngine)
  - measured shaft position (encoder abs degrees, unwrapped)

  Detection thresholds are configurable (FULL STEPS):
  - hold / run / fast threshold, selected by measured speed (steps/s)
  - confirm times per speed region (ms)

  Outputs:
  - error in steps (microstep steps) for telemetry
  - lossActive flag
  - recoveryActive flag
  - fault request when recovery fails
*/

#pragma once
#include <Arduino.h>
#include "config/ConfigDefs.h"

class StepLossMonitor {
public:
  enum SpeedBand { BAND_HOLD, BAND_RUN, BAND_FAST };
  void reset();

  // Update monitor. Provide:
  // - cfg: current active config
  // - nowMs: millis()
  // - posSteps: motor commanded position in microsteps
  // - speedSpsEst: estimated motor speed in microsteps/s
  // - encAbsDegQ100: unwrapped encoder degrees (deg*100)
  void update(const Cfg::ActiveConfig& cfg,
              uint32_t nowMs,
              int32_t posSteps,
              int32_t speedSpsEst,
              int32_t encAbsDegQ100);

  // Telemetry outputs
  int32_t errorSteps() const { return _errorSteps; }    // in microsteps
  bool lossActive() const { return _lossActive; }
  bool recovering() const { return _recovering; }
  bool faultRequested() const { return _faultRequested; }

  // Recovery request: returns a suggested correction delta in microsteps.
  // (Controller may apply this as a nudge when safe.)
  int32_t suggestedCorrectionSteps(const Cfg::ActiveConfig& cfg) const;

  void clearFaultRequest() { _faultRequested = false; }

private:
 
  SpeedBand bandForSpeed(const Cfg::ActiveConfig& cfg, int32_t absSpeedSps) const;

  int32_t _errorSteps = 0;
  bool _lossActive = false;
  bool _recovering = false;
  bool _faultRequested = false;

  uint32_t _overThreshSinceMs = 0;
  uint32_t _recoverStartMs = 0;
};
