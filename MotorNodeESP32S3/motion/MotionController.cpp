/*
  MotionController.cpp
  --------------------
  Adds StepLossMonitor:
  - speed-band thresholds (hold/run/fast)
  - timed confirmation
  - recovery nudges while moving (position/velocity/homing)
  - fault on recovery timeout -> disables motor for disable_time_ms
*/

#include "motion/MotionController.h"
#include "motion_engine/MotionEngine.h"
#include "sensors/EncoderAS5047P.h"
#include "sensors/Thermistor.h"
#include "drivers/TMC2209Driver.h"
#include "config/ConfigStore.h"
#include <math.h>
#include "system/SharedState.h"


void MotionController::begin(MotionEngine* eng,
                            EncoderAS5047P* enc,
                            Thermistor* therm,
                            TMC2209Driver* tmc,
                            ConfigStore* cfg,
                            SharedState* state) {
  _eng = eng;
  _enc = enc;
  _therm = therm;
  _tmc = tmc;
  _cfg = cfg;
  _state = state;

  _st = MotorState::MOTOR_DISABLED;
  _homeSteps = 0;
  _targetSteps = 0;
  _velCmdSps = 0;

  _loss.reset();
  _faultCode = 0;
  _disabledUntilMs = 0;

  _lastSpeedMs = millis();
  _lastPosSteps = 0;
  _speedSpsEst = 0;
}

int32_t MotionController::stepsPerRev() const {
  const int32_t fullSteps = 200; // 1.8deg
  int32_t ms = (int32_t)_cfg->active().microsteps;
  if (ms < 16) ms = 16;
  return fullSteps * ms;
}

int32_t MotionController::degQ100ToSteps(int32_t degQ100) const {
  return (int32_t)llround((double)degQ100 * (double)stepsPerRev() / 36000.0);
}

int32_t MotionController::estimateSpeedSps(uint32_t nowMs, int32_t posSteps) {
  uint32_t dt = nowMs - _lastSpeedMs;
  if (dt < 10) return _speedSpsEst; // avoid noisy tiny dt
  int32_t dp = posSteps - _lastPosSteps;

  // microsteps per second
  double sps = (double)dp * 1000.0 / (double)dt;
  _speedSpsEst = (int32_t)llround(sps);

  _lastSpeedMs = nowMs;
  _lastPosSteps = posSteps;
  return _speedSpsEst;
}

void MotionController::enterFault(uint16_t code) {
  _faultCode = code;
  _st = MotorState::FAULT;
  _homingActive = false;
  _velCmdSps = 0;

  _eng->enable(false);
  _disabledUntilMs = millis() + _cfg->active().disable_time_ms;
}

void MotionController::cmdEnable(bool en) {
  if (!_eng) return;

  if (en) {
    // If we were faulted and disable time not elapsed, keep disabled
    if (_st == MotorState::FAULT && millis() < _disabledUntilMs) return;

    _faultCode = 0;
    _loss.clearFaultRequest();
    _eng->enable(true);
    _st = MotorState::IDLE;
  } else {
    _eng->enable(false);
    _st = MotorState::MOTOR_DISABLED;
  }
}

void MotionController::cmdStop() {
  if (!_eng) return;
  _eng->stop();
  _homingActive = false;
  _velCmdSps = 0;
  if (_st != MotorState::MOTOR_DISABLED && _st != MotorState::FAULT) _st = MotorState::IDLE;
}

void MotionController::cmdMoveAbsDegQ100(int32_t degQ100) {
  if (!_eng) return;
  if (_st == MotorState::MOTOR_DISABLED || _st == MotorState::FAULT) return;

  const auto& c = _cfg->active();
  uint32_t maxSps = (uint32_t)llround((double)c.max_speed_rev_s_q100 / 100.0 * (double)stepsPerRev());
  uint32_t accSps2 = (uint32_t)llround((double)c.accel_rev_s2_q100 / 100.0 * (double)stepsPerRev());
  _eng->setMaxSpeedSps(std::max<uint32_t>(1, maxSps));
  _eng->setAccelSps2(std::max<uint32_t>(1, accSps2));

  _targetSteps = degQ100ToSteps(degQ100);
  _eng->moveTo(_targetSteps);

  _st = MotorState::MOVING_POSITION;
  _homingActive = false;
  _velCmdSps = 0;
}

void MotionController::cmdVelocityDegPerSecQ100(int32_t dpsQ100) {
  if (!_eng) return;
  if (_st == MotorState::MOTOR_DISABLED || _st == MotorState::FAULT) return;

  double degPerSec = (double)dpsQ100 / 100.0;
  double revPerSec = degPerSec / 360.0;
  int32_t sps = (int32_t)llround(revPerSec * (double)stepsPerRev());

  const auto& c = _cfg->active();
  uint32_t accSps2 = (uint32_t)llround((double)c.accel_rev_s2_q100 / 100.0 * (double)stepsPerRev());
  _eng->setAccelSps2(max<uint32_t>(1, accSps2));

  _velCmdSps = sps;
  _eng->setVelocitySps(sps);
  _st = (sps == 0) ? MotorState::IDLE : MotorState::MOVING_VELOCITY;
  _homingActive = false;
}

void MotionController::cmdHomeStart() {
  if (_st == MotorState::MOTOR_DISABLED || _st == MotorState::FAULT) return;
  _homingActive = true;
  _homeStartMs = millis();
  _st = MotorState::HOMING;
}

void MotionController::tick() {
  const uint32_t now = millis();
  const auto& c = _cfg->active();

  // Auto-release from FAULT after disable window if someone re-enables
  if (_st == MotorState::FAULT && now >= _disabledUntilMs) {
    // stay FAULT until explicit enable, but allow telemetry to show not-disabled timer elapsed
  }

  // Thermistor + encoder
  _therm->sample(c.r_fixed_ohm, c.ntc_r25_ohm, c.ntc_beta);
  _enc->readAngle();

  // Motion parameter refresh
  uint32_t maxSps = (uint32_t)llround((double)c.max_speed_rev_s_q100 / 100.0 * (double)stepsPerRev());
  uint32_t accSps2 = (uint32_t)llround((double)c.accel_rev_s2_q100 / 100.0 * (double)stepsPerRev());
  _eng->setMaxSpeedSps(max<uint32_t>(1, maxSps));
  _eng->setAccelSps2(max<uint32_t>(1, accSps2));

  // Homing (same as before)
  if (_homingActive) {
    int32_t target = c.home_angle_deg_q100;
    while (target < 0) target += 36000;
    while (target >= 36000) target -= 36000;

    int32_t cur = _enc->angleDegQ100(); // wrapped
    int32_t err = target - cur;
    if (err > 18000)  err -= 36000;
    if (err < -18000) err += 36000;

    uint32_t homeMaxSps = (uint32_t)llround((double)c.home_speed_rev_s_q100 / 100.0 * (double)stepsPerRev());
    uint32_t homeAccSps2 = (uint32_t)llround((double)c.home_accel_rev_s2_q100 / 100.0 * (double)stepsPerRev());
    _eng->setMaxSpeedSps(max<uint32_t>(1, homeMaxSps));
    _eng->setAccelSps2(max<uint32_t>(1, homeAccSps2));

    const int32_t win = (int32_t)c.home_window_deg_q100;
    static uint32_t inWinSince = 0;

    if (abs(err) <= win) {
      if (inWinSince == 0) inWinSince = now;
      if (now - inWinSince >= c.home_settle_ms) {
        _eng->stop();
        _homeSteps = _eng->currentPosition();
        _homingActive = false;
        _st = MotorState::IDLE;
        inWinSince = 0;
      } else {
        int32_t slowDpsQ100 = (err > 0) ? 600 : -600; // 6 deg/s
        cmdVelocityDegPerSecQ100(slowDpsQ100);
        _st = MotorState::HOMING;
      }
    } else {
      inWinSince = 0;
      if (!_eng->isRunning()) {
        int32_t deltaSteps = degQ100ToSteps(err);
        _eng->move(deltaSteps);
      }
      _st = MotorState::HOMING;
    }

    if (c.home_timeout_ms > 0 && (now - _homeStartMs) > c.home_timeout_ms) {
      _homingActive = false;
      enterFault(/*code=*/2);
    }
  }

  // Step-loss monitor + recovery
  int32_t posSteps = _eng->currentPosition();
  int32_t spsEst = estimateSpeedSps(now, posSteps);

  _loss.update(c, now, posSteps, spsEst, _enc->absAngleDegQ100());

  // If loss active, apply small nudges to re-lock (works for position + velocity + homing)
  if (_loss.lossActive() && _st != MotorState::FAULT && _st != MotorState::MOTOR_DISABLED) {
    int32_t corr = _loss.suggestedCorrectionSteps(c);

    // Apply a correction nudge only if motor is enabled
    if (_eng->isEnabled() && corr != 0) {
      _eng->move(corr);
    }

    if (_loss.faultRequested()) {
      enterFault(/*code=*/3);
    }
  }

  // Basic state transitions
  if ((_st == MotorState::MOVING_POSITION) && !_eng->isRunning()) _st = MotorState::IDLE;
  if ((_st == MotorState::MOVING_VELOCITY) && _velCmdSps == 0) _st = MotorState::IDLE;

  // Telemetry snapshot
  Telemetry t;
  t.ms = now;
  t.state = _st;
  t.motorEnabled = _eng->isEnabled();
  t.moving = _eng->isRunning() || (_st == MotorState::MOVING_VELOCITY) || (_st == MotorState::HOMING);
  t.pos_steps = posSteps;
  t.target_steps = _targetSteps;
  t.home_steps = _homeSteps;
  t.enc_deg_q100 = _enc->angleDegQ100();
  t.enc_abs_deg_q100 = _enc->absAngleDegQ100();
  t.error_steps = _loss.errorSteps();
  t.lossActive = _loss.lossActive();
  t.faultCode = _faultCode;
  t.temp_c_q10 = _therm->tempCq10();
  t.config_revision = _cfg->revision();

  _state->setTelemetry(t);
}
