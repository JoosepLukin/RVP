/*
  MotionController.h
  ------------------
  Motor state machine + conversions (degrees <-> steps) + step-loss monitor integration.

  Supports:
  - enable/disable
  - stop
  - move-to absolute angle (deg*100)
  - velocity (deg/s*100)
  - encoder-based homing (slow + settle)
  - step-loss detect + recovery nudges + fault on timeout
*/

#pragma once
#include <Arduino.h>
#include <algorithm>


#include "system/Types.h"
class SharedState;

class MotionEngine;
class EncoderAS5047P;
class Thermistor;
class TMC2209Driver;
class ConfigStore;

#include "motion/StepLossMonitor.h"

class MotionController {
public:
  void begin(MotionEngine* eng,
             EncoderAS5047P* enc,
             Thermistor* therm,
             TMC2209Driver* tmc,
             ConfigStore* cfg,
             SharedState* state);

  void tick(); // called frequently by MotionTask

  // Commands (called by ESP-NOW handler)
  void cmdEnable(bool en);
  void cmdStop();
  void cmdMoveAbsDegQ100(int32_t degQ100);
  void cmdVelocityDegPerSecQ100(int32_t dpsQ100);
  void cmdHomeStart();

private:
  int32_t stepsPerRev() const;
  int32_t degQ100ToSteps(int32_t degQ100) const;

  int32_t estimateSpeedSps(uint32_t nowMs, int32_t posSteps);

  void enterFault(uint16_t code);

  MotorState _st = MotorState::MOTOR_DISABLED;

  MotionEngine* _eng = nullptr;
  EncoderAS5047P* _enc = nullptr;
  Thermistor* _therm = nullptr;
  TMC2209Driver* _tmc = nullptr;
  ConfigStore* _cfg = nullptr;
  SharedState* _state = nullptr;

  int32_t _homeSteps = 0;
  int32_t _targetSteps = 0;

  // Velocity mode tracking (for possible future catch-up tuning)
  int32_t _velCmdSps = 0;

  // Homing
  bool _homingActive = false;
  uint32_t _homeStartMs = 0;

  // Step-loss
  StepLossMonitor _loss;
  uint16_t _faultCode = 0;
  uint32_t _disabledUntilMs = 0;

  // Speed estimation
  uint32_t _lastSpeedMs = 0;
  int32_t  _lastPosSteps = 0;
  int32_t  _speedSpsEst = 0;
};
