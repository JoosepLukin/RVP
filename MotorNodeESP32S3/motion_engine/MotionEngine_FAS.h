/*
  MotionEngine_FAS.h
  ------------------
  FastAccelStepper-based MotionEngine implementation.
  - Generates reliable STEP pulses using ESP32 hardware timing
*/

#pragma once
#include "motion_engine/MotionEngine.h"
#include <FastAccelStepper.h>

class MotionEngine_FAS : public MotionEngine {
public:
  bool begin(int stepPin, int dirPin, int enPin) override;

  void enable(bool en) override;
  bool isEnabled() const override { return _enabled; }

  void setMaxSpeedSps(uint32_t stepsPerSec) override;
  void setAccelSps2(uint32_t stepsPerSec2) override;

  void moveTo(int32_t targetSteps) override;
  void move(int32_t deltaSteps) override;
  void setVelocitySps(int32_t stepsPerSec) override;

  void stop() override;

  int32_t currentPosition() const override;
  bool isRunning() const override;

private:
  FastAccelStepperEngine _engine;
  FastAccelStepper* _stepper = nullptr;

  int _enPin = -1;
  bool _enabled = false;

  uint32_t _maxSpeed = 1000;
  uint32_t _accel = 1000;
};
