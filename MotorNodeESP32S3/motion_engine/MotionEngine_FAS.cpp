/*
  MotionEngine_FAS.cpp
  --------------------
  Implements MotionEngine using FastAccelStepper.
*/

#include "motion_engine/MotionEngine_FAS.h"

bool MotionEngine_FAS::begin(int stepPin, int dirPin, int enPin) {
  _enPin = enPin;

  _engine.init();
  _stepper = _engine.stepperConnectToPin(stepPin);
  if (!_stepper) return false;

  _stepper->setDirectionPin(dirPin);
  _stepper->setEnablePin(enPin, true); // true = enable active HIGH (matches your board)
  _stepper->setAutoEnable(false);      // we control enable explicitly

  setMaxSpeedSps(_maxSpeed);
  setAccelSps2(_accel);

  enable(false);
  return true;
}

void MotionEngine_FAS::enable(bool en) {
  _enabled = en;
  if (_stepper) {
    if (en) _stepper->enableOutputs();
    else    _stepper->disableOutputs();
  }
}

void MotionEngine_FAS::setMaxSpeedSps(uint32_t stepsPerSec) {
  _maxSpeed = max<uint32_t>(1, stepsPerSec);
  if (_stepper) _stepper->setSpeedInHz(_maxSpeed);
}

void MotionEngine_FAS::setAccelSps2(uint32_t stepsPerSec2) {
  _accel = max<uint32_t>(1, stepsPerSec2);
  if (_stepper) _stepper->setAcceleration(_accel);
}

void MotionEngine_FAS::moveTo(int32_t targetSteps) {
  if (!_stepper) return;
  _stepper->moveTo(targetSteps);
}

void MotionEngine_FAS::move(int32_t deltaSteps) {
  if (!_stepper) return;
  _stepper->move(deltaSteps);
}

void MotionEngine_FAS::setVelocitySps(int32_t stepsPerSec) {
  if (!_stepper) return;

  // FastAccelStepper continuous run:
  // - setSpeedInHz sets magnitude
  // - runForward/runBackward starts continuous motion
  uint32_t spd = (uint32_t)std::min<int64_t>(std::llabs((int64_t)stepsPerSec), 2000000LL);
  _stepper->setSpeedInHz(std::max<uint32_t>(1, spd));

  if (stepsPerSec > 0) _stepper->runForward();
  else if (stepsPerSec < 0) _stepper->runBackward();
  else _stepper->stopMove();
}

void MotionEngine_FAS::stop() {
  if (_stepper) _stepper->stopMove();
}

int32_t MotionEngine_FAS::currentPosition() const {
  if (!_stepper) return 0;
  return _stepper->getCurrentPosition();
}

bool MotionEngine_FAS::isRunning() const {
  if (!_stepper) return false;
  return _stepper->isRunning();
}
