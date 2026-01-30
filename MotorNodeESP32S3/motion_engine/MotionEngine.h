/*
  MotionEngine.h
  --------------
  Abstract motion engine interface.
  - MotionController uses this interface so we can swap implementations.
*/

#pragma once
#include <Arduino.h>

class MotionEngine {
public:
  virtual ~MotionEngine() = default;

  virtual bool begin(int stepPin, int dirPin, int enPin) = 0;

  virtual void enable(bool en) = 0;
  virtual bool isEnabled() const = 0;

  virtual void setMaxSpeedSps(uint32_t stepsPerSec) = 0;
  virtual void setAccelSps2(uint32_t stepsPerSec2) = 0;

  virtual void moveTo(int32_t targetSteps) = 0;
  virtual void move(int32_t deltaSteps) = 0;
  virtual void setVelocitySps(int32_t stepsPerSec) = 0; // continuous mode

  virtual void stop() = 0;

  virtual int32_t currentPosition() const = 0;
  virtual bool isRunning() const = 0;
};
