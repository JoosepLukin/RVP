/*
  TMC2209Driver.h
  ---------------
  Minimal TMC2209 setup wrapper.
  - For now: "applyConfig" placeholder so we can expand later
  - You requested no periodic status reads.
*/

#pragma once
#include <Arduino.h>
#include "config/ConfigDefs.h"
#include <TMCStepper.h>

class TMC2209Driver {
public:
  void begin();
  void applyConfig(const Cfg::ActiveConfig& cfg);

private:
  HardwareSerial* _ser = nullptr;
  TMC2209Stepper* _drv = nullptr;
};
