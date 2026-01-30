/*
  Tasks.h
  -------
  FreeRTOS tasks and core pinning.
  Core 0: ESP-NOW TX + sensors (WiFi stack tends to live here)
  Core 1: Motion/control loop (time-critical)
*/

#pragma once
#include <Arduino.h>

class SharedState;
class ConfigStore;
class EspNowManager;
class EncoderAS5047P;
class Thermistor;
class MotionController;

namespace Tasks {
  void startAll(SharedState*, ConfigStore*, EspNowManager*, EncoderAS5047P*, Thermistor*, MotionController*);
}
