/*
  Thermistor.h
  ------------
  Thermistor ADC reader for a divider: 3.3V -> R_fixed -> ADC -> NTC -> GND (pull-down NTC).
  - Uses Beta equation (configurable B, R25, R_fixed)
  - Produces temperature in q10 (°C * 10)
*/

#pragma once
#include <Arduino.h>

class Thermistor {
public:
  void begin(uint8_t adcPin);

  // Call periodically (e.g. 1 Hz). Stores last value.
  void sample(uint32_t rFixedOhm = 10000, uint32_t r25Ohm = 47000, uint32_t beta = 3950);

  int16_t tempCq10() const { return _tempCq10; }

private:
  uint8_t _pin = 255;
  int16_t _tempCq10 = 0;
};
