/*
  Thermistor.cpp
  --------------
  Reads ADC and converts to temperature using the Beta model:
    1/T = 1/T0 + (1/B) * ln(R/R0)
  Where:
    T0 = 298.15K (25°C), R0 = R25
*/

#include "sensors/Thermistor.h"
#include <math.h>

void Thermistor::begin(uint8_t adcPin) {
  _pin = adcPin;
  analogReadResolution(12);
  // Optional: analogSetAttenuation(ADC_11db);  // use if needed for range
}

void Thermistor::sample(uint32_t rFixedOhm, uint32_t r25Ohm, uint32_t beta) {
  // Read raw (0..4095)
  uint32_t raw = analogRead(_pin);
  if (raw == 0 || raw >= 4095) {
    // out-of-range, keep old value
    return;
  }

  // Divider: Vadc = 3.3 * (Rntc / (Rfixed + Rntc))
  // => Rntc = Rfixed * Vadc / (3.3 - Vadc)
  // With ADC ratio: Vadc/3.3 = raw/4095
  double vRatio = (double)raw / 4095.0;
  double rNtc = (double)rFixedOhm * vRatio / (1.0 - vRatio);

  // Beta equation
  const double T0 = 298.15; // K
  double invT = (1.0 / T0) + (1.0 / (double)beta) * log(rNtc / (double)r25Ohm);
  double tempK = 1.0 / invT;
  double tempC = tempK - 273.15;

  _tempCq10 = (int16_t)lround(tempC * 10.0);
}
