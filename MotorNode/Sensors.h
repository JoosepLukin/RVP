#pragma once
#include <Arduino.h>

namespace Sensors {

// Thermistor ADC pin
static const int PIN_THERM_ADC = 8;

// Divider: 3.3V -> Rfixed(4.7k) -> ADC -> NTC(47k) -> GND
static uint32_t g_rFixed_ohm = 4700;     // R9
static uint32_t g_r0_ohm     = 47000;    // NTC nominal at T0
static uint16_t g_beta       = 3950;     // change to your part
static int16_t  g_t0_c_x10   = 250;      // 25.0C
static uint8_t  g_samples    = 8;        // averaging

static inline void setThermistorParams(uint32_t rFixed, uint32_t r0, uint16_t beta,
                                       int16_t t0_c_x10, uint8_t samples) {
  if (rFixed > 0) g_rFixed_ohm = rFixed;
  if (r0 > 0)     g_r0_ohm     = r0;
  if (beta > 0)   g_beta       = beta;
  g_t0_c_x10 = t0_c_x10;

  if (samples == 0) samples = 1;
  if (samples > 64) samples = 64;
  g_samples = samples;
}

static inline void begin() {
  analogReadResolution(12);
#if defined(ARDUINO_ARCH_ESP32)
  analogSetPinAttenuation(PIN_THERM_ADC, ADC_11db);
#endif
}

// Returns temperature in Celsius, NAN if invalid
static inline float readThermistorC() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < g_samples; i++) {
#if defined(ARDUINO_ARCH_ESP32)
    acc += analogReadMilliVolts(PIN_THERM_ADC);
#else
    acc += analogRead(PIN_THERM_ADC);
#endif
    delayMicroseconds(200);
  }

#if defined(ARDUINO_ARCH_ESP32)
  float v_mv = (float)acc / (float)g_samples;
  float v = v_mv / 1000.0f;
  const float vcc = 3.3f;
#else
  float adc = (float)acc / (float)g_samples;
  float v = adc * (3.3f / 4095.0f);
  const float vcc = 3.3f;
#endif

  if (v <= 0.001f || v >= (vcc - 0.001f)) return NAN;

  // Rntc = Rfixed * V / (Vcc - V)
  float rntc = (float)g_rFixed_ohm * v / (vcc - v);
  if (rntc <= 0.0f) return NAN;

  // Beta equation
  float t0 = ((float)g_t0_c_x10 / 10.0f) + 273.15f;
  float invT = (1.0f / t0) + (1.0f / (float)g_beta) * logf(rntc / (float)g_r0_ohm);
  float tK = 1.0f / invT;
  return tK - 273.15f;
}

} // namespace Sensors
