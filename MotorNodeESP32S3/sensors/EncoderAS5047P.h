/*
  EncoderAS5047P.h
  ----------------
  AS5047P SPI absolute magnetic encoder driver.
  - Reads angle (0..360deg wrapped) and unwraps to multi-turn absolute angle
  - Provides degrees in q100 (deg*100) for easy integer transport/telemetry
  Notes:
  - Uses a simple "command then NOP to fetch response" SPI read pattern.
  - If you find readings are wrong, SPI MODE or command bits may need adjusting per datasheet.
*/

#pragma once
#include <Arduino.h>
#include <SPI.h>

class EncoderAS5047P {
public:
  bool begin(uint8_t csPin);
  bool readAngle();                 // updates internal angle values
  int32_t angleDegQ100() const { return _angleDegQ100; }         // 0..36000 (wrapped)
  int32_t absAngleDegQ100() const { return _absAngleDegQ100; }   // unwrapped, can grow +/- with turns
  void setInverted(bool inverted) { _invert = inverted; }
  bool inverted() const { return _invert; }
  bool    ok() const { return _ok; }

private:
  uint16_t spiRead16(uint16_t cmd);
  static uint16_t addParity(uint16_t x);
  static uint8_t  evenParity15(uint16_t x);

  uint8_t _cs = 255;
  bool _ok = false;

  int32_t _angleDegQ100 = 0;
  int32_t _absAngleDegQ100 = 0;

  int32_t _prevAngleDegQ100 = 0;
  int32_t _turns = 0;
  bool _invert = false;

  SPISettings _spi = SPISettings(1000000, MSBFIRST, SPI_MODE1); // conservative; adjust if needed
};
