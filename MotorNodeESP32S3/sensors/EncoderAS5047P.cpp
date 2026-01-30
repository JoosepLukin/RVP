/*
  EncoderAS5047P.cpp
  ------------------
  Implements SPI reads and unwrap logic.
  - Wrapped angle: 0..36000 (deg*100)
  - Unwrap: detects wrap-around by >180deg jump and increments/decrements turn counter
*/

#include "sensors/EncoderAS5047P.h"
#include "system/PinMap.h"

// Common AS5047* angle register address is typically 0x3FFF (14-bit angle).
// Command frame assumed: [parity bit][R/W bit][14-bit address].
// If your datasheet indicates different, change here.
static constexpr uint16_t REG_ANGLE = 0x3FFF;
static constexpr uint16_t CMD_READ  = 0x4000; // assumed read bit in bit14

bool EncoderAS5047P::begin(uint8_t csPin) {
  _cs = csPin;
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, _cs);

  // Prime state
  _ok = false;
  _turns = 0;
  _prevAngleDegQ100 = 0;
  _angleDegQ100 = 0;
  _absAngleDegQ100 = 0;

  // Try one read
  readAngle();
  return true;
}

uint8_t EncoderAS5047P::evenParity15(uint16_t x) {
  // compute parity over bits 0..14 (exclude bit15)
  x &= 0x7FFF;
  uint8_t p = 0;
  while (x) { p ^= (x & 1); x >>= 1; }
  return p; // 1 if odd number of ones
}

uint16_t EncoderAS5047P::addParity(uint16_t x) {
  // Set bit15 so that total parity over 0..15 becomes even
  uint8_t p = evenParity15(x);     // odd? -> set parity bit to 1
  if (p) x |= 0x8000;
  else   x &= 0x7FFF;
  return x;
}

uint16_t EncoderAS5047P::spiRead16(uint16_t cmd) {
  uint16_t resp = 0;

  SPI.beginTransaction(_spi);
  digitalWrite(_cs, LOW);
  SPI.transfer16(cmd);
  digitalWrite(_cs, HIGH);
  delayMicroseconds(2);

  // second frame fetches response
  digitalWrite(_cs, LOW);
  resp = SPI.transfer16(0x0000);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  return resp;
}

bool EncoderAS5047P::readAngle() {
  // Build read command
  uint16_t cmd = CMD_READ | (REG_ANGLE & 0x3FFF);
  cmd = addParity(cmd);

  uint16_t resp = spiRead16(cmd);

  // Response assumed: bit15 error flag? and 14-bit data in bits 13..0 (common pattern).
  // We'll accept 14-bit angle.
  uint16_t raw14 = resp & 0x3FFF;

  // Convert 0..16383 -> 0..36000 (deg*100)
  // deg = raw14 * 360 / 16384
  int32_t degQ100 = (int32_t)raw14 * 36000L / 16384L;
  if (_invert) {
    degQ100 = 36000 - degQ100;
    if (degQ100 >= 36000) degQ100 -= 36000;
  }

  // Unwrap:
  // delta > +180deg => wrapped backward
  // delta < -180deg => wrapped forward
  int32_t delta = degQ100 - _prevAngleDegQ100;
  if (delta > 18000)  _turns -= 1;
  if (delta < -18000) _turns += 1;

  _prevAngleDegQ100 = degQ100;
  _angleDegQ100 = degQ100;
  _absAngleDegQ100 = degQ100 + _turns * 36000L;

  _ok = true;
  return true;
}
