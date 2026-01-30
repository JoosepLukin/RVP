/*
  SharedState.h
  -------------
  Thread-safe shared runtime state + telemetry snapshot.
  - Motion task is the "truth owner"
  - Other tasks read snapshots (atomic copy)
*/

#pragma once
#include <Arduino.h>
//#include "comms/Protocol.h"
#include "system/Types.h"

class SharedState {
public:
  void begin() { _mux = portMUX_INITIALIZER_UNLOCKED; }

  void setRxBlink() {
    portENTER_CRITICAL(&_mux);
    _rxBlinkMs = millis();
    portEXIT_CRITICAL(&_mux);
  }

  uint32_t lastRxBlinkMs() const { return _rxBlinkMs; }

  void setTelemetry(const Telemetry& t) {
    portENTER_CRITICAL(&_mux);
    _telemetry = t;
    portEXIT_CRITICAL(&_mux);
  }

  Telemetry getTelemetry() const {
    Telemetry t;
    portENTER_CRITICAL((portMUX_TYPE*)&_mux);
    t = _telemetry;
    portEXIT_CRITICAL((portMUX_TYPE*)&_mux);
    return t;
  }

private:
  mutable portMUX_TYPE _mux;
  volatile uint32_t _rxBlinkMs = 0;
  Telemetry _telemetry;
};
