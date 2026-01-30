/*
  Tasks.cpp
  ---------
  Task implementations:
  - MotionTask (core 1): runs motion state machine + updates telemetry
  - SensorTask (core 0): reads thermistor (1 Hz configurable)
  - TelemetryTask (core 0): sends ESP-NOW status periodically
  - LedTask (core 0): LED patterns (move + comms blink)
*/

#include "system/Tasks.h"
#include "system/PinMap.h"
#include "system/SharedState.h"
#include "config/ConfigStore.h"
#include "comms/EspNowManager.h"
#include "sensors/EncoderAS5047P.h"
#include "sensors/Thermistor.h"
#include "motion/MotionController.h"

static SharedState*      S;
static ConfigStore*      C;
static EspNowManager*    N;
static EncoderAS5047P*   E;
static Thermistor*       T;
static MotionController* M;

static void MotionTask(void*) {
  for (;;) {
    M->tick();                 // updates motion + telemetry snapshot
    vTaskDelay(pdMS_TO_TICKS(2)); // 500 Hz-ish control tick (tune later)
  }
}

static void SensorTask(void*) {
  for (;;) {
    const auto& c = C->active();
    T->sample(c.r_fixed_ohm, c.ntc_r25_ohm, c.ntc_beta);
    vTaskDelay(pdMS_TO_TICKS(c.therm_interval_ms));
  }
}

static void TelemetryTask(void*) {
  for (;;) {
    N->sendStatus(nullptr); // default peer or broadcast
    vTaskDelay(pdMS_TO_TICKS(C->active().telem_interval_ms));
  }
}

static void LedTask(void*) {
  for (;;) {
    auto tel = S->getTelemetry();

    // MOVE LED: solid on when moving, fast blink if fault, slow blink if lossActive
    uint32_t ms = millis();
    bool moveOn = false;
    if (tel.state == MotorState::FAULT) moveOn = (ms / 100) % 2;
    else if (tel.lossActive)            moveOn = (ms / 250) % 2;
    else                                moveOn = tel.moving;

    digitalWrite(PIN_LED_MOVE, moveOn ? HIGH : LOW);

    // COMMS LED: pulse for 60ms on RX
    bool commOn = (ms - S->lastRxBlinkMs()) < 60;
    digitalWrite(PIN_LED_COMMS, commOn ? HIGH : LOW);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

namespace Tasks {
  void startAll(SharedState* s, ConfigStore* c, EspNowManager* n, EncoderAS5047P* e, Thermistor* t, MotionController* m) {
    S=s; C=c; N=n; E=e; T=t; M=m;

    xTaskCreatePinnedToCore(MotionTask, "MotionTask", 8192, nullptr, 4, nullptr, 1);
    xTaskCreatePinnedToCore(SensorTask, "SensorTask", 4096, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(TelemetryTask, "TelemTask", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2048, nullptr, 1, nullptr, 0);
  }
}
