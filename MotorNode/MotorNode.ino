#include <Arduino.h>

#define MN_DEBUG 1

#include "Sensors.h"
#include "MotionControl.h"
#include "EspNowComms.h"

// LEDs
static const int PIN_LED_MISSED = 35; // ON when mismatch active
static const int PIN_LED_STATUS = 36; // toggles each status send

static uint32_t g_nextStatusMs = 0;
static bool g_statusLedState = false;

static void sendStatusTick() {
  if (!EspNowComms::hasMaster()) return;

  EspNowComms::MsgStatus st{};
  EspNowComms::fillStatus(st);

  // Thermistor temperature
  float tc = Sensors::readThermistorC();
  if (isnan(tc) || isinf(tc)) {
    st.temp_c_x10 = INT16_MIN;
  } else {
    long t10 = lroundf(tc * 10.0f);
    if (t10 < INT16_MIN) t10 = INT16_MIN;
    if (t10 > INT16_MAX) t10 = INT16_MAX;
    st.temp_c_x10 = (int16_t)t10;
  }

  EspNowComms::sendStatus(st);

  // Toggle status LED each send
  g_statusLedState = !g_statusLedState;
  digitalWrite(PIN_LED_STATUS, g_statusLedState ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_LED_MISSED, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_MISSED, LOW);
  digitalWrite(PIN_LED_STATUS, LOW);

  Sensors::begin();
  Sensors::loadConfigFromNvs();

  if (!MotionControl::begin()) {
    Serial.println("[MN] MotionControl init FAILED");
    while (true) delay(1000);
  }

  if (!EspNowComms::begin()) {
    Serial.println("[MN] ESP-NOW init FAILED");
    while (true) delay(1000);
  }

  Serial.print("[MN] Ready. My MAC: ");
  Serial.println(EspNowComms::selfMacStr());

  Serial.print("[MN] Encoder start raw14=");
  Serial.print(MotionControl::encoderRaw14());
  Serial.print(" enc_user=");
  Serial.println(MotionControl::encoderUserPos());

  g_nextStatusMs = millis() + 500;
}

void loop() {
  MotionControl::service();
  EspNowComms::service();

  // Fast status response when requested (GUI "Update config from node")
  if (EspNowComms::consumeStatusRequest()) {
    sendStatusTick();
  }

  // Missed steps LED ON while mismatch is active
  digitalWrite(PIN_LED_MISSED, MotionControl::mismatchActive() ? HIGH : LOW);

  uint32_t now = millis();
  if ((int32_t)(now - g_nextStatusMs) >= 0) {
    g_nextStatusMs = now + 500;
    sendStatusTick();
  }
}
