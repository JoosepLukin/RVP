/*
  MotorNodeESP32S3.ino
  --------------------
  Arduino entry point.
  - Initializes pins, config, sensors, motion engine, ESP-NOW
  - Creates FreeRTOS tasks pinned to cores
  - loop() stays empty (work is done in tasks)
*/

#include <Arduino.h>
#include "system/PinMap.h"
#include "system/SharedState.h"
#include "system/Tasks.h"

#include "config/ConfigStore.h"
#include "comms/EspNowManager.h"
#include "sensors/EncoderAS5047P.h"
#include "sensors/Thermistor.h"
#include "drivers/TMC2209Driver.h"
#include "motion_engine/MotionEngine_FAS.h"
#include "motion/MotionController.h"

SharedState gState;

ConfigStore     gConfig;
EspNowManager   gEspNow;
EncoderAS5047P  gEncoder;
Thermistor      gTherm;
TMC2209Driver   gTmc;
MotionEngine_FAS gMotionEngine;
MotionController gMotion;

static void initPins() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);

  pinMode(PIN_LED_MOVE, OUTPUT);
  pinMode(PIN_LED_COMMS, OUTPUT);

  digitalWrite(PIN_LED_MOVE, LOW);
  digitalWrite(PIN_LED_COMMS, LOW);

  // EN is active-high per your board.
  digitalWrite(PIN_EN, LOW); // start disabled
}

void setup() {
  Serial.begin(115200);
  delay(200);

  initPins();

  gState.begin();
  gConfig.begin();        // loads defaults + NVS if present
  gTherm.begin(PIN_THERM_ADC);
  gEncoder.begin(PIN_SPI_CS); // SPI pins are in PinMap.h

  // Driver init/config (no status reads)
  gTmc.begin();
  gTmc.applyConfig(gConfig.active());

  // Motion engine (FastAccelStepper) + controller
  gMotionEngine.begin(PIN_STEP, PIN_DIR, PIN_EN);
  gMotion.begin(&gMotionEngine, &gEncoder, &gTherm, &gTmc, &gConfig, &gState);

  // ESP-NOW
  gEspNow.begin(/*channel=*/6);
  gEspNow.setSharedState(&gState);
  gEspNow.setConfigStore(&gConfig);
  gEspNow.setMotionController(&gMotion);

  // Start tasks (core split)
  Tasks::startAll(&gState, &gConfig, &gEspNow, &gEncoder, &gTherm, &gMotion);
}

void loop() {
  // Intentionally empty. All work is done in FreeRTOS tasks.
  delay(1000);
}
