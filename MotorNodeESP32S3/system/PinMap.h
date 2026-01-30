/*
  PinMap.h
  --------
  Single source of truth for all GPIO assignments.
  Keep *all* pin numbers here (no magic numbers elsewhere).
*/

#pragma once

// Stepper
#define PIN_STEP        10
#define PIN_DIR         11
#define PIN_EN          47   // active-high on your board

// TMC2209 UART
#define PIN_UART_TX      7
#define PIN_UART_RX     16   // through 1k resistor on board

// Encoder SPI
#define PIN_SPI_SCK     13
#define PIN_SPI_MOSI    14
#define PIN_SPI_MISO    12
#define PIN_SPI_CS      21

// Thermistor ADC
#define PIN_THERM_ADC    8   // assume ADC1-capable

// LEDs (active-high)
#define PIN_LED_MOVE    36
#define PIN_LED_COMMS   35
