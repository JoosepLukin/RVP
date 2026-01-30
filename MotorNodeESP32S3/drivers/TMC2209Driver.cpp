/*
  TMC2209Driver.cpp
  -----------------
  UART + register configuration for:
  - RDSon current sensing (NO external sense resistors)
  - RREF from 5VOUT -> VREF (15k on your PCB)
  - BRA/BRB tied to GND (per datasheet requirement)

  NOTE (datasheet): internal_Rsense must be set BEFORE motor enable.
  With RREF=15k, CS=31, vsense=0 => ~1.2A peak / ~0.84A RMS max. (table)
*/

#include "drivers/TMC2209Driver.h"
#include "system/PinMap.h"

// TMCStepper needs a value here even if you don't use external sense resistors.
// We do NOT call rms_current() so this is not used for your current setting mode.
static constexpr float   R_SENSE  = 0.11f;
static constexpr uint8_t TMC_ADDR = 0b00; // per MS1/MS2 address pins

static uint16_t clampMicrosteps(uint16_t ms) {
  // TMC2209 supports up to 256 microsteps.
  // Also: your config clamps to >=16 already, keep it power-of-two-ish.
  if (ms < 16) ms = 16;
  if (ms > 256) ms = 256;

  // Snap to nearest supported value (optional but avoids odd values from config):
  if (ms <= 16) return 16;
  if (ms <= 32) return 32;
  if (ms <= 64) return 64;
  if (ms <= 128) return 128;
  return 256;
}

void TMC2209Driver::begin() {
  _ser = &Serial1;
  _ser->begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  _drv = new TMC2209Stepper((Stream*)_ser, R_SENSE, TMC_ADDR);
  _drv->begin();

  // --- MUST for UART mode ---
  _drv->pdn_disable(true);        // required when using UART :contentReference[oaicite:5]{index=5}

  // --- MUST for your PCB wiring (no sense resistors, BRA/BRB to GND, RREF 5VOUT->VREF) ---
  // Enables RDSon current measurement mode; datasheet: set internal_Rsense in GCONF.
  _drv->internal_Rsense(true);    // IMPORTANT :contentReference[oaicite:6]{index=6}

  // In internal_Rsense mode, the reference is based on current into VREF (via RREF),
  // and I_scale_analog=0 uses internal reference derived from 5VOUT. :contentReference[oaicite:7]{index=7}
  _drv->I_scale_analog(false);

  // Use UART register for microstep resolution
  _drv->mstep_reg_select(true);   // use MRES register :contentReference[oaicite:8]{index=8}

  // Keep chopper disabled until applyConfig() sets everything (avoids enabling before config)
  _drv->toff(0);
}

void TMC2209Driver::applyConfig(const Cfg::ActiveConfig& cfg) {
  if (!_drv) return;

  // --- Current scaling behavior ---
  // Datasheet current table you’re relying on uses vsense=0. :contentReference[oaicite:9]{index=9}
  _drv->vsense(false);

  // Apply run/hold current scalers (0..31)
  uint8_t irun  = (cfg.tmc_irun  > 31) ? 31 : cfg.tmc_irun;
  uint8_t ihold = (cfg.tmc_ihold > 31) ? 31 : cfg.tmc_ihold;
  if (ihold > irun) ihold = irun;   // common-sense clamp

  _drv->irun(irun);
  _drv->ihold(ihold);
  _drv->iholddelay(1);              // 0..15; small delay is fine

  // --- Microsteps ---
  _drv->microsteps(clampMicrosteps(cfg.microsteps));

  // --- Mode selection ---
  // NOTE: datasheet says RDSon sensing works best with StealthChop; SpreadCycle can be noisier.
  // You asked for SpreadCycle, so we force it. :contentReference[oaicite:10]{index=10}
  _drv->en_spreadCycle(true);

  // Ensure no “auto switching” surprises:
  _drv->TPWMTHRS(0);     // don’t switch into StealthChop by velocity threshold
  _drv->TCOOLTHRS(0);    // disable coolStep thresholding path (not using it)

  // --- SpreadCycle chopper defaults (reasonable starting point) ---
  _drv->toff(4);                 // enable chopper (nonzero enables driver)
  _drv->blank_time(24);          // allowed values typically 16/24/36/54
  _drv->hysteresis_end(2);
  _drv->hysteresis_start(3);

  // --- Misc ---
  _drv->intpol(true);            // interpolate to 256 internally (smoother)
  _drv->pwm_autoscale(false);    // irrelevant for forced spreadCycle, but ok
  _drv->pwm_autograd(false);
}
