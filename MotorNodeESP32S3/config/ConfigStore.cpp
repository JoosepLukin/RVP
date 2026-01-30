/*
  ConfigStore.cpp
  ---------------
  TLV config set/get + NVS persistence.
  This drop expands setParam() and TLV serialization.
*/

#include "config/ConfigStore.h"

static constexpr const char* NVS_NS = "motorcfg";

void ConfigStore::begin() { loadFromNvs(); }

void ConfigStore::bumpRevision() {
  _cfg.revision++;
  if (_cfg.revision == 0) _cfg.revision = 1;
}

void ConfigStore::hardClamp() {
  if (_cfg.tmc_irun > 31)  _cfg.tmc_irun = 31;
  if (_cfg.tmc_ihold > 31) _cfg.tmc_ihold = 31;

  if (_cfg.microsteps < 16) _cfg.microsteps = 16;
  if (_cfg.telem_interval_ms < 100) _cfg.telem_interval_ms = 100;
  if (_cfg.node_id == 0) _cfg.node_id = 1;

  if (_cfg.home_window_deg_q100 < 10) _cfg.home_window_deg_q100 = 10;
  if (_cfg.home_settle_ms < 10) _cfg.home_settle_ms = 10;

  // sane thermistor defaults
  if (_cfg.ntc_r25_ohm < 1000) _cfg.ntc_r25_ohm = 47000;
  if (_cfg.r_fixed_ohm < 1000) _cfg.r_fixed_ohm = 10000;
  if (_cfg.ntc_beta < 1000) _cfg.ntc_beta = 3950;
  if (_cfg.therm_interval_ms < 200) _cfg.therm_interval_ms = 200;

  if (_cfg.encoder_invert > 1) _cfg.encoder_invert = 1;
}

void ConfigStore::saveToNvs() {
  _prefs.begin(NVS_NS, false);
  _prefs.putBytes("cfg", &_cfg, sizeof(_cfg));
  _prefs.end();
}

void ConfigStore::loadFromNvs() {
  _prefs.begin(NVS_NS, true);
  size_t n = _prefs.getBytesLength("cfg");
  if (n == sizeof(_cfg)) _prefs.getBytes("cfg", &_cfg, sizeof(_cfg));
  _prefs.end();
  hardClamp();
}

bool ConfigStore::setParam(uint16_t id, uint8_t type, const uint8_t* v, uint8_t len,
                           uint8_t setFlags, uint8_t& entryResult) {
  (void)setFlags;
  entryResult = 0;

  auto needLen = [&](uint8_t n) { return len == n; };
  auto U16 = [&](){ return *((uint16_t*)v); };
  auto U32 = [&](){ return *((uint32_t*)v); };
  auto I32 = [&](){ return *((int32_t*)v); };

  switch ((Cfg::ParamId)id) {
	case Cfg::P_TMC_IRUN:
	  if (type != Proto::VT_U8 || !needLen(1)) { entryResult = 2; return false; }
	  _cfg.tmc_irun = (v[0] > 31) ? 31 : v[0];
	  bumpRevision(); return true;

	case Cfg::P_TMC_IHOLD:
	  if (type != Proto::VT_U8 || !needLen(1)) { entryResult = 2; return false; }
	  _cfg.tmc_ihold = (v[0] > 31) ? 31 : v[0];
	  bumpRevision(); return true;

    case Cfg::P_NODE_ID:
      if (type != Proto::VT_U8 || !needLen(1)) { entryResult = 2; return false; }
      _cfg.node_id = v[0] ? v[0] : 1;
      bumpRevision(); return true;

    case Cfg::P_MICROSTEPS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.microsteps = U16();
      hardClamp();
      bumpRevision(); return true;

    case Cfg::P_MAX_SPEED_REV_S_Q100:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.max_speed_rev_s_q100 = U32();
      bumpRevision(); return true;

    case Cfg::P_ACCEL_REV_S2_Q100:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.accel_rev_s2_q100 = U32();
      bumpRevision(); return true;

    // Homing
    case Cfg::P_HOME_ANGLE_DEG_Q100:
      if (type != Proto::VT_I32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.home_angle_deg_q100 = I32();
      bumpRevision(); return true;

    case Cfg::P_HOME_WINDOW_DEG_Q100:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.home_window_deg_q100 = U16();
      hardClamp();
      bumpRevision(); return true;

    case Cfg::P_HOME_SETTLE_MS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.home_settle_ms = U16();
      hardClamp();
      bumpRevision(); return true;

    case Cfg::P_HOME_TIMEOUT_MS:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.home_timeout_ms = U32();
      bumpRevision(); return true;

    case Cfg::P_HOME_SPEED_REV_S_Q100:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.home_speed_rev_s_q100 = U32();
      bumpRevision(); return true;

    case Cfg::P_HOME_ACCEL_REV_S2_Q100:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.home_accel_rev_s2_q100 = U32();
      bumpRevision(); return true;

    // Step-loss (full steps thresholds)
    case Cfg::P_LOSS_THR_HOLD_FS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.loss_thr_hold_fs = U16();
      bumpRevision(); return true;

    case Cfg::P_LOSS_THR_RUN_FS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.loss_thr_run_fs = U16();
      bumpRevision(); return true;

    case Cfg::P_LOSS_THR_FAST_FS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.loss_thr_fast_fs = U16();
      bumpRevision(); return true;

    case Cfg::P_LOSS_SPEED_HOLD_SPS:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.loss_speed_hold_sps = U32();
      bumpRevision(); return true;

    case Cfg::P_LOSS_SPEED_FAST_SPS:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.loss_speed_fast_sps = U32();
      bumpRevision(); return true;

    case Cfg::P_LOSS_SPEED_HYST_SPS:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.loss_speed_hyst_sps = U32();
      bumpRevision(); return true;

    case Cfg::P_LOSS_CONFIRM_HOLD_MS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.loss_confirm_hold_ms = U16();
      bumpRevision(); return true;

    case Cfg::P_LOSS_CONFIRM_RUN_MS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.loss_confirm_run_ms = U16();
      bumpRevision(); return true;

    case Cfg::P_LOSS_CONFIRM_FAST_MS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.loss_confirm_fast_ms = U16();
      bumpRevision(); return true;

    case Cfg::P_RECOVER_TIME_MS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.recover_time_ms = U16();
      bumpRevision(); return true;

    case Cfg::P_DISABLE_TIME_MS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.disable_time_ms = U16();
      bumpRevision(); return true;

    // Thermistor
    case Cfg::P_NTC_R25_OHM:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.ntc_r25_ohm = U32();
      hardClamp();
      bumpRevision(); return true;

    case Cfg::P_R_FIXED_OHM:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.r_fixed_ohm = U32();
      hardClamp();
      bumpRevision(); return true;

    case Cfg::P_NTC_BETA:
      if (type != Proto::VT_U32 || !needLen(4)) { entryResult = 2; return false; }
      _cfg.ntc_beta = U32();
      hardClamp();
      bumpRevision(); return true;

    case Cfg::P_THERM_INTERVAL_MS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.therm_interval_ms = U16();
      hardClamp();
      bumpRevision(); return true;

    // Telemetry
    case Cfg::P_TELEM_INTERVAL_MS:
      if (type != Proto::VT_U16 || !needLen(2)) { entryResult = 2; return false; }
      _cfg.telem_interval_ms = U16();
      hardClamp();
      bumpRevision(); return true;

    case Cfg::P_ENCODER_INVERT:
      if (type != Proto::VT_BOOL || !needLen(1)) { entryResult = 2; return false; }
      _cfg.encoder_invert = v[0] ? 1 : 0;
      bumpRevision(); return true;

    default:
      entryResult = 1; // unknown param
      return false;
  }
}

uint8_t ConfigStore::applyTLV(const uint8_t* payload, size_t len, uint8_t setFlags,
                              uint8_t* resultBuf, size_t resultBufMax, size_t& resultLen) {
  resultLen = 0;
  uint8_t overall = 0;

  size_t i = 0;
  while (i + sizeof(Proto::TLV) <= len) {
    const Proto::TLV* tlv = (const Proto::TLV*)(payload + i);
    i += sizeof(Proto::TLV);
    if (i + tlv->len > len) { overall = 2; break; }

    uint8_t entryRes = 0;
    bool ok = setParam(tlv->param_id, tlv->type, payload + i, tlv->len, setFlags, entryRes);

    // Append result: [param_id u16][res u8]
    if (resultLen + 3 <= resultBufMax) {
      memcpy(resultBuf + resultLen, &tlv->param_id, 2); resultLen += 2;
      resultBuf[resultLen++] = ok ? 0 : entryRes;
    }

    if (!ok) overall = 1;
    i += tlv->len;
  }

  if (setFlags & Proto::SET_SAVE_AFTER_APPLY) saveToNvs();
  return overall;
}

static inline void putTLV(uint8_t* out, size_t outMax, size_t& w,
                          uint16_t id, uint8_t type, const void* val, uint8_t vlen) {
  if (w + sizeof(Proto::TLV) + vlen > outMax) return;
  Proto::TLV tlv{ id, type, vlen };
  memcpy(out + w, &tlv, sizeof(tlv)); w += sizeof(tlv);
  memcpy(out + w, val, vlen); w += vlen;
}

size_t ConfigStore::serializeAllTLV(uint8_t* out, size_t outMax) const {
  size_t w = 0;
  putTLV(out, outMax, w, Cfg::P_NODE_ID, Proto::VT_U8,  &_cfg.node_id, 1);
  putTLV(out, outMax, w, Cfg::P_MICROSTEPS, Proto::VT_U16, &_cfg.microsteps, 2);
  putTLV(out, outMax, w, Cfg::P_MAX_SPEED_REV_S_Q100, Proto::VT_U32, &_cfg.max_speed_rev_s_q100, 4);
  putTLV(out, outMax, w, Cfg::P_ACCEL_REV_S2_Q100, Proto::VT_U32, &_cfg.accel_rev_s2_q100, 4);

  putTLV(out, outMax, w, Cfg::P_TMC_IRUN,  Proto::VT_U8, &_cfg.tmc_irun,  1);
  putTLV(out, outMax, w, Cfg::P_TMC_IHOLD, Proto::VT_U8, &_cfg.tmc_ihold, 1);


  putTLV(out, outMax, w, Cfg::P_HOME_ANGLE_DEG_Q100, Proto::VT_I32, &_cfg.home_angle_deg_q100, 4);
  putTLV(out, outMax, w, Cfg::P_HOME_WINDOW_DEG_Q100, Proto::VT_U16, &_cfg.home_window_deg_q100, 2);
  putTLV(out, outMax, w, Cfg::P_HOME_SETTLE_MS, Proto::VT_U16, &_cfg.home_settle_ms, 2);
  putTLV(out, outMax, w, Cfg::P_HOME_TIMEOUT_MS, Proto::VT_U32, &_cfg.home_timeout_ms, 4);
  putTLV(out, outMax, w, Cfg::P_HOME_SPEED_REV_S_Q100, Proto::VT_U32, &_cfg.home_speed_rev_s_q100, 4);
  putTLV(out, outMax, w, Cfg::P_HOME_ACCEL_REV_S2_Q100, Proto::VT_U32, &_cfg.home_accel_rev_s2_q100, 4);

  putTLV(out, outMax, w, Cfg::P_LOSS_THR_HOLD_FS, Proto::VT_U16, &_cfg.loss_thr_hold_fs, 2);
  putTLV(out, outMax, w, Cfg::P_LOSS_THR_RUN_FS, Proto::VT_U16, &_cfg.loss_thr_run_fs, 2);
  putTLV(out, outMax, w, Cfg::P_LOSS_THR_FAST_FS, Proto::VT_U16, &_cfg.loss_thr_fast_fs, 2);
  putTLV(out, outMax, w, Cfg::P_LOSS_SPEED_HOLD_SPS, Proto::VT_U32, &_cfg.loss_speed_hold_sps, 4);
  putTLV(out, outMax, w, Cfg::P_LOSS_SPEED_FAST_SPS, Proto::VT_U32, &_cfg.loss_speed_fast_sps, 4);
  putTLV(out, outMax, w, Cfg::P_LOSS_SPEED_HYST_SPS, Proto::VT_U32, &_cfg.loss_speed_hyst_sps, 4);
  putTLV(out, outMax, w, Cfg::P_LOSS_CONFIRM_HOLD_MS, Proto::VT_U16, &_cfg.loss_confirm_hold_ms, 2);
  putTLV(out, outMax, w, Cfg::P_LOSS_CONFIRM_RUN_MS, Proto::VT_U16, &_cfg.loss_confirm_run_ms, 2);
  putTLV(out, outMax, w, Cfg::P_LOSS_CONFIRM_FAST_MS, Proto::VT_U16, &_cfg.loss_confirm_fast_ms, 2);
  putTLV(out, outMax, w, Cfg::P_RECOVER_TIME_MS, Proto::VT_U16, &_cfg.recover_time_ms, 2);
  putTLV(out, outMax, w, Cfg::P_DISABLE_TIME_MS, Proto::VT_U16, &_cfg.disable_time_ms, 2);

  putTLV(out, outMax, w, Cfg::P_NTC_R25_OHM, Proto::VT_U32, &_cfg.ntc_r25_ohm, 4);
  putTLV(out, outMax, w, Cfg::P_R_FIXED_OHM, Proto::VT_U32, &_cfg.r_fixed_ohm, 4);
  putTLV(out, outMax, w, Cfg::P_NTC_BETA, Proto::VT_U32, &_cfg.ntc_beta, 4);
  putTLV(out, outMax, w, Cfg::P_THERM_INTERVAL_MS, Proto::VT_U16, &_cfg.therm_interval_ms, 2);

  putTLV(out, outMax, w, Cfg::P_TELEM_INTERVAL_MS, Proto::VT_U16, &_cfg.telem_interval_ms, 2);
  putTLV(out, outMax, w, Cfg::P_ENCODER_INVERT, Proto::VT_BOOL, &_cfg.encoder_invert, 1);
  return w;
}

size_t ConfigStore::serializeSelectedTLV(const uint16_t* ids, size_t idCount, uint8_t* out, size_t outMax) const {
  // simple implementation: serialize all, then filter is overkill; just switch on ids.
  size_t w = 0;
  for (size_t k = 0; k < idCount; k++) {
    uint16_t id = ids[k];
    switch ((Cfg::ParamId)id) {
      case Cfg::P_NODE_ID: putTLV(out, outMax, w, id, Proto::VT_U8, &_cfg.node_id, 1); break;
      case Cfg::P_MICROSTEPS: putTLV(out, outMax, w, id, Proto::VT_U16, &_cfg.microsteps, 2); break;
      case Cfg::P_MAX_SPEED_REV_S_Q100: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.max_speed_rev_s_q100, 4); break;
      case Cfg::P_ACCEL_REV_S2_Q100: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.accel_rev_s2_q100, 4); break;

      case Cfg::P_TMC_IRUN:  putTLV(out, outMax, w, id, Proto::VT_U8, &_cfg.tmc_irun, 1); break;
      case Cfg::P_TMC_IHOLD: putTLV(out, outMax, w, id, Proto::VT_U8, &_cfg.tmc_ihold, 1); break;


      case Cfg::P_HOME_ANGLE_DEG_Q100: putTLV(out, outMax, w, id, Proto::VT_I32, &_cfg.home_angle_deg_q100, 4); break;
      case Cfg::P_HOME_WINDOW_DEG_Q100: putTLV(out, outMax, w, id, Proto::VT_U16, &_cfg.home_window_deg_q100, 2); break;
      case Cfg::P_HOME_SETTLE_MS: putTLV(out, outMax, w, id, Proto::VT_U16, &_cfg.home_settle_ms, 2); break;
      case Cfg::P_HOME_TIMEOUT_MS: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.home_timeout_ms, 4); break;
      case Cfg::P_HOME_SPEED_REV_S_Q100: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.home_speed_rev_s_q100, 4); break;
      case Cfg::P_HOME_ACCEL_REV_S2_Q100: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.home_accel_rev_s2_q100, 4); break;

      case Cfg::P_LOSS_THR_HOLD_FS: putTLV(out, outMax, w, id, Proto::VT_U16, &_cfg.loss_thr_hold_fs, 2); break;
      case Cfg::P_LOSS_THR_RUN_FS: putTLV(out, outMax, w, id, Proto::VT_U16, &_cfg.loss_thr_run_fs, 2); break;
      case Cfg::P_LOSS_THR_FAST_FS: putTLV(out, outMax, w, id, Proto::VT_U16, &_cfg.loss_thr_fast_fs, 2); break;
      case Cfg::P_LOSS_SPEED_HOLD_SPS: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.loss_speed_hold_sps, 4); break;
      case Cfg::P_LOSS_SPEED_FAST_SPS: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.loss_speed_fast_sps, 4); break;
      case Cfg::P_LOSS_SPEED_HYST_SPS: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.loss_speed_hyst_sps, 4); break;

      case Cfg::P_NTC_R25_OHM: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.ntc_r25_ohm, 4); break;
      case Cfg::P_R_FIXED_OHM: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.r_fixed_ohm, 4); break;
      case Cfg::P_NTC_BETA: putTLV(out, outMax, w, id, Proto::VT_U32, &_cfg.ntc_beta, 4); break;

      case Cfg::P_TELEM_INTERVAL_MS: putTLV(out, outMax, w, id, Proto::VT_U16, &_cfg.telem_interval_ms, 2); break;
      case Cfg::P_ENCODER_INVERT: putTLV(out, outMax, w, id, Proto::VT_BOOL, &_cfg.encoder_invert, 1); break;

      default:
        // unknown -> skip
        break;
    }
  }
  return w;
}
