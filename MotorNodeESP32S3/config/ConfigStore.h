/*
  ConfigStore.h
  -------------
  Stores active config + supports runtime updates via TLV set/get.
  This drop adds:
  - applyTLV() coverage for MOST params in ConfigDefs.h
  - serializeSelectedTLV() for CMD_GET_CONFIG(param list)
*/

#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config/ConfigDefs.h"


class ConfigStore {
public:
  void begin();

  const Cfg::ActiveConfig& active() const { return _cfg; }
  Cfg::ActiveConfig& activeMut() { return _cfg; }

  uint32_t revision() const { return _cfg.revision; }

  uint8_t applyTLV(const uint8_t* payload, size_t len, uint8_t setFlags,
                   uint8_t* resultBuf, size_t resultBufMax, size_t& resultLen);

  size_t serializeAllTLV(uint8_t* out, size_t outMax) const;
  size_t serializeSelectedTLV(const uint16_t* ids, size_t idCount, uint8_t* out, size_t outMax) const;

  void saveToNvs();
  void loadFromNvs();

private:
  bool setParam(uint16_t id, uint8_t type, const uint8_t* value, uint8_t len,
                uint8_t setFlags, uint8_t& entryResult);

  void bumpRevision();
  void hardClamp();

  Preferences _prefs;
  Cfg::ActiveConfig _cfg;
};
