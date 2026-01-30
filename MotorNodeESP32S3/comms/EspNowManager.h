/*
  EspNowManager.h
  ---------------
  ESP-NOW transport:
  - init WiFi/ESP-NOW on fixed channel
  - receive callback -> validates header/CRC -> dispatches to MotionController/ConfigStore
  - periodic telemetry sending (handled by Tasks)
*/

#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "comms/Protocol.h"

class SharedState;
class ConfigStore;
class MotionController; // forward

class EspNowManager {
public:
  uint8_t _channel = 0;

  bool begin(uint8_t channel);

  void setSharedState(SharedState* s) { _state = s; }
  void setConfigStore(ConfigStore* c) { _cfg = c; }
  void setMotionController(MotionController* m) { _motion = m; }

  // Send helpers
  bool sendStatus(const uint8_t* peerMacOrNullBroadcast);
  bool sendAck(const uint8_t* peerMac, uint8_t seq, uint8_t result);

  // Called by task to broadcast or to known peer later
  void setDefaultPeer(const uint8_t mac[6]);

private:
  static void onRecvStatic(const esp_now_recv_info_t* info, const uint8_t* data, int len);
  void onRecv(const uint8_t* mac, const uint8_t* data, int len);


  bool ensurePeer(const uint8_t* mac);

  SharedState* _state = nullptr;
  ConfigStore* _cfg = nullptr;
  MotionController* _motion = nullptr;

  uint8_t _defaultPeer[6] = {0};
  bool _hasDefaultPeer = false;

  uint8_t _seqTx = 1;

  static EspNowManager* _self;
};
