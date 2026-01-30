/*
  EspNowManager.cpp
  -----------------
  RX parsing + CRC checks + dispatch.
  This drop adds:
  - CMD_ENABLE / CMD_STOP / CMD_MOVE_ABS_DEG / CMD_VELOCITY_DPS / CMD_HOME_START
  - CMD_GET_CONFIG -> RSP_CONFIG (TLVs)
  - CMD_SAVE_CONFIG
*/
#include <esp_wifi.h>
#include "comms/EspNowManager.h"
#include "motion/MotionController.h"
#include "config/ConfigStore.h"
#include "system/SharedState.h"



EspNowManager* EspNowManager::_self = nullptr;

static bool buildAndSend(const uint8_t* peerMac, uint8_t msgType, uint8_t flags, uint8_t seqTx,
                         const uint8_t* payload, uint8_t payloadLen) {
  Proto::Header h{};
  h.magic = Proto::MAGIC;
  h.version = Proto::VERSION;
  h.msg_type = msgType;
  h.flags = flags;
  h.seq = seqTx;
  h.node_id = 0;
  h.payload_len = payloadLen;
  h.crc16 = 0;

  // stack buffer sized for common packets
  static uint8_t buf[300];
  if (sizeof(Proto::Header) + payloadLen > sizeof(buf)) return false;

  memcpy(buf, &h, sizeof(h));
  if (payloadLen) memcpy(buf + sizeof(h), payload, payloadLen);

  Proto::Header* ph = (Proto::Header*)buf;
  Proto::Header tmp = *ph; tmp.crc16 = 0;

  uint16_t crc = Proto::crc16_ccitt((uint8_t*)&tmp, sizeof(tmp));
  if (payloadLen) crc = Proto::crc16_ccitt(buf + sizeof(Proto::Header), payloadLen, crc);
  ph->crc16 = crc;

  return esp_now_send(peerMac, buf, sizeof(Proto::Header) + payloadLen) == ESP_OK;
}

bool EspNowManager::begin(uint8_t channel) {
  _self = this;
  _channel = channel;


  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(&EspNowManager::onRecvStatic);
  return true;
}

void EspNowManager::setDefaultPeer(const uint8_t mac[6]) {
  memcpy(_defaultPeer, mac, 6);
  _hasDefaultPeer = true;
  ensurePeer(_defaultPeer);
}

bool EspNowManager::ensurePeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return true;

  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = _channel;   // explicit channel
  p.encrypt = false;
  return esp_now_add_peer(&p) == ESP_OK;
}

void EspNowManager::onRecvStatic(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!_self || !info) return;
  _self->onRecv(info->src_addr, data, len);   // src MAC
}


void EspNowManager::onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (!_state || len < (int)sizeof(Proto::Header)) return;

  _state->setRxBlink();

  Proto::Header hdr;
  memcpy(&hdr, data, sizeof(hdr));
  if (hdr.magic != Proto::MAGIC || hdr.version != Proto::VERSION) return;
  if (sizeof(Proto::Header) + hdr.payload_len != (uint16_t)len) return;

  // CRC check
  Proto::Header tmp = hdr;
  tmp.crc16 = 0;
  uint16_t crc = Proto::crc16_ccitt((const uint8_t*)&tmp, sizeof(tmp));
  crc = Proto::crc16_ccitt(data + sizeof(Proto::Header), hdr.payload_len, crc);
  if (crc != hdr.crc16) {
    sendAck(mac, hdr.seq, /*result=*/3);
    return;
  }

  const uint8_t* payload = data + sizeof(Proto::Header);

  switch (hdr.msg_type) {
    case Proto::CMD_PING:
      sendAck(mac, hdr.seq, 0);
      break;

    case Proto::CMD_ENABLE: {
      if (!_motion || hdr.payload_len != sizeof(Proto::CmdEnablePayload)) { sendAck(mac, hdr.seq, 2); break; }
      auto p = (const Proto::CmdEnablePayload*)payload;
      _motion->cmdEnable(p->enable != 0);
      sendAck(mac, hdr.seq, 0);
      break;
    }

    case Proto::CMD_STOP:
      if (!_motion) { sendAck(mac, hdr.seq, 2); break; }
      _motion->cmdStop();
      sendAck(mac, hdr.seq, 0);
      break;

    case Proto::CMD_MOVE_ABS_DEG: {
      if (!_motion || hdr.payload_len != sizeof(Proto::CmdMoveAbsDegPayload)) { sendAck(mac, hdr.seq, 2); break; }
      auto p = (const Proto::CmdMoveAbsDegPayload*)payload;
      _motion->cmdMoveAbsDegQ100(p->target_deg_q100);
      sendAck(mac, hdr.seq, 0);
      break;
    }

    case Proto::CMD_VELOCITY_DPS: {
      if (!_motion || hdr.payload_len != sizeof(Proto::CmdVelocityDpsPayload)) { sendAck(mac, hdr.seq, 2); break; }
      auto p = (const Proto::CmdVelocityDpsPayload*)payload;
      _motion->cmdVelocityDegPerSecQ100(p->vel_dps_q100);
      sendAck(mac, hdr.seq, 0);
      break;
    }

    case Proto::CMD_ZERO_POS:
      if (!_motion) { sendAck(mac, hdr.seq, 2); break; }
      _motion->cmdZeroPosition();
      sendAck(mac, hdr.seq, 0);
      break;

    case Proto::CMD_HOME_START:
      if (!_motion) { sendAck(mac, hdr.seq, 2); break; }
      _motion->cmdHomeStart();
      sendAck(mac, hdr.seq, 0);
      break;

    case Proto::CMD_SET_CONFIG: {
      if (!_cfg || hdr.payload_len < 1) { sendAck(mac, hdr.seq, 4); break; }
      uint8_t setFlags = payload[0];

      uint8_t results[220];
      size_t resLen = 0;
      uint8_t overall = _cfg->applyTLV(payload + 1, hdr.payload_len - 1, setFlags, results, sizeof(results), resLen);

      // For now, we only ACK overall status. (We can add RSP_SET_RESULT next drop.)
      sendAck(mac, hdr.seq, overall);
      break;
    }

    case Proto::CMD_SAVE_CONFIG:
      if (!_cfg) { sendAck(mac, hdr.seq, 4); break; }
      _cfg->saveToNvs();
      sendAck(mac, hdr.seq, 0);
      break;

    case Proto::CMD_GET_CONFIG: {
      if (!_cfg) { sendAck(mac, hdr.seq, 4); break; }
      if (hdr.payload_len < sizeof(Proto::CmdGetConfigPayload)) { sendAck(mac, hdr.seq, 2); break; }

      auto p = (const Proto::CmdGetConfigPayload*)payload;
      uint8_t out[240];
      size_t outLen = 0;

      if (p->list_len == 0) {
        outLen = _cfg->serializeAllTLV(out, sizeof(out));
      } else {
        size_t need = sizeof(Proto::CmdGetConfigPayload) + (size_t)p->list_len * 2;
        if ((size_t)hdr.payload_len < need) { sendAck(mac, hdr.seq, 2); break; }
        const uint16_t* ids = (const uint16_t*)(payload + sizeof(Proto::CmdGetConfigPayload));
        outLen = _cfg->serializeSelectedTLV(ids, p->list_len, out, sizeof(out));
      }

      ensurePeer(mac);
      buildAndSend(mac, Proto::RSP_CONFIG, 0, _seqTx++, out, (uint8_t)outLen);
      sendAck(mac, hdr.seq, 0);
      break;
    }

    default:
      sendAck(mac, hdr.seq, 1); // unsupported
      break;
  }
}

bool EspNowManager::sendAck(const uint8_t* peerMac, uint8_t seq, uint8_t result) {
  if (!ensurePeer(peerMac)) return false;
  Proto::AckPayload p{ seq, result };
  return buildAndSend(peerMac, Proto::RSP_ACK, Proto::FLAG_IS_ACK, _seqTx++, (const uint8_t*)&p, sizeof(p));
}

bool EspNowManager::sendStatus(const uint8_t* peerMacOrNullBroadcast) {
  if (!_state) return false;

  const Telemetry t = _state->getTelemetry();
  Proto::StatusPayload s{};
  s.ms = t.ms;
  s.state = (uint8_t)t.state;
  s.motorEnabled = t.motorEnabled ? 1 : 0;
  s.moving = t.moving ? 1 : 0;
  s.lossActive = t.lossActive ? 1 : 0;
  s.pos_steps = t.pos_steps;
  s.target_steps = t.target_steps;
  s.home_steps = t.home_steps;
  s.enc_deg_q100 = t.enc_deg_q100;
  s.enc_abs_deg_q100 = t.enc_abs_deg_q100;
  s.error_steps = t.error_steps;
  s.faultCode = t.faultCode;
  s.temp_c_q10 = t.temp_c_q10;
  s.config_revision = t.config_revision;

  if (peerMacOrNullBroadcast) {
    ensurePeer(peerMacOrNullBroadcast);
    return buildAndSend(peerMacOrNullBroadcast, Proto::RSP_STATUS, 0, _seqTx++, (const uint8_t*)&s, sizeof(s));
  }
  if (_hasDefaultPeer) {
    ensurePeer(_defaultPeer);
    return buildAndSend(_defaultPeer, Proto::RSP_STATUS, 0, _seqTx++, (const uint8_t*)&s, sizeof(s));
  }

  uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  ensurePeer(bcast);
  return buildAndSend(bcast, Proto::RSP_STATUS, 0, _seqTx++, (const uint8_t*)&s, sizeof(s));
}
