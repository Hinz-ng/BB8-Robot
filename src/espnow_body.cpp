// module:  espnow_body.cpp
// layer:   7 — transport; ESP-NOW body-side implementation
// date:    2025

#include "espnow_body.h"
#include "project_wide_defs.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include <cstring>

ESPNowBody* ESPNowBody::_instance = nullptr;

bool ESPNowBody::begin() {
    _instance = this;

    // WiFi: STA mode, no connection, fixed channel.
    // Body uses WiFi solely to provide the RF hardware for ESP-NOW.
    // esp_wifi_set_channel() is mandatory — without it, the STA radio may
    // settle on a different channel after background scanning and silently
    // miss all packets from the head unit.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false);
    delay(100);

    if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        Serial.printf("[ESPNOW] WARNING: set_channel(%u) failed\n", ESPNOW_CHANNEL);
    }

    // Print STA MAC — operator fills this into ESPNOW_BODY_STA_MAC.
    uint8_t myMac[6];
    esp_read_mac(myMac, ESP_MAC_WIFI_STA);
    Serial.printf("[ESPNOW] My STA MAC: %02X:%02X:%02X:%02X:%02X:%02X  Channel: %u\n",
                  myMac[0], myMac[1], myMac[2],
                  myMac[3], myMac[4], myMac[5], ESPNOW_CHANNEL);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] esp_now_init() FAILED");
        return false;
    }
    esp_now_register_send_cb(_onSendCb);
    esp_now_register_recv_cb(_onRecvCb);

    if (ESPNOW_PEER_DISCOVERY) {
        static const uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
        _addPeer(bcast);
        Serial.println("[ESPNOW] Discovery mode: broadcasting.");
    } else {
        if (!_addPeer(ESPNOW_HEAD_AP_MAC)) return false;
        memcpy(_peerMac, ESPNOW_HEAD_AP_MAC, 6);
        _peerAdded = true;
        Serial.printf("[ESPNOW] Unicast peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      ESPNOW_HEAD_AP_MAC[0], ESPNOW_HEAD_AP_MAC[1],
                      ESPNOW_HEAD_AP_MAC[2], ESPNOW_HEAD_AP_MAC[3],
                      ESPNOW_HEAD_AP_MAC[4], ESPNOW_HEAD_AP_MAC[5]);
    }

    Serial.println("[ESPNOW] Body handler ready");
    return true;
}

void ESPNowBody::update() {
    const uint32_t now = millis();
    if (_linked && (now - _lastAckMs > ESPNOW_LINK_TIMEOUT_MS)) {
        _linked = false;
        Serial.println("[ESPNOW] Link LOST — no delivery ACK from head");
    }
    static uint32_t lastStatus = 0;
    if (now - lastStatus > 5000) {
        lastStatus = now;
        Serial.printf("[ESPNOW] Link: %s  SeqTx: %u\n",
                      _linked ? "OK" : "LOST", _seqNum);
    }
}

void ESPNowBody::sendTelemetry(const RawIMUData&    raw,
                                const IMUState&      est,
                                const BalanceOutput& bal) {
    if (!_peerAdded) return;

    BodyTelemetry pkt{};
    pkt.msg_type         = static_cast<uint8_t>(MsgType::TELEMETRY);
    pkt.accel_x_ms2      = raw.accel_x_ms2;
    pkt.accel_y_ms2      = raw.accel_y_ms2;
    pkt.accel_z_ms2      = raw.accel_z_ms2;
    pkt.gyro_x_rads      = raw.gyro_x_rads;
    pkt.gyro_y_rads      = raw.gyro_y_rads;
    pkt.gyro_z_rads      = raw.gyro_z_rads;
    pkt.pitch_deg        = est.pitch_deg;
    pkt.roll_deg         = est.roll_deg;
    // Use already-converted deg/s from BalanceOutput — avoids a duplicate
    // rad->deg conversion and stays consistent with what balance_controller used.
    pkt.pitch_rate_degs  = bal.pitch_rate_degs;
    pkt.correction_vel_y = bal.correction_vel_y;
    pkt.correction_vel_x = bal.correction_vel_x;
    pkt.uptime_ms        = millis();
    pkt.seq_num          = _seqNum++;
    pkt.imu_valid        = raw.valid;
    pkt.bal_enabled      = bal.balance_enabled;
    pkt.estop_active     = bal.estop_active;

    const uint8_t* target = ESPNOW_PEER_DISCOVERY
                          ? (const uint8_t*)"\xFF\xFF\xFF\xFF\xFF\xFF"
                          : _peerMac;
    esp_now_send(target, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void ESPNowBody::_onSendCb(const uint8_t*, esp_now_send_status_t status) {
    if (_instance && status == ESP_NOW_SEND_SUCCESS) _instance->_onAck();
}

void ESPNowBody::_onRecvCb(const esp_now_recv_info_t* info,
                            const uint8_t* data, int len) {
    if (_instance) _instance->_onRecv(info->src_addr, data, len);
}

void ESPNowBody::_onAck() {
    _lastAckMs = millis();
    if (!_linked) {
        _linked = true;
        Serial.println("[ESPNOW] Link established — head ACK received");
    }
}

void ESPNowBody::_onRecv(const uint8_t* senderMac, const uint8_t* data, int len) {
    if (ESPNOW_PEER_DISCOVERY && !_peerAdded) {
        Serial.printf("[ESPNOW] Peer discovered: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      senderMac[0], senderMac[1], senderMac[2],
                      senderMac[3], senderMac[4], senderMac[5]);
        Serial.println("[ESPNOW] Copy above MAC -> ESPNOW_HEAD_AP_MAC in espnow_config.h");
        memcpy(_peerMac, senderMac, 6);
        _addPeer(senderMac);
        _peerAdded = true;
    }

    if (len < 1) return;
    const MsgType type = static_cast<MsgType>(data[0]);

    if (type == MsgType::PING) {
        HeadCommand pong{};
        pong.msg_type = static_cast<uint8_t>(MsgType::PONG);
        esp_now_send(_peerMac, reinterpret_cast<const uint8_t*>(&pong), sizeof(pong));
        return;
    }

    if (static_cast<size_t>(len) < sizeof(HeadCommand)) return;
    const HeadCommand& cmd = *reinterpret_cast<const HeadCommand*>(data);

    switch (type) {
    case MsgType::DRIVE:
        if (_driveCb) _driveCb(_clamp1(cmd.f1), _clamp1(cmd.f2));
        break;
    case MsgType::SPEED:
        if (_speedCb) {
            const float s = cmd.f1 < 0.0f ? 0.0f : cmd.f1;
            _speedCb(s);
            _currentSpeed = s;
        }
        break;
    case MsgType::STOP:
        if (_driveCb) _driveCb(0.0f, 0.0f);
        break;
    case MsgType::BAL_ENABLE:
        if (_balEnableCb) _balEnableCb(cmd.u1 != 0);
        break;
    case MsgType::BAL_TUNE:
        if (_balTuneCb) _balTuneCb(cmd.f1, cmd.f2, cmd.f3, cmd.f4);
        break;
    default: break;
    }
}

bool ESPNowBody::_addPeer(const uint8_t* mac) {
    if (esp_now_is_peer_exist(mac)) return true;
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;  // body is always STA mode
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[ESPNOW] esp_now_add_peer() failed");
        return false;
    }
    return true;
}