// module:  espnow_body.h
// layer:   7 — transport; replaces webcomm.h on the body unit
// purpose: ESP-NOW telemetry sender (body->head at 20 Hz) and command receiver
//          (head->body, event-driven). Callback API is identical to removed
//          WebComm so main.cpp wiring changes are minimal.
// date:    2025

#pragma once
#include <functional>
#include <cstdint>
#include "imu.h"
#include "state_estimator.h"
#include "balance_controller.h"
#include "espnow_protocol.h"
#include "espnow_config.h"

// Callback types — identical to removed webcomm.h
using DriveCallback         = std::function<void(float vel_x, float vel_y)>;
using SpeedCallback         = std::function<void(float scale)>;
using BalanceEnableCallback = std::function<void(bool enabled)>;
using BalanceTuneCallback   = std::function<void(float kp, float kd,
                                                   float kp_roll, float kd_roll)>;

class ESPNowBody {
public:
    // Call in setup() after Serial is ready.
    // Initialises WiFi STA mode (no connection), locks channel, starts ESP-NOW.
    bool begin();

    // Call every loop() tick — updates link health.
    void update();

    // Packs raw + est + bal into BodyTelemetry and sends.
    // Call at WEBCOMM_BROADCAST_HZ — mirrors former broadcastState().
    void sendTelemetry(const RawIMUData&    raw,
                       const IMUState&      est,
                       const BalanceOutput& bal);

    // Register before begin() — identical signature to removed WebComm setters.
    void setDriveCallback(DriveCallback cb)                { _driveCb    = cb; }
    void setSpeedCallback(SpeedCallback cb)                { _speedCb    = cb; }
    void setBalanceEnableCallback(BalanceEnableCallback cb){ _balEnableCb = cb; }
    void setBalanceTuneCallback(BalanceTuneCallback cb)    { _balTuneCb  = cb; }

    void setCurrentSpeed(float s) { _currentSpeed = s; }
    bool isLinked() const { return _linked; }

private:
    DriveCallback         _driveCb;
    SpeedCallback         _speedCb;
    BalanceEnableCallback _balEnableCb;
    BalanceTuneCallback   _balTuneCb;

    uint16_t _seqNum       = 0;
    bool     _linked       = false;
    bool     _peerAdded    = false;
    uint8_t  _peerMac[6]   = {};
    float    _currentSpeed = 0.0f;
    uint32_t _lastAckMs    = 0;

    void _onAck();
    void _onRecv(const uint8_t* senderMac, const uint8_t* data, int len);
    bool _addPeer(const uint8_t* mac);

    static void _onSendCb(const uint8_t* mac, esp_now_send_status_t status);
    static void _onRecvCb(const esp_now_recv_info_t* info,
                          const uint8_t* data, int len);

    static ESPNowBody* _instance;
    static float _clamp1(float v) {
        return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
    }
};