// module:  webcomm.h
// layer:   7 — network I/O only; no control logic lives here
// purpose: AsyncWebServer + WebSocket server; broadcasts IMU/balance telemetry,
//          parses drive/config/balance commands from phone UI
// inputs:  RawIMUData (from imu.h), IMUState (from state_estimator.h),
//          BalanceOutput (from balance_controller.h)
// outputs: JSON telemetry over WebSocket; parsed commands via callbacks
// deps:    ESPAsyncWebServer, AsyncTCP, ArduinoJson, imu.h,
//          state_estimator.h, balance_controller.h
// date:    2025

#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <functional>
#include "imu.h"
#include "state_estimator.h"
#include "balance_controller.h"
#include "project_wide_defs.h"

// ── Command callbacks ─────────────────────────────────────────────────────────
using DriveCallback         = std::function<void(float vel_x, float vel_y)>;
using SpeedCallback         = std::function<void(float scale)>;
// Balance enable/disable — called when CMD:BALANCE:enable=X arrives
using BalanceEnableCallback = std::function<void(bool enabled)>;
// Balance tune — called when CMD:BALANCE_TUNE:Kp=X,Kd=Y,Kpr=X,Kdr=Y arrives
// Gains are already clamped to [0, KP_MAX/KD_MAX] before the callback fires.
using BalanceTuneCallback   = std::function<void(float kp, float kd, float kp_roll, float kd_roll)>;

// ── WebComm ───────────────────────────────────────────────────────────────────
class WebComm {
public:
    // Call once in setup() after Serial is ready.
    bool begin(const char* ssid, const char* pass);

    // Call every loop() tick — handles WebSocket cleanup.
    void update();

    // Called from main control loop at WEBCOMM_BROADCAST_HZ.
    // Assembles JSON and pushes to all connected clients.
    void broadcastState(const RawIMUData& raw, const IMUState& est, const BalanceOutput& bal);

    // Register before begin()
    void setDriveCallback(DriveCallback cb)               { _driveCb = cb; }
    void setSpeedCallback(SpeedCallback cb)               { _speedCb = cb; }
    void setBalanceEnableCallback(BalanceEnableCallback cb){ _balanceEnableCb = cb; }
    void setBalanceTuneCallback(BalanceTuneCallback cb)   { _balanceTuneCb = cb; }

    void setCurrentSpeed(float s) {
        _currentSpeed = (s < 0.0f) ? 0.0f : ((s > DRIVE_SPEED_MAX) ? DRIVE_SPEED_MAX : s);
    }

    bool    isConnected() const { return _connected; }
    uint8_t clientCount() const;
    String  ipAddress() const;

private:
    AsyncWebServer  _server{80};
    AsyncWebSocket  _ws{"/ws"};
    DriveCallback         _driveCb;
    SpeedCallback         _speedCb;
    BalanceEnableCallback _balanceEnableCb;
    BalanceTuneCallback   _balanceTuneCb;
    float           _currentSpeed{DRIVE_SPEED_DEFAULT};
    bool            _connected{false};

    static void _wsEventStatic(AsyncWebSocket* server,
                                AsyncWebSocketClient* client,
                                AwsEventType type, void* arg,
                                uint8_t* data, size_t len);

    void _wsEvent(AsyncWebSocket* server,
                  AsyncWebSocketClient* client,
                  AwsEventType type, void* arg,
                  uint8_t* data, size_t len);

    void _parseCommand(const char* msg, size_t len);
    void _sendWelcome(AsyncWebSocketClient* client);

    static float _clamp1(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }
};