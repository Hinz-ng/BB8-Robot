// module:  webcomm.h
// layer:   7 — network I/O only; no control logic lives here
// purpose: AsyncWebServer + WebSocket server; broadcasts IMU telemetry,
//          parses drive/config commands from phone UI
// inputs:  RawIMUData (from imu.h), optional DriveCallback
// outputs: JSON telemetry over WebSocket; parsed commands via callbacks
// deps:    ESPAsyncWebServer, AsyncTCP, ArduinoJson, imu.h
// date:    2025

#pragma once

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <functional>
#include "imu.h"

// ── Command callbacks ─────────────────────────────────────────────────────────
// Drive callback: vel_x and vel_y in range [-1.0, +1.0]
// Register via setDriveCallback() before calling begin()
using DriveCallback = std::function<void(float vel_x, float vel_y)>;

// ── Telemetry packet ──────────────────────────────────────────────────────────
// Snapshot assembled each broadcast tick and sent to all connected clients
struct TelemetryPacket {
    // IMU — raw calibrated values
    float accel_x_ms2;
    float accel_y_ms2;
    float accel_z_ms2;
    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;
    bool  imu_valid;

    // System
    uint32_t uptime_ms;
    uint8_t  client_count;
};

// ── WebComm ───────────────────────────────────────────────────────────────────
class WebComm {
public:
    // Call once in setup() after Serial is ready
    // ssid/pass: WiFi credentials; if pass is "", opens open network
    bool begin(const char* ssid, const char* pass);

    // Call every loop() tick — handles WebSocket cleanup
    void update();

    // Called from main control loop at WEBCOMM_BROADCAST_HZ
    // Assembles JSON and pushes to all connected clients
    void broadcastIMU(const RawIMUData& data);

    // Register before begin() — called when a drive command arrives from UI
    // If not registered, drive commands are silently dropped
    void setDriveCallback(DriveCallback cb) { _driveCb = cb; }

    // True once WiFi is associated and server is listening
    bool isConnected() const { return _connected; }

    // Current WebSocket client count
    uint8_t clientCount() const;

    // IP address as string — for Serial print after begin()
    String ipAddress() const;

private:
    AsyncWebServer  _server{80};
    AsyncWebSocket  _ws{"/ws"};
    DriveCallback   _driveCb;
    bool            _connected{false};

    // WebSocket event handler — static trampoline into instance method
    static void _wsEventStatic(AsyncWebSocket* server,
                                AsyncWebSocketClient* client,
                                AwsEventType type,
                                void* arg,
                                uint8_t* data,
                                size_t len);

    void _wsEvent(AsyncWebSocket* server,
                  AsyncWebSocketClient* client,
                  AwsEventType type,
                  void* arg,
                  uint8_t* data,
                  size_t len);

    // Parse a text frame from the client
    // Expected format: "CMD:DRIVE:x=0.5,y=-0.3" or "CMD:STOP"
    void _parseCommand(const char* msg, size_t len);

    // Send welcome packet to a newly connected client
    void _sendWelcome(AsyncWebSocketClient* client);

    // Clamp helper — keeps drive values in [-1.0, +1.0]
    static float _clamp1(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }
};