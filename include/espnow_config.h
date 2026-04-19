// module:  espnow_config.h
// layer:   transport configuration
// purpose: RF channel, MAC addresses, link timeouts shared by both MCUs.
// CRITICAL: IDENTICAL COPY required in both projects.
//
// ── Initial MAC Discovery Procedure ──────────────────────────────────────────
//  1. Set ESPNOW_PEER_DISCOVERY = true. Flash and power on the HEAD unit first.
//     → Serial will print: "[ESPNOW] My AP MAC: XX:XX:XX:XX:XX:XX"
//     → Copy this value into ESPNOW_HEAD_AP_MAC below.
//
//  2. Flash and power on the BODY unit.
//     → Serial will print: "[ESPNOW] My STA MAC: XX:XX:XX:XX:XX:XX"
//     → Copy this value into ESPNOW_BODY_STA_MAC below.
//
//  3. Verify both units print:
//       "[ESPNOW] Peer discovered and registered"
//     and the body prints "[ESPNOW] Link: OK" within a few seconds.
//
//  4. Fill in both MAC arrays below.
//  5. Set ESPNOW_PEER_DISCOVERY = false.
//  6. Update BOTH copies, reflash BOTH units.
// ─────────────────────────────────────────────────────────────────────────────

#pragma once
#include <cstdint>

// ── RF channel ────────────────────────────────────────────────────────────────
// Must equal WIFI_AP_CHANNEL in the head unit's project_wide_defs.h.
// Changing this without updating BOTH constants will silently break ESP-NOW.
constexpr uint8_t ESPNOW_CHANNEL = 1;

// ── Peer discovery ────────────────────────────────────────────────────────────
// true  = broadcast mode; units discover each other at runtime (initial setup)
// false = unicast mode; uses hardcoded MACs below (production; faster reconnect)
constexpr bool ESPNOW_PEER_DISCOVERY = true;  // set false after MACs filled in

// ── Hardcoded MAC addresses ───────────────────────────────────────────────────
// HEAD_AP_MAC:  head unit's softAP MAC  → from "[ESPNOW] My AP MAC:"
// BODY_STA_MAC: body unit's STA MAC     → from "[ESPNOW] My STA MAC:"
// Why different MAC types? Head is in AP mode; ESP-NOW uses the AP interface MAC.
// Body is in STA mode (no connection); ESP-NOW uses the STA interface MAC.
constexpr uint8_t ESPNOW_HEAD_AP_MAC[6]  = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
constexpr uint8_t ESPNOW_BODY_STA_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ── Link health timeouts ──────────────────────────────────────────────────────
// LINK_TIMEOUT_MS:  body declares link lost if no command received for this long.
//                   head declares link lost if no telemetry received for this long.
// PING_INTERVAL_MS: how often a PING is sent when no normal traffic is flowing.
constexpr uint32_t ESPNOW_LINK_TIMEOUT_MS  = 2000;   // ms
constexpr uint32_t ESPNOW_PING_INTERVAL_MS = 1000;   // ms