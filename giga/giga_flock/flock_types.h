// ============================================================
// flock_types.h — shared data model + global state (extern)
// ============================================================
// Globals are DEFINED once in giga_flock.ino and referenced here as extern so
// ui.cpp / storage.cpp / audio.cpp can read them.  Everything is touched only
// from loop() context (single-threaded), so no locking is needed.
#pragma once

#include <Arduino.h>
#include <string.h>
#include "config.h"

#define MAX_DEVICES 200

// One row of the in-RAM device table (for the UI / dedupe / event decisions).
struct DeviceEntry
{
  char     mac[18];        // "aa:bb:cc:dd:ee:ff"
  char     method[16];     // "oui_addr2" / "oui_addr1" / "oui_addr3" / "wildcard_probe" / "ssid"
  char     ssid[33];       // "" unless an ssid hit populated it
  int8_t   rssi;           // most recent RSSI
  int8_t   bestRssi;       // strongest (closest to 0) RSSI seen
  uint8_t  channel;        // most recent channel
  uint16_t count;          // running hit count reported by the ESP32
  uint32_t firstSeenMs;    // millis() at first local sighting
  uint32_t lastSeenMs;     // millis() at latest local sighting

  // --- event-log bookkeeping (append-only CSV sampling) ---
  bool     hasLoggedEvent; // at least one event row written for this MAC
  double   lastEventLat;   // GPS position at last logged event
  double   lastEventLon;
  uint32_t lastEventMs;    // millis() of last logged event
};

// Last status heartbeat from the ESP32 radio.
struct LinkStatus
{
  bool     everSeen;       // any status ever received
  uint32_t lastStatusMs;   // millis() of last status line
  uint8_t  channel;        // ESP32's current channel
  uint32_t uptime;         // ESP32 uptime (s)
  uint32_t pkts;           // total mgmt/data frames the ESP32 has seen
  int      uniq;           // unique count reported by the ESP32
};

// Current GPS fix state (parsed from Serial2 NMEA).
struct GpsState
{
  bool     hasFix;
  double   lat;
  double   lon;
  uint32_t utc;            // unix epoch, UTC (0 if unknown)
  uint32_t sats;
  double   hdop;
};

// ---- Globals (defined in giga_flock.ino) ----
extern DeviceEntry g_dev[MAX_DEVICES];
extern int         g_devCount;
extern uint32_t    g_totalEvents;   // CSV event rows that met the sampling rule
extern LinkStatus  g_link;
extern GpsState    g_gps;
extern int         g_lastDevIdx;    // most-recent detection (for ALERT screen), -1 = none
extern uint32_t    g_lastDetMs;     // millis() of most-recent detection
extern bool        g_logReady;      // QSPI log mounted & writable
extern volatile bool g_uiDirty;     // device table changed -> rebuild LIVE list

// ---- Class-color helper (0xRRGGBB), shared everywhere ----
static inline uint32_t methodColorHex(const char *method)
{
  if (!method) return COL_UNKNOWN;
  if (!strcmp(method, "wildcard_probe")) return COL_WILDCARD;
  if (!strcmp(method, "oui_addr2"))      return COL_ADDR2;
  if (!strcmp(method, "oui_addr1"))      return COL_ADDR1;
  if (!strcmp(method, "oui_addr3"))      return COL_ADDR3;
  if (!strcmp(method, "ssid"))           return COL_SSID;
  return COL_UNKNOWN;
}
