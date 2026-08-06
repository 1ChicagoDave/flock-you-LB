// ============================================================
// storage.h — QSPI flash CSV event log
// ============================================================
#pragma once

#include <Arduino.h>

// Mounts the 16 MB QSPI flash (FAT) and opens the append-only event log.
// Returns true on success.  On failure the firmware runs WITHOUT logging
// (never hangs) — see storage.cpp for the one-time-format note.
bool storage_init();

// Append one detection EVENT row.  Columns (exactly):
//   mac,method,rssi,channel,lat,lon,utc,sats,hdop,ssid
// Returns true if the row was written (and flushed).
bool storage_append_event(const char *mac, const char *method, int rssi,
                          int channel, double lat, double lon, uint32_t utc,
                          uint32_t sats, double hdop, const char *ssid);

// True once the log is mounted and writable.
bool storage_ready();
