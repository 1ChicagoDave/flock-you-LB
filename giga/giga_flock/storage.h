// ============================================================
// storage.h — internal QSPI flash CSV event log
// ============================================================
#pragma once

#include <Arduino.h>

// Mounts the internal 16 MB QSPI flash's FAT user-data partition (MBR
// partition 2) via the mbed FATFileSystem API and opens the append-only event
// log at /fs/detections.csv.  Returns true on success.  If the partition is not
// present/formatted (one-time QSPIFormat example — see storage.cpp) the
// firmware runs WITHOUT logging and never hangs.
bool storage_init();

// Append one detection EVENT row.  Columns (exactly):
//   mac,method,rssi,channel,lat,lon,utc,sats,hdop,ssid
// Returns true if the row was written (and flushed).
bool storage_append_event(const char *mac, const char *method, int rssi,
                          int channel, double lat, double lon, uint32_t utc,
                          uint32_t sats, double hdop, const char *ssid);

// True once the log is mounted and writable.
bool storage_ready();
