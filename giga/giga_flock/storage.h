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

// Stream the whole /fs/detections.csv to USB Serial (for `d` serial command).
void storage_dump_csv();

// Append a non-detection marker row (breadcrumb / session boundary). Keeps the
// same 10-column schema so the CSV stays machine-readable; `tag` goes in the mac
// column and always starts with '#' (e.g. "#BOOT", "#TRK") so analysis tools can
// filter marker rows out with a single startswith('#') test.
// `kind` lands in the method column: "track" for a breadcrumb, or the reset
// cause ("power", "WATCHDOG", ...) for a #BOOT row, so a log read back later
// shows WHY each session started.
bool storage_append_mark(const char *tag, const char *kind, double lat, double lon,
                         uint32_t utc, uint32_t sats, double hdop);

// Append a Strength mark from the NAV screen.  Same 10-column schema:
//   mac      = "#MARK"        method  = "strength"
//   rssi     = strength 1..5  channel = heading deg (0..359), or -1 if unknown
//   lat/lon/utc/sats/hdop as usual; ssid carries a readable "S3 hdg 127" note.
bool storage_append_strength(int strength, int headingDeg, double lat, double lon,
                             uint32_t utc, uint32_t sats, double hdop);

// Free-text marker row: tag,kind,0,0,0,0,utc,0,0,"note". Used for #CAL rows so
// a calibration attempt (raw corner readings, accepted/rejected) is readable
// later from the log instead of only on a live serial console.
bool storage_append_note(const char *tag, const char *kind, const char *note);

// Touch calibration (2-point scale/offset) persisted at /fs/touchcal.bin.
// save rejects implausible values; load returns false if absent/corrupt.
bool storage_save_touchcal(float sx, float ox, float sy, float oy);
bool storage_load_touchcal(float *sx, float *ox, float *sy, float *oy);
void storage_clear_touchcal();

// Display backlight level (0..100) persisted at /fs/backlight.bin. This has to
// persist because the device power-cycles at every engine-off: without it the
// screen would come back at full brightness on every single trip.
// stepUs is persisted alongside the level: this backlight only dims at certain
// PWM frequencies, so a frequency dialled in with serial f has to survive the
// power cycle that happens at every engine-off, exactly like the level does.
bool storage_save_backlight(uint8_t pct, uint32_t stepUs);
bool storage_load_backlight(uint8_t *pct, uint32_t *stepUs);

// Device-table snapshot so hit counts survive a reboot/power cycle.
// save writes /fs/fy_table.bin; load restores g_dev / g_devCount / g_totalEvents.
void storage_save_table();
bool storage_load_table();
