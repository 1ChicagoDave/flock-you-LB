// ============================================================
// config.h — pins, cadence, orientation, colors
// ============================================================
// All tunables live here so the other translation units stay clean.
#pragma once

#include <Arduino.h>

// ---- Serial links -----------------------------------------------------------
// ESP32 radio link on Serial4 (GIGA D14=TX, D15=RX per the variant). ESP32 TX
// (GPIO17) -> GIGA D15, ESP32 RX (GPIO16) <- GIGA D14, common GND. Newline JSON
// @115200 (see esp32_companion/companion.cpp).
// NOTE: D18/D19 (Serial2) do NOT carry the link on this unit (display shield /
// bad pins) despite a good signal both ends — hardware-verified. Use Serial4.
#define ESP32_SERIAL   Serial4
#define ESP32_BAUD     115200

// GPS on Serial3 (GIGA D16=TX, D17=RX ARE Serial3 per the variant). 9600 NMEA.
#define GPS_SERIAL     Serial3
#define GPS_BAUD       9600
#define GPS_STALE_MS   3000     // a fix older than this is treated as "no fix"

// ---- Speaker ----------------------------------------------------------------
// DAC0 == analog pin A12 on the GIGA R1.  A rising two-note chirp fires on each
// brand-new unique MAC.  (See audio.cpp for the DAC synthesis approach.)
#define SPEAKER_PIN    A12      // DAC0

// ---- Onboard RGB LED (Arduino_GigaDisplay :: GigaDisplayRGB) -----------------
#define LED_FLASH_MS   1200     // class-color flash duration on each detection
#define LED_BRIGHT     120      // 0..255 per channel cap (full is very bright)

// ---- UI / display -----------------------------------------------------------
// Landscape.  Arduino_H7_Video handles the panel rotation from these dims.
#define SCREEN_W       800
#define SCREEN_H       480
#define TABBAR_H       64       // taller tab bar = bigger, easier touch targets
#define LIVE_MAX_ROWS  14       // rows that fit on the LIVE list
// HUNTER gauge diameter. Must leave room for the title + MAC labels inside
// (SCREEN_H - TABBAR_H); oversizing it makes the tab scroll, which drags the
// readout off-screen and costs a large repaint on every scroll frame.
#define HUNTER_ARC_D   260
// Touch release debounce. The GT911 updates at ~100 Hz but LVGL polls it on
// its own clock, so a poll can land between controller updates and return
// "no points" while a finger is still down. Without this, LVGL sees a
// release/press flicker mid-touch: clicks misfire and the 2-point calibration
// reads one held finger as two taps at the same spot. A gap shorter than this
// is treated as still pressed.
#define TOUCH_RELEASE_MS 60
#define UI_TICK_MS     200      // label refresh cadence
#define LIVE_REBUILD_MS 400     // min interval between LIVE list rebuilds

// ---- NAV tab: bearing / distance to a fixed target + Strength marks --------
// Target = 35.92963 N, 84.30987 W.  The compass is heading-up (straight up on
// the screen = direction of travel) using GPS course-over-ground.  Below
// NAV_MIN_SPEED_MPS the course is noise, so we fall back to north-up and say so.
#define NAV_TARGET_LAT      35.92963
#define NAV_TARGET_LON     -84.30987
#define NAV_MIN_SPEED_MPS   1.5      // ~3.4 mph
#define NAV_TICK_MS         500      // readout / compass refresh cadence
#define NAV_SAVED_MS        1500     // how long the "SAVED S3" confirmation shows

// ---- Watchdog ---------------------------------------------------------------
// Hardware IWDG timeout. Loop normally cycles in ~5 ms, so this only fires on a
// genuine hang (e.g. a display flush that never returns).
#define WATCHDOG_MS    8000
// Software hang detector. Must fire BEFORE the hardware watchdog so it has time
// to write the #HANG marker before the board resets.
#define HANG_DETECT_MS 4000

// ---- Link liveness ----------------------------------------------------------
#define LINK_ALIVE_MS  6000     // ESP32 "alive" if a status arrived < this ago

// ---- Event log sampling (stationary suppression) ----------------------------
// Per MAC, an event row is appended only when we have a GPS fix AND it is the
// first event for that MAC, OR we've moved >= EVENT_MIN_DISTANCE_M since its
// last logged event, OR EVENT_MIN_INTERVAL_MS has elapsed (keepalive). This
// mirrors main.cpp's model: keep a live device table for the UI plus an
// append-only event log for later triangulation.
#define EVENT_MIN_DISTANCE_M   20.0     // metres (equirectangular approximation)
#define EVENT_MIN_INTERVAL_MS  60000UL  // keepalive interval

// ---- Breadcrumb track log ---------------------------------------------------
// A position row every TRACK_INTERVAL_MS (only with a fix, and only if we've
// actually moved) plus a #BOOT row each startup. This makes a quiet stretch
// PROVABLE: you can see the detector was alive and where it was, so "no cameras
// heard here" is distinguishable from "the log just stopped".
// 30 s: at highway speed a 60 s cadence leaves ~1.2 km gaps between points,
// which is too coarse to tell whether you actually passed a given camera.
#define TRACK_INTERVAL_MS      30000UL  // breadcrumb cadence
// 50 m: GPS jitter on a marginal fix exceeded the old 25 m threshold and wrote
// breadcrumbs while parked (24 of them in one stationary session).
#define TRACK_MIN_MOVE_M       50.0     // skip the breadcrumb if parked

// ---- Audio re-alert ---------------------------------------------------------
// A known camera goes silent forever once it's in the persisted table, which
// makes a familiar commute feel dead. Re-chirp when a device reappears after
// this long unseen (the ESP32 does the same at 30 s on its own buzzer).
#define REDISCOVER_CHIRP_MS    300000UL // 5 minutes

// ---- Alert screen behaviour -------------------------------------------------
// How long the full alert holds before returning to the screen you were on.
#define ALERT_HOLD_MS          6000UL
#define LOG_PATH               "/fs/detections.csv"  // mbed FATFileSystem mount root is /fs
#define LOG_HEADER             "mac,method,rssi,channel,lat,lon,utc,sats,hdop,ssid"

// ---- Detection-class colors (0xRRGGBB) --------------------------------------
// Must match esp32_companion/companion.cpp :: alertTypeColor and main.cpp.
//   wildcard_probe = red, oui_addr2 = amber, oui_addr1 = blue,
//   oui_addr3 = cyan, ssid = magenta.
#define COL_WILDCARD   0xFF2A2A   // red
#define COL_ADDR2      0xFFB000   // amber
#define COL_ADDR1      0x2A6BFF   // blue
#define COL_ADDR3      0x00E5E5   // cyan
#define COL_SSID       0xFF3CFF   // magenta
#define COL_UNKNOWN    0xE0E0E0   // white/grey fallback

// UI chrome
#define COL_BG         0x0A0E14
#define COL_TEXT       0xE6EAF0
#define COL_DIM        0x8A94A6
#define COL_OK         0x30D158
#define COL_BAD        0xFF453A
