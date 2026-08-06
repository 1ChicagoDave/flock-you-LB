// ============================================================
// config.h — pins, cadence, orientation, colors
// ============================================================
// All tunables live here so the other translation units stay clean.
#pragma once

#include <Arduino.h>

// ---- Serial links -----------------------------------------------------------
// ESP32 radio link on Serial1 (GIGA D18=TX, D19=RX).  ESP32 TX -> GIGA D19,
// ESP32 RX <- GIGA D18, common GND.  Newline-delimited JSON @115200 (see
// esp32_companion/companion.cpp for the exact wire format we parse).
#define ESP32_SERIAL   Serial1
#define ESP32_BAUD     115200

// GPS on Serial2 (GIGA D16=TX, D17=RX).  Adafruit Ultimate GPS, 9600 NMEA.
#define GPS_SERIAL     Serial2
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
#define TABBAR_H       44
#define LIVE_MAX_ROWS  14       // rows that fit on the LIVE list
#define UI_TICK_MS     200      // label refresh cadence
#define LIVE_REBUILD_MS 400     // min interval between LIVE list rebuilds

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
#define LOG_PATH               "/qspi/detections.csv"  // QSPI FAT mount root is /qspi
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
