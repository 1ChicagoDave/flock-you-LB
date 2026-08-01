// ============================================================================
// FlockYou — LCDWIKI E32R40T port
// ============================================================================
// Passive 2.4 GHz promiscuous-mode WiFi detector for Flock Safety surveillance
// gear, ported from the Lonely Binary "Gold Edition" (main.cpp) onto the LCDWIKI
// E32R40T display board (ESP32-D0WD-V3, ST7796S 320x480, XPT2046 touch, microSD,
// SC8002B audio amp, RGB LED, GPS header, Li-battery ADC).
//
// The detection engine (OUI list, 802.11 frame parsing, wildcard-probe
// signature, dedupe, channel hopping) is copied VERBATIM from main.cpp — it is
// upstream research and is left untouched. Only the peripheral layers differ:
//   LED   — common-anode RGB on 22/16/17 (was a single WS2812 NeoPixel)
//   AUDIO — DAC tone through the SC8002B amp (was a piezo buzzer)
//   GPS   — Serial2 on 25/32          (was Serial1 on 16/17)
//   STORE — SD-card CSV session files (was a SPIFFS binary snapshot)
//   UI    — a minimal TFT_eSPI status screen (new)
// BLE and SPIFFS persistence are removed entirely.
// ----------------------------------------------------------------------------

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <ctype.h>
#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <TinyGPSPlus.h>
#include <Preferences.h> // persist rotation-specific touch calibration in NVS
#include <math.h>

// ============================================================
// BOARD PIN MAP  (E32R40T — verified in hosyond/bringup.cpp)
// ============================================================
// Display + touch pins are owned by TFT_eSPI (set via platformio build_flags);
// everything below is driven directly here.

#define PIN_LED_R 22 // RGB LED, COMMON ANODE: drive LOW to light
#define PIN_LED_G 16
#define PIN_LED_B 17
#define LED_ON LOW
#define LED_OFF HIGH

#define PIN_SD_CS 5 // microSD on VSPI (separate bus from the display HSPI)
#define PIN_SD_SCK 18
#define PIN_SD_MISO 19
#define PIN_SD_MOSI 23

#define PIN_AUDIO_DAC 26 // ESP32 DAC2 -> SC8002B AUDIO_IN
#define PIN_AUDIO_EN 4   // SC8002B enable (drive HIGH to un-mute the amp)

#define PIN_BAT_ADC 34 // battery voltage via 100K/100K divider (x2), input-only

#define PIN_GPS_RX 25 // GPS on the 4-pin connector, remapped as UART2 RX
#define PIN_GPS_TX 32 // ESP32 UART2 TX -> GPS RX

#define SCREEN_ROTATION 1 // 0/2 = portrait 320x480, 1/3 = landscape 480x320 (UI is landscape)

// ============================================================
// CONFIG
// ============================================================

// Detection-class LED color mapping is scaled to LED_BRIGHTNESS in
// alertTypeColor(); on this board the RGB LED is a plain digital common-anode
// device (no PWM), so any nonzero channel value simply lights that channel.
#define LED_BRIGHTNESS 64
// Long enough to catch out of the corner of your eye while driving; the flash
// is non-blocking (ledTick() clears it), so this never stalls the sniffer.
#define LED_FLASH_MS 2000
// Boot self-test: dwell per color while stepping the detection palette.
#define BOOT_CYCLE_MS 500

// ---- On-board GPS (NMEA over UART2, parsed by TinyGPS++) ----
// Detections are stamped with the current fix (lat/lon + UTC) when one is
// available; with no fix they're still recorded, just without geodata.
#define GPS_BAUD     9600 // Adafruit Ultimate GPS default
#define GPS_STALE_MS 5000 // a fix older than this counts as lost

#define CHANNEL_MODE_FULL_HOP 0
#define CHANNEL_MODE_CUSTOM 1
#define CHANNEL_MODE_SINGLE 2

#define CHANNEL_MODE CHANNEL_MODE_CUSTOM
#define CHANNEL_DWELL_MS 350
#define SINGLE_CHANNEL 1

static const uint8_t customChannels[] = {1, 6, 11};
static const size_t customChannelCount = sizeof(customChannels) / sizeof(customChannels[0]);

static const uint8_t fullHopChannels[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const size_t fullHopChannelCount = sizeof(fullHopChannels) / sizeof(fullHopChannels[0]);

#define HEARTBEAT_MS 30000
#define RSSI_MIN -95
#define ALERT_COOLDOWN_MS 5000

// Audio cadence: two fast ascending beeps on a NEW MAC, then while any
// target is still in range (seen within HB_DEVICE_ACTIVE_MS), two monotone
// heartbeat beeps every HB_BEEP_INTERVAL_MS.
#define HB_DEVICE_ACTIVE_MS 3000
#define HB_BEEP_INTERVAL_MS 10000
// A MAC we haven't heard from in REDISCOVER_MS counts as a fresh discovery
// next time it shows up — fires the ascending chirp again. Shorter than a
// Flock's burst-sleep gap would mean false chirps; longer means you'd miss
// a drive-away/return. 30 s is a good middle ground.
#define REDISCOVER_MS 30000
#define NEW_CHIRP_LO_HZ 2000
#define NEW_CHIRP_HI_HZ 2800
#define NEW_CHIRP_NOTE_MS 55
#define NEW_CHIRP_GAP_MS 25
#define HB_BEEP_HZ 1500
#define HB_BEEP_NOTE_MS 70
#define HB_BEEP_GAP_MS 70

#define ENABLE_SSID_MATCH 0
#define CHECK_ADDR1 1 // dst/rx — catches Flock STAs receiving probe responses
#define CHECK_ADDR3 0 // bssid fallback for randomised addr2
static const char *target_ssid_keywords[] = {"flock"};
static const size_t SSID_KEYWORD_COUNT = sizeof(target_ssid_keywords) / sizeof(target_ssid_keywords[0]);

#define STOP_ON_SSID_HIT 0
#define STOP_ON_OUI_HIT 0
#define PROCESS_MGMT_FRAMES 1
#define PROCESS_DATA_FRAMES 1

// Persistence — session CSV on the SD card, latest file reloaded on boot so
// counts survive power loss.
#define MAX_DETECTIONS 200
#define AUTOSAVE_INTERVAL_MS 60000

// ---- Touchscreen UI (Phase 2a) ----
// Landscape 480x320 layout. The status strip lives across the top, the active
// screen body in the middle, and a persistent 4-tab bar along the bottom.
#define UI_W 480
#define UI_H 320
#define STRIP_H 22             // shared status strip: y 0..22
#define BODY_TOP 24            // screen body starts below the strip
#define TABBAR_Y 276           // tab bar: y 276..320
#define TABBAR_H (UI_H - TABBAR_Y)
#define BODY_BOT TABBAR_Y      // body ends where the tab bar begins
#define TAB_W (UI_W / 4)       // four equal tabs, 120 px each

// Hunter refresh rates.
#define HUNTER_UPDATE_MS 200   // radar/gauge animation ~5 Hz
#define SLOW_UPDATE_MS 1000    // Alert/Live/Stats bodies + status strip ~1 Hz

// Proximity tracking: the "strongest current target" is the highest-RSSI unique
// device seen within this rolling window; blips fade out over BLIP_FADE_MS.
#define PROX_WINDOW_MS 8000
#define BLIP_FADE_MS 6000

// RSSI -> gauge percent mapping (dBm).
#define RSSI_GAUGE_MIN -95
#define RSSI_GAUGE_MAX -40

// Sonar radar geometry (a TFT_Sprite of just this square region — never a
// full-screen buffer). 160x160x2 = 51,200 B of heap, allocated once at boot.
#define RADAR_D 160
#define RADAR_R (RADAR_D / 2)
#define RADAR_X 8              // top-left of the radar sprite within the body
#define RADAR_Y 36
#define SWEEP_STEP_DEG 9       // sweep advance per Hunter update
#define DEG2RAD 0.01745329252f

// ============================================================
// TARGET OUI LIST  (all lowercase, colons only)
// ============================================================

static const char *target_ouis[] = {
    "70:c9:4e", "3c:91:80", "d8:f3:bc", "80:30:49", "b8:35:32",
    "14:5a:fc", "74:4c:a1", "08:3a:88", "9c:2f:9d", "c0:35:32",
    "94:08:53", "e4:aa:ea", "f4:6a:dd", "f8:a2:d6", "24:b2:b9",
    "00:f4:8d", "d0:39:57", "e8:d0:fc", "e0:4f:43", "b8:1e:a4",
    "70:08:94", "58:8e:81", "ec:1b:bd", "3c:71:bf", "58:00:e3",
    "90:35:ea", "5c:93:a2", "64:6e:69", "48:27:ea", "a4:cf:12",
    // Contributed by Michael / DeFlockJoplin — discovered via wildcard-probe
    // + OUI signature during field testing. The 12th camera in his drive-test
    // used this prefix and wasn't in @NitekryDPaul's original 30.
    "82:6b:f2"

};
static const size_t OUI_COUNT = sizeof(target_ouis) / sizeof(target_ouis[0]);

// Pre-compiled byte table — populated once in setup(), never touched again.
// Keeps matchOuiRaw entirely in IRAM with no flash-resident function calls.
static uint8_t oui_bytes[OUI_COUNT][3];

// ============================================================
// ALERT QUEUE  (callback → loop, avoids Serial in WiFi task)
// ============================================================

#define ALERT_QUEUE_SIZE 64

typedef enum : uint8_t
{
  ALERT_OUI_ADDR2 = 0,
  ALERT_OUI_ADDR1 = 1,
  ALERT_OUI_ADDR3 = 2,
  ALERT_SSID = 3,
  // Probe Request + wildcard SSID (tag 0, length 0) from a known-OUI addr2.
  // Tight signature from Michael / DeFlockJoplin field research:
  //   https://github.com/DeflockJoplin/flock-you
  ALERT_WILDCARD_PROBE = 4,
} AlertType;

typedef struct
{
  AlertType type;
  uint8_t mac[6];
  int8_t rssi;
  uint8_t channel;
  char ssid[33]; // populated for SSID hits
  char frameKind[12];
} AlertEntry;

static volatile AlertEntry alertQueue[ALERT_QUEUE_SIZE];
static volatile size_t alertHead = 0; // written by callback
static volatile size_t alertTail = 0; // read by loop()
static portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;

// RF / throughput diagnostics — tell weak-RX hardware (low pkt rate, RSSI near
// the floor) apart from UI starvation (alerts dropped because loop() is behind
// draining the queue). Declared here so enqueueAlert/wifiSniffer can see them.
static volatile uint32_t fyPktSeen = 0;       // frames the sniffer processed
static volatile uint32_t fyAlertsEnqueued = 0; // matched hits queued
static volatile uint32_t fyAlertsDropped = 0;  // matched hits lost (queue full)

static void IRAM_ATTR enqueueAlert(AlertType type, const uint8_t *mac, int8_t rssi,
                                   uint8_t ch, const char *ssid, const char *kind)
{
  portENTER_CRITICAL_ISR(&queueMux);
  size_t next = (alertHead + 1) % ALERT_QUEUE_SIZE;
  if (next == alertTail)
  { // drop if full — loop() is behind
    fyAlertsDropped++;
    portEXIT_CRITICAL_ISR(&queueMux);
    return;
  }

  AlertEntry *e = (AlertEntry *)&alertQueue[alertHead];
  e->type = type;
  e->rssi = rssi;
  e->channel = ch;
  memcpy((void *)e->mac, mac, 6);

  if (ssid)
  {
    strncpy((char *)e->ssid, ssid, 32);
    ((char *)e->ssid)[32] = '\0';
  }
  else
  {
    ((char *)e->ssid)[0] = '\0';
  }

  if (kind)
  {
    strncpy((char *)e->frameKind, kind, 11);
    ((char *)e->frameKind)[11] = '\0';
  }
  else
  {
    ((char *)e->frameKind)[0] = '\0';
  }

  alertHead = next;
  fyAlertsEnqueued++;
  portEXIT_CRITICAL_ISR(&queueMux);
}

// ============================================================
// DETECTION TABLE  (on-device storage, persisted to SD CSV)
// ============================================================
//
// Single-threaded: only touched from loop() — drainAlertQueue() adds, and
// fySaveSessionCSV() reads. No mutex needed. The WiFi-task callback never
// touches this table; it only writes to the lock-free alert ring buffer.

typedef struct
{
  char mac[18];
  char method[16]; // "oui_addr2" / "oui_addr1" / "oui_addr3" / "ssid"
  int8_t rssi;
  uint8_t channel;
  uint32_t firstSeen; // millis() at first hit
  uint32_t lastSeen;  // millis() at latest hit
  uint16_t count;
  char ssid[33];  // "" unless an SSID hit populated it
  float lat;      // GPS latitude at first hit (0 if no fix)
  float lon;      // GPS longitude at first hit (0 if no fix)
  uint32_t utc;   // GPS UTC as unix epoch at first hit (0 if no fix)
  uint8_t hasFix; // 1 if lat/lon/utc are valid
} FYDetection;

static FYDetection fyDet[MAX_DETECTIONS];
static int fyDetCount = 0;
static bool fyDirty = false;
static unsigned long fyLastSaveAt = 0;
static int fyLastSaveCount = 0;

// ============================================================
// STATE
// ============================================================

static uint8_t currentChannel = 1;
static size_t customChannelIndex = 0;
static size_t fullHopIndex = 0;
static unsigned long lastHop = 0;
static unsigned long lastHeartbeat = 0;
static volatile bool sniffingStopped = false;

// Dedupe table (small circular, avoids single-slot eviction bug).
// This is the *serial-rate-limit* dedup — it suppresses beep + emit within
// ALERT_COOLDOWN_MS of a prior hit on the same MAC. The detection table
// (above) still counts every hit regardless of this suppression.
#define DEDUPE_SLOTS 8
static struct
{
  char mac[18];
  unsigned long ts;
} dedupeTable[DEDUPE_SLOTS];
static size_t dedupeIdx = 0;

// LED one-shot pulse timer
static volatile unsigned long ledOffAt = 0;

// Heartbeat audio state: last time any target was seen, last time the
// heartbeat beep-pair was played. When nothing has been seen for
// HB_DEVICE_ACTIVE_MS the heartbeat stops until the next new detection.
static unsigned long fyLastTargetSeen = 0;
static unsigned long fyLastHeartbeatAt = 0;

// GPS fix state — refreshed by the GPS reader in loop(). Stays "no fix" until
// the module gets a lock; detections are geotagged from these when hasFix.
static double gpsLat = 0.0;
static double gpsLon = 0.0;
static uint32_t gpsUtc = 0; // unix epoch, UTC (0 if unknown)
static double gpsHdop = 0.0;
static uint8_t gpsSats = 0;
static bool gpsHasFix = false;

// SD / session-file state.
static SPIClass sdSPI(VSPI);
static bool sdReady = false;
static int fySessionNum = 0;      // NNN of the file this power-on writes
static char fySessionPath[24];    // "/session_NNN.csv"

// Most-recent detection, for the TFT UI. These are the shared "last event"
// fields the UI observes; drainAlertQueue() writes them on every emitted hit.
static char fyLastMac[18] = "";
static char fyLastMethod[16] = "";
static AlertType fyLastType = ALERT_OUI_ADDR2;
static int8_t fyLastRssi = RSSI_MIN;
static unsigned long fyLastEventAt = 0;
static bool fyHaveLast = false;

// Front-facing detection flash: the whole screen flashes the class color with
// big text on each hit — the only onboard LED is on the BACK, dead-center, and
// is nearly invisible in daylight. Set in drainAlertQueue, rendered by tftTick.
#define FLASH_HOLD_MS 700
#ifndef TFT_BL
#define TFT_BL 27 // LCD backlight (also set via build_flags); LOW = off
#endif
static unsigned long uiFlashUntil = 0;
static uint16_t uiFlashColor = 0;      // class color (565) — full-screen fill
static uint16_t uiFlashText = 0xFFFF;  // contrasting text color
static bool uiFlashPainted = false;
static bool uiBlanked = false;         // 'b' test: backlight off to check RF desense

// Touch-calibration store (rotation-specific uint16_t cal[5] in NVS).
static Preferences uiPrefs;

// UI redraw request — set on boot, screen switch, or after recalibration.
// Declared here (ahead of serialCommandTick, which sets it on 'k'); the rest of
// the UI framework lives further down.
static bool screenDirty = true;
static void touchCalibrate(bool force);

// ============================================================
// 802.11 HEADER
// ============================================================

typedef struct __attribute__((packed))
{
  uint16_t frame_ctrl;
  uint16_t duration;
  uint8_t addr1[6];
  uint8_t addr2[6];
  uint8_t addr3[6];
  uint16_t seq_ctrl;
} wifi_ieee80211_mac_hdr_t;

// ============================================================
// SERIAL OUTPUT
// ============================================================
// On the Gold Edition these mirrored to Serial1 / BLE; here they are plain
// USB-serial wrappers so the rest of the code can keep the dualPrintf/dualPrintln
// call sites unchanged.

static char _dualBuf[384];

static void dualPrintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void dualPrintf(const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(_dualBuf, sizeof(_dualBuf), fmt, args);
  va_end(args);
  if (n > 0)
    Serial.write(_dualBuf, n);
}

static void dualPrintln(const char *str)
{
  Serial.println(str);
}

// ============================================================
// DISPLAY / GPS / AUDIO OBJECTS
// ============================================================

static TFT_eSPI tft = TFT_eSPI();
static TinyGPSPlus gps;
static int screenW = 320, screenH = 480;

// ============================================================
// RGB LED  (common anode on 22/16/17 — LOW lights a channel)
// ============================================================
//
// A digital, PWM-free port of the WS2812 layer: any nonzero channel value from
// alertTypeColor() simply lights that channel. red=R, amber=R+G, blue=B,
// cyan=G+B, magenta=R+B, white=all.

static inline void rgbShow(uint8_t r, uint8_t g, uint8_t b)
{
  digitalWrite(PIN_LED_R, r > 0 ? LED_ON : LED_OFF);
  digitalWrite(PIN_LED_G, g > 0 ? LED_ON : LED_OFF);
  digitalWrite(PIN_LED_B, b > 0 ? LED_ON : LED_OFF);
}
static inline void rgbOff() { rgbShow(0, 0, 0); }

// One-shot colored pulse; ledTick() clears it after the timer expires.
static void ledFlashColor(uint8_t r, uint8_t g, uint8_t b, unsigned ms)
{
  rgbShow(r, g, b);
  ledOffAt = millis() + ms;
  if (ledOffAt == 0)
    ledOffAt = 1; // avoid the "off" sentinel
}

static void ledTick()
{
  if (ledOffAt && (long)(millis() - ledOffAt) >= 0)
  {
    rgbOff();
    ledOffAt = 0;
  }
}

// Idle "still alive" indicator: a slow, brief dim-green blink shown whenever the
// LED isn't mid detection-flash, so a glance confirms the unit is powered and
// scanning. Digital (no PWM), so "dim" = a short on-pulse rather than a low duty.
static unsigned long fyLastBreatheAt = 0;
static bool fyBreatheOn = false;
static void breatheTick()
{
  if (ledOffAt)
    return; // a detection flash owns the LED
  unsigned long now = millis();
  if (!fyBreatheOn && now - fyLastBreatheAt >= 2000)
  {
    digitalWrite(PIN_LED_G, LED_ON);
    fyBreatheOn = true;
    fyLastBreatheAt = now;
  }
  else if (fyBreatheOn && now - fyLastBreatheAt >= 40)
  {
    digitalWrite(PIN_LED_G, LED_OFF);
    fyBreatheOn = false;
    fyLastBreatheAt = now;
  }
}

// ============================================================
// AUDIO  (DAC tone through the SC8002B amp)
// ============================================================
//
// Square-wave tone by toggling the DAC between two levels (see bringup.cpp).
// Blocking, like the Gold Edition's tone()/delay() beeps — only called from
// loop() context, never the sniffer callback.

static void beep(unsigned freqHz, unsigned ms)
{
  if (freqHz == 0)
  {
    delay(ms);
    return;
  }
  const unsigned halfUs = 500000UL / freqHz;
  const unsigned long end = millis() + ms;
  while (millis() < end)
  {
    dacWrite(PIN_AUDIO_DAC, 230);
    delayMicroseconds(halfUs);
    dacWrite(PIN_AUDIO_DAC, 25);
    delayMicroseconds(halfUs);
  }
  dacWrite(PIN_AUDIO_DAC, 0);
}

// Two fast ascending beeps — played on the FIRST sighting of a MAC.
static void newDetectChirp()
{
  beep(NEW_CHIRP_LO_HZ, NEW_CHIRP_NOTE_MS);
  delay(NEW_CHIRP_GAP_MS);
  beep(NEW_CHIRP_HI_HZ, NEW_CHIRP_NOTE_MS);
}

// Two monotone beeps — periodic heartbeat while at least one target is still
// in range (last seen within HB_DEVICE_ACTIVE_MS).
static void heartbeatBeep()
{
  beep(HB_BEEP_HZ, HB_BEEP_NOTE_MS);
  delay(HB_BEEP_GAP_MS);
  beep(HB_BEEP_HZ, HB_BEEP_NOTE_MS);
}

static void startupBeep()
{
  // First 6 notes of SMB World 1-2 (underground). Koji Kondo's descending
  // pattern: C5 → C4 → A4 → A3 → G#4 → G#3 (alternating-octave pairs).
  static const uint16_t notes[6] = {523, 262, 440, 220, 415, 208};
  for (int i = 0; i < 6; i++)
  {
    beep(notes[i], (i == 5) ? 160 : 95);
    if (i < 5)
      delay(22);
  }
}

// ============================================================
// HELPERS
// ============================================================

static void macToStr(const uint8_t *mac, char *buf, size_t len)
{
  snprintf(buf, len, "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
static void ouiFromMac(const uint8_t *mac, char *buf, size_t len)
{
  snprintf(buf, len, "%02x:%02x:%02x", mac[0], mac[1], mac[2]);
}

static void precompileOuis()
{
  for (size_t i = 0; i < OUI_COUNT; i++)
  {
    const char *o = target_ouis[i];
    oui_bytes[i][0] = (uint8_t)strtol(o, nullptr, 16);
    oui_bytes[i][1] = (uint8_t)strtol(o + 3, nullptr, 16);
    oui_bytes[i][2] = (uint8_t)strtol(o + 6, nullptr, 16);
  }
}

// Bit 0 of byte 0 set = multicast/broadcast — never a real device transmitter or receiver
// we care about. Guards addr1 checks against 01:xx, 33:33:xx, ff:ff:ff:ff:ff:ff etc.
static inline bool IRAM_ATTR isMulticast(const uint8_t *mac)
{
  return mac[0] & 0x01;
}

static bool IRAM_ATTR matchOuiRaw(const uint8_t *mac)
{
  // Locally-administered (randomised) MACs have bit 1 of byte 0 set.
  // Fixed infrastructure devices never use them — skip immediately.
  if (mac[0] & 0x02)
    return false;

  for (size_t i = 0; i < OUI_COUNT; i++)
  {
    if (mac[0] == oui_bytes[i][0] &&
        mac[1] == oui_bytes[i][1] &&
        mac[2] == oui_bytes[i][2])
      return true;
  }
  return false;
}

static char *strcasestr_local(const char *haystack, const char *needle)
{
  if (!*needle)
    return (char *)haystack;
  for (; *haystack; ++haystack)
  {
    const char *h = haystack;
    const char *n = needle;
    while (*h && *n && tolower((unsigned char)*h) == tolower((unsigned char)*n))
    {
      ++h;
      ++n;
    }
    if (!*n)
      return (char *)haystack;
  }
  return nullptr;
}
static bool matchSsidKeyword(const char *ssid)
{
  for (size_t i = 0; i < SSID_KEYWORD_COUNT; i++)
    if (strcasestr_local(ssid, target_ssid_keywords[i]))
      return true;
  return false;
}

static const char *channelModeName()
{
  switch (CHANNEL_MODE)
  {
  case CHANNEL_MODE_FULL_HOP:
    return "FULL_HOP";
  case CHANNEL_MODE_CUSTOM:
    return "CUSTOM";
  case CHANNEL_MODE_SINGLE:
    return "SINGLE";
  default:
    return "UNKNOWN";
  }
}

static inline uint16_t channelFreqMhz(uint8_t ch)
{
  return (ch >= 1 && ch <= 14) ? (uint16_t)(2407 + 5 * ch) : 0;
}

static bool shouldSuppressDuplicate(const char *macStr)
{
  unsigned long now = millis();
  for (size_t i = 0; i < DEDUPE_SLOTS; i++)
  {
    if (strcmp(dedupeTable[i].mac, macStr) == 0)
    {
      if ((now - dedupeTable[i].ts) < ALERT_COOLDOWN_MS)
        return true;
      dedupeTable[i].ts = now;
      return false;
    }
  }
  // Not found — insert into next slot
  strlcpy(dedupeTable[dedupeIdx].mac, macStr, 18);
  dedupeTable[dedupeIdx].ts = now;
  dedupeIdx = (dedupeIdx + 1) % DEDUPE_SLOTS;
  return false;
}

static void stopSniffing(const char *reason)
{
  if (sniffingStopped)
    return;
  sniffingStopped = true;
  esp_wifi_set_promiscuous(false);
  dualPrintf("[flockyou] sniffing stopped: %s\n", reason);
}

static void applyInitialChannel()
{
#if CHANNEL_MODE == CHANNEL_MODE_SINGLE
  currentChannel = SINGLE_CHANNEL;
#elif CHANNEL_MODE == CHANNEL_MODE_CUSTOM
  currentChannel = customChannels[0];
#else
  currentChannel = fullHopChannels[0];
#endif
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  lastHop = millis(); // start dwell timer precisely when channel is first set
}

static void updateChannelMode()
{
  if (sniffingStopped)
    return;
#if CHANNEL_MODE == CHANNEL_MODE_SINGLE
  if (currentChannel != SINGLE_CHANNEL)
  {
    currentChannel = SINGLE_CHANNEL;
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  }
  return;
#else
  if (millis() - lastHop < CHANNEL_DWELL_MS)
    return;
#if CHANNEL_MODE == CHANNEL_MODE_CUSTOM
  customChannelIndex = (customChannelIndex + 1) % customChannelCount;
  currentChannel = customChannels[customChannelIndex];
#else
  fullHopIndex = (fullHopIndex + 1) % fullHopChannelCount;
  currentChannel = fullHopChannels[fullHopIndex];
#endif
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  lastHop = millis();
#endif
}

static void printHeartbeat()
{
  if (millis() - lastHeartbeat >= HEARTBEAT_MS)
  {
    dualPrintf("[flockyou] scanning (ch=%u mode=%s det=%d gps=%s sats=%u)\n",
               currentChannel, channelModeName(), fyDetCount,
               gpsHasFix ? "fix" : "none", (unsigned)gpsSats);
    lastHeartbeat = millis();
  }
}

// ============================================================
// DETECTION TABLE OPS
// ============================================================

static const char *alertTypeToMethod(AlertType t)
{
  switch (t)
  {
  case ALERT_OUI_ADDR2:
    return "oui_addr2";
  case ALERT_OUI_ADDR1:
    return "oui_addr1";
  case ALERT_OUI_ADDR3:
    return "oui_addr3";
  case ALERT_SSID:
    return "ssid";
  case ALERT_WILDCARD_PROBE:
    return "wildcard_probe";
  default:
    return "unknown";
  }
}

// Distinct color per detection class, so the RGB LED alone tells you what kind
// of hit fired at a glance. Values are scaled to LED_BRIGHTNESS; on this board
// the LED is digital, so any nonzero channel lights (see rgbShow).
static void alertTypeColor(AlertType t, uint8_t &r, uint8_t &g, uint8_t &b)
{
  const uint8_t B = LED_BRIGHTNESS;
  switch (t)
  {
  case ALERT_WILDCARD_PROBE:
    r = B;
    g = 0;
    b = 0;
    break; // red     — high-precision probe signature
  case ALERT_OUI_ADDR2:
    r = B;
    g = B / 3;
    b = 0;
    break; // amber   — transmitter-side OUI
  case ALERT_OUI_ADDR1:
    r = 0;
    g = 0;
    b = B;
    break; // blue    — receiver-side sleeper catch
  case ALERT_OUI_ADDR3:
    r = 0;
    g = B;
    b = B;
    break; // cyan    — BSSID fallback
  case ALERT_SSID:
    r = B;
    g = 0;
    b = B;
    break; // magenta — SSID keyword
  default:
    r = B;
    g = B;
    b = B;
    break; // white
  }
}

// TFT color for a detection class — same channel logic as the LED, promoted to
// full-brightness 565 so the on-screen "last hit" text matches the LED color.
static uint16_t alertTftColor(AlertType t)
{
  uint8_t r, g, b;
  alertTypeColor(t, r, g, b);
  return tft.color565(r ? 255 : 0, g ? 255 : 0, b ? 255 : 0);
}

// Boot self-test: step through every detection color, in the same order the
// README/table lists them, so a glance confirms the LED works and re-learns the
// code before driving. Blocking by design — setup() only, before the sniffer
// starts. Colors come from alertTypeColor() so this can never drift from the
// live mapping.
static void ledBootColorCycle(unsigned ms)
{
  static const AlertType kOrder[] = {
      ALERT_WILDCARD_PROBE, // red
      ALERT_OUI_ADDR2,      // amber
      ALERT_OUI_ADDR1,      // blue
      ALERT_OUI_ADDR3,      // cyan
      ALERT_SSID,           // magenta
  };
  for (size_t i = 0; i < sizeof(kOrder) / sizeof(kOrder[0]); i++)
  {
    uint8_t r, g, b;
    alertTypeColor(kOrder[i], r, g, b);
    rgbShow(r, g, b);
    delay(ms);
  }
  rgbOff();
}

// Stamp a detection record with the current GPS fix at first sighting.
static void fyStampGps(FYDetection &d)
{
  if (gpsHasFix)
  {
    d.lat = (float)gpsLat;
    d.lon = (float)gpsLon;
    d.utc = gpsUtc;
    d.hasFix = 1;
  }
  else
  {
    d.lat = 0.0f;
    d.lon = 0.0f;
    d.utc = 0;
    d.hasFix = 0;
  }
}

// Returns index of entry (new or updated), or -1 if table is full.
// Returns index, and sets *outChirpWorthy = true when the caller should fire
// the ascending new-discovery chirp. Chirp-worthy means either (a) MAC is
// brand new to this session, or (b) MAC is known but hasn't been seen in
// REDISCOVER_MS — i.e. it left RF range and came back.
static int fyAddDetection(const char *mac, const char *method,
                          int8_t rssi, uint8_t ch, const char *ssid,
                          bool *outChirpWorthy)
{
  uint32_t now = millis();
  for (int i = 0; i < fyDetCount; i++)
  {
    if (strcasecmp(fyDet[i].mac, mac) == 0)
    {
      bool rediscover = (now - fyDet[i].lastSeen) > REDISCOVER_MS;
      if (fyDet[i].count < 0xFFFF)
        fyDet[i].count++;
      fyDet[i].lastSeen = now;
      fyDet[i].rssi = rssi;
      fyDet[i].channel = ch;
      if (ssid && ssid[0] && !fyDet[i].ssid[0])
      {
        strlcpy(fyDet[i].ssid, ssid, sizeof(fyDet[i].ssid));
      }
      fyDirty = true;
      if (outChirpWorthy)
        *outChirpWorthy = rediscover;
      return i;
    }
  }
  if (fyDetCount >= MAX_DETECTIONS)
  {
    if (outChirpWorthy)
      *outChirpWorthy = false;
    return -1;
  }
  FYDetection &d = fyDet[fyDetCount];
  strlcpy(d.mac, mac, sizeof(d.mac));
  strlcpy(d.method, method ? method : "", sizeof(d.method));
  d.rssi = rssi;
  d.channel = ch;
  d.firstSeen = now;
  d.lastSeen = now;
  d.count = 1;
  if (ssid && ssid[0])
    strlcpy(d.ssid, ssid, sizeof(d.ssid));
  else
    d.ssid[0] = '\0';
  fyStampGps(d); // geotag + timestamp the first sighting (no-op without a fix)
  fyDetCount++;
  fyDirty = true;
  if (outChirpWorthy)
    *outChirpWorthy = true;
  return fyDetCount - 1;
}

// ============================================================
// JSON ESCAPE  — only needed for SSIDs (user-controlled bytes)
// ============================================================

static size_t jsonEscape(char *dst, size_t cap, const char *src)
{
  size_t o = 0;
  if (cap == 0)
    return 0;
  for (size_t i = 0; src[i]; i++)
  {
    char c = src[i];
    if (c == '"' || c == '\\')
    {
      if (o + 2 >= cap)
        break;
      dst[o++] = '\\';
      dst[o++] = c;
    }
    else if ((unsigned char)c < 0x20)
    {
      if (o + 6 >= cap)
        break;
      int n = snprintf(dst + o, cap - o, "\\u%04x", (unsigned)(unsigned char)c);
      if (n <= 0 || (size_t)n >= cap - o)
        break;
      o += (size_t)n;
    }
    else
    {
      if (o + 1 >= cap)
        break;
      dst[o++] = c;
    }
  }
  dst[o] = '\0';
  return o;
}

// ============================================================
// SD SESSION PERSISTENCE  — CSV, latest file reloaded on boot
// ============================================================
//
// Each power-on writes a new /session_NNN.csv (zero-padded 3-digit). On boot we
// scan / for the highest existing NNN, reload that file into the live table so
// counts / firstSeen / GPS carry over, then open /session_(NNN+1).csv and seed
// it from the carried-over table. The columns match the 'd' CSV export exactly:
//   mac,method,rssi,channel,count,firstSeen_ms,lastSeen_ms,lat,lon,utc,hasFix,ssid
// The ssid field is double-quoted.

static const char *kSessionHeader =
    "mac,method,rssi,channel,count,firstSeen_ms,lastSeen_ms,lat,lon,utc,hasFix,ssid";

// Rewrite the current session file from the in-RAM table.
static void fySaveSessionCSV()
{
  if (!sdReady)
    return;
  if (!fyDirty && fyDetCount == fyLastSaveCount)
    return;

  File f = SD.open(fySessionPath, FILE_WRITE); // "w" — truncates and rewrites
  if (!f)
  {
    dualPrintf("[flockyou] save failed: cannot open %s\n", fySessionPath);
    return;
  }
  f.println(kSessionHeader);
  for (int i = 0; i < fyDetCount; i++)
  {
    FYDetection &d = fyDet[i];
    f.printf("%s,%s,%d,%u,%u,%lu,%lu,%.6f,%.6f,%lu,%u,\"%s\"\n",
             d.mac, d.method, d.rssi, (unsigned)d.channel, (unsigned)d.count,
             (unsigned long)d.firstSeen, (unsigned long)d.lastSeen,
             d.lat, d.lon, (unsigned long)d.utc, (unsigned)d.hasFix, d.ssid);
  }
  f.close();

  fyLastSaveAt = millis();
  fyLastSaveCount = fyDetCount;
  fyDirty = false;
}

// Parse one saved CSV row back into the live table.
static void fyParseSessionRow(char *line)
{
  if (fyDetCount >= MAX_DETECTIONS)
    return;

  char mac[18] = {0};
  char method[16] = {0};
  int rssi = 0, ch = 0, cnt = 0, hasFix = 0;
  unsigned long firstSeen = 0, lastSeen = 0, utc = 0;
  float lat = 0.0f, lon = 0.0f;
  int consumed = 0;

  int got = sscanf(line,
                   "%17[^,],%15[^,],%d,%d,%d,%lu,%lu,%f,%f,%lu,%d,%n",
                   mac, method, &rssi, &ch, &cnt, &firstSeen, &lastSeen,
                   &lat, &lon, &utc, &hasFix, &consumed);
  if (got < 11)
    return;

  FYDetection &d = fyDet[fyDetCount];
  memset(&d, 0, sizeof(d));
  strlcpy(d.mac, mac, sizeof(d.mac));
  strlcpy(d.method, method, sizeof(d.method));
  d.rssi = (int8_t)rssi;
  d.channel = (uint8_t)ch;
  d.count = (uint16_t)cnt;
  d.firstSeen = (uint32_t)firstSeen;
  d.lastSeen = (uint32_t)lastSeen;
  d.utc = (uint32_t)utc;
  d.lat = lat;
  d.lon = lon;
  d.hasFix = (uint8_t)hasFix;

  // ssid is the double-quoted remainder after the 11th comma.
  const char *s = line + consumed;
  if (*s == '"')
    s++;
  size_t sl = strlen(s);
  while (sl > 0 && (s[sl - 1] == '\r' || s[sl - 1] == '\n'))
    sl--;
  if (sl > 0 && s[sl - 1] == '"')
    sl--;
  size_t n = (sl < sizeof(d.ssid) - 1) ? sl : sizeof(d.ssid) - 1;
  memcpy(d.ssid, s, n);
  d.ssid[n] = '\0';

  fyDetCount++;
}

// Load a saved session CSV into the live table.
static void fyLoadSessionCSV(int num)
{
  char path[24];
  snprintf(path, sizeof(path), "/session_%03d.csv", num);
  File f = SD.open(path, FILE_READ);
  if (!f)
    return;

  bool header = true;
  char line[256];
  while (f.available())
  {
    size_t len = f.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';
    if (len == 0)
      continue;
    if (header)
    {
      header = false; // skip the column header row
      continue;
    }
    fyParseSessionRow(line);
    if (fyDetCount >= MAX_DETECTIONS)
      break;
  }
  f.close();
}

// Scan / for the highest existing session_NNN.csv. Returns NNN, or -1 if none.
static int fyScanHighestSession()
{
  int highest = -1;
  File root = SD.open("/");
  if (!root)
    return -1;
  for (File entry = root.openNextFile(); entry; entry = root.openNextFile())
  {
    const char *name = entry.name();
    const char *base = strrchr(name, '/');
    base = base ? base + 1 : name;
    int n = -1;
    if (sscanf(base, "session_%d.csv", &n) == 1 && n > highest)
      highest = n;
    entry.close();
  }
  root.close();
  return highest;
}

// Boot: reload the newest session (carry the table over), then open a fresh
// file for this power-on and seed it from the carried-over table.
static void fySessionBegin()
{
  if (!sdReady)
  {
    dualPrintln("[flockyou] SD not mounted — running without logging");
    return;
  }
  int highest = fyScanHighestSession();
  if (highest >= 0)
  {
    fyLoadSessionCSV(highest);
    dualPrintf("[flockyou] restored %d detections from /session_%03d.csv\n",
               fyDetCount, highest);
  }
  else
  {
    dualPrintln("[flockyou] no prior session — starting fresh");
  }
  fyLastSaveCount = fyDetCount;

  fySessionNum = highest + 1;
  snprintf(fySessionPath, sizeof(fySessionPath), "/session_%03d.csv", fySessionNum);
  fyDirty = true;         // force the initial seed write
  fySaveSessionCSV();     // create + seed the new file from the carried table
  dualPrintf("[flockyou] logging to %s\n", fySessionPath);
}

// ============================================================
// FLASK-COMPATIBLE JSON EMISSION
// ============================================================
//
// The Flask app (flock-you/api/flockyou.py) reads one JSON object per line
// from the USB CDC serial port. It filters by presence of `detection_method`
// and extracts these fields:  mac_address, rssi, channel, frequency, ssid,
// device_name, gps.latitude, gps.longitude, gps.accuracy.

static void emitDetectionJSON(const char *mac, const char *method,
                              int8_t rssi, uint8_t ch, const char *ssid)
{
  char ssidEsc[sizeof(((FYDetection *)0)->ssid) * 6 + 1];
  jsonEscape(ssidEsc, sizeof(ssidEsc), ssid ? ssid : "");
  char oui[9];
  uint8_t mbytes[6] = {0};
  sscanf(mac, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &mbytes[0], &mbytes[1], &mbytes[2], &mbytes[3], &mbytes[4], &mbytes[5]);
  ouiFromMac(mbytes, oui, sizeof(oui));

  // GPS block (only when we have a current fix). accuracy is a rough metres
  // estimate from HDOP. Slots in before "ssid" and ends with its own comma.
  char gpsField[112];
  if (gpsHasFix)
    snprintf(gpsField, sizeof(gpsField),
             "\"gps\":{\"latitude\":%.6f,\"longitude\":%.6f,\"accuracy\":%.1f},"
             "\"utc\":%lu,",
             gpsLat, gpsLon, (gpsHdop > 0.0) ? gpsHdop * 2.5 : 0.0,
             (unsigned long)gpsUtc);
  else
    gpsField[0] = '\0';

  dualPrintf(
      "{\"event\":\"detection\","
      "\"detection_method\":\"wifi_%s\","
      "\"protocol\":\"wifi_2_4ghz\","
      "\"mac_address\":\"%s\","
      "\"oui\":\"%s\","
      "\"device_name\":\"\","
      "\"rssi\":%d,"
      "\"channel\":%u,"
      "\"frequency\":%u,"
      "%s"
      "\"ssid\":\"%s\"}\n",
      method, mac, oui, rssi,
      (unsigned)ch, (unsigned)channelFreqMhz(ch), gpsField, ssidEsc);
}

// ============================================================
// PROMISCUOUS CALLBACK  — keep it fast, no Serial, no malloc
// ============================================================

static bool IRAM_ATTR extractSsidFromMgmtBody(const uint8_t *body, int len,
                                              char *outSsid, size_t outLen)
{
  if (!body || len <= 0 || !outSsid || outLen == 0)
    return false;
  while (len >= 2)
  {
    uint8_t id = body[0], elen = body[1];
    if ((int)elen + 2 > len)
      break;
    if (id == 0)
    {
      size_t n = (elen < (outLen - 1)) ? elen : (outLen - 1);
      memcpy(outSsid, body + 2, n);
      outSsid[n] = '\0';
      return true;
    }
    body += elen + 2;
    len -= elen + 2;
  }
  return false;
}

// Returns:
//   1  = wildcard SSID IE found (tag 0, length 0)  → Flock-style probe
//   0  = SSID IE found, non-zero length            → directed probe, not ours
//  -1  = no SSID IE found at all                   → caller should retry with
//                                                    FCS-stripped length, then bail
static int IRAM_ATTR isWildcardProbeIE(const uint8_t *body, int len)
{
  if (!body || len < 2)
    return -1;
  while (len >= 2)
  {
    uint8_t id = body[0];
    uint8_t elen = body[1];
    if ((int)elen + 2 > len)
      break;
    if (id == 0)
      return (elen == 0) ? 1 : 0;
    body += elen + 2;
    len -= elen + 2;
  }
  return -1;
}

static void IRAM_ATTR wifiSniffer(void *buf, wifi_promiscuous_pkt_type_t type)
{
  if (!buf || sniffingStopped)
    return;

#if PROCESS_MGMT_FRAMES && PROCESS_DATA_FRAMES
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA)
    return;
#elif PROCESS_MGMT_FRAMES
  if (type != WIFI_PKT_MGMT)
    return;
#elif PROCESS_DATA_FRAMES
  if (type != WIFI_PKT_DATA)
    return;
#else
  return; // nothing configured to process
#endif

  fyPktSeen++; // RX-activity counter (diagnostics)

  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  if (pkt->rx_ctrl.sig_len < sizeof(wifi_ieee80211_mac_hdr_t))
    return;
  wifi_ieee80211_mac_hdr_t *hdr = (wifi_ieee80211_mac_hdr_t *)pkt->payload;
  int8_t rssi = pkt->rx_ctrl.rssi;

  if (rssi < RSSI_MIN)
    return;

  uint8_t ch = (uint8_t)pkt->rx_ctrl.channel; // actual rx channel from driver

  // --- OUI check: addr2 (transmitter/source) ---
  //
  // For mgmt Probe Requests (type=0 subtype=4) from a matched OUI, tighten
  // to the DeFlockJoplin wildcard-probe signature: SSID IE (tag 0) length
  // must be zero. This reduces false positives dramatically (Michael's field
  // test: 11/12 true-positive with only 2 false-positives in Joplin).
  //
  // Non-probe frames from the same OUI still emit the broad ADDR2 alert.
  // See: https://github.com/DeflockJoplin/flock-you
  if (matchOuiRaw(hdr->addr2))
  {
    bool emitted = false;
    if (type == WIFI_PKT_MGMT)
    {
      uint8_t fc0 = hdr->frame_ctrl & 0xFF;
      uint8_t ftype = (fc0 >> 2) & 0x03;
      uint8_t subtype = (fc0 >> 4) & 0x0F;
      if (ftype == 0 && subtype == 4)
      { // Probe Request
        int sigLen = (int)pkt->rx_ctrl.sig_len;
        int bodyLen = sigLen - (int)sizeof(wifi_ieee80211_mac_hdr_t);
        const uint8_t *body = pkt->payload + sizeof(wifi_ieee80211_mac_hdr_t);
        int r = (bodyLen > 0) ? isWildcardProbeIE(body, bodyLen) : -1;
        // FCS-trailer retry: only when the first parse found no SSID IE AT
        // ALL (-1). A found-but-nonzero (0) means legit directed probe; do
        // not retry — it would mis-classify.
        if (r == -1 && bodyLen > 4)
          r = isWildcardProbeIE(body, bodyLen - 4);
        if (r == 1)
        {
          enqueueAlert(ALERT_WILDCARD_PROBE, hdr->addr2, rssi, ch,
                       nullptr, "probe_req");
          emitted = true;
        }
      }
    }
    if (!emitted)
    {
      enqueueAlert(ALERT_OUI_ADDR2, hdr->addr2, rssi, ch, nullptr, "addr2");
    }
  }

#if CHECK_ADDR1
  // addr1 (receiver/destination): catches Flock STAs that appear only as the
  // dst of probe responses and data frames — never transmitting in the capture
  // window due to their burst-sleep duty cycle. Multicast guard is mandatory
  // here since addr1 is broadcast (ff:ff:ff:ff:ff:ff) in beacons/broadcasts.
  if (!isMulticast(hdr->addr1) && matchOuiRaw(hdr->addr1))
  {
    enqueueAlert(ALERT_OUI_ADDR1, hdr->addr1, rssi, ch, nullptr, "addr1");
  }
#endif

#if CHECK_ADDR3
  // addr3 fallback: catches cases where addr2 is randomised but addr3
  // carries the real BSSID OUI (management frames only).
  if (type == WIFI_PKT_MGMT && matchOuiRaw(hdr->addr3))
  {
    enqueueAlert(ALERT_OUI_ADDR3, hdr->addr3, rssi, ch, nullptr, "addr3");
  }
#endif

#if ENABLE_SSID_MATCH
  if (type == WIFI_PKT_MGMT)
  {
    uint8_t fc0 = hdr->frame_ctrl & 0xFF;
    uint8_t subtype = (fc0 >> 4) & 0x0F;
    uint8_t ftype = (fc0 >> 2) & 0x03;

    if (ftype == 0)
    {
      int sigLen = pkt->rx_ctrl.sig_len - 4; // strip 4-byte FCS
      if (sigLen < (int)sizeof(wifi_ieee80211_mac_hdr_t))
        return;

      const uint8_t *mgmtBody = nullptr;
      int mgmtBodyLen = 0;
      const char *frameKind = nullptr;

      if (subtype == 8 || subtype == 5)
      {
        // Beacon / Probe Response: fixed params = 12 bytes after MAC hdr
        int off = sizeof(wifi_ieee80211_mac_hdr_t) + 12;
        if (sigLen > off)
        {
          frameKind = (subtype == 8) ? "beacon" : "probe_resp";
          mgmtBody = pkt->payload + off;
          mgmtBodyLen = sigLen - off;
        }
      }
      else if (subtype == 4)
      {
        // Probe Request: IEs follow directly after MAC hdr
        int off = sizeof(wifi_ieee80211_mac_hdr_t);
        if (sigLen > off)
        {
          frameKind = "probe_req";
          mgmtBody = pkt->payload + off;
          mgmtBodyLen = sigLen - off;
        }
      }

      if (mgmtBody && mgmtBodyLen > 0)
      {
        char ssid[33] = {0};
        if (extractSsidFromMgmtBody(mgmtBody, mgmtBodyLen, ssid, sizeof(ssid)))
        {
          if (matchSsidKeyword(ssid))
          {
            enqueueAlert(ALERT_SSID, hdr->addr2, rssi, ch, ssid, frameKind);
          }
        }
      }
    }
  }
#endif
}

// ============================================================
// DRAIN QUEUE — called from loop(), safe to Serial.print here
// ============================================================

static void drainAlertQueue()
{
  while (true)
  {
    portENTER_CRITICAL(&queueMux);
    if (alertTail == alertHead)
    {
      portEXIT_CRITICAL(&queueMux);
      break;
    }
    AlertEntry e;
    memcpy(&e, (const void *)&alertQueue[alertTail], sizeof(AlertEntry));
    alertTail = (alertTail + 1) % ALERT_QUEUE_SIZE;
    portEXIT_CRITICAL(&queueMux);

    char macStr[18];
    macToStr(e.mac, macStr, sizeof(macStr));
    const char *method = alertTypeToMethod(e.type);

    // Always update the on-device detection table (survives reboot via SD CSV).
    // chirpWorthy = true for brand-new MACs AND for MACs rediscovered after
    // REDISCOVER_MS of silence (drove away and came back).
    bool chirpWorthy = false;
    int idx = fyAddDetection(macStr, method, e.rssi, e.channel,
                             (e.type == ALERT_SSID) ? e.ssid : nullptr,
                             &chirpWorthy);

    // Refresh the global "still around" timer for the heartbeat tick.
    // Done unconditionally so a device counts as active even when serial is
    // rate-limited (still audible via heartbeat, just quieter on the wire).
    fyLastTargetSeen = millis();

    // Serial-rate-limit: suppress emit/beep/flash within ALERT_COOLDOWN_MS.
    if (shouldSuppressDuplicate(macStr))
      continue;

    // Track the latest detection for the TFT UI (shared "last event").
    strlcpy(fyLastMac, macStr, sizeof(fyLastMac));
    strlcpy(fyLastMethod, method, sizeof(fyLastMethod));
    fyLastType = e.type;
    fyLastRssi = e.rssi;
    fyLastEventAt = millis();
    fyHaveLast = true;

    // Human-readable line (for serial terminal).
    char oui[9];
    ouiFromMac(e.mac, oui, sizeof(oui));
    char gtag[52];
    if (gpsHasFix)
      snprintf(gtag, sizeof(gtag), " gps=%.6f,%.6f", gpsLat, gpsLon);
    else
      gtag[0] = '\0';
    if (e.type == ALERT_SSID)
    {
      dualPrintf("[flockyou] DETECT-SSID type=%s mac=%s ssid=\"%s\" rssi=%d ch=%u count=%d%s\n",
                 e.frameKind, macStr, e.ssid, e.rssi, e.channel,
                 (idx >= 0) ? (int)fyDet[idx].count : 0, gtag);
    }
    else
    {
      dualPrintf("[flockyou] DETECT-OUI mac=%s oui=%s rssi=%d ch=%u addr=%s count=%d%s\n",
                 macStr, oui, e.rssi, e.channel,
                 e.frameKind[0] ? e.frameKind : "addr2",
                 (idx >= 0) ? (int)fyDet[idx].count : 0, gtag);
    }

    // Flask-compatible JSON line (parsed by api/flockyou.py over USB CDC).
    emitDetectionJSON(macStr, method, e.rssi, e.channel,
                      (e.type == ALERT_SSID) ? e.ssid : "");

    // Audio feedback:
    //   - NEW MAC  → two fast ascending beeps (clearly distinct sound)
    //   - REPEAT   → silent; the heartbeat tick covers continued presence
    // LED flashes on every emitted detection either way.
    if (chirpWorthy)
    {
      newDetectChirp();
      // Reset the heartbeat phase so the first follow-up beep lands
      // HB_BEEP_INTERVAL_MS after the initial chirp, not mid-window.
      fyLastHeartbeatAt = millis();
    }
    // Flash the RGB LED in the color for this detection class.
    uint8_t lr, lg, lb;
    alertTypeColor(e.type, lr, lg, lb);
    ledFlashColor(lr, lg, lb, LED_FLASH_MS);

    // Front-facing screen flash in the same class color (the back LED is
    // near-invisible in daylight). Rendered by tftTick; text auto-contrasts.
    uiFlashColor = tft.color565(lr, lg, lb);
    int luma = (lr * 30 + lg * 59 + lb * 11) / 100;
    uiFlashText = (luma > 110) ? TFT_BLACK : TFT_WHITE;
    uiFlashUntil = millis() + FLASH_HOLD_MS;
    uiFlashPainted = false;

#if STOP_ON_OUI_HIT
    if (e.type != ALERT_SSID)
      stopSniffing("OUI hit");
#endif
#if STOP_ON_SSID_HIT
    if (e.type == ALERT_SSID)
      stopSniffing("SSID hit");
#endif
  }
}

// ============================================================
// AUTOSAVE
// ============================================================

static void autosaveTick()
{
  if (!sdReady || !fyDirty)
    return;
  if (millis() - fyLastSaveAt < AUTOSAVE_INTERVAL_MS)
    return;
  fySaveSessionCSV();
}

// Heartbeat beep while at least one target was seen in the last
// HB_DEVICE_ACTIVE_MS. Fires HB_BEEP_INTERVAL_MS apart.
static void heartbeatTick()
{
  if (fyLastTargetSeen == 0)
    return; // never seen one
  unsigned long now = millis();
  if (now - fyLastTargetSeen > HB_DEVICE_ACTIVE_MS)
    return; // gone silent
  if (now - fyLastHeartbeatAt < HB_BEEP_INTERVAL_MS)
    return; // too soon
  heartbeatBeep();
  fyLastHeartbeatAt = now;
}

// ============================================================
// LOG EXPORT  — dump the stored detection table over USB serial
// ============================================================
//
// Type a key in the serial monitor to export the accumulated table:
//   d = CSV (spreadsheet-friendly),  j = JSON (one object per line)

static void dumpDetectionsCSV()
{
  dualPrintln("mac,method,rssi,channel,count,firstSeen_ms,lastSeen_ms,lat,lon,utc,hasFix,ssid");
  for (int i = 0; i < fyDetCount; i++)
  {
    FYDetection &d = fyDet[i];
    dualPrintf("%s,%s,%d,%u,%u,%lu,%lu,%.6f,%.6f,%lu,%u,\"%s\"\n",
               d.mac, d.method, d.rssi, (unsigned)d.channel, (unsigned)d.count,
               (unsigned long)d.firstSeen, (unsigned long)d.lastSeen,
               d.lat, d.lon, (unsigned long)d.utc, (unsigned)d.hasFix, d.ssid);
  }
  dualPrintf("[flockyou] dumped %d detections (CSV)\n", fyDetCount);
}

static void dumpDetectionsJSON()
{
  for (int i = 0; i < fyDetCount; i++)
  {
    FYDetection &d = fyDet[i];
    char ssidEsc[sizeof(d.ssid) * 6 + 1];
    jsonEscape(ssidEsc, sizeof(ssidEsc), d.ssid);
    char gpsField[96];
    if (d.hasFix)
      snprintf(gpsField, sizeof(gpsField),
               "\"gps\":{\"latitude\":%.6f,\"longitude\":%.6f},\"utc\":%lu,",
               d.lat, d.lon, (unsigned long)d.utc);
    else
      gpsField[0] = '\0';
    dualPrintf("{\"record\":%d,\"mac\":\"%s\",\"method\":\"%s\",\"rssi\":%d,"
               "\"channel\":%u,\"count\":%u,%s\"ssid\":\"%s\"}\n",
               i, d.mac, d.method, d.rssi, (unsigned)d.channel,
               (unsigned)d.count, gpsField, ssidEsc);
  }
  dualPrintf("[flockyou] dumped %d detections (JSON)\n", fyDetCount);
}

// Poll USB serial for a one-key export command.
static void serialCommandTick()
{
  while (Serial.available())
  {
    int c = Serial.read();
    if (c == 'd' || c == 'D')
      dumpDetectionsCSV();
    else if (c == 'j' || c == 'J')
      dumpDetectionsJSON();
    else if (c == 'k' || c == 'K')
    {
      touchCalibrate(true); // force re-run the corner-arrow calibration
      screenDirty = true;   // full redraw once calibration finishes
    }
    else if (c == 'b' || c == 'B')
    {
      // RF-desense test: blank the display + backlight. If detection improves
      // with the screen off, the LCD is desensitizing the 2.4 GHz receiver.
      uiBlanked = !uiBlanked;
      pinMode(TFT_BL, OUTPUT);
      digitalWrite(TFT_BL, uiBlanked ? LOW : HIGH);
      if (!uiBlanked)
        screenDirty = true;
      dualPrintf("[flockyou] display %s\n", uiBlanked ? "BLANKED (RF test)" : "on");
    }
  }
}

// ============================================================
// GPS READER  (NMEA over Serial2, parsed by TinyGPS++)
// ============================================================

// Days since the Unix epoch for a civil (y,m,d) date — Howard Hinnant's algo.
static long daysFromCivil(long y, unsigned m, unsigned d)
{
  y -= m <= 2;
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (long)doe - 719468;
}

static void gpsBegin()
{
  Serial2.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
}

// Feed NMEA bytes to the parser and refresh the shared fix state.
static void gpsTick()
{
  while (Serial2.available())
    gps.encode((char)Serial2.read());

  if (gps.location.isValid() && gps.location.age() < GPS_STALE_MS)
  {
    gpsLat = gps.location.lat();
    gpsLon = gps.location.lng();
    gpsHdop = gps.hdop.isValid() ? (gps.hdop.value() / 100.0) : 0.0; // value() = HDOP×100
    gpsSats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
    if (gps.date.isValid() && gps.time.isValid() && gps.date.age() < GPS_STALE_MS)
    {
      long days = daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day());
      gpsUtc = (uint32_t)days * 86400UL + (uint32_t)gps.time.hour() * 3600UL +
               (uint32_t)gps.time.minute() * 60UL + (uint32_t)gps.time.second();
    }
    gpsHasFix = true;
  }
  else
  {
    gpsHasFix = false; // no current fix — detections recorded without geodata
  }
}

// ============================================================
// BATTERY
// ============================================================

static float readBatteryVolts()
{
  // analogReadMilliVolts applies the eFuse ADC calibration; x2 for the divider.
  return (analogReadMilliVolts(PIN_BAT_ADC) * 2.0f) / 1000.0f;
}

// ============================================================
// TOUCHSCREEN UI  (Phase 2a — landscape 480x320 framework)
// ============================================================
// A persistent top status strip, a switchable screen body, and a bottom tab
// bar. The flagship SCR_HUNTER screen renders a sonar radar (in a small sprite,
// never a full-screen buffer), an RSSI gauge, and the strongest-target readout.
// Everything is driven from tftTick() in loop() — non-blocking, except the
// one-time touch calibration at boot.

enum UiScreen
{
  SCR_HUNTER = 0,
  SCR_ALERT,
  SCR_LIVE,
  SCR_STATS
};

static UiScreen currentScreen = SCR_HUNTER;
// screenDirty is declared up in the STATE section (serialCommandTick sets it).

static const char *kTabLabels[4] = {"HUNT", "ALERT", "LIVE", "STATS"};

// Radar sonar sprite — a single small region, allocated once at boot.
static TFT_eSprite radarSpr = TFT_eSprite(&tft);
static bool radarSprOk = false;
static int sweepAngle = 0;

// UI cadence timers + touch-edge debounce.
static unsigned long uiHunterAt = 0;
static unsigned long uiSlowAt = 0;
static unsigned long uiStripAt = 0;
static bool touchDown = false;

// Hunter dynamic caches (dirty tracking to avoid repainting steady text).
static char huntCacheMac[18] = "";
static bool huntCacheHadTarget = false;
static bool huntForce = false; // force the target block to paint on screen entry

// ---- small color / math helpers ----------------------------------------

// Scale a 565 color's brightness by lvl (0..255) — used for blip/sweep fades.
static uint16_t dim565(uint16_t c, uint8_t lvl)
{
  uint8_t r = (c >> 11) & 0x1F;
  uint8_t g = (c >> 5) & 0x3F;
  uint8_t b = c & 0x1F;
  r = (uint8_t)((r * lvl) / 255);
  g = (uint8_t)((g * lvl) / 255);
  b = (uint8_t)((b * lvl) / 255);
  return (uint16_t)((r << 11) | (g << 5) | b);
}

// Green (far / weak) -> yellow -> red (near / strong) for a 0..100 percent.
static uint16_t gaugeColor565(int pct)
{
  if (pct < 0)
    pct = 0;
  if (pct > 100)
    pct = 100;
  uint8_t r, g;
  if (pct < 50)
  {
    r = (uint8_t)(255 * pct / 50);
    g = 255;
  }
  else
  {
    r = 255;
    g = (uint8_t)(255 * (100 - pct) / 50);
  }
  return tft.color565(r, g, 0);
}

static int rssiToPct(int rssi)
{
  if (rssi <= RSSI_GAUGE_MIN)
    return 0;
  if (rssi >= RSSI_GAUGE_MAX)
    return 100;
  return (int)(((long)(rssi - RSSI_GAUGE_MIN) * 100) / (RSSI_GAUGE_MAX - RSSI_GAUGE_MIN));
}

// djb2 over the MAC string — stable pseudo-angle so a device always plots at
// the same bearing on the radar.
static uint32_t macHash(const char *s)
{
  uint32_t h = 5381;
  for (; *s; ++s)
    h = ((h << 5) + h) + (uint8_t)*s;
  return h;
}

// Reverse of alertTypeToMethod() — the detection table stores the method
// string, so map it back to a class for coloring/labeling.
static AlertType methodToAlertType(const char *m)
{
  if (strcmp(m, "oui_addr1") == 0)
    return ALERT_OUI_ADDR1;
  if (strcmp(m, "oui_addr3") == 0)
    return ALERT_OUI_ADDR3;
  if (strcmp(m, "ssid") == 0)
    return ALERT_SSID;
  if (strcmp(m, "wildcard_probe") == 0)
    return ALERT_WILDCARD_PROBE;
  return ALERT_OUI_ADDR2;
}

// Human label per class for the target readout (cop-show flavor).
static const char *alertTypeLabel(AlertType t)
{
  switch (t)
  {
  case ALERT_WILDCARD_PROBE:
    return "WILDCARD PROBE";
  case ALERT_OUI_ADDR2:
    return "OUI TX";
  case ALERT_OUI_ADDR1:
    return "OUI RX (sleeper)";
  case ALERT_OUI_ADDR3:
    return "OUI BSSID";
  case ALERT_SSID:
    return "SSID MATCH";
  default:
    return "UNKNOWN";
  }
}

// Strongest current target: highest-RSSI unique device seen within the rolling
// PROX_WINDOW_MS. Returns a fyDet[] index, or -1 if nothing is in the window.
// Observes the same table the detector fills — no detection logic is touched.
static int fyStrongestTarget()
{
  unsigned long now = millis();
  int best = -1;
  int bestRssi = -128;
  for (int i = 0; i < fyDetCount; i++)
  {
    if (now - fyDet[i].lastSeen > PROX_WINDOW_MS)
      continue;
    if (fyDet[i].rssi > bestRssi)
    {
      bestRssi = fyDet[i].rssi;
      best = i;
    }
  }
  return best;
}

// ---- touch calibration (rotation-specific, NVS-backed) -----------------

static void touchCalibrate(bool force)
{
  uint16_t cal[5];
  uiPrefs.begin("flockui", false);
  bool have = !force && uiPrefs.getBytesLength("cal") == sizeof(cal);
  if (have)
  {
    uiPrefs.getBytes("cal", cal, sizeof(cal));
    tft.setTouch(cal);
    dualPrintf("[flockyou] touch cal loaded: %u %u %u %u %u\n",
               cal[0], cal[1], cal[2], cal[3], cal[4]);
  }
  else
  {
    tft.fillScreen(TFT_BLACK);
    tft.setTextFont(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 10);
    tft.println("Touch the corner arrows to calibrate");
    tft.calibrateTouch(cal, TFT_MAGENTA, TFT_BLACK, 15); // blocking, boot-only
    tft.setTouch(cal);
    uiPrefs.putBytes("cal", cal, sizeof(cal));
    dualPrintf("[flockyou] touch cal saved: %u %u %u %u %u\n",
               cal[0], cal[1], cal[2], cal[3], cal[4]);
  }
  uiPrefs.end();
}

// ---- display + sprite init --------------------------------------------

static void tftInit()
{
  tft.init();
  tft.setRotation(SCREEN_ROTATION); // 1 => 480x320 landscape
  screenW = tft.width();
  screenH = tft.height();
  tft.fillScreen(TFT_BLACK);

  // Allocate the radar sprite once, up front while heap is least fragmented
  // (before WiFi is started). 16bpp so pushSprite is a straight blit.
  radarSpr.setColorDepth(16);
  radarSprOk = (radarSpr.createSprite(RADAR_D, RADAR_D) != nullptr);
  if (!radarSprOk)
    dualPrintln("[flockyou] radar sprite alloc failed — radar disabled");
}

// ---- shared chrome: status strip + tab bar -----------------------------

static void drawStatusStrip()
{
  tft.fillRect(0, 0, UI_W, STRIP_H, TFT_BLACK);
  tft.drawFastHLine(0, STRIP_H, UI_W, TFT_DARKGREY);
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  const int y = 3;

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(6, y);
  tft.print("FLOCK");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(64, y);
  tft.printf("ch%u", currentChannel);

  tft.setTextColor(gpsHasFix ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  tft.setCursor(116, y);
  tft.printf("GPS:%s/%u", gpsHasFix ? "fix" : "no", (unsigned)gpsSats);

  tft.setTextColor(sdReady ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.setCursor(230, y);
  tft.printf("SD:%s", sdReady ? "ok" : "--");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(300, y);
  tft.printf("%.2fV", readBatteryVolts());

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(400, y);
  tft.printf("U:%d", fyDetCount);
}

static void drawTabBar()
{
  tft.setTextFont(4);
  tft.setTextDatum(MC_DATUM);
  for (int i = 0; i < 4; i++)
  {
    int x = i * TAB_W;
    bool active = (i == (int)currentScreen);
    uint16_t bg = active ? TFT_DARKCYAN : TFT_BLACK;
    uint16_t fg = active ? TFT_BLACK : TFT_CYAN;
    tft.fillRect(x, TABBAR_Y, TAB_W, TABBAR_H, bg);
    tft.drawRect(x, TABBAR_Y, TAB_W, TABBAR_H, TFT_DARKGREY);
    tft.setTextColor(fg, bg);
    tft.drawString(kTabLabels[i], x + TAB_W / 2, TABBAR_Y + TABBAR_H / 2);
  }
  tft.setTextDatum(TL_DATUM);
}

// ---- SCR_HUNTER --------------------------------------------------------

static void drawRadar()
{
  const int cx = RADAR_R, cy = RADAR_R;
  unsigned long now = millis();

  if (!radarSprOk)
  {
    sweepAngle += SWEEP_STEP_DEG;
    if (sweepAngle >= 360)
      sweepAngle -= 360;
    return;
  }

  radarSpr.fillSprite(TFT_BLACK);

  // faint range rings + crosshair
  for (int k = 1; k <= 3; k++)
    radarSpr.drawCircle(cx, cy, RADAR_R * k / 3, dim565(TFT_GREEN, 70));
  radarSpr.drawCircle(cx, cy, RADAR_R - 1, dim565(TFT_GREEN, 140));
  radarSpr.drawFastHLine(0, cy, RADAR_D, dim565(TFT_GREEN, 40));
  radarSpr.drawFastVLine(cx, 0, RADAR_D, dim565(TFT_GREEN, 40));

  // sweep line + comet trail
  for (int t = 0; t < 8; t++)
  {
    float a = (sweepAngle - t * 5) * DEG2RAD;
    uint8_t lvl = (uint8_t)(220 - t * 26);
    int ex = cx + (int)(cosf(a) * (RADAR_R - 2));
    int ey = cy + (int)(sinf(a) * (RADAR_R - 2));
    radarSpr.drawLine(cx, cy, ex, ey, dim565(TFT_GREEN, lvl));
  }

  // blips straight from the live detection table (recent, fading by age).
  // angle = hash(MAC), radius = closer-to-center for stronger RSSI.
  for (int i = 0; i < fyDetCount; i++)
  {
    unsigned long age = now - fyDet[i].lastSeen;
    if (age > BLIP_FADE_MS)
      continue;
    int pct = rssiToPct(fyDet[i].rssi);
    float a = (macHash(fyDet[i].mac) % 360) * DEG2RAD;
    float rr = (RADAR_R - 6) * (1.0f - pct / 100.0f * 0.85f);
    int bx = cx + (int)(cosf(a) * rr);
    int by = cy + (int)(sinf(a) * rr);
    uint8_t lvl = (uint8_t)(255 - (age * 255 / BLIP_FADE_MS));
    uint16_t col = dim565(gaugeColor565(pct), lvl);
    radarSpr.fillCircle(bx, by, 3, col);
    if (age < 1200)
      radarSpr.drawCircle(bx, by, 5, dim565(col, 180)); // fresh-ping halo
  }

  radarSpr.pushSprite(RADAR_X, RADAR_Y);

  sweepAngle += SWEEP_STEP_DEG;
  if (sweepAngle >= 360)
    sweepAngle -= 360;
}

static void drawHunterStatic()
{
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(2);

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(RADAR_X + RADAR_R - 20, RADAR_Y + RADAR_D + 3);
  tft.print("SONAR");

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(190, 30);
  tft.print("SIGNAL");
  tft.drawRect(190, 52, 280, 30, TFT_DARKGREY); // gauge frame

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(190, 150);
  tft.print("TARGET");
}

static void updateHunter()
{
  int ti = fyStrongestTarget();
  bool haveTarget = (ti >= 0);
  int rssi = haveTarget ? fyDet[ti].rssi : RSSI_GAUGE_MIN;
  int pct = rssiToPct(rssi);

  // sonar radar (animated, in its sprite)
  drawRadar();

  // RSSI gauge bar
  const int gx = 192, gy = 54, gw = 276, gh = 26;
  int fillw = gw * pct / 100;
  uint16_t gc = gaugeColor565(pct);
  tft.fillRect(gx, gy, fillw, gh, haveTarget ? gc : TFT_BLACK);
  tft.fillRect(gx + fillw, gy, gw - fillw, gh, TFT_BLACK);

  // big dBm readout
  tft.fillRect(190, 92, 200, 50, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setCursor(190, 92);
  if (haveTarget)
  {
    tft.setTextFont(6);
    tft.setTextColor(gc, TFT_BLACK);
    tft.printf("%d", rssi);
    tft.setTextFont(2);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(300, 122);
    tft.print("dBm");
  }
  else
  {
    tft.setTextFont(6);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print("--");
  }

  // target readout — MAC + class label repainted only when it changes.
  const char *tmac = haveTarget ? fyDet[ti].mac : "";
  bool changed = huntForce || (haveTarget != huntCacheHadTarget) ||
                 (haveTarget && strcmp(tmac, huntCacheMac) != 0);
  huntForce = false;
  if (changed)
  {
    tft.fillRect(190, 172, 285, 50, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    if (haveTarget)
    {
      AlertType t = methodToAlertType(fyDet[ti].method);
      tft.setTextFont(4);
      tft.setTextColor(alertTftColor(t), TFT_BLACK);
      tft.setCursor(190, 172);
      tft.print(fyDet[ti].mac);
      tft.setTextFont(2);
      tft.setTextColor(alertTftColor(t), TFT_BLACK);
      tft.setCursor(190, 200);
      tft.print(alertTypeLabel(t));
      strlcpy(huntCacheMac, tmac, sizeof(huntCacheMac));
    }
    else
    {
      tft.setTextFont(4);
      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      tft.setCursor(190, 172);
      tft.print("NO TARGET");
      huntCacheMac[0] = '\0';
    }
    huntCacheHadTarget = haveTarget;
  }

  // keep the idle "scanning ch N" line fresh as channels hop
  if (!haveTarget)
  {
    tft.fillRect(190, 200, 285, 18, TFT_BLACK);
    tft.setTextFont(2);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(190, 200);
    tft.printf("scanning ch %u ...", currentChannel);
  }
}

// ---- SCR_ALERT / SCR_LIVE / SCR_STATS (2b/2c placeholders) -------------

static void drawAlertStatic()
{
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(4);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setCursor(10, BODY_TOP + 10);
  tft.print("ALERTS");
  tft.setTextFont(2);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(10, BODY_TOP + 44);
  tft.print("(2b) live alert feed lands here");
}

static void updateAlert()
{
  tft.fillRect(0, BODY_TOP + 70, UI_W, 40, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(2);
  tft.setCursor(10, BODY_TOP + 74);
  if (fyHaveLast)
  {
    tft.setTextColor(alertTftColor(fyLastType), TFT_BLACK);
    tft.printf("Last: %s  %s  %ddBm", fyLastMac, alertTypeLabel(fyLastType),
               (int)fyLastRssi);
  }
  else
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("no active alert");
  }
}

static void drawLiveStatic()
{
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(4);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, BODY_TOP + 10);
  tft.print("LIVE");
  tft.setTextFont(2);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(10, BODY_TOP + 44);
  tft.print("(2c) scrolling detection list lands here");
}

static void updateLive()
{
  tft.fillRect(0, BODY_TOP + 70, UI_W, 40, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, BODY_TOP + 74);
  tft.printf("%d detections", fyDetCount);
}

static void drawStatsStatic()
{
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, BODY_TOP + 10);
  tft.print("STATS");
}

static void updateStats()
{
  tft.fillRect(0, BODY_TOP + 70, UI_W, 80, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, BODY_TOP + 74);
  tft.printf("Unique: %d", fyDetCount);
  unsigned long s = millis() / 1000;
  tft.setCursor(10, BODY_TOP + 104);
  tft.printf("Uptime: %02lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
}

// ---- dispatch: full redraw, dynamic update, touch, tick ----------------

static void drawScreenFull()
{
  tft.fillRect(0, BODY_TOP, UI_W, BODY_BOT - BODY_TOP, TFT_BLACK);
  drawStatusStrip();
  switch (currentScreen)
  {
  case SCR_HUNTER:
    huntCacheMac[0] = '\0';
    huntCacheHadTarget = false;
    huntForce = true; // guarantee the target block paints on entry
    drawHunterStatic();
    break;
  case SCR_ALERT:
    drawAlertStatic();
    break;
  case SCR_LIVE:
    drawLiveStatic();
    break;
  case SCR_STATS:
    drawStatsStatic();
    break;
  }
  drawTabBar();
}

static void updateCurrentScreen()
{
  switch (currentScreen)
  {
  case SCR_HUNTER:
    updateHunter();
    break;
  case SCR_ALERT:
    updateAlert();
    break;
  case SCR_LIVE:
    updateLive();
    break;
  case SCR_STATS:
    updateStats();
    break;
  }
}

static void handleTouch()
{
  uint16_t tx, ty;
  bool pressed = tft.getTouch(&tx, &ty);
  if (pressed && !touchDown)
  {
    touchDown = true; // act on the press edge only
    if (ty >= TABBAR_Y)
    {
      int idx = tx / TAB_W;
      if (idx < 0)
        idx = 0;
      if (idx > 3)
        idx = 3;
      if ((int)currentScreen != idx)
      {
        currentScreen = (UiScreen)idx;
        screenDirty = true;
      }
    }
  }
  else if (!pressed)
  {
    touchDown = false;
  }
}

// Full-screen detection flash — front-facing alert in the class color.
static void drawDetectionFlash()
{
  tft.fillScreen(uiFlashColor);
  tft.setTextColor(uiFlashText, uiFlashColor);
  tft.setTextDatum(MC_DATUM);
  tft.setTextFont(4);
  tft.drawString("FLOCK DETECTED", UI_W / 2, 55);
  tft.drawString(fyLastMac, UI_W / 2, 130);
  char sub[48];
  snprintf(sub, sizeof(sub), "%s   %d dBm", fyLastMethod, (int)fyLastRssi);
  tft.drawString(sub, UI_W / 2, 200);
  tft.setTextDatum(TL_DATUM); // restore default
}

static void tftTick()
{
  unsigned long now = millis();

  // Display blanked for the RF-desense test ('b') — draw nothing.
  if (uiBlanked)
    return;

  // Detection flash owns the whole screen for FLASH_HOLD_MS. Painted once, then
  // we restore the normal screen when it expires.
  if (now < uiFlashUntil)
  {
    if (!uiFlashPainted)
    {
      drawDetectionFlash();
      uiFlashPainted = true;
    }
    return;
  }
  if (uiFlashPainted)
  {
    uiFlashPainted = false;
    screenDirty = true; // restore the normal UI after the flash
  }

  handleTouch();

  if (screenDirty)
  {
    drawScreenFull();
    updateCurrentScreen(); // paint one dynamic frame immediately
    screenDirty = false;
    uiHunterAt = uiSlowAt = uiStripAt = now;
    return;
  }

  // shared status strip ~1 Hz on every screen
  if (now - uiStripAt >= SLOW_UPDATE_MS)
  {
    drawStatusStrip();
    uiStripAt = now;
  }

  if (currentScreen == SCR_HUNTER)
  {
    if (now - uiHunterAt >= HUNTER_UPDATE_MS)
    {
      updateHunter();
      uiHunterAt = now;
    }
  }
  else if (now - uiSlowAt >= SLOW_UPDATE_MS)
  {
    updateCurrentScreen();
    uiSlowAt = now;
  }
}

// ============================================================
// SD MOUNT
// ============================================================

static void sdBegin()
{
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  const uint32_t freqs[] = {20000000, 4000000, 1000000};
  for (uint8_t i = 0; i < 3 && !sdReady; i++)
    sdReady = SD.begin(PIN_SD_CS, sdSPI, freqs[i]);

  if (sdReady)
    dualPrintln("[flockyou] SD mounted");
  else
    dualPrintln("[flockyou] SD mount FAILED — running without logging");
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(300);

  // RGB LED (common anode: HIGH = off) — start dark.
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  rgbOff();

  // Audio amp enable (SC8002B: HIGH un-mutes).
  pinMode(PIN_AUDIO_EN, OUTPUT);
  digitalWrite(PIN_AUDIO_EN, HIGH);

  // GPS on UART2 (RX=25, TX=32).
  gpsBegin();

  // Display first, so status is visible during the rest of bring-up.
  tftInit();

  // Rotation-specific touch calibration: load from NVS, or run the blocking
  // corner-arrow routine on first boot / after 'k'. Boot-only, so the one
  // long block here is acceptable.
  touchCalibrate(false);

  // Boot self-test: startup jingle + green pulse + detection-palette cycle.
  startupBeep();
  rgbShow(0, LED_BRIGHTNESS, 0);
  delay(200);
  ledBootColorCycle(BOOT_CYCLE_MS); // ends dark

  precompileOuis();
  memset(dedupeTable, 0, sizeof(dedupeTable));

  // SD card + session persistence. Non-fatal if the card is missing.
  sdBegin();
  fySessionBegin();

  WiFi.mode(WIFI_MODE_NULL);
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_start();

  applyInitialChannel();

  wifi_promiscuous_filter_t filt = {
      .filter_mask = 0
#if PROCESS_MGMT_FRAMES
                     | WIFI_PROMIS_FILTER_MASK_MGMT
#endif
#if PROCESS_DATA_FRAMES
                     | WIFI_PROMIS_FILTER_MASK_DATA
#endif
  };
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&wifiSniffer);
  esp_wifi_set_promiscuous(true);

  dualPrintln("[flockyou] merged WiFi detector started (E32R40T)");
  dualPrintln("[flockyou] serial cmds: 'd'=dump table CSV, 'j'=dump table JSON");
  dualPrintf("[flockyou] mode=%s dwell_ms=%u start_channel=%u rssi_min=%d sd=%d\n",
             channelModeName(), CHANNEL_DWELL_MS, currentChannel,
             RSSI_MIN, sdReady ? 1 : 0);

  lastHeartbeat = millis();
  fyLastSaveAt = millis();
  screenDirty = true; // draw the first UI frame immediately
}

// Every 5 s: packet rate + queue drops + heap. The key hardware-vs-software
// tell — drops>0 means the UI is starving the drain; drops~0 with a low pkt/s
// and floor-hugging RSSI means weak RX (antenna / display desense).
static unsigned long fyDiagAt = 0;
static uint32_t fyPktPrev = 0;
static void diagTick()
{
  unsigned long now = millis();
  if (now - fyDiagAt < 5000)
    return;
  unsigned long dt = now - fyDiagAt;
  fyDiagAt = now;
  uint32_t pkt = fyPktSeen;
  uint32_t rate = (uint32_t)((uint64_t)(pkt - fyPktPrev) * 1000UL / (dt ? dt : 1));
  fyPktPrev = pkt;
  dualPrintf("[diag] pkt/s=%lu total=%lu enq=%lu dropped=%lu uniq=%d heap=%u\n",
             (unsigned long)rate, (unsigned long)pkt,
             (unsigned long)fyAlertsEnqueued, (unsigned long)fyAlertsDropped,
             fyDetCount, (unsigned)ESP.getFreeHeap());
}

void loop()
{
  gpsTick();           // feed NMEA + refresh fix before detections are stamped
  serialCommandTick(); // 'd'/'j'/'k'/'b' over USB serial
  updateChannelMode();
  drainAlertQueue();   // Serial.printf happens here, not in callback
  autosaveTick();      // periodic SD CSV write if dirty
  heartbeatTick();     // audible beep-pair while a target is still in range
  ledTick();           // turn off LED after LED_FLASH_MS
  breatheTick();       // subtle idle "still alive" blink between detections
  tftTick();           // refresh the UI (+ full-screen detection flash)
  diagTick();          // 5 s RF/throughput diagnostics to serial
  printHeartbeat();
  delay(1);
}
