#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <ctype.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>

// ============================================================
// SNIFFER COMPANION — RADIO half of a split-brain detector
// ============================================================
//
// Ported from the "Gold Edition" (main.cpp). The detection engine is copied
// VERBATIM — OUI/SSID matching, the promiscuous RX callback and 802.11 frame
// parsing, channel hopping, dedupe, the alert ring buffer, and the in-RAM
// detection table. What was removed vs main.cpp: on-board GPS (TinyGPS++ /
// Serial1 NMEA), SPIFFS persistence (binary snapshot / CRC / autosave), and
// NimBLE. Serial1's freed pins now carry a UART link to an Arduino GIGA, which
// owns the UI / GPS / logging. Each emitted detection is streamed to the GIGA
// as one newline-delimited JSON line; a status heartbeat goes out ~every 3 s.
// The RGB LED + piezo stay as local indicators. USB Serial keeps the human
// DETECT debug output.

// ============================================================
// CONFIG
// ============================================================

// External piezo buzzer. GPIO4 is a plain, non-strapping, header-exposed pin on
// the Lonely Binary ESP32 "Gold Edition" — safe to drive at boot.
#define BUZZER_PIN 4
#define USE_BUZZER 1

// Onboard WS2812 RGB LED on the "Gold Edition" is a single addressable NeoPixel
// on GPIO2 (per the board's reference card). It's driven with the Adafruit
// NeoPixel library (portable across all Arduino-ESP32 core versions — the
// core's built-in rgbLedWrite() only exists on newer cores). Detection class is
// encoded as color (see alertTypeColor); LED_BRIGHTNESS caps each channel
// because the WS2812B is blinding at full 255.
#define LED_PIN 2
#define USE_LED 1
// Long enough to catch out of the corner of your eye while driving; the flash
// is non-blocking (ledTick() clears it), so this never stalls the sniffer.
#define LED_FLASH_MS 2000
#define LED_BRIGHTNESS 64
// Boot self-test: dwell per color while stepping the detection palette.
#define BOOT_CYCLE_MS 500

// Idle "still alive" breathing glow. A slow, subtle pulse shown whenever the
// LED isn't mid detection-flash, so a glance confirms the unit is powered and
// scanning. BREATHE_R/G/B is the peak color (kept dim so it never competes with
// the bright, color-coded detection flashes); the pulse scales it 0..peak.
#define BREATHE_ENABLE 1
#define BREATHE_PERIOD_MS 2600 // full dim->bright->dim cycle
#define BREATHE_UPDATE_MS 30   // LED refresh cadence
#define BREATHE_R 8
#define BREATHE_G 0
#define BREATHE_B 14 // dim teal — reads as "idle / scanning"

// ---- GIGA UART link (Serial1) ----
// The freed old-GPS pins now carry a UART to an Arduino GIGA that does the
// UI/GPS/logging. GPIO16 = RX (<- GIGA TX), GPIO17 = TX (-> GIGA RX). Each
// emitted detection and a periodic status heartbeat are streamed as one
// newline-delimited JSON object. No GIGA->ESP32 command handling in v1; any
// bytes the GIGA sends are read and discarded.
#define GIGA_RX_PIN 16
#define GIGA_TX_PIN 17
#define GIGA_BAUD 115200
#define GIGA_STATUS_MS 3000 // status heartbeat cadence to the GIGA

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

// In-RAM detection table cap (no persistence in the companion build).
#define MAX_DETECTIONS 200

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

#define ALERT_QUEUE_SIZE 32

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

static void IRAM_ATTR enqueueAlert(AlertType type, const uint8_t *mac, int8_t rssi,
                                   uint8_t ch, const char *ssid, const char *kind)
{
  portENTER_CRITICAL_ISR(&queueMux);
  size_t next = (alertHead + 1) % ALERT_QUEUE_SIZE;
  if (next == alertTail)
  { // drop if full — loop() is behind
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
  portEXIT_CRITICAL_ISR(&queueMux);
}

// ============================================================
// DETECTION TABLE  (in-RAM only — not persisted in this build)
// ============================================================
//
// Single-threaded: only touched from loop() via drainAlertQueue(). No mutex
// needed. The WiFi-task callback never touches this table; it only writes to
// the lock-free alert ring buffer.

typedef struct
{
  char mac[18];
  char method[16]; // "oui_addr2" / "oui_addr1" / "oui_addr3" / "ssid"
  int8_t rssi;
  uint8_t channel;
  uint32_t firstSeen; // millis() at first hit
  uint32_t lastSeen;  // millis() at latest hit
  uint16_t count;
  char ssid[33]; // "" unless an SSID hit populated it
} FYDetection;

static FYDetection fyDet[MAX_DETECTIONS];
static int fyDetCount = 0;

// ============================================================
// STATE
// ============================================================

static uint8_t currentChannel = 1;
static size_t customChannelIndex = 0;
static size_t fullHopIndex = 0;
static unsigned long lastHop = 0;
static unsigned long lastHeartbeat = 0;
static volatile bool sniffingStopped = false;

// Lightweight packet counter — every mgmt/data frame the promiscuous callback
// accepts bumps this. Reported to the GIGA in the status heartbeat ("pkts").
static volatile uint32_t pktCount = 0;

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

// GIGA status-heartbeat timer.
static unsigned long lastGigaStatus = 0;

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
// HELPERS
// ============================================================

// USB-only human/debug output. (Serial1 is reserved for the GIGA JSON link.)
static char _dbgBuf[384];

static void dbgPrintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void dbgPrintf(const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(_dbgBuf, sizeof(_dbgBuf), fmt, args);
  va_end(args);
  if (n > 0)
  {
    Serial.write(_dbgBuf, n);
  }
}

static void dbgPrintln(const char *str)
{
  Serial.println(str);
}

// Single onboard WS2812. Constructed here; rgbLed.begin() runs once in setup().
static Adafruit_NeoPixel rgbLed(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// WS2812 write — raw per-channel values, already brightness-limited by callers.
static inline void rgbShow(uint8_t r, uint8_t g, uint8_t b)
{
#if USE_LED
  rgbLed.setPixelColor(0, rgbLed.Color(r, g, b));
  rgbLed.show();
#endif
}
static inline void rgbOff() { rgbShow(0, 0, 0); }

// One-shot colored pulse; ledTick() clears it after the timer expires.
static void ledFlashColor(uint8_t r, uint8_t g, uint8_t b, unsigned ms)
{
#if USE_LED
  rgbShow(r, g, b);
  ledOffAt = millis() + ms;
  if (ledOffAt == 0)
    ledOffAt = 1; // avoid the "off" sentinel
#endif
}

static void ledTick()
{
#if USE_LED
  if (ledOffAt && (long)(millis() - ledOffAt) >= 0)
  {
    rgbOff();
    ledOffAt = 0;
  }
#endif
}

// Slow breathing pulse while idle. Skips entirely during a detection flash
// (ledOffAt != 0), so a hit always takes over the LED cleanly.
static unsigned long fyLastBreatheAt = 0;
static void breatheTick()
{
#if USE_LED && BREATHE_ENABLE
  if (ledOffAt)
    return; // a detection flash owns the LED
  unsigned long now = millis();
  if (now - fyLastBreatheAt < BREATHE_UPDATE_MS)
    return;
  fyLastBreatheAt = now;

  // Smooth 0..1 sine breathe: dark at cycle start, full at the midpoint.
  float phase = (float)(now % BREATHE_PERIOD_MS) / (float)BREATHE_PERIOD_MS;
  float level = (1.0f - cosf(phase * TWO_PI)) * 0.5f;
  rgbShow((uint8_t)(BREATHE_R * level),
          (uint8_t)(BREATHE_G * level),
          (uint8_t)(BREATHE_B * level));
#endif
}

static void buzzerBeep(unsigned int ms)
{
#if USE_BUZZER
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
#endif
}

// Two fast ascending beeps — played on the FIRST sighting of a MAC.
static void newDetectChirp()
{
#if USE_BUZZER
  tone(BUZZER_PIN, NEW_CHIRP_LO_HZ);
  delay(NEW_CHIRP_NOTE_MS);
  noTone(BUZZER_PIN);
  delay(NEW_CHIRP_GAP_MS);
  tone(BUZZER_PIN, NEW_CHIRP_HI_HZ);
  delay(NEW_CHIRP_NOTE_MS);
  noTone(BUZZER_PIN);
#endif
}

// Two monotone beeps — periodic heartbeat while at least one target is still
// in range (last seen within HB_DEVICE_ACTIVE_MS).
static void heartbeatBeep()
{
#if USE_BUZZER
  tone(BUZZER_PIN, HB_BEEP_HZ);
  delay(HB_BEEP_NOTE_MS);
  noTone(BUZZER_PIN);
  delay(HB_BEEP_GAP_MS);
  tone(BUZZER_PIN, HB_BEEP_HZ);
  delay(HB_BEEP_NOTE_MS);
  noTone(BUZZER_PIN);
#endif
}
static void startupBeep()
{
#if USE_BUZZER
  // First 6 notes of SMB World 1-2 (underground). Koji Kondo's descending
  // pattern: C5 → C4 → A4 → A3 → G#4 → G#3 (alternating-octave pairs).
  static const uint16_t notes[6] = {523, 262, 440, 220, 415, 208};
  for (int i = 0; i < 6; i++)
  {
    tone(BUZZER_PIN, notes[i]);
    delay((i == 5) ? 160 : 95);
    noTone(BUZZER_PIN);
    if (i < 5)
      delay(22);
  }
#endif
}

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
  dbgPrintf("[flockyou] sniffing stopped: %s\n", reason);
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
    dbgPrintf("[flockyou] scanning (ch=%u mode=%s det=%d pkts=%lu)\n",
              currentChannel, channelModeName(), fyDetCount,
              (unsigned long)pktCount);
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

// Distinct WS2812 color per detection class, so the RGB LED alone tells you
// what kind of hit fired at a glance. Values are already scaled to
// LED_BRIGHTNESS (the WS2812B is blinding at full 255).
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

// Boot self-test: step through every detection color, in the same order the
// README/table lists them, so a glance confirms the LED works and re-learns the
// code before driving. Blocking by design — setup() only, before the sniffer
// starts. Colors come from alertTypeColor() so this can never drift from the
// live mapping.
static void ledBootColorCycle(unsigned ms)
{
#if USE_LED
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
#endif
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
  fyDetCount++;
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
// GIGA UART EMISSION  (Serial1 — one JSON object per line)
// ============================================================

// One detection line to the GIGA. Emitted post-dedupe, alongside the LED flash.
static void emitGigaDetection(const char *mac, const char *method,
                              int8_t rssi, uint8_t ch, const char *ssid,
                              uint16_t count)
{
  char ssidEsc[sizeof(((FYDetection *)0)->ssid) * 6 + 1];
  jsonEscape(ssidEsc, sizeof(ssidEsc), ssid ? ssid : "");
  char line[160];
  int n = snprintf(line, sizeof(line),
                   "{\"t\":\"det\",\"mac\":\"%s\",\"method\":\"%s\",\"rssi\":%d,"
                   "\"ch\":%u,\"ssid\":\"%s\",\"count\":%u}\n",
                   mac, method, rssi, (unsigned)ch, ssidEsc, (unsigned)count);
  if (n > 0)
    Serial1.write(line, (n < (int)sizeof(line)) ? (size_t)n : sizeof(line));
}

// Periodic status heartbeat to the GIGA (~every GIGA_STATUS_MS).
static void gigaStatusTick()
{
  unsigned long now = millis();
  if (now - lastGigaStatus < GIGA_STATUS_MS)
    return;
  lastGigaStatus = now;
  char line[128];
  int n = snprintf(line, sizeof(line),
                   "{\"t\":\"status\",\"ch\":%u,\"uptime\":%lu,\"pkts\":%lu,\"uniq\":%d}\n",
                   (unsigned)currentChannel, (unsigned long)(now / 1000UL),
                   (unsigned long)pktCount, fyDetCount);
  if (n > 0)
    Serial1.write(line, (n < (int)sizeof(line)) ? (size_t)n : sizeof(line));
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

  pktCount++; // total mgmt/data frames seen (reported to GIGA as "pkts")

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

    // Always update the in-RAM detection table (for the running count).
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

    uint16_t count = (idx >= 0) ? fyDet[idx].count : 0;

    // Human-readable line (USB serial terminal only).
    char oui[9];
    ouiFromMac(e.mac, oui, sizeof(oui));
    if (e.type == ALERT_SSID)
    {
      dbgPrintf("[flockyou] DETECT-SSID type=%s mac=%s ssid=\"%s\" rssi=%d ch=%u count=%d\n",
                e.frameKind, macStr, e.ssid, e.rssi, e.channel, (int)count);
    }
    else
    {
      dbgPrintf("[flockyou] DETECT-OUI mac=%s oui=%s rssi=%d ch=%u addr=%s count=%d\n",
                macStr, oui, e.rssi, e.channel,
                e.frameKind[0] ? e.frameKind : "addr2", (int)count);
    }

    // Stream one JSON detection line to the GIGA over Serial1.
    emitGigaDetection(macStr, method, e.rssi, e.channel,
                      (e.type == ALERT_SSID) ? e.ssid : "", count);

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

// Drain any bytes the GIGA sends. No command handling in v1 — read & discard so
// the RX buffer never backs up.
static void gigaRxDrain()
{
  while (Serial1.available())
    (void)Serial1.read();
}

// ============================================================
// LOG EXPORT  — dump the in-RAM detection table over USB serial
// ============================================================
//
// Type a key in the serial monitor to export the accumulated table:
//   d = CSV (spreadsheet-friendly),  j = JSON (one object per line)

static void dumpDetectionsCSV()
{
  dbgPrintln("mac,method,rssi,channel,count,firstSeen_ms,lastSeen_ms,ssid");
  for (int i = 0; i < fyDetCount; i++)
  {
    FYDetection &d = fyDet[i];
    dbgPrintf("%s,%s,%d,%u,%u,%lu,%lu,\"%s\"\n",
              d.mac, d.method, d.rssi, (unsigned)d.channel, (unsigned)d.count,
              (unsigned long)d.firstSeen, (unsigned long)d.lastSeen, d.ssid);
  }
  dbgPrintf("[flockyou] dumped %d detections (CSV)\n", fyDetCount);
}

static void dumpDetectionsJSON()
{
  for (int i = 0; i < fyDetCount; i++)
  {
    FYDetection &d = fyDet[i];
    char ssidEsc[sizeof(d.ssid) * 6 + 1];
    jsonEscape(ssidEsc, sizeof(ssidEsc), d.ssid);
    dbgPrintf("{\"record\":%d,\"mac\":\"%s\",\"method\":\"%s\",\"rssi\":%d,"
              "\"channel\":%u,\"count\":%u,\"ssid\":\"%s\"}\n",
              i, d.mac, d.method, d.rssi, (unsigned)d.channel,
              (unsigned)d.count, ssidEsc);
  }
  dbgPrintf("[flockyou] dumped %d detections (JSON)\n", fyDetCount);
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
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup()
{
  Serial.begin(115200);
  // On native-USB parts (ESP32-S3/C3) this stops Serial.write() blocking
  // forever on the USB-CDC port when no host is attached. The classic ESP32
  // uses a hardware UART bridge and HardwareSerial has no such method, so
  // guard it out there.
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  delay(300);

  // GIGA UART link on Serial1 (freed old-GPS pins).
  Serial1.begin(GIGA_BAUD, SERIAL_8N1, /*RX=*/GIGA_RX_PIN, /*TX=*/GIGA_TX_PIN);

#if USE_BUZZER
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
#endif

#if USE_LED
  rgbLed.begin(); // init the WS2812 driver
  rgbOff();       // start dark
#endif

  startupBeep();
#if USE_LED
  // Green boot pulse, then the detection-palette self-test. Driven blocking
  // (not via ledFlashColor) so the green actually dwells for its 200 ms instead
  // of being overwritten by the first cycle color on the very next line.
  rgbShow(0, LED_BRIGHTNESS, 0);
  delay(200);
  ledBootColorCycle(BOOT_CYCLE_MS); // ends dark
#endif

  precompileOuis();
  memset(dedupeTable, 0, sizeof(dedupeTable));

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

  dbgPrintln("[flockyou] sniffer companion started (GIGA link on Serial1)");
  dbgPrintln("[flockyou] serial cmds: 'd'=dump table CSV, 'j'=dump table JSON");
  dbgPrintf("[flockyou] mode=%s dwell_ms=%u start_channel=%u rssi_min=%d\n",
            channelModeName(), CHANNEL_DWELL_MS, currentChannel, RSSI_MIN);

  lastHeartbeat = millis();
  lastGigaStatus = millis();
}

void loop()
{
  gigaRxDrain();       // read & discard any GIGA->ESP32 bytes (no cmds in v1)
  serialCommandTick(); // 'd'/'j' over USB serial exports the in-RAM table
  updateChannelMode();
  drainAlertQueue();   // detections -> LED/piezo + JSON line to the GIGA
  gigaStatusTick();    // ~3 s status heartbeat to the GIGA
  heartbeatTick();     // audible beep-pair while a target is still in range
  ledTick();           // turn off LED after LED_FLASH_MS
  breatheTick();       // subtle idle "still alive" glow between detections
  printHeartbeat();    // periodic USB debug line
  delay(1);
}
