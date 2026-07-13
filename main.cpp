#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <ctype.h>
#include <string.h>
#include <SPIFFS.h>
#include <Adafruit_NeoPixel.h>

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
#define LED_FLASH_MS 120
#define LED_BRIGHTNESS 64

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

// Serial1 TX-only debug mirror. The classic ESP32 has no native USB — the
// onboard bridge drives Serial (USB) over UART0 (GPIO1/3), so we mirror on
// GPIO17, the free UART2 TX pin. (On ESP32-S3 boards use GPIO43 instead.)
#define MIRROR_SERIAL 1
#define MIRROR_TX_PIN 17
#define MIRROR_BAUD 115200

// ---- BLE serial mirror (for the Circuit Magic "BLE Controller" iOS app) ----
// iOS can't use Bluetooth Classic SPP (the BluetoothSerial example on the
// vendor page is Android-only), so we expose a BLE "UART" using the Nordic
// UART Service — the de-facto standard iOS BLE terminals/controllers scan for.
// Every line that goes to USB Serial is also notified over BLE.
//
// The classic ESP32 shares one 2.4 GHz radio between WiFi and BLE, so with BLE
// enabled the promiscuous sniffer loses frames to coexistence — field-tested to
// cause many missed detections. DISABLED by default for that reason. The code
// stays behind this guard so it can be re-enabled, but for detection work leave
// it 0. (If you re-enable and the app can't find the device, swap these UUIDs to
// whatever it expects, e.g. HM-10 style: service FFE0, characteristic FFE1.)
#define USE_BLE          0
#define BLE_DEVICE_NAME  "FlockYou"
#define BLE_SVC_UUID     "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // notify: device -> phone
#define BLE_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // write:  phone -> device
#define BLE_MTU          247    // request a large ATT MTU so whole lines fit in 1-2 notifies
#define BLE_CHUNK        20     // fallback notify size before the MTU is negotiated

// Lazy connection: a long interval + slave latency lets a connected phone
// barely touch the shared radio while idle — the ESP32 skips connection events
// when it has nothing to send, and transmits at the next event (~200 ms) the
// instant a detection fires. Keeps the link alive while handing almost all
// airtime back to the sniffer. Values satisfy Apple's BLE parameter rules:
//   IntervalMax*(latency+1) <= 2 s, *3 < timeout, latency <= 30, timeout <= 6 s.
#define BLE_CONN_MIN_INTERVAL  80    // x1.25ms = 100 ms
#define BLE_CONN_MAX_INTERVAL  160   // x1.25ms = 200 ms
#define BLE_CONN_LATENCY       6     // connection events the peripheral may skip when idle
#define BLE_CONN_TIMEOUT       600   // x10ms  = 6 s supervision timeout
// Advertising interval (pre-connection radio saver — slower = less airtime).
#define BLE_ADV_MIN_INTERVAL   800   // x0.625ms = 500 ms
#define BLE_ADV_MAX_INTERVAL   1600  // x0.625ms = 1 s

#if USE_BLE
#include <NimBLEDevice.h>
#include "esp_coexist.h"
#endif

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

// Persistence — binary snapshot, reloaded on boot so counts survive power loss.
#define MAX_DETECTIONS 200
#define FY_SESSION_FILE "/fy_sess.bin"
#define FY_SESSION_TMP "/fy_sess.tmp"
#define FY_FILE_MAGIC 0x464C4B32u // 'FLK2'
#define FY_FILE_VERSION 2
#define AUTOSAVE_INTERVAL_MS 60000

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
// DETECTION TABLE  (on-device storage, persisted to SPIFFS)
// ============================================================
//
// Single-threaded: only touched from loop() — drainAlertQueue() adds, and
// fySaveSession() reads. No mutex needed. The WiFi-task callback never
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
static bool fySpiffsReady = false;
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
// BLE SERIAL MIRROR  (Nordic UART Service — notify device -> phone)
// ============================================================
//
// Runs only from loop() context (via dualPrintf/dualPrintln), never from the
// WiFi promiscuous callback, so notifying here is safe. blePrint() is a no-op
// until a phone subscribes, so it costs nothing while disconnected.

#if USE_BLE
static NimBLECharacteristic* bleTxChar     = nullptr;
static NimBLEServer*         bleServer      = nullptr;
static volatile uint16_t     bleConnHandle  = 0;
static volatile bool         bleConnected   = false;

class FYServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, ble_gap_conn_desc* desc) override {
    bleConnHandle = desc->conn_handle;
    bleConnected  = true;
    // Ask the phone for a lazy connection so BLE stops hogging the radio while
    // idle; notifications still go out promptly at the next connection event.
    s->updateConnParams(desc->conn_handle,
                        BLE_CONN_MIN_INTERVAL, BLE_CONN_MAX_INTERVAL,
                        BLE_CONN_LATENCY, BLE_CONN_TIMEOUT);
  }
  void onDisconnect(NimBLEServer*) override {
    bleConnected = false;
    NimBLEDevice::startAdvertising();   // allow reconnect
  }
};

static void bleBegin() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(BLE_MTU);        // negotiate a large MTU so lines aren't over-fragmented
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new FYServerCallbacks());

  NimBLEService* svc = bleServer->createService(BLE_SVC_UUID);
  bleTxChar = svc->createCharacteristic(BLE_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  // RX is unused (we don't act on phone->device writes) but present so
  // controller apps that expect a writable characteristic still bind cleanly.
  svc->createCharacteristic(BLE_RX_UUID, NIMBLE_PROPERTY::WRITE);
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponse(true);
  adv->setMinInterval(BLE_ADV_MIN_INTERVAL);   // slow advertising = less pre-connect airtime
  adv->setMaxInterval(BLE_ADV_MAX_INTERVAL);
  adv->start();
}

// Notify a byte span to the phone, split into MTU-safe chunks. Using the
// negotiated ATT MTU (minus the 3-byte ATT header) keeps whole lines to 1-2
// notifications, so the lazy connection can drain them without dropping any.
static void blePrint(const char* buf, int len) {
  if (!bleConnected || !bleTxChar || len <= 0) return;
  int chunk = BLE_CHUNK;                                  // fallback until MTU is known
  if (bleServer) {
    uint16_t mtu = bleServer->getPeerMTU(bleConnHandle);
    if (mtu > 3) chunk = (int)mtu - 3;
  }
  for (int off = 0; off < len; off += chunk) {
    int n = len - off;
    if (n > chunk) n = chunk;
    bleTxChar->setValue((const uint8_t*)(buf + off), (size_t)n);
    bleTxChar->notify();
  }
}
#else
static inline void blePrint(const char*, int) {}
#endif

// ============================================================
// HELPERS
// ============================================================

// Dual-output: prints to both Serial (USB) and Serial1 (MIRROR_TX_PIN)
static char _dualBuf[384];

static void dualPrintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void dualPrintf(const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(_dualBuf, sizeof(_dualBuf), fmt, args);
  va_end(args);
  if (n > 0)
  {
    Serial.write(_dualBuf, n);
#if MIRROR_SERIAL
    Serial1.write(_dualBuf, n);
#endif
    blePrint(_dualBuf, n);
  }
}

static void dualPrintln(const char *str)
{
  Serial.println(str);
#if MIRROR_SERIAL
  Serial1.println(str);
#endif
  blePrint(str, (int)strlen(str));
  blePrint("\n", 1);
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
    dualPrintf("[flockyou] scanning (ch=%u mode=%s det=%d)\n",
               currentChannel, channelModeName(), fyDetCount);
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

// ------------------------------------------------------------
// GPS fix state — updated by the GPS reader in loop() (Stage 3). Until a real
// fix arrives this stays "no fix", and detections are stored without geodata.
// ------------------------------------------------------------
static double gpsLat = 0.0;
static double gpsLon = 0.0;
static uint32_t gpsUtc = 0; // unix epoch, UTC
static bool gpsHasFix = false;

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
// CRC32  (zlib / SPIFFS-tool compatible polynomial 0xEDB88320)
// ============================================================

static uint32_t fyCRC32Update(uint32_t crc, const uint8_t *data, size_t len)
{
  crc = ~crc;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (int k = 0; k < 8; k++)
      crc = (crc >> 1) ^ (0xEDB88320u & -(int32_t)(crc & 1));
  }
  return ~crc;
}

// ============================================================
// SPIFFS SESSION PERSISTENCE  — binary snapshot, reloaded on boot
// ============================================================
//
// The on-disk file is only for on-device persistence (Flask reads the live
// serial stream, not this file), so we dump the detection table as a compact
// binary snapshot: a small header + a raw array of FYDetection records. Simple
// and reliable to round-trip, so the table survives power loss and is reloaded
// on boot — the count keeps accumulating instead of resetting to 0.
//
// Atomic write: header+records -> /fy_sess.tmp, then rename to /fy_sess.bin.
// Boot: read /fy_sess.bin (fallback /fy_sess.tmp), validate magic/version/
// recSize/CRC, load records into the live table. recSize guards against struct
// layout changes across firmware versions — a mismatched file is ignored.

typedef struct
{
  uint32_t magic;   // FY_FILE_MAGIC
  uint16_t version; // FY_FILE_VERSION
  uint16_t recSize; // sizeof(FYDetection) — rejects files from a different layout
  uint32_t count;   // number of FYDetection records that follow
  uint32_t crc;     // CRC32 over the records region
} FYFileHdr;

static bool fySpiffsCopy(const char *src, const char *dst)
{
  File s = SPIFFS.open(src, "r");
  if (!s)
    return false;
  File d = SPIFFS.open(dst, "w");
  if (!d)
  {
    s.close();
    return false;
  }
  uint8_t buf[256];
  int n;
  bool ok = true;
  while ((n = s.read(buf, sizeof(buf))) > 0)
  {
    if (d.write(buf, (size_t)n) != (size_t)n)
    {
      ok = false;
      break;
    }
  }
  s.close();
  d.close();
  return ok;
}

static bool fyAtomicPromote(const char *src, const char *dst)
{
  if (SPIFFS.rename(src, dst))
    return true;
  if (!fySpiffsCopy(src, dst))
    return false;
  SPIFFS.remove(src);
  return true;
}

static void fySaveSession()
{
  if (!fySpiffsReady)
    return;
  if (!fyDirty && fyDetCount == fyLastSaveCount)
    return;

  int savedCount = fyDetCount;
  size_t bytes = (size_t)savedCount * sizeof(FYDetection);
  uint32_t crc = fyCRC32Update(0, (const uint8_t *)fyDet, bytes);

  File f = SPIFFS.open(FY_SESSION_TMP, "w");
  if (!f)
  {
    dualPrintf("[flockyou] save failed: cannot open %s\n", FY_SESSION_TMP);
    return;
  }
  FYFileHdr hdr = {FY_FILE_MAGIC, (uint16_t)FY_FILE_VERSION,
                   (uint16_t)sizeof(FYDetection), (uint32_t)savedCount, crc};
  bool ok = f.write((const uint8_t *)&hdr, sizeof(hdr)) == sizeof(hdr);
  if (ok && bytes > 0)
    ok = f.write((const uint8_t *)fyDet, bytes) == bytes;
  f.close();
  if (!ok)
  {
    dualPrintf("[flockyou] save WRITE failed — old session preserved\n");
    return;
  }

  SPIFFS.remove(FY_SESSION_FILE);
  if (!fyAtomicPromote(FY_SESSION_TMP, FY_SESSION_FILE))
  {
    dualPrintf("[flockyou] promote FAILED — data in %s for recovery\n", FY_SESSION_TMP);
    return;
  }

  fyLastSaveAt = millis();
  fyLastSaveCount = savedCount;
  fyDirty = false;
  dualPrintf("[flockyou] session saved: %d det, %u bytes, crc=0x%08lX\n",
             savedCount, (unsigned)(sizeof(hdr) + bytes), (unsigned long)crc);
}

// Read a binary snapshot into the live table. Returns records loaded, or 0 if
// the file is missing / from a different layout / corrupt.
static int fyLoadSessionFrom(const char *path)
{
  if (!SPIFFS.exists(path))
    return 0;
  File f = SPIFFS.open(path, "r");
  if (!f)
    return 0;

  FYFileHdr hdr;
  if (f.read((uint8_t *)&hdr, sizeof(hdr)) != (int)sizeof(hdr))
  {
    f.close();
    return 0;
  }
  if (hdr.magic != FY_FILE_MAGIC || hdr.version != FY_FILE_VERSION ||
      hdr.recSize != (uint16_t)sizeof(FYDetection) || hdr.count > (uint32_t)MAX_DETECTIONS)
  {
    f.close();
    return 0;
  }
  size_t bytes = (size_t)hdr.count * sizeof(FYDetection);
  if ((size_t)f.size() < sizeof(hdr) + bytes)
  {
    f.close();
    return 0;
  }
  int loaded = 0;
  if (hdr.count > 0)
  {
    if (f.read((uint8_t *)fyDet, bytes) != (int)bytes)
    {
      f.close();
      return 0;
    }
    if (fyCRC32Update(0, (const uint8_t *)fyDet, bytes) != hdr.crc)
    {
      f.close();
      return 0; // corrupt — ignore
    }
    loaded = (int)hdr.count;
  }
  f.close();
  return loaded;
}

// Load the saved table on boot so detections persist across power loss.
static void fyLoadSession()
{
  if (!fySpiffsReady)
    return;
  int n = fyLoadSessionFrom(FY_SESSION_FILE);
  if (n == 0)
    n = fyLoadSessionFrom(FY_SESSION_TMP); // interrupted-save fallback
  fyDetCount = n;
  fyLastSaveCount = n;
  fyDirty = false;
  if (n > 0)
    dualPrintf("[flockyou] restored %d detections from flash\n", n);
  else
    dualPrintln("[flockyou] no prior session — starting fresh");
}

// ============================================================
// FLASK-COMPATIBLE JSON EMISSION
// ============================================================
//
// The Flask app (flock-you/api/flockyou.py) reads one JSON object per line
// from the USB CDC serial port. It filters by presence of `detection_method`
// and extracts these fields:  mac_address, rssi, channel, frequency, ssid,
// device_name, gps.latitude, gps.longitude, gps.accuracy.
//
// GPS is handled Flask-side via its own USB NMEA puck or browser geolocation;
// we don't embed GPS here because there's no on-device AP / phone link.

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
      "\"ssid\":\"%s\"}\n",
      method, mac, oui, rssi,
      (unsigned)ch, (unsigned)channelFreqMhz(ch), ssidEsc);
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

    // Always update the on-device detection table (survives reboot via SPIFFS).
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

    // Human-readable line (for serial terminal / mirror).
    char oui[9];
    ouiFromMac(e.mac, oui, sizeof(oui));
    if (e.type == ALERT_SSID)
    {
      dualPrintf("[flockyou] DETECT-SSID type=%s mac=%s ssid=\"%s\" rssi=%d ch=%u count=%d\n",
                 e.frameKind, macStr, e.ssid, e.rssi, e.channel,
                 (idx >= 0) ? (int)fyDet[idx].count : 0);
    }
    else
    {
      dualPrintf("[flockyou] DETECT-OUI mac=%s oui=%s rssi=%d ch=%u addr=%s count=%d\n",
                 macStr, oui, e.rssi, e.channel,
                 e.frameKind[0] ? e.frameKind : "addr2",
                 (idx >= 0) ? (int)fyDet[idx].count : 0);
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
  if (!fySpiffsReady || !fyDirty)
    return;
  if (millis() - fyLastSaveAt < AUTOSAVE_INTERVAL_MS)
    return;
  fySaveSession();
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

#if MIRROR_SERIAL
  Serial1.begin(MIRROR_BAUD, SERIAL_8N1, -1, MIRROR_TX_PIN); // TX-only on MIRROR_TX_PIN
#endif

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
  ledFlashColor(0, LED_BRIGHTNESS, 0, 200); // green boot pulse
#endif

  precompileOuis();
  memset(dedupeTable, 0, sizeof(dedupeTable));

  // SPIFFS — format on first boot if missing. Non-fatal if it fails.
  if (SPIFFS.begin(true))
  {
    fySpiffsReady = true;
    dualPrintln("[flockyou] SPIFFS ready");
    fyLoadSession();
  }
  else
  {
    dualPrintln("[flockyou] SPIFFS init FAILED — running without persistence");
  }

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

#if USE_BLE
  // Stand up the BLE UART after WiFi, then bias the shared radio toward
  // sniffing so BLE coexistence costs the fewest captured frames.
  bleBegin();
  esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
  dualPrintln("[flockyou] BLE serial mirror advertising as \"" BLE_DEVICE_NAME "\"");
#endif

  dualPrintln("[flockyou] merged WiFi detector started");
  dualPrintf("[flockyou] mode=%s dwell_ms=%u start_channel=%u rssi_min=%d spiffs=%d\n",
             channelModeName(), CHANNEL_DWELL_MS, currentChannel,
             RSSI_MIN, fySpiffsReady ? 1 : 0);

  lastHeartbeat = millis();
  fyLastSaveAt = millis();
}

void loop()
{
  updateChannelMode();
  drainAlertQueue(); // Serial.printf happens here, not in callback
  autosaveTick();    // periodic SPIFFS write if dirty
  heartbeatTick();   // audible beep-pair while a target is still in range
  ledTick();         // turn off LED after LED_FLASH_MS
  breatheTick();     // subtle idle "still alive" glow between detections
  printHeartbeat();
  delay(1);
}
