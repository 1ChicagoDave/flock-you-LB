// ============================================================================
// giga_flock.ino — Arduino GIGA R1 "brain" of a split-brain Flock detector
// ============================================================================
//
// HARDWARE: Arduino GIGA R1 WiFi + GIGA Display Shield (3.97" 480x800 IPS,
// GT911 capacitive touch, onboard RGB LED).  A Lonely Binary ESP32 runs the
// promiscuous-mode radio (esp32_companion/companion.cpp) and streams detections
// to this board over UART.  An Adafruit Ultimate GPS and a speaker are attached.
//
// ROLE SPLIT:  ESP32 = radio + detection engine.  GIGA = UI + GPS geotag + QSPI
// event logging + audio.  This mirrors the ESP32 project's model: a live device
// table for the screen PLUS an append-only event log for later triangulation.
//
// ----------------------------------------------------------------------------
// BOARD PACKAGE: "Arduino Mbed OS GIGA Boards"  (FQBN arduino:mbed_giga:giga)
//
// DISPLAY DRIVER — DO NOT "FIX" THIS INCLUDE:
//   The display driver on this core is Arduino_H7_Video, and it is BUNDLED WITH
//   THE CORE — it is NOT a Library Manager library.  It lives at
//     <arduino15>/packages/arduino/hardware/mbed_giga/<ver>/libraries/Arduino_H7_Video
//   and its header is "Arduino_H7_Video.h".
//
//   There IS a Library Manager library called "Arduino_Video" whose header is
//   "Arduino_Video.h".  It is NOT for this board: its library.properties says
//   architectures=zephyr_main, i.e. it only applies to the Arduino *Zephyr*
//   core, and it targets LVGL v9 through a different API.  Because the arch tag
//   does not match mbed_giga, arduino-cli silently EXCLUDES it from the build,
//   so #include "Arduino_Video.h" fails with a bare "No such file or directory"
//   even though the folder is plainly sitting in your libraries directory.
//   That mismatch is what broke this sketch. Keep the include below as-is.
//
// LIBRARY MANAGER LIBRARIES TO INSTALL:
//   * lvgl                        — v9.x (this code targets v9; see ui.cpp)
//   * Arduino_GigaDisplayTouch    — GT911 capacitive touch
//   * Arduino_GigaDisplay         — onboard RGB LED (GigaDisplayRGB)
//   * TinyGPSPlus                 — NMEA parsing (Serial2)
//   * ArduinoJson                 — UART line parsing (this code uses the v7 API)
//   (No Arduino_POSIXStorage — see storage.cpp; it cannot reach the QSPI.)
//
// lv_conf.h NOTE — the copy in this folder is INERT:
//   Arduino_H7_Video ships its own lv_conf.h in its src/ directory, which
//   auto-selects lv_conf_8.h or lv_conf_9.h by probing for a v9-only header.
//   That is the config the build actually uses; it sets LV_COLOR_DEPTH 16 and
//   enables LV_FONT_MONTSERRAT_14, which is all this UI needs.  The lv_conf.h
//   sitting next to this sketch is NOT picked up (verified with a #warning
//   probe: zero hits).  Edit the core's copy if you need to change LVGL config
//   — editing the local one will appear to do nothing.
//
// ----------------------------------------------------------------------------
// !!! KNOWN-UNCERTAIN — these are RUNTIME behaviours a clean compile cannot
// prove.  The sketch builds; these still want eyes on first flash:
//   1. QSPI filesystem (storage.cpp): the QSPI must be FAT-formatted ONCE via
//      File > Examples > STM32H747_System > QSPIFormat (pick the option that
//      KEEPS the Wi-Fi firmware partition).  Until then mount() returns non-zero
//      and we run without logging — handled gracefully, UI still works.
//   2. DAC tone (audio.cpp): analogWrite(A12) assumes A12 routes to the DAC on
//      your core version; an mbed AnalogOut fallback is provided.
//   3. GT911 touch (ui.cpp): the portrait->landscape coordinate remap
//      (TOUCH_SWAP_XY / TOUCH_INV_*) may need flipping for your panel.
// ----------------------------------------------------------------------------

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>

#include "Arduino_H7_Video.h"   // core-bundled. NOT "Arduino_Video.h" — see above.
#include "Arduino_GigaDisplayTouch.h"
#include "Arduino_GigaDisplay.h"        // GigaDisplayRGB (onboard LED)
#include "lvgl.h"

#include "config.h"
#include "flock_types.h"
#include "storage.h"
#include "audio.h"
#include "ui.h"

// ---- display / touch / LED objects ----
Arduino_H7_Video       Display(SCREEN_W, SCREEN_H, GigaDisplayShield);
Arduino_GigaDisplayTouch TouchDetector;    // referenced (extern) by ui.cpp
static GigaDisplayRGB  rgbLed;

// ---- GPS parser ----
static TinyGPSPlus gps;

// ============================================================================
// GLOBAL STATE  (declared extern in flock_types.h)
// ============================================================================
DeviceEntry   g_dev[MAX_DEVICES];
int           g_devCount    = 0;
uint32_t      g_totalEvents = 0;
LinkStatus    g_link        = {};
GpsState      g_gps         = {};
int           g_lastDevIdx  = -1;
uint32_t      g_lastDetMs   = 0;
bool          g_logReady    = false;
volatile bool g_uiDirty     = false;

// ---- LED one-shot flash ----
static uint32_t ledOffAt = 0;

static void ledFlashHex(uint32_t rgb)
{
  uint8_t r = (rgb >> 16) & 0xFF, g = (rgb >> 8) & 0xFF, b = rgb & 0xFF;
  // Scale to the brightness cap.
  r = (uint16_t)r * LED_BRIGHT / 255;
  g = (uint16_t)g * LED_BRIGHT / 255;
  b = (uint16_t)b * LED_BRIGHT / 255;
  rgbLed.on(r, g, b);
  ledOffAt = millis() + LED_FLASH_MS;
  if (ledOffAt == 0) ledOffAt = 1;
}

static void ledTick(uint32_t now)
{
  if (ledOffAt && (int32_t)(now - ledOffAt) >= 0)
  {
    rgbLed.off();
    ledOffAt = 0;
  }
}

// ============================================================================
// DEVICE TABLE
// ============================================================================
static int findDevice(const char *mac)
{
  for (int i = 0; i < g_devCount; i++)
    if (strcasecmp(g_dev[i].mac, mac) == 0) return i;
  return -1;
}

// Upsert; returns {index, isNew}.
static int upsertDevice(const char *mac, const char *method, int rssi,
                        int ch, const char *ssid, int count, bool *isNew)
{
  uint32_t now = millis();
  int idx = findDevice(mac);
  if (idx >= 0)
  {
    DeviceEntry &d = g_dev[idx];
    strlcpy(d.method, method, sizeof(d.method));   // method can change per hit
    d.rssi    = (int8_t)rssi;
    d.channel = (uint8_t)ch;
    if (rssi > d.bestRssi) d.bestRssi = (int8_t)rssi;
    d.count   = (uint16_t)count;
    d.lastSeenMs = now;
    if (ssid && ssid[0] && !d.ssid[0]) strlcpy(d.ssid, ssid, sizeof(d.ssid));
    if (isNew) *isNew = false;
    return idx;
  }

  if (g_devCount >= MAX_DEVICES)
  {
    if (isNew) *isNew = false;
    return -1;   // table full — drop (matches the ESP32's fixed cap behavior)
  }

  DeviceEntry &d = g_dev[g_devCount];
  memset(&d, 0, sizeof(d));
  strlcpy(d.mac, mac, sizeof(d.mac));
  strlcpy(d.method, method, sizeof(d.method));
  if (ssid) strlcpy(d.ssid, ssid, sizeof(d.ssid));
  d.rssi = d.bestRssi = (int8_t)rssi;
  d.channel = (uint8_t)ch;
  d.count = (uint16_t)count;
  d.firstSeenMs = d.lastSeenMs = now;
  d.hasLoggedEvent = false;
  int newIdx = g_devCount++;
  if (isNew) *isNew = true;
  return newIdx;
}

// ============================================================================
// EVENT LOG DECISION  (time + distance sampling / stationary suppression)
// ============================================================================
static double metresBetween(double lat1, double lon1, double lat2, double lon2)
{
  // Equirectangular approximation — plenty accurate at these short ranges.
  const double R = 6371000.0;
  double x = radians(lon2 - lon1) * cos(radians((lat1 + lat2) * 0.5));
  double y = radians(lat2 - lat1);
  return sqrt(x * x + y * y) * R;
}

static void maybeLogEvent(int idx)
{
  if (idx < 0) return;
  if (!g_gps.hasFix) return;                 // no fix -> no event row

  DeviceEntry &d = g_dev[idx];
  uint32_t now = millis();

  bool doLog = false;
  if (!d.hasLoggedEvent)
  {
    doLog = true;                            // first event for this MAC
  }
  else
  {
    double moved = metresBetween(d.lastEventLat, d.lastEventLon,
                                 g_gps.lat, g_gps.lon);
    if (moved >= EVENT_MIN_DISTANCE_M) doLog = true;
    else if (now - d.lastEventMs >= EVENT_MIN_INTERVAL_MS) doLog = true;  // keepalive
  }
  if (!doLog) return;

  // Count the event regardless of whether the write persists (reflects activity).
  g_totalEvents++;
  d.hasLoggedEvent = true;
  d.lastEventLat = g_gps.lat;
  d.lastEventLon = g_gps.lon;
  d.lastEventMs  = now;

  if (g_logReady)
  {
    storage_append_event(d.mac, d.method, d.rssi, d.channel,
                         g_gps.lat, g_gps.lon, g_gps.utc,
                         g_gps.sats, g_gps.hdop, d.ssid);
  }
}

// ============================================================================
// UART LINK  (Serial1 <- ESP32)  newline-delimited JSON
// ============================================================================
static char   rxBuf[256];
static size_t rxLen = 0;

static void handleDet(JsonDocument &doc)
{
  const char *mac    = doc["mac"]    | "";
  const char *method = doc["method"] | "";
  int   rssi  = doc["rssi"]  | 0;
  int   ch    = doc["ch"]    | 0;
  const char *ssid = doc["ssid"] | "";
  int   count = doc["count"] | 0;
  if (!mac[0]) return;

  bool isNew = false;
  int idx = upsertDevice(mac, method, rssi, ch, ssid, count, &isNew);
  if (idx < 0) return;

  g_lastDevIdx = idx;
  g_lastDetMs  = millis();
  g_uiDirty    = true;

  // Geotag + append-only event log (stationary suppression inside).
  maybeLogEvent(idx);

  // RGB LED flashes the detection-class color on every hit.
  ledFlashHex(methodColorHex(method));

  // Audio only on a brand-new unique MAC (rising two-note chirp).
  if (isNew) audio_chirp_new();
}

static void handleStatus(JsonDocument &doc)
{
  g_link.everSeen     = true;
  g_link.lastStatusMs = millis();
  g_link.channel = (uint8_t)(doc["ch"]     | (int)g_link.channel);
  g_link.uptime  = (uint32_t)(doc["uptime"] | (long)g_link.uptime);
  g_link.pkts    = (uint32_t)(doc["pkts"]   | (long)g_link.pkts);
  g_link.uniq    = (int)(doc["uniq"]        | g_link.uniq);
}

static void handleLine(const char *line)
{
  JsonDocument doc;                         // ArduinoJson v7 elastic document
  DeserializationError err = deserializeJson(doc, line);
  if (err) return;                          // ignore malformed/partial lines

  const char *t = doc["t"] | "";
  if (!strcmp(t, "det"))         handleDet(doc);
  else if (!strcmp(t, "status")) handleStatus(doc);
}

static void pollEsp32()
{
  while (ESP32_SERIAL.available())
  {
    char c = (char)ESP32_SERIAL.read();
    if (c == '\n')
    {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) handleLine(rxBuf);
      rxLen = 0;
    }
    else if (c != '\r')
    {
      if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
      else rxLen = 0;                        // overlong line -> resync
    }
  }
}

// ============================================================================
// GPS  (Serial2)
// ============================================================================
// days-from-civil (Howard Hinnant) — same routine main.cpp uses for the epoch.
static long daysFromCivil(int y, unsigned m, unsigned d)
{
  y -= (m <= 2);
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (long)doe - 719468;
}

static void pollGps()
{
  while (GPS_SERIAL.available())
    gps.encode((char)GPS_SERIAL.read());

  if (gps.location.isValid() && gps.location.age() < GPS_STALE_MS)
  {
    g_gps.lat  = gps.location.lat();
    g_gps.lon  = gps.location.lng();
    g_gps.hdop = gps.hdop.isValid() ? (gps.hdop.value() / 100.0) : 0.0; // value()=HDOP*100
    g_gps.sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    if (gps.date.isValid() && gps.time.isValid() && gps.date.age() < GPS_STALE_MS)
    {
      long days = daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day());
      g_gps.utc = (uint32_t)days * 86400UL + (uint32_t)gps.time.hour() * 3600UL +
                  (uint32_t)gps.time.minute() * 60UL + (uint32_t)gps.time.second();
    }
    g_gps.hasFix = true;
  }
  else
  {
    g_gps.hasFix = false;   // detections still recorded, just without geodata
  }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup()
{
  Serial.begin(115200);           // USB debug only
  ESP32_SERIAL.begin(ESP32_BAUD); // Serial1: D18=TX, D19=RX
  GPS_SERIAL.begin(GPS_BAUD);     // Serial2: D16=TX, D17=RX

  audio_init();
  rgbLed.begin();
  rgbLed.off();

  // Display.begin() initializes LVGL (lv_init) and registers the panel driver.
  Display.begin();
  TouchDetector.begin();
  ui_init();                      // builds screens + registers touch indev

  // QSPI event log — non-fatal if it fails (see storage.cpp one-time-format note).
  g_logReady = storage_init();
  Serial.println(g_logReady ? "[giga] QSPI log ready" : "[giga] QSPI log DISABLED");

  audio_boot();
}

void loop()
{
  uint32_t now = millis();

  pollEsp32();          // parse ESP32 detection/status JSON
  pollGps();            // parse NMEA, refresh fix state
  ledTick(now);         // clear the class-color flash after LED_FLASH_MS
  ui_tick(now);         // refresh LVGL labels / LIVE list from global state

  lv_timer_handler();   // LVGL rendering + input (Arduino_H7_Video drives the tick)
  delay(3);
}
