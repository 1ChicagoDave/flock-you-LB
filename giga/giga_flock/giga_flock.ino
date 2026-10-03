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
#include "backlight.h"

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

// This-drive counters (reset every boot; NOT persisted).  g_sessUniq = distinct
// devices sighted this session (incl. ones reloaded from a prior drive but seen
// again now); g_sessEvents = event rows logged this session.
uint32_t      g_sessUniq    = 0;
uint32_t      g_sessEvents  = 0;
static bool   sessionSeen[MAX_DEVICES] = { false };  // per-device "seen this session"

// Per-device "first discovered during THIS session" — i.e. it was not in the
// table restored from flash at boot.  Drives the NEW / KNOWN badge in the UI.
// Index-aligned with g_dev (entries are only ever appended, never removed).
bool          g_devNew[MAX_DEVICES] = { false };

// ---- hang detection ---------------------------------------------------------
// Hardware reset-cause reporting is unavailable on this board: RCC->RSR reads 0
// after a confirmed watchdog reset (the bootloader clears it), and an RTC backup
// register doesn't survive either. Both were tried and verified useless.
//
// Instead a high-priority RTOS thread watches the main loop. If loop() stops
// updating its liveness stamp, the UI has wedged — the thread writes a #HANG
// marker (with position/time) to the log and reboots. A "#HANG" row immediately
// followed by "#BOOT" is then unambiguous proof of a hang, versus a bare "#BOOT"
// which is an ordinary power/ignition cycle. The 8 s hardware IWDG stays as the
// backstop for the case where even this thread can't run.
static volatile uint32_t g_lastLoopMs = 0;
static rtos::Thread      s_hangWatch(osPriorityHigh, 4096, nullptr, "hangwatch");

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

  DeviceEntry &d = g_dev[idx];
  uint32_t now = millis();

  // Log EVERY detection; geotag it when we have a fix (matches the ESP32 build).
  // With a fix we suppress stationary repeats by distance moved; without a fix we
  // fall back to a time-based keepalive, so hits are still captured (ungeotagged).
  bool haveLastPos = (d.lastEventLat != 0.0) || (d.lastEventLon != 0.0);
  bool doLog = false;
  if (!d.hasLoggedEvent)
  {
    doLog = true;                            // first event for this MAC
  }
  else if (g_gps.hasFix && haveLastPos)
  {
    double moved = metresBetween(d.lastEventLat, d.lastEventLon,
                                 g_gps.lat, g_gps.lon);
    if (moved >= EVENT_MIN_DISTANCE_M) doLog = true;
    else if (now - d.lastEventMs >= EVENT_MIN_INTERVAL_MS) doLog = true;  // keepalive
  }
  else
  {
    if (now - d.lastEventMs >= EVENT_MIN_INTERVAL_MS) doLog = true;       // no fix: keepalive only
  }
  if (!doLog) return;

  g_totalEvents++;
  g_sessEvents++;
  d.hasLoggedEvent = true;
  if (g_gps.hasFix) { d.lastEventLat = g_gps.lat; d.lastEventLon = g_gps.lon; }
  d.lastEventMs = now;

  if (g_logReady)
  {
    storage_append_event(d.mac, d.method, d.rssi, d.channel,
                         g_gps.hasFix ? g_gps.lat : 0.0,
                         g_gps.hasFix ? g_gps.lon : 0.0,
                         g_gps.hasFix ? g_gps.utc : 0,
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

  // Look up BEFORE the upsert so we can see how long this device has been
  // silent (upsertDevice overwrites lastSeenMs).
  int      existing = findDevice(mac);
  bool     wasKnown = (existing >= 0);
  uint32_t prevSeen = wasKnown ? g_dev[existing].lastSeenMs : 0;

  bool isNew = false;
  int idx = upsertDevice(mac, method, rssi, ch, ssid, count, &isNew);
  if (idx < 0) return;
  if (isNew && idx < MAX_DEVICES) g_devNew[idx] = true;   // discovered this session

  g_lastDevIdx = idx;
  g_lastDetMs  = millis();
  g_uiDirty    = true;

  // Count this device once per session (first time it's sighted since boot).
  if (idx >= 0 && idx < MAX_DEVICES && !sessionSeen[idx])
  {
    sessionSeen[idx] = true;
    g_sessUniq++;
  }

  // Geotag + append-only event log (stationary suppression inside).
  maybeLogEvent(idx);

  // RGB LED flashes the detection-class color on every hit.
  ledFlashHex(methodColorHex(method));

  // Audio on a brand-new unique MAC, and again when a KNOWN camera reappears
  // after REDISCOVER_CHIRP_MS of silence — otherwise a camera you pass daily is
  // permanently mute once it lands in the persisted table.  (Devices restored
  // from flash have lastSeenMs zeroed, so they re-chirp once uptime passes the
  // threshold, which is exactly the "first encounter this drive" case.)
  bool rediscovered = wasKnown && (millis() - prevSeen >= REDISCOVER_CHIRP_MS);
  if (isNew || rediscovered)
  {
    audio_chirp_new();
    if (isNew && g_logReady) storage_save_table();  // persist the new unique device now
  }
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
    // Heading for the NAV compass.  Course-over-ground is only meaningful when
    // moving; parked, it wanders randomly, so gate it on speed.
    g_gps.speedMps    = gps.speed.isValid()  ? gps.speed.mps()  : 0.0;
    g_gps.course      = gps.course.isValid() ? gps.course.deg() : 0.0;
    g_gps.courseValid = gps.course.isValid() && gps.speed.isValid() &&
                        gps.course.age() < GPS_STALE_MS &&
                        g_gps.speedMps >= NAV_MIN_SPEED_MPS;
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
    g_gps.courseValid = false;
  }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup()
{
  // FIRST thing on purpose. The backlight pin comes out of reset enabled, so the
  // panel is at full brightness from power-on; every instruction before this one
  // is time spent blinding the driver at night. The GIGA bootloader runs before
  // any of this, so a brief bright flash at power-on is not avoidable from here,
  // but the application no longer adds to it. The saved level is applied further
  // down, once QSPI is mounted.
  backlight_begin(BL_BOOT_PCT);

  Serial.begin(115200);           // USB debug only
  ESP32_SERIAL.begin(ESP32_BAUD); // Serial4: D14=TX, D15=RX (D18/D19 don't work here)
  GPS_SERIAL.begin(GPS_BAUD);     // Serial3: D16=TX, D17=RX (GPS)

  audio_init();
  rgbLed.begin();
  rgbLed.off();

  // Display.begin() initializes LVGL (lv_init) and registers the panel driver.
  Display.begin();

  // Panel init reconfigures GPIO banks, which leaves the backlight pin no longer
  // driving as an output: the dimmer kept running and the screen stayed at full
  // brightness. Re-assert it here, after the display is up.
  backlight_reassert();

  TouchDetector.begin();
  ui_init();                      // builds screens + registers touch indev

  // QSPI event log — non-fatal if it fails (see storage.cpp one-time-format note).
  g_logReady = storage_init();
  Serial.println(g_logReady ? "[giga] QSPI log ready" : "[giga] QSPI log DISABLED");

  // Saved backlight level. Without persistence this would reset to full on every
  // trip, because the device power-cycles whenever the ignition goes off.
  {
    uint8_t  pct  = BL_DEFAULT_PCT;
    uint32_t step = BL_STEP_US_DEFAULT;
    bool restored = (g_logReady && storage_load_backlight(&pct, &step));
    backlight_set_step_us(step);
    backlight_set(pct);
    ui_backlight_refresh();
    // Logged, not just printed: the USB console is not attached at boot, so the
    // banner is unobservable in practice. This row is how we tell a genuine
    // restore from a silent fallback to the default after the fact.
    {
      char nb[72];
      snprintf(nb, sizeof(nb), "%u%% step %luus", (unsigned)pct, (unsigned long)step);
      if (g_logReady) storage_append_note("#BL", restored ? "restored" : "default", nb);
    }
    Serial.print(restored ? "[giga] backlight restored: " : "[giga] backlight default: ");
    Serial.print(pct);
    Serial.print("% ");
    Serial.print(backlight_label(pct));
    Serial.print("  step="); Serial.print((unsigned long)step);
    Serial.print("us pwm="); Serial.print(1000000UL / (step * 20));
    Serial.println("Hz");
  }

  // Touch calibration from the STATS screen, if one has been saved.
  {
    float sx, ox, sy, oy;
    if (g_logReady && storage_load_touchcal(&sx, &ox, &sy, &oy))
    {
      TouchCal c = { sx, ox, sy, oy };
      ui_set_touchcal(c);
      Serial.print("[giga] touch cal loaded: sx="); Serial.print(sx, 4);
      Serial.print(" ox="); Serial.print(ox, 1);
      Serial.print(" sy="); Serial.print(sy, 4);
      Serial.print(" oy="); Serial.println(oy, 1);
    }
  }

  // Reload the persisted device table so hit counts survive a power cycle.
  if (g_logReady && storage_load_table())
  {
    Serial.print("[giga] restored "); Serial.print(g_devCount);
    Serial.print(" devices, "); Serial.print((unsigned long)g_totalEvents);
    Serial.println(" events from snapshot");
    g_uiDirty = true;
  }

  // Session boundary marker. GPS almost never has a fix this early, so the
  // position is usually blank — the row's value is marking where one power-on
  // ends and the next begins when reading the log back.
  if (g_logReady)
    storage_append_mark("#BOOT", "boot", g_gps.hasFix ? g_gps.lat : 0.0,
                        g_gps.hasFix ? g_gps.lon : 0.0,
                        g_gps.hasFix ? g_gps.utc : 0, g_gps.sats, g_gps.hdop);

  audio_boot();

  // Independent hardware watchdog (STM32H7 IWDG).  If the UI ever wedges again
  // (a display flush that never returns takes the whole loop with it), the chip
  // resets itself after WATCHDOG_MS instead of stranding the detector mid-drive.
  // Safe to lose a reboot now: the CSV is committed per row and the device table
  // is reloaded from /fs/fy_table.bin on boot, so counts and log survive.
  mbed::Watchdog::get_instance().start(WATCHDOG_MS);

  // Software hang detector — fires before the hardware IWDG so it can record
  // WHY the board rebooted (see hangWatchFn).
  g_lastLoopMs = millis();
  s_hangWatch.start(hangWatchFn);
}

// Simple USB-serial console:  t = status,  d = dump the CSV log,
// c = clear touch calibration,  x = toggle raw touch echo,  n = NAV demo soak,
// b = cycle backlight brightness,  i = list /fs + backlight record,  L = visible backlight pin test,
// f = step backlight PWM frequency,  s = brightness sweep,  m = measure pad,
// W = test hang.
static void serialCmdTick()
{
  if (!Serial.available()) return;
  int c = Serial.read();
  if (c == 't' || c == 'T')
  {
    Serial.println("=== GIGA status ===");
    Serial.print("QSPI log   : "); Serial.println(g_logReady ? "READY" : "DISABLED (needs one-time QSPIFormat)");
    ui_fit_report();
    Serial.print("backlight  : "); Serial.print(backlight_get());
    Serial.print("% "); Serial.print(backlight_label(backlight_get()));
    Serial.print("  step="); Serial.print((unsigned long)backlight_step_us());
    Serial.print("us pwm="); Serial.print(1000000UL / (backlight_step_us() * 20));
    Serial.print("Hz isr="); Serial.println((unsigned long)backlight_isr_count());
    Serial.print("unique dev : "); Serial.println(g_devCount);
    Serial.print("events log : "); Serial.println((unsigned long)g_totalEvents);
    Serial.print("ESP32 link : "); Serial.println(g_link.everSeen ? "seen" : "never");
    Serial.print("GPS        : ");
    if (g_gps.hasFix)
    {
      Serial.print("FIX sats="); Serial.print((unsigned long)g_gps.sats);
      Serial.print(" "); Serial.print(g_gps.lat, 5);
      Serial.print(","); Serial.println(g_gps.lon, 5);
    }
    else
    {
      Serial.print("no fix (sats="); Serial.print((unsigned long)g_gps.sats); Serial.println(")");
    }
  }
  else if (c == 'd' || c == 'D')
  {
    storage_dump_csv();
  }
  else if (c == 'c' || c == 'C')
  {
    // Rescue path: wipe a bad touch calibration and go back to identity.
    storage_clear_touchcal();
    TouchCal id = { 1.0f, 0.0f, 1.0f, 0.0f };
    ui_set_touchcal(id);
    Serial.println("[giga] touch calibration cleared (identity mapping)");
  }
  else if (c == 'x' || c == 'X')
  {
    // Diagnostic: echo raw GT911 coordinates so the real panel range is visible.
    static bool dbg = false;
    dbg = !dbg;
    ui_touch_debug(dbg);
    Serial.println(dbg ? "[giga] raw touch echo ON (tap the screen)" : "[giga] raw touch echo OFF");
  }
  else if (c == 'b' || c == 'B')
  {
    // Same cycle the STATS button walks, reachable without touching the screen.
    uint8_t pct = backlight_cycle();
    ui_backlight_refresh();
    bool saved = (g_logReady && storage_save_backlight(pct, backlight_step_us()));
    Serial.print("[giga] backlight "); Serial.print(pct);
    Serial.print("% "); Serial.print(backlight_label(pct));
    Serial.println(saved ? "  (saved)" : "  (NOT saved)");
  }
  else if (c == 'i' || c == 'I')
  {
    storage_list_files();
  }
  else if (c == 's' || c == 'S')
  {
    Serial.println("[giga] brightness sweep: 17 steps, ~1.8 s each, ~30 s total.");
    Serial.println("[giga] watch the screen; note where it stops getting darker.");
    backlight_sweep_begin();
  }
  else if (c == 'm' || c == 'M')
  {
    // Configured level vs what the pad is really doing.
    uint8_t want = backlight_get();
    uint8_t got  = backlight_measure_duty();
    Serial.print("[giga] level "); Serial.print(want);
    Serial.print("%  pad measured "); Serial.print(got);
    Serial.print("% high  step="); Serial.print((unsigned long)backlight_step_us());
    Serial.print("us isr="); Serial.println((unsigned long)backlight_isr_count());
  }
  else if (c == 'L')
  {
    // Visible proof of whether this pin drives the backlight at all.
    Serial.println("[giga] backlight pin test: watch the screen for ~5 s");
    backlight_test_begin();
  }
  else if (c == 'f' || c == 'F')
  {
    // Step the PWM period. If the interrupt is firing but nothing dims, the
    // backlight converter is likely ignoring a short off-phase; slowing it down
    // lengthens that phase until the panel actually responds.
    uint32_t cur = backlight_step_us();
    uint32_t next = (cur >= 4000) ? 250 : cur * 2;
    backlight_set_step_us(next);
    // Persisted: a frequency that actually dims this panel is worth keeping.
    bool fsaved = (g_logReady && storage_save_backlight(backlight_get(), next));
    Serial.print("[giga] backlight PWM step "); Serial.print((unsigned long)next);
    Serial.print(" us = "); Serial.print(1000000UL / (next * 20));
    Serial.println(fsaved ? " Hz (saved)" : " Hz (NOT saved)");
  }
  else if (c == 'n' || c == 'N')
  {
    static bool demo = false;
    demo = !demo;
    ui_nav_demo(demo);
    Serial.println(demo ? "[giga] NAV demo ON: compass spinning (soak test)" : "[giga] NAV demo OFF");
  }
  else if (c == 'W')
  {
    // Diagnostic: deliberately wedge the loop so the watchdog fires. Used to
    // prove the reset-cause plumbing actually reports WATCHDOG (vs "unknown"),
    // so a mid-drive reboot in the log can be read as "it hung" or "it lost
    // power". Uppercase only — hard to hit by accident.
    Serial.println("[giga] hanging on purpose; watchdog should reset in ~8s...");
    Serial.flush();
    for (;;) { /* no kick -> IWDG expires */ }
  }
}

// Why did the board start?  Distinguishes a normal ignition/power cycle from a
// WATCHDOG reset — i.e. tells us whether the UI is still hanging and silently
// self-recovering.  Recorded in the #BOOT row of the log.
// mbed::ResetReason::get() returns UNKNOWN on this board even after a verified
// watchdog reset, so read the STM32H7 reset-status register ourselves. Captured
// at the very top of setup() (before anything can clear it) and then cleared via
// RMVF so the NEXT boot reports its own cause instead of a stale accumulation.
// Watches main-loop liveness; on a stall, records it and reboots (see the note
// on the thread declaration for why this exists instead of a reset-cause read).
static void hangWatchFn()
{
  for (;;)
  {
    rtos::ThisThread::sleep_for(500);
    uint32_t last = g_lastLoopMs;
    if (last == 0) continue;                       // loop hasn't started yet
    if (millis() - last < HANG_DETECT_MS) continue;

    // Main loop is wedged. It is not touching the filesystem while stuck in the
    // display driver, and mbed's FATFileSystem takes its own lock, so recording
    // the event here is safe.
    if (g_logReady)
      storage_append_mark("#HANG", "ui-stall",
                          g_gps.hasFix ? g_gps.lat : 0.0,
                          g_gps.hasFix ? g_gps.lon : 0.0,
                          g_gps.hasFix ? g_gps.utc : 0,
                          g_gps.sats, g_gps.hdop);
    NVIC_SystemReset();
  }
}

// Breadcrumb track: one position row per TRACK_INTERVAL_MS while we have a fix
// and are actually moving.  Parked (or indoors with no fix) it writes nothing,
// so the log doesn't fill with a stationary device repeating itself.
static void trackTick(uint32_t now)
{
  static uint32_t lastTrackMs = 0;
  static double   lastLat = 0.0, lastLon = 0.0;
  static bool     haveLast = false;

  if (!g_logReady || !g_gps.hasFix) return;
  if (now - lastTrackMs < TRACK_INTERVAL_MS) return;

  if (haveLast && metresBetween(lastLat, lastLon, g_gps.lat, g_gps.lon) < TRACK_MIN_MOVE_M)
  {
    lastTrackMs = now;      // still parked — re-arm without writing a row
    return;
  }

  storage_append_mark("#TRK", "track", g_gps.lat, g_gps.lon, g_gps.utc, g_gps.sats, g_gps.hdop);
  lastLat = g_gps.lat;
  lastLon = g_gps.lon;
  haveLast = true;
  lastTrackMs = now;
}

// Periodic table checkpoint so re-sighting count growth survives a power cut
// (new unique devices are saved immediately in handleDet).
static void tableSaveTick(uint32_t now)
{
  static uint32_t lastSave = 0;
  if (!g_logReady || g_devCount == 0) return;
  if (now - lastSave < 60000UL) return;
  lastSave = now;
  storage_save_table();
}

void loop()
{
  uint32_t now = millis();
  g_lastLoopMs = now;   // liveness stamp for the hang detector

  pollEsp32();          // parse ESP32 detection/status JSON
  pollGps();            // parse NMEA, refresh fix state
  ledTick(now);         // clear the class-color flash after LED_FLASH_MS
  if (backlight_test_tick(now))  ui_backlight_refresh();
  if (backlight_sweep_tick(now)) ui_backlight_refresh();
  ui_tick(now);         // refresh LVGL labels / LIVE list from global state
  serialCmdTick();      // USB-serial console (t=status, d=dump CSV)
  tableSaveTick(now);   // periodic device-table snapshot to QSPI
  trackTick(now);       // breadcrumb position row (proves the detector was alive)

  lv_timer_handler();   // LVGL rendering + input (Arduino_H7_Video drives the tick)

  mbed::Watchdog::get_instance().kick();   // loop is alive; defer the reset
  delay(3);
}
