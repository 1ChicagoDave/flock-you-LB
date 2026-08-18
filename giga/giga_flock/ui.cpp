// ============================================================
// ui.cpp — LVGL v8 UI for the GIGA Display Shield (800x480 landscape)
// ============================================================
//
// Follows Arduino's official "LVGL on the GIGA Display Shield" pattern:
//   Arduino_H7_Video Display(800, 480, GigaDisplayShield);  // in the .ino
//   Display.begin();          // <-- this calls lv_init() + registers the display
//   TouchDetector.begin();
//   ...build widgets on lv_scr_act()/tabview...
//   loop(): lv_timer_handler();
//
// We register the GT911 touch as an LVGL pointer input device here.
//
// LVGL MAJOR VERSION: this file targets **LVGL v9** (verified against lvgl
// 9.5.0).  It was originally written for v8; the v8->v9 deltas that mattered
// here were exactly three:
//   1. lv_indev_drv_t / lv_indev_drv_init / lv_indev_drv_register  ->
//      lv_indev_create() + lv_indev_set_type() + lv_indev_set_read_cb().
//   2. The read callback's first arg is lv_indev_t*, not lv_indev_drv_t*.
//   3. lv_tabview_create(parent, dir, size) -> lv_tabview_create(parent)
//      followed by lv_tabview_set_tab_bar_size(tv, size).
// LV_PART/LV_STATE names, the flex API, and lv_label_set_recolor are unchanged.
//
// FONTS: this file only uses lv_font_montserrat_14.  That font is enabled by
// the lv_conf the core supplies (see the lv_conf note in giga_flock.ino).  If
// you switch any readout to montserrat_20/28/48 you must enable those there
// first, or the build will fail at link time with an undefined reference.
//  * TOUCH ORIENTATION.  GT911 native frame is portrait (480x800).  The remap in
//    touch_read_cb() converts to our 800x480 landscape; flip TOUCH_* below if
//    taps land rotated/mirrored on your unit.

#include "ui.h"
#include "config.h"
#include "flock_types.h"

#include "lvgl.h"
#include "Arduino_GigaDisplayTouch.h"

// The touch object is created in the .ino; we just reference it.
extern Arduino_GigaDisplayTouch TouchDetector;

// ---- touch coordinate remap ----
// On this unit the GT911 already reports landscape-aligned coords (x:0..799
// right, y:0..479 down), so the mapping is identity.  The earlier swap+invert
// rotated every gesture 90° (horizontal swipes scrolled, vertical swipes changed
// tabs).  If a single axis ends up MIRRORED (taps land, but left/right or
// up/down is reversed), flip just that INV_* bit — do NOT re-enable the swap.
#define TOUCH_SWAP_XY   0
#define TOUCH_INV_X     0
#define TOUCH_INV_Y     0
#define PANEL_NATIVE_W  480      // GT911 native width (portrait)
#define PANEL_NATIVE_H  800      // GT911 native height (portrait)

// ---- widget handles ----
static lv_obj_t *tabview;
static lv_obj_t *tabLive, *tabStats, *tabAlert, *tabHunter;

static lv_obj_t *liveList;              // flex container of row labels
static uint32_t  lastLiveRebuild = 0;
static uint32_t  lastLabelTick    = 0;

// STATS labels
static lv_obj_t *stUniq, *stEvents, *stSessUniq, *stSessEvents,
                *stChan, *stLink, *stPkts, *stGps, *stLog;
// ALERT labels
static lv_obj_t *alMac, *alMethod, *alRssi, *alAge, *alChGps, *alBadge;
// HUNTER
static lv_obj_t *hnMac, *hnRssiLbl, *hnArc;
// Always-visible status LEDs (top-right overlay)
static lv_obj_t *ledLink, *ledGps;
// Detection flash: instead of a full-screen overlay we TINT the ALERT tab's own
// background to the detection-class color and jump to that tab.  The old
// approach put an opaque 800x480 object on lv_layer_top, which forced a
// full-screen repaint on every detection and reliably hung the display mid-paint
// (symptom: frozen device, tab bar half-covered).  Tinting an existing pane
// repaints only that pane — same look, a fraction of the draw work.
static lv_obj_t *alertPane;             // the ALERT tab's content container
static uint32_t  alertUntil = 0;        // hold the tint until this millis()
static int32_t   alertPrevTab = -1;     // tab to restore when the alert expires

// ---- small helpers ----
static lv_color_t hx(uint32_t rgb) { return lv_color_hex(rgb); }

// Scale an 0xRRGGBB color to `pct` percent brightness (for a readable tinted bg).
static uint32_t dimColor(uint32_t rgb, uint8_t pct)
{
  uint32_t r = ((rgb >> 16) & 0xFF) * pct / 100;
  uint32_t g = ((rgb >> 8)  & 0xFF) * pct / 100;
  uint32_t b = ( rgb        & 0xFF) * pct / 100;
  return (r << 16) | (g << 8) | b;
}

// GT911 point struct name in Arduino_GigaDisplayTouch is GDTpoint_t.
static void touch_read_cb(lv_indev_t *drv, lv_indev_data_t *data)
{
  (void)drv;
  GDTpoint_t pts[5];
  uint8_t n = TouchDetector.getTouchPoints(pts);
  if (n > 0)
  {
    int rx = pts[0].x;
    int ry = pts[0].y;
    int sx, sy;
#if TOUCH_SWAP_XY
    sx = ry; sy = rx;
#else
    sx = rx; sy = ry;
#endif
#if TOUCH_INV_X
    sx = (SCREEN_W - 1) - sx;
#endif
#if TOUCH_INV_Y
    sy = (SCREEN_H - 1) - sy;
#endif
    if (sx < 0) sx = 0; if (sx >= SCREEN_W) sx = SCREEN_W - 1;
    if (sy < 0) sy = 0; if (sy >= SCREEN_H) sy = SCREEN_H - 1;
    data->point.x = sx;
    data->point.y = sy;
    data->state   = LV_INDEV_STATE_PRESSED;
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void register_touch()
{
  // LVGL v9 input-device registration (see the v8->v9 note at the top).
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touch_read_cb);
}

// A titled value label pair packed into a parent; returns the value label.
static lv_obj_t *makeStatRow(lv_obj_t *parent, const char *title)
{
  lv_obj_t *row = lv_obj_create(parent);
  lv_obj_set_width(row, LV_PCT(100));
  lv_obj_set_height(row, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(row, 0, 0);
  lv_obj_set_style_pad_top(row, 2, 0);
  lv_obj_set_style_pad_bottom(row, 2, 0);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);

  lv_obj_t *t = lv_label_create(row);
  lv_label_set_text(t, title);
  lv_obj_set_width(t, 190);
  lv_obj_set_style_text_color(t, hx(COL_DIM), 0);
  lv_obj_set_style_text_font(t, &lv_font_montserrat_20, 0);

  lv_obj_t *v = lv_label_create(row);
  lv_label_set_recolor(v, true);
  lv_label_set_text(v, "-");
  lv_obj_set_style_text_color(v, hx(COL_TEXT), 0);
  lv_obj_set_style_text_font(v, &lv_font_montserrat_20, 0);
  return v;
}

static void buildLive()
{
  liveList = lv_obj_create(tabLive);
  lv_obj_set_size(liveList, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_color(liveList, hx(COL_BG), 0);
  lv_obj_set_style_border_width(liveList, 0, 0);
  lv_obj_set_style_pad_all(liveList, 6, 0);
  lv_obj_set_flex_flow(liveList, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_scroll_dir(liveList, LV_DIR_VER);
}

static void buildStats()
{
  lv_obj_t *col = lv_obj_create(tabStats);
  lv_obj_set_size(col, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_color(col, hx(COL_BG), 0);
  lv_obj_set_style_border_width(col, 0, 0);
  lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_all(col, 8, 0);

  stUniq       = makeStatRow(col, "Unique devices");
  stEvents     = makeStatRow(col, "Total events");
  stSessUniq   = makeStatRow(col, "Unique (this drive)");
  stSessEvents = makeStatRow(col, "Events (this drive)");
  stChan       = makeStatRow(col, "Channel");
  stLink   = makeStatRow(col, "ESP32 link");
  stPkts   = makeStatRow(col, "Pkts / uniq");
  stGps    = makeStatRow(col, "GPS");
  stLog    = makeStatRow(col, "Log");
}

static void buildAlert()
{
  lv_obj_t *col = lv_obj_create(tabAlert);
  alertPane = col;                       // tinted to the class color on detection
  lv_obj_set_size(col, LV_PCT(100), LV_PCT(100));
  lv_obj_remove_flag(col, LV_OBJ_FLAG_SCROLLABLE);   // fixed content, never scroll
  lv_obj_set_style_bg_color(col, hx(COL_BG), 0);
  lv_obj_set_style_border_width(col, 0, 0);
  lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(col, 6, 0);

  // NEW / KNOWN badge — the headline read while driving. The hit count is in the
  // detail line but is too small to parse at speed; this is a single word.
  alBadge = lv_label_create(col);
  lv_label_set_text(alBadge, "");
  lv_obj_set_style_text_font(alBadge, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(alBadge, hx(COL_TEXT), 0);

  alMethod = lv_label_create(col);
  lv_label_set_text(alMethod, "— no detections —");
  lv_obj_set_style_text_font(alMethod, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(alMethod, hx(COL_DIM), 0);

  alRssi = lv_label_create(col);
  lv_label_set_text(alRssi, "--");
  lv_obj_set_style_text_font(alRssi, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(alRssi, hx(COL_TEXT), 0);

  alMac = lv_label_create(col);
  lv_label_set_text(alMac, "--:--:--:--:--:--");
  lv_obj_set_style_text_font(alMac, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(alMac, hx(COL_TEXT), 0);

  alAge = lv_label_create(col);
  lv_label_set_text(alAge, "");
  lv_obj_set_style_text_font(alAge, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(alAge, hx(COL_DIM), 0);

  alChGps = lv_label_create(col);
  lv_label_set_text(alChGps, "");
  lv_obj_set_style_text_font(alChGps, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(alChGps, hx(COL_DIM), 0);
}

static void buildHunter()
{
  lv_obj_t *col = lv_obj_create(tabHunter);
  lv_obj_set_size(col, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_color(col, hx(COL_BG), 0);
  lv_obj_set_style_border_width(col, 0, 0);
  lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(col, 6, 0);
  lv_obj_set_style_pad_row(col, 8, 0);
  // Fixed-content screen: must fit the tab area with no scrolling.  A scrollable
  // HUNTER let the readout be dragged off-screen while driving, and scroll
  // momentum on the 300px arc is a large repaint (the class of draw work behind
  // the display hangs).  Content is sized below to fit 480 - TABBAR_H.
  lv_obj_remove_flag(col, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *title = lv_label_create(col);
  lv_label_set_text(title, "STRONGEST TARGET");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(title, hx(COL_DIM), 0);

  hnMac = lv_label_create(col);
  lv_label_set_text(hnMac, "— none —");
  lv_obj_set_style_text_font(hnMac, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(hnMac, hx(COL_TEXT), 0);

  // Radial signal-strength gauge.  270° sweep; the big dBm readout lives in the
  // middle.  Display-only (not draggable), knob hidden.  The indicator color and
  // the center number shift green -> amber -> red as the target gets closer.
  hnArc = lv_arc_create(col);
  lv_obj_set_size(hnArc, HUNTER_ARC_D, HUNTER_ARC_D);
  lv_arc_set_rotation(hnArc, 135);
  lv_arc_set_bg_angles(hnArc, 0, 270);
  lv_arc_set_range(hnArc, 0, 100);
  lv_arc_set_value(hnArc, 0);
  lv_obj_remove_flag(hnArc, LV_OBJ_FLAG_CLICKABLE);   // display-only
  lv_obj_set_style_arc_width(hnArc, 22, LV_PART_MAIN);
  lv_obj_set_style_arc_color(hnArc, hx(0x203040), LV_PART_MAIN);
  lv_obj_set_style_arc_width(hnArc, 22, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(hnArc, hx(COL_OK), LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(hnArc, LV_OPA_TRANSP, LV_PART_KNOB);  // hide knob

  hnRssiLbl = lv_label_create(hnArc);
  lv_label_set_text(hnRssiLbl, "--");
  lv_obj_set_style_text_font(hnRssiLbl, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(hnRssiLbl, hx(COL_TEXT), 0);
  lv_obj_center(hnRssiLbl);
}

void ui_init()
{
  // Global dark background on the active screen.
  lv_obj_set_style_bg_color(lv_scr_act(), hx(COL_BG), 0);

  // LVGL v9: create, then size the tab bar as a separate call.
  tabview = lv_tabview_create(lv_scr_act());
  lv_tabview_set_tab_bar_size(tabview, TABBAR_H);
  lv_obj_set_style_bg_color(tabview, hx(COL_BG), 0);
  // Bigger tab labels -> easier touch targets.
  lv_obj_set_style_text_font(lv_tabview_get_tab_bar(tabview), &lv_font_montserrat_20, 0);

  tabLive   = lv_tabview_add_tab(tabview, "LIVE");
  tabStats  = lv_tabview_add_tab(tabview, "STATS");
  tabAlert  = lv_tabview_add_tab(tabview, "ALERT");
  tabHunter = lv_tabview_add_tab(tabview, "HUNTER");

  buildLive();
  buildStats();
  buildAlert();
  buildHunter();

  // Always-visible status dots on the top layer (render above every tab). Made
  // non-clickable so taps fall through to the tab bar underneath.
  //   ledLink : green = ESP32 link alive, red = no link
  //   ledGps  : blue  = GPS fix, dim = no fix
  ledGps = lv_led_create(lv_layer_top());
  lv_obj_set_size(ledGps, 14, 14);
  lv_obj_align(ledGps, LV_ALIGN_TOP_RIGHT, -30, TABBAR_H + 8);
  lv_obj_remove_flag(ledGps, LV_OBJ_FLAG_CLICKABLE);
  lv_led_set_color(ledGps, hx(COL_ADDR1));
  lv_led_off(ledGps);

  ledLink = lv_led_create(lv_layer_top());
  lv_obj_set_size(ledLink, 14, 14);
  lv_obj_align(ledLink, LV_ALIGN_TOP_RIGHT, -8, TABBAR_H + 8);
  lv_obj_remove_flag(ledLink, LV_OBJ_FLAG_CLICKABLE);
  lv_led_set_color(ledLink, hx(COL_BAD));
  lv_led_on(ledLink);

  register_touch();
}

// ---- refresh helpers ----------------------------------------------------

static void rebuildLiveList(uint32_t now)
{
  lv_obj_clean(liveList);

  // Show most-recent first: pick the top LIVE_MAX_ROWS by lastSeenMs.
  // Simple selection (device count is small).
  bool used[MAX_DEVICES] = {false};
  int rows = 0;
  while (rows < LIVE_MAX_ROWS)
  {
    int best = -1;
    uint32_t bestMs = 0;
    for (int i = 0; i < g_devCount; i++)
    {
      if (used[i]) continue;
      if (best < 0 || g_dev[i].lastSeenMs >= bestMs)
      {
        best = i;
        bestMs = g_dev[i].lastSeenMs;
      }
    }
    if (best < 0) break;
    used[best] = true;

    DeviceEntry &d = g_dev[best];
    lv_obj_t *lbl = lv_label_create(liveList);
    lv_label_set_recolor(lbl, true);
    lv_obj_set_width(lbl, LV_PCT(100));
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl, hx(COL_TEXT), 0);
    // MAC in default color, method token recolored to its class color.
    // Pre-format with the C library snprintf (LVGL's mini-printf is unreliable
    // for %06lX), then hand the finished recolor string to the label.
    // Leading NEW/KNOWN tag, color-coded — readable at a glance while driving.
    bool isNewDev = g_devNew[best];
    char row[176];
    snprintf(row, sizeof(row), "#%06lX %-5s#  %s  #%06lX %s#  %ddBm  x%u",
             (unsigned long)((isNewDev ? COL_OK : COL_DIM) & 0xFFFFFF),
             isNewDev ? "NEW" : "KNOWN",
             d.mac, (unsigned long)(methodColorHex(d.method) & 0xFFFFFF),
             d.method, (int)d.rssi, (unsigned)d.count);
    lv_label_set_text(lbl, row);
    rows++;
  }

  if (rows == 0)
  {
    lv_obj_t *lbl = lv_label_create(liveList);
    lv_label_set_text(lbl, "waiting for detections...");
    lv_obj_set_style_text_color(lbl, hx(COL_DIM), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
  }
  lastLiveRebuild = now;
}

static int strongestTarget(uint32_t now)
{
  // Strongest RSSI among devices seen in the last 8 s.
  int best = -1;
  int bestR = -128;
  for (int i = 0; i < g_devCount; i++)
  {
    if (now - g_dev[i].lastSeenMs > 8000) continue;
    if (g_dev[i].rssi > bestR) { bestR = g_dev[i].rssi; best = i; }
  }
  return best;
}

static void refreshLabels(uint32_t now)
{
  char buf[96];

  // ---- STATS ----
  lv_label_set_text_fmt(stUniq,   "%d", g_devCount);
  lv_label_set_text_fmt(stEvents, "%lu", (unsigned long)g_totalEvents);
  lv_label_set_text_fmt(stSessUniq,   "%lu", (unsigned long)g_sessUniq);
  lv_label_set_text_fmt(stSessEvents, "%lu", (unsigned long)g_sessEvents);
  lv_label_set_text_fmt(stChan,   "%u", (unsigned)g_link.channel);

  bool alive = g_link.everSeen && (now - g_link.lastStatusMs < LINK_ALIVE_MS);
  snprintf(buf, sizeof(buf), "#%06lX %s#",
           (unsigned long)(alive ? COL_OK : COL_BAD), alive ? "ALIVE" : "no link");
  lv_label_set_text(stLink, buf);

  lv_label_set_text_fmt(stPkts, "%lu / %d",
                        (unsigned long)g_link.pkts, g_link.uniq);

  if (g_gps.hasFix)
    snprintf(buf, sizeof(buf), "#%06lX fix# %lus  %.5f,%.5f",
             (unsigned long)COL_OK, (unsigned long)g_gps.sats, g_gps.lat, g_gps.lon);
  else
    snprintf(buf, sizeof(buf), "#%06lX no fix# sats %lu",
             (unsigned long)COL_DIM, (unsigned long)g_gps.sats);
  lv_label_set_text(stGps, buf);

  if (g_logReady)
    snprintf(buf, sizeof(buf), "#%06lX ok# %s", (unsigned long)COL_OK, LOG_PATH);
  else
    snprintf(buf, sizeof(buf), "#%06lX off# (unformatted QSPI?)", (unsigned long)COL_BAD);
  lv_label_set_text(stLog, buf);

  // ---- ALERT ----
  if (g_lastDevIdx >= 0 && g_lastDevIdx < g_devCount)
  {
    DeviceEntry &d = g_dev[g_lastDevIdx];
    uint32_t c = methodColorHex(d.method);

    bool isNewDev = g_devNew[g_lastDevIdx];
    lv_label_set_text(alBadge, isNewDev ? "NEW CAMERA" : "KNOWN");
    lv_obj_set_style_text_color(alBadge, hx(isNewDev ? COL_OK : COL_DIM), 0);

    snprintf(buf, sizeof(buf), "#%06lX %s#", (unsigned long)c, d.method);
    lv_label_set_text(alMethod, buf);
    lv_obj_set_style_text_color(alMethod, hx(c), 0);
    lv_label_set_recolor(alMethod, true);

    lv_label_set_text_fmt(alRssi, "%d dBm", (int)d.rssi);
    lv_obj_set_style_text_color(alRssi, hx(c), 0);
    lv_label_set_text(alMac, d.mac);

    uint32_t ageS = (now - g_lastDetMs) / 1000;
    lv_label_set_text_fmt(alAge, "%lus ago   count %u", (unsigned long)ageS, (unsigned)d.count);

    if (g_gps.hasFix)
      snprintf(buf, sizeof(buf), "ch %u   %.5f, %.5f",
               (unsigned)d.channel, g_gps.lat, g_gps.lon);
    else
      snprintf(buf, sizeof(buf), "ch %u   (no GPS fix)", (unsigned)d.channel);
    lv_label_set_text(alChGps, buf);
  }

  // ---- HUNTER ----
  int s = strongestTarget(now);
  if (s >= 0)
  {
    DeviceEntry &d = g_dev[s];
    lv_label_set_text(hnMac, d.mac);
    lv_label_set_text_fmt(hnRssiLbl, "%d", (int)d.rssi);   // dBm; gauge shows strength
    // Map -95..-35 dBm -> 0..100.
    int v = (int)((d.rssi + 95) * 100 / 60);
    if (v < 0) v = 0; if (v > 100) v = 100;
    lv_arc_set_value(hnArc, v);
    // Getting-warmer color: cool green when far/weak, hot red when close/strong.
    uint32_t ac = (v >= 66) ? COL_BAD : (v >= 33) ? COL_ADDR2 : COL_OK;
    lv_obj_set_style_arc_color(hnArc, hx(ac), LV_PART_INDICATOR);
    lv_obj_set_style_text_color(hnRssiLbl, hx(ac), 0);
  }
  else
  {
    lv_label_set_text(hnMac, "— none —");
    lv_label_set_text(hnRssiLbl, "--");
    lv_arc_set_value(hnArc, 0);
    lv_obj_set_style_text_color(hnRssiLbl, hx(COL_TEXT), 0);
  }

  // ---- detection flash (tint the ALERT pane, like the RGB LED) ----
  // Re-arm whenever a new sighting lands (g_lastDetMs bumps on every hit); hold
  // the tint >=5 s past the latest sighting so it stays while the camera is in
  // range and lingers long enough to read.  On the rising edge (no alert already
  // showing) jump to the ALERT tab so the detail is on screen; re-sightings only
  // extend the hold, so we never yank the user around mid-look.
  static uint32_t shownDetMs = 0;
  if (g_lastDetMs != shownDetMs && g_lastDevIdx >= 0 && g_lastDevIdx < g_devCount)
  {
    bool rising = (alertUntil == 0);
    shownDetMs = g_lastDetMs;
    DeviceEntry &d = g_dev[g_lastDevIdx];
    lv_obj_set_style_bg_color(alertPane, hx(dimColor(methodColorHex(d.method), 40)), 0);
    if (rising)
    {
      // Remember where the user was so we can put them back afterwards.
      alertPrevTab = (int32_t)lv_tabview_get_tab_active(tabview);
      lv_tabview_set_active(tabview, 2, LV_ANIM_OFF);   // 2 = ALERT
    }
    alertUntil = now + ALERT_HOLD_MS;
  }
  if (alertUntil != 0 && now >= alertUntil)
  {
    lv_obj_set_style_bg_color(alertPane, hx(COL_BG), 0);          // back to normal
    // Return to whatever screen was showing when the alert fired — but only if
    // the user hasn't already swiped somewhere themselves during the hold.
    if (alertPrevTab >= 0 && lv_tabview_get_tab_active(tabview) == 2)
      lv_tabview_set_active(tabview, (uint32_t)alertPrevTab, LV_ANIM_OFF);
    alertPrevTab = -1;
    alertUntil   = 0;
  }

  // ---- status LEDs (always visible) ----
  lv_led_set_color(ledLink, hx(alive ? COL_OK : COL_BAD));
  lv_led_on(ledLink);
  if (g_gps.hasFix) { lv_led_set_color(ledGps, hx(COL_ADDR1)); lv_led_on(ledGps); }
  else              { lv_led_off(ledGps); }

  lastLabelTick = now;
}

void ui_tick(uint32_t now)
{
  if (now - lastLabelTick >= UI_TICK_MS)
    refreshLabels(now);

  if (g_uiDirty && (now - lastLiveRebuild >= LIVE_REBUILD_MS))
  {
    g_uiDirty = false;
    rebuildLiveList(now);
  }
}
