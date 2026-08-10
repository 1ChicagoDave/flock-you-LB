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

// ---- touch coordinate remap (portrait GT911 -> landscape screen) ----
// Start here; adjust if taps are off.  For a panel physically rotated 90°:
//   screen_x = touch_y ; screen_y = (PANEL_W-1) - touch_x
#define TOUCH_SWAP_XY   1
#define TOUCH_INV_X     0
#define TOUCH_INV_Y     1
#define PANEL_NATIVE_W  480      // GT911 native width (portrait)
#define PANEL_NATIVE_H  800      // GT911 native height (portrait)

// ---- widget handles ----
static lv_obj_t *tabview;
static lv_obj_t *tabLive, *tabStats, *tabAlert, *tabHunter;

static lv_obj_t *liveList;              // flex container of row labels
static uint32_t  lastLiveRebuild = 0;
static uint32_t  lastLabelTick    = 0;

// STATS labels
static lv_obj_t *stUniq, *stEvents, *stChan, *stLink, *stPkts, *stGps, *stLog;
// ALERT labels
static lv_obj_t *alMac, *alMethod, *alRssi, *alAge, *alChGps;
// HUNTER
static lv_obj_t *hnMac, *hnRssiLbl, *hnBar;

// ---- small helpers ----
static lv_color_t hx(uint32_t rgb) { return lv_color_hex(rgb); }

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
  lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);

  lv_obj_t *v = lv_label_create(row);
  lv_label_set_recolor(v, true);
  lv_label_set_text(v, "-");
  lv_obj_set_style_text_color(v, hx(COL_TEXT), 0);
  lv_obj_set_style_text_font(v, &lv_font_montserrat_14, 0);
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

  stUniq   = makeStatRow(col, "Unique devices");
  stEvents = makeStatRow(col, "Total events");
  stChan   = makeStatRow(col, "Channel");
  stLink   = makeStatRow(col, "ESP32 link");
  stPkts   = makeStatRow(col, "Pkts / uniq");
  stGps    = makeStatRow(col, "GPS");
  stLog    = makeStatRow(col, "Log");
}

static void buildAlert()
{
  lv_obj_t *col = lv_obj_create(tabAlert);
  lv_obj_set_size(col, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_color(col, hx(COL_BG), 0);
  lv_obj_set_style_border_width(col, 0, 0);
  lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(col, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(col, 6, 0);

  alMethod = lv_label_create(col);
  lv_label_set_text(alMethod, "— no detections —");
  lv_obj_set_style_text_font(alMethod, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(alMethod, hx(COL_DIM), 0);

  alRssi = lv_label_create(col);
  lv_label_set_text(alRssi, "--");
  lv_obj_set_style_text_font(alRssi, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(alRssi, hx(COL_TEXT), 0);

  alMac = lv_label_create(col);
  lv_label_set_text(alMac, "--:--:--:--:--:--");
  lv_obj_set_style_text_font(alMac, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(alMac, hx(COL_TEXT), 0);

  alAge = lv_label_create(col);
  lv_label_set_text(alAge, "");
  lv_obj_set_style_text_font(alAge, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(alAge, hx(COL_DIM), 0);

  alChGps = lv_label_create(col);
  lv_label_set_text(alChGps, "");
  lv_obj_set_style_text_font(alChGps, &lv_font_montserrat_14, 0);
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
  lv_obj_set_style_pad_all(col, 10, 0);

  lv_obj_t *title = lv_label_create(col);
  lv_label_set_text(title, "STRONGEST TARGET");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(title, hx(COL_DIM), 0);

  hnMac = lv_label_create(col);
  lv_label_set_text(hnMac, "— none —");
  lv_obj_set_style_text_font(hnMac, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(hnMac, hx(COL_TEXT), 0);

  hnRssiLbl = lv_label_create(col);
  lv_label_set_text(hnRssiLbl, "-- dBm");
  lv_obj_set_style_text_font(hnRssiLbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(hnRssiLbl, hx(COL_TEXT), 0);

  hnBar = lv_bar_create(col);
  lv_obj_set_size(hnBar, 640, 34);
  lv_bar_set_range(hnBar, 0, 100);
  lv_bar_set_value(hnBar, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(hnBar, hx(0x203040), LV_PART_MAIN);
  lv_obj_set_style_bg_color(hnBar, hx(COL_OK), LV_PART_INDICATOR);
}

void ui_init()
{
  // Global dark background on the active screen.
  lv_obj_set_style_bg_color(lv_scr_act(), hx(COL_BG), 0);

  // LVGL v9: create, then size the tab bar as a separate call.
  tabview = lv_tabview_create(lv_scr_act());
  lv_tabview_set_tab_bar_size(tabview, TABBAR_H);
  lv_obj_set_style_bg_color(tabview, hx(COL_BG), 0);

  tabLive   = lv_tabview_add_tab(tabview, "LIVE");
  tabStats  = lv_tabview_add_tab(tabview, "STATS");
  tabAlert  = lv_tabview_add_tab(tabview, "ALERT");
  tabHunter = lv_tabview_add_tab(tabview, "HUNTER");

  buildLive();
  buildStats();
  buildAlert();
  buildHunter();

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
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl, hx(COL_TEXT), 0);
    // MAC in default color, method token recolored to its class color.
    // Pre-format with the C library snprintf (LVGL's mini-printf is unreliable
    // for %06lX), then hand the finished recolor string to the label.
    char row[128];
    snprintf(row, sizeof(row), "%s  #%06lX %s#  %ddBm  x%u",
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
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
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
    lv_label_set_text_fmt(hnRssiLbl, "%d dBm", (int)d.rssi);
    lv_obj_set_style_text_color(hnRssiLbl, hx(methodColorHex(d.method)), 0);
    // Map -95..-35 dBm -> 0..100.
    int v = (int)((d.rssi + 95) * 100 / 60);
    if (v < 0) v = 0; if (v > 100) v = 100;
    lv_bar_set_value(hnBar, v, LV_ANIM_ON);
  }
  else
  {
    lv_label_set_text(hnMac, "— none —");
    lv_label_set_text(hnRssiLbl, "-- dBm");
    lv_bar_set_value(hnBar, 0, LV_ANIM_OFF);
  }

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
