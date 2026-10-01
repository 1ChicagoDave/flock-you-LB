// ============================================================
// ui.h — LVGL screens (LIVE / STATS / ALERT / HUNTER)
// ============================================================
#pragma once

#include <Arduino.h>

// Build all screens + the tab bar, and register the GT911 touch input device.
// Call AFTER Display.begin() and TouchDetector.begin() (LVGL is initialized by
// Arduino_H7_Video's Display.begin(), so do NOT call lv_init() yourself).
void ui_init();

// Refresh widget contents from the global state.  Call every loop; internally
// throttled (labels every UI_TICK_MS, LIVE list on change + LIVE_REBUILD_MS).
void ui_tick(uint32_t now);

// ---- touch calibration ----
// 2-point scale/offset applied to raw GT911 coordinates: screen = raw*s + o.
// Identity {1,0,1,0} until the user runs "Calibrate touch" on the STATS tab.
// Persisted by the .ino via storage_save/load_touchcal.
struct TouchCal { float sx, ox, sy, oy; };
void ui_set_touchcal(const TouchCal &c);

// Echo raw GT911 coordinates to USB serial on every press (diagnostic; serial 'x').
void ui_touch_debug(bool on);
