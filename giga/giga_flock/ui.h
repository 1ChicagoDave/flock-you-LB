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
