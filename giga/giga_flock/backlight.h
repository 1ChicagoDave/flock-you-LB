// ============================================================================
// backlight.h — display brightness control for night driving
// ============================================================================
// The GIGA Display Shield backlight is a plain GPIO enable on PB_12 (= D74).
// The core-bundled Arduino_H7_Video driver never touches that pin, so the panel
// comes up at full brightness from power-on and stays there — blinding at night.
//
// Arduino_GigaDisplay does ship a GigaDisplayBacklight class, but it software
// PWMs the pin from an mbed Ticker at 2 ms per 10% step: a 20 ms period, i.e.
// 50 Hz. A 50 Hz square wave on an LED backlight strobes visibly at low duty,
// which is precisely the setting wanted at night, and only 10 levels exist.
// This module does the same job at 200 Hz (20 steps of 250 us) and, at either
// extreme, parks the pin and detaches the interrupt entirely.
//
// Levels are whole percents, 0..100. Nothing above calls for 0: see config.h.
// ============================================================================
#pragma once

#include <stdint.h>

// Create the pin and apply an initial level. Safe to call once, early in setup.
void backlight_begin(uint8_t pct);

// Apply a level now. Clamped to 0..100. Cheap, no flash access.
void backlight_set(uint8_t pct);

// Current level.
uint8_t backlight_get();

// Step to the next preset in BL_PRESET_LIST and return the level landed on.
// The list wraps, so one tap at the dimmest setting goes straight back to full.
uint8_t backlight_cycle();

// Short human label for a level, e.g. "NIGHT". Never returns null.
const char *backlight_label(uint8_t pct);
