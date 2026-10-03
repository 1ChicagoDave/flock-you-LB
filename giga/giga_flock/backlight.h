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

// Re-assert the pin as a plain output and re-apply the current level. MUST be
// called after the display is initialised: bringing up the panel reconfigures
// GPIO banks and leaves PB_12 no longer driving as a GPIO output, so every write
// after that lands on a pad that ignores it. The symptom is brutal to diagnose
// from the firmware side, because the PWM interrupt keeps firing at the right
// rate and the level bookkeeping all reads correct while the panel stays at full
// brightness. Note the Arduino video library also configures this pin only at the
// very END of its init for the same reason.
void backlight_reassert();

// ---- diagnostics ----
// Measure what the PAD is actually doing, by sampling the GPIO input data
// register for ~50 ms and returning the percentage of samples found high. IDR
// reflects the real pad level regardless of how the pin is configured, so this
// separates "we wrote the right duty" from "the write reached the pin" from "the
// backlight ignored it". Blocks for 50 ms; well inside the 4 s hang watchdog.
uint8_t backlight_measure_duty();


// Times the PWM interrupt has fired. If this is not climbing while the level is
// between the extremes, the Ticker is not running and the pin is simply stuck at
// whatever it was last parked at, which looks exactly like "dimming does nothing".
uint32_t backlight_isr_count();

// Current PWM step period in microseconds, and the matching period setter.
// Runtime-adjustable because a backlight boost converter driven on its enable pin
// may not follow a fast square wave at all: the output capacitor holds up across
// a short off-phase and the panel never visibly dims. Lowering the frequency
// lengthens the off-phase until the converter actually drops out.
uint32_t backlight_step_us();
void backlight_set_step_us(uint32_t us);

// Characterisation sweep: walks duty from 100% down to 5% in uneven steps,
// holding each ~1.8 s and printing it, then restores the level it started from.
// Needed because perceived brightness on this panel is not monotonic in duty, so
// the only honest way to pick a ladder is to watch one. Non-blocking; drive from
// loop(). Returns true on the tick that finishes it.
void backlight_sweep_begin();
bool backlight_sweep_tick(uint32_t now);

// Slow visible pin test: parks the pin dark/lit in ~1.2 s phases so it is obvious
// to the eye whether this pin controls the backlight at all. Non-blocking, so it
// cannot trip the 4 s hang watchdog; drive it from loop(). Returns true on the
// tick that finishes the test, so the caller can refresh the UI.
void backlight_test_begin();
bool backlight_test_tick(uint32_t now);
bool backlight_test_active();
