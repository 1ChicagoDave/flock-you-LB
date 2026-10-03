// ============================================================================
// backlight.cpp — 200 Hz software dimmer on the display backlight pin
// ============================================================================
#include <Arduino.h>
#include "mbed.h"

#include "backlight.h"
#include "config.h"

static const uint8_t  BL_PRESETS[] = BL_PRESET_LIST;
static const char    *BL_LABELS[]  = BL_LABEL_LIST;
static const uint8_t  BL_N = (uint8_t)(sizeof(BL_PRESETS) / sizeof(BL_PRESETS[0]));

// 20 steps x 250 us = 5 ms period = 200 Hz, 5% granularity. Well above the
// frequency at which backlight dimming reads as flicker in peripheral vision,
// which is the whole point of not reusing the stock 50 Hz class.
#define BL_STEPS    20
#define BL_STEP_US  250

static mbed::DigitalOut *s_pin  = nullptr;
static mbed::Ticker     *s_tick = nullptr;
static volatile uint8_t  s_duty = BL_STEPS;   // steps lit per period, 0..BL_STEPS
static volatile uint8_t  s_step = 0;
static uint8_t           s_pct  = BL_DEFAULT_PCT;
static bool              s_running = false;

// One register write and a counter bump. This fires 4000 times a second next to
// the LTDC flush, the UART reads and LVGL, so it must stay trivial and must
// never touch anything that can block or allocate.
static void bl_isr()
{
  *s_pin = (s_step < s_duty) ? 1 : 0;
  if (++s_step >= BL_STEPS) s_step = 0;
}

void backlight_begin(uint8_t pct)
{
  if (s_pin == nullptr)  s_pin  = new mbed::DigitalOut(PB_12);
  if (s_tick == nullptr) s_tick = new mbed::Ticker();
  backlight_set(pct);
}

void backlight_set(uint8_t pct)
{
  if (pct > 100) pct = 100;
  s_pct = pct;
  if (s_pin == nullptr) return;              // backlight_begin not called yet

  // Round to the nearest whole step so e.g. 55% lands on 11 of 20 exactly.
  uint8_t d = (uint8_t)(((uint16_t)pct * BL_STEPS + 50) / 100);

  // Fully on or fully off needs no modulation: hold the pin and stop the
  // interrupt rather than PWM a constant value forever.
  if (d == 0 || d >= BL_STEPS)
  {
    if (s_running) { s_tick->detach(); s_running = false; }
    s_duty = d;
    *s_pin = (d >= BL_STEPS) ? 1 : 0;
    return;
  }

  s_duty = d;
  if (!s_running)
  {
    s_step = 0;
    s_tick->attach(mbed::callback(bl_isr), std::chrono::microseconds(BL_STEP_US));
    s_running = true;
  }
}

uint8_t backlight_get()
{
  return s_pct;
}

uint8_t backlight_cycle()
{
  uint8_t i = 0;
  while (i < BL_N && BL_PRESETS[i] != s_pct) i++;
  // Current level not in the list (serial override): restart at the first entry.
  uint8_t next = (i >= BL_N) ? 0 : (uint8_t)((i + 1) % BL_N);
  backlight_set(BL_PRESETS[next]);
  return s_pct;
}

const char *backlight_label(uint8_t pct)
{
  for (uint8_t i = 0; i < BL_N; i++)
    if (BL_PRESETS[i] == pct) return BL_LABELS[i];
  return "custom";
}
