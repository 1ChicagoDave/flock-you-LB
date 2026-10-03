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

// 20 steps per cycle = 5% granularity. The step period, and so the PWM frequency,
// lives in config.h: see BL_STEP_US_DEFAULT. It is NOT a free choice - this
// backlight ignores a fast switching signal entirely.
// PB_12 as an Arduino pin number, for pinMode. Re-asserting the mode through the
// Arduino API rather than deleting and re-creating the mbed DigitalOut avoids any
// race against the PWM interrupt, which dereferences that object.
#define BL_PIN_D    74

#define BL_STEPS    20

static mbed::DigitalOut *s_pin  = nullptr;
static mbed::Ticker     *s_tick = nullptr;
static volatile uint8_t  s_duty = BL_STEPS;   // steps lit per period, 0..BL_STEPS
static volatile uint8_t  s_step = 0;
static uint8_t           s_pct  = BL_DEFAULT_PCT;
static bool              s_running = false;
static volatile uint32_t s_isrCount = 0;
static uint32_t          s_stepUs = BL_STEP_US_DEFAULT;

// One register write and a counter bump. This fires 4000 times a second next to
// the LTDC flush, the UART reads and LVGL, so it must stay trivial and must
// never touch anything that can block or allocate.
static void bl_isr()
{
  s_isrCount++;
  *s_pin = (s_step < s_duty) ? 1 : 0;
  if (++s_step >= BL_STEPS) s_step = 0;
}

// Park the pin at a fixed state WITHOUT disturbing the remembered level, so the
// visible pin test can flash the panel and then hand control straight back.
static void bl_park(int lit)
{
  if (s_running) { s_tick->detach(); s_running = false; }
  if (s_pin) *s_pin = lit ? 1 : 0;
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
    s_tick->attach(mbed::callback(bl_isr), std::chrono::microseconds(s_stepUs));
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

void backlight_reassert()
{
  pinMode(BL_PIN_D, OUTPUT);
  backlight_set(s_pct);        // re-applies duty and re-arms the Ticker
}

uint8_t backlight_measure_duty()
{
  uint32_t hi = 0, n = 0;
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < 50000UL)
  {
    if (GPIOB->IDR & (1UL << 12)) hi++;
    n++;
  }
  if (n == 0) return 255;
  return (uint8_t)((hi * 100UL) / n);
}

uint32_t backlight_isr_count() { return s_isrCount; }
uint32_t backlight_step_us()   { return s_stepUs; }

void backlight_set_step_us(uint32_t us)
{
  if (us < 50) us = 50;
  s_stepUs = us;
  // Re-apply so the Ticker is detached and re-attached at the new period.
  uint8_t p = s_pct;
  bl_park(1);
  backlight_set(p);
}

// ---- visible pin test ----
// Phases: dark, lit, dark, lit, restore. Long enough that any backlight which
// this pin actually controls will visibly follow it.
static uint8_t  s_testStep = 0;
static uint32_t s_testNext = 0;

void backlight_test_begin()
{
  s_testStep = 1;
  s_testNext = millis();
}

bool backlight_test_active() { return s_testStep != 0; }

bool backlight_test_tick(uint32_t now)
{
  if (s_testStep == 0) return false;
  if ((int32_t)(now - s_testNext) < 0) return false;

  switch (s_testStep)
  {
    case 1:
    case 3:
      bl_park(0);
      Serial.println("[bl test] pin LOW  - screen should go DARK");
      break;
    case 2:
    case 4:
      bl_park(1);
      Serial.println("[bl test] pin HIGH - screen should be LIT");
      break;
    default:
      backlight_set(s_pct);          // hand control back to the saved level
      Serial.print("[bl test] done. If the screen never changed, this pin does ");
      Serial.println("not drive the backlight.");
      s_testStep = 0;
      return true;
  }
  s_testStep++;
  s_testNext = now + 1200;
  return false;
}

// ---- characterisation sweep ----
// Sampled finer where it matters: the top end of this panel is perceptually
// compressed (55% looks close to 100%), the middle holds the useful range, and
// the bottom is included because it misbehaves and that needs mapping.
static const uint8_t BL_SWEEP[] = {
  100, 85, 70, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 12, 10, 8, 5
};
static const uint8_t BL_SWEEP_N = (uint8_t)(sizeof(BL_SWEEP) / sizeof(BL_SWEEP[0]));
static uint8_t  s_sweepIdx   = 255;        // 255 = idle
static uint32_t s_sweepNext  = 0;
static uint8_t  s_sweepSaved = 0;

void backlight_sweep_begin()
{
  s_sweepSaved = s_pct;
  s_sweepIdx   = 0;
  s_sweepNext  = millis();
}

bool backlight_sweep_tick(uint32_t now)
{
  if (s_sweepIdx == 255) return false;
  if ((int32_t)(now - s_sweepNext) < 0) return false;

  if (s_sweepIdx >= BL_SWEEP_N)
  {
    s_sweepIdx = 255;
    backlight_set(s_sweepSaved);
    Serial.print("[sweep] done, restored to ");
    Serial.print(s_sweepSaved);
    Serial.println("%");
    return true;
  }

  uint8_t pct = BL_SWEEP[s_sweepIdx];
  backlight_set(pct);
  Serial.print("[sweep] ");
  Serial.print(s_sweepIdx + 1); Serial.print("/"); Serial.print(BL_SWEEP_N);
  Serial.print("  duty "); Serial.print(pct); Serial.println("%");
  s_sweepIdx++;
  s_sweepNext = now + 1800;
  return false;
}
