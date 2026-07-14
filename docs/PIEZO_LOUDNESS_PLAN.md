# Piezo loudness plan — BTL (anti-phase) drive + resonance retune

Goal: make the bare piezo disc dramatically louder without new parts. Two
independent, stacking wins:

1. **BTL / bridge drive (~+6 dB):** drive BOTH piezo terminals with square waves
   180° out of phase instead of one terminal to a GPIO and the other to GND.
   When pin A = 3.3 V and pin B = 0 V the disc sees +3.3 V; a half-cycle later
   A = 0 V, B = 3.3 V → −3.3 V. So it swings ±3.3 V = **6.6 V p-p, double** the
   single-ended 3.3 V p-p. Roughly +6 dB.
2. **Resonance retune (~+10–20 dB):** a piezo disc is far louder at its
   mechanical resonance (usually ~2.6–4.5 kHz) and near the ear's most sensitive
   band. Current alert tones (1.5–2.8 kHz) and the startup melody (~200 Hz) sit
   below the loud zone.

(Enclosure/mounting is a third big win but that's hardware, not firmware.)

## Wiring change (BTL)
- Piezo terminal 1 → **GPIO4** (`BUZZER_PIN`, existing).
- Piezo terminal 2 → **GPIO18** (`BUZZER_PIN_B`, new). No GND to the piezo.
- GPIO18 is free, non-strapping, and broken out. Avoid strapping pins
  (0/2/5/12/15) and the used pins (2 LED, 4 buzzerA, 16/17 GPS, 1/3 USB).
- Piezo is capacitive, so no DC-block needed. A series resistor is optional and
  usually hurts (RC roll-off) — start with none.

## Implementation (recommended: two LEDC channels on one timer, phase-offset)
Both channels share one timer (so identical frequency, phase-locked). Channel B
is shifted 180° via `hpoint` — cleaner than the output-invert flag because
setting duty=0 on both silences to 0 V (invert would idle one pin HIGH → DC
across the disc). Uses the IDF LEDC driver directly; drop Arduino `tone()`.

```c
#define BUZZER_BTL   1
#define BUZZER_PIN_B 18
#define PIEZO_RESONANCE_HZ 4000   // sweep to your disc's loudest point (see below)

#if BUZZER_BTL
#include "driver/ledc.h"
#define BTL_MODE   LEDC_LOW_SPEED_MODE
#define BTL_TIMER  LEDC_TIMER_0
#define BTL_CH_A   LEDC_CHANNEL_0
#define BTL_CH_B   LEDC_CHANNEL_1
#define BTL_RES    LEDC_TIMER_8_BIT   // duty 128 = 50%, hpoint 128 = 180°

static void btlSetup()
{
  ledc_timer_config_t t = {};
  t.speed_mode = BTL_MODE; t.duty_resolution = BTL_RES;
  t.timer_num = BTL_TIMER; t.freq_hz = PIEZO_RESONANCE_HZ; t.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  ledc_channel_config_t c = {};
  c.speed_mode = BTL_MODE; c.timer_sel = BTL_TIMER; c.duty = 0;
  c.channel = BTL_CH_A; c.gpio_num = BUZZER_PIN;   c.hpoint = 0;   ledc_channel_config(&c);
  c.channel = BTL_CH_B; c.gpio_num = BUZZER_PIN_B; c.hpoint = 128; ledc_channel_config(&c);
}

static void toneBTL(uint32_t freq)
{
  ledc_set_freq(BTL_MODE, BTL_TIMER, freq);
  ledc_set_duty_with_hpoint(BTL_MODE, BTL_CH_A, 128, 0);   ledc_update_duty(BTL_MODE, BTL_CH_A);
  ledc_set_duty_with_hpoint(BTL_MODE, BTL_CH_B, 128, 128); ledc_update_duty(BTL_MODE, BTL_CH_B);
}
static void noToneBTL()
{
  ledc_set_duty(BTL_MODE, BTL_CH_A, 0); ledc_update_duty(BTL_MODE, BTL_CH_A);
  ledc_set_duty(BTL_MODE, BTL_CH_B, 0); ledc_update_duty(BTL_MODE, BTL_CH_B); // both low → 0 V, silent
}
#endif
```

### Route the existing beeps through a shim
Add one indirection so single-ended vs BTL is a compile switch, then change the
beep functions to use it (they currently call `tone`/`noTone` directly):

```c
static inline void beepTone(uint32_t f)
{
#if BUZZER_BTL
  toneBTL(f);
#else
  tone(BUZZER_PIN, f);
#endif
}
static inline void beepOff()
{
#if BUZZER_BTL
  noToneBTL();
#else
  noTone(BUZZER_PIN);
#endif
}
```
- Replace every `tone(BUZZER_PIN, X)` → `beepTone(X)` and `noTone(BUZZER_PIN)` →
  `beepOff()` in `newDetectChirp()`, `heartbeatBeep()`, `startupBeep()`.
- `buzzerBeep()` (raw `digitalWrite`) is unused by the alert path; either delete
  it or leave it (single-ended, harmless).
- In `setup()`: `#if BUZZER_BTL btlSetup(); #else pinMode(BUZZER_PIN,OUTPUT);
  digitalWrite(BUZZER_PIN,LOW); #endif`. Remove the old buzzer `pinMode` block
  when BTL is on (LEDC owns the pins).

### Alternative BTL method (if you prefer)
One LEDC signal routed to two pins via the GPIO matrix, second pin inverted:
`ledcAttachPin(BUZZER_PIN, ch)` then
`gpio_matrix_out(BUZZER_PIN_B, <ledc_sig_idx_for_ch>, /*invert=*/true, false)`.
Perfect phase, fewer channels, but the LEDC signal index is core-version
specific — the hpoint method above avoids that.

## Resonance retune
Tie the alert tones to the resonance knob so one number tunes loudness:
```c
#define NEW_CHIRP_LO_HZ (PIEZO_RESONANCE_HZ - 400)
#define NEW_CHIRP_HI_HZ  PIEZO_RESONANCE_HZ
#define HB_BEEP_HZ       PIEZO_RESONANCE_HZ
```
The startup melody is intentionally musical (low notes) — leave it, or transpose
up an octave or two if you want it audible on a bare disc.

## Finding the disc's resonance
With BTL wired, play a steady tone and sweep `PIEZO_RESONANCE_HZ` in ~200 Hz
steps from 2600→4600; keep the loudest. (Or read the disc's datasheet resonant
frequency if printed.) A quick test sketch: `toneBTL(f); delay(400); noToneBTL();`
stepping `f`. Expect a sharp peak — that's resonance.

## Verify
- Scope or ear: BTL should be clearly louder than single-ended at the same freq.
- Confirm silence is truly silent (0 V across the disc, no whine) when idle —
  proves the duty-0 stop works and neither pin idles high.
- Watch that LEDC (used here) doesn't collide with anything else; the WS2812 uses
  RMT and WiFi uses the radio, so no conflict.

## Expected result
BTL (+6 dB) × resonance (+10–20 dB) × a resonant enclosure/mount = the bare disc
goes from "extremely quiet" to genuinely attention-getting. Firmware gets you the
first two for free.
