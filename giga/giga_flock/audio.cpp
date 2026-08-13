// ============================================================
// audio.cpp — bit-banged square-wave chirps on the speaker pin
// ============================================================
//
// APPROACH: drive SPEAKER_PIN (A12) as a DIGITAL square wave by toggling it
// with digitalWrite in a short timed loop — the same way the ESP32 build drove
// its speaker straight off a GPIO.  A digital pin swings rail-to-rail and
// sources enough current to rattle a tiny 8 ohm / 1 W speaker wired directly to
// the pin — no amp, no DAC.
//
// Why bit-bang instead of tone()?  A12 is also DAC0 (PA_4); on the GIGA it has
// no general-purpose PWM timer channel, so tone() (which uses PwmOut on this
// core) can silently fail on that pin.  Toggling the pin by hand always works.
//
// Why not the DAC?  The DAC output buffer only sources a couple mA, so a bare
// 8 ohm load loads it down to near-silence.
//
// TRADE-OFF: square wave = buzzy, not a smooth sine — fine (and louder) for a
// "camera nearby!" alert.  8 ohm straight off a pin is over the pin's current
// rating, but the chirps are short and low-duty (the risk the ESP32 tolerated).
// Add a class-D amp (PAM8302A) on A12 later if you want a clean loud sine.

#include "audio.h"
#include "config.h"

// Blocking square-wave note.  freq in Hz, duration in ms.
static void playNote(float freqHz, uint16_t ms)
{
  if (freqHz <= 0.0f || ms == 0) return;
  const uint32_t halfUs = (uint32_t)(500000.0f / freqHz);   // half period, µs
  if (halfUs == 0) return;
  const uint32_t cycles = ((uint32_t)ms * 1000UL) / (halfUs * 2);
  for (uint32_t i = 0; i < cycles; i++)
  {
    digitalWrite(SPEAKER_PIN, HIGH);
    delayMicroseconds(halfUs);
    digitalWrite(SPEAKER_PIN, LOW);
    delayMicroseconds(halfUs);
  }
}

void audio_init()
{
  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, LOW);
}

// Brand-new unique MAC: rising two-note chirp.
void audio_chirp_new()
{
  playNote(2000.0f, 90);
  delay(15);
  playNote(2800.0f, 110);
}

// Power-up: an unmistakable rising three-note jingle so you know audio is live.
void audio_boot()
{
  playNote(1000.0f, 130);
  delay(30);
  playNote(1500.0f, 130);
  delay(30);
  playNote(2200.0f, 170);
}
