// ============================================================
// audio.cpp — square-wave chirps on the speaker pin
// ============================================================
//
// APPROACH: drive SPEAKER_PIN (A12) as a DIGITAL square wave via tone(), the
// same way the ESP32 build drove its piezo/speaker straight off a GPIO.  A
// digital pin swings rail-to-rail and sources enough current to rattle a tiny
// 8 ohm / 1 W speaker wired directly to the pin — no amp, no DAC.
//
// Why not the DAC?  A12 is also DAC0, but the DAC output buffer only sources a
// couple mA, so a bare 8 ohm load just loads it down to near-silence.  tone()
// toggles the pin as a plain push-pull output instead, which is loud.
//
// TRADE-OFF: square wave = buzzy, not the smooth sine the DAC path produced.
// For a "camera nearby!" alert that's fine (and louder).  8 ohm straight off a
// pin is technically over the pin's current rating, but the chirps are short
// and low-duty — the same low risk the ESP32 already tolerated.  If you later
// add a class-D amp (PAM8302A) fed from A12, you can go back to a DAC sine.

#include "audio.h"
#include "config.h"

// Blocking square-wave note.  freq in Hz, duration in ms.
static void playNote(float freqHz, uint16_t ms)
{
  if (freqHz <= 0.0f || ms == 0) return;
  tone(SPEAKER_PIN, (unsigned int)freqHz);
  delay(ms);
  noTone(SPEAKER_PIN);
}

void audio_init()
{
  pinMode(SPEAKER_PIN, OUTPUT);
  digitalWrite(SPEAKER_PIN, LOW);
}

void audio_chirp_new()
{
  playNote(2000.0f, 70);   // low note
  delay(20);
  playNote(2800.0f, 80);   // rising note
}

void audio_boot()
{
  playNote(1200.0f, 60);
}
