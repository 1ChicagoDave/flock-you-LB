// ============================================================
// audio.cpp — DAC0 (A12) tone synthesis
// ============================================================
//
// APPROACH: DAC0 is analog pin A12 on the GIGA R1.  On the Arduino Mbed core,
// A12/A13 are true DAC outputs, so analogWrite(A12, level) sets an output
// VOLTAGE (not PWM).  We synthesize a note by writing sine samples in a short
// timed loop.  Chirps only fire on a brand-new MAC, so a ~160 ms blocking burst
// is acceptable.
//
// KNOWN-UNCERTAIN: on some core versions analogWrite() drives A12 as PWM rather
// than the DAC.  If you hear nothing, switch to the mbed AnalogOut path shown in
// the #else branch below (uncomment ALT_MBED_ANALOGOUT), or use the AdvancedDAC
// library.  Also verify the DAC full-scale level matches your amp/speaker.

#include "audio.h"
#include "config.h"

// #define ALT_MBED_ANALOGOUT 1   // uncomment to use mbed::AnalogOut instead

#ifdef ALT_MBED_ANALOGOUT
  #include "mbed.h"
  // PA_4 is the STM32H7 DAC1 channel exposed as DAC0 / A12 on the GIGA silk.
  static mbed::AnalogOut s_dac(PA_4);
#endif

static const int  DAC_BITS   = 12;         // 12-bit DAC
static const int  DAC_MAX    = (1 << DAC_BITS) - 1;
static const int  DAC_MID    = DAC_MAX / 2;
static const uint32_t SR_HZ  = 20000;      // sample rate for the synth loop

void audio_init()
{
#ifndef ALT_MBED_ANALOGOUT
  analogWriteResolution(DAC_BITS);
  analogWrite(SPEAKER_PIN, 0);
#else
  s_dac.write_u16(0);
#endif
}

// Blocking sine note.  amp is 0..1 volume.
static void playNote(float freqHz, uint16_t ms, float amp)
{
  if (freqHz <= 0.0f || ms == 0) return;
  const uint32_t totalSamples = (uint32_t)((uint64_t)SR_HZ * ms / 1000UL);
  const float    step         = TWO_PI * freqHz / (float)SR_HZ;
  const uint32_t usPerSample   = 1000000UL / SR_HZ;
  float phase = 0.0f;

  for (uint32_t i = 0; i < totalSamples; i++)
  {
    float s = sinf(phase);                 // -1..+1
    phase += step;
    if (phase >= TWO_PI) phase -= TWO_PI;

    int level = DAC_MID + (int)(s * amp * DAC_MID);
    if (level < 0)        level = 0;
    if (level > DAC_MAX)  level = DAC_MAX;

#ifndef ALT_MBED_ANALOGOUT
    analogWrite(SPEAKER_PIN, level);
#else
    s_dac.write_u16((uint16_t)(level << (16 - DAC_BITS)));
#endif
    delayMicroseconds(usPerSample);
  }

  // Return to rest so the speaker doesn't hold a DC level.
#ifndef ALT_MBED_ANALOGOUT
  analogWrite(SPEAKER_PIN, 0);
#else
  s_dac.write_u16(0);
#endif
}

void audio_chirp_new()
{
  playNote(2000.0f, 70, 0.85f);   // low note
  delay(20);
  playNote(2800.0f, 80, 0.85f);   // rising note
}

void audio_boot()
{
  playNote(1200.0f, 60, 0.6f);
}
