// ============================================================
// audio.h — DAC0 speaker chirps
// ============================================================
#pragma once

#include <Arduino.h>

void audio_init();

// Two rising notes — played on each NEW unique MAC.  Short & blocking (~160 ms).
void audio_chirp_new();

// Optional startup blip so you know the speaker is wired.
void audio_boot();
