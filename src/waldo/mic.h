// The DevKit's two on-board PDM microphones (PDM0 channels 4 and 5: clock P6_7, data
// P5_4, already muxed by the board's pin config), captured at 16 kHz, 16 bit.
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MIC_RATE 16000

int mic_init(void);   // 0 on success; capture runs from then on, in the background
// Copies up to max unread mono samples (mic 0), oldest first, into out; returns how many.
// The ring holds ~1 s: a reader that falls further behind loses the oldest audio.
uint32_t mic_read(int16_t *out, uint32_t max);
// RMS of each mic since the last call (full scale 32768), and PDM FIFO overflows.
void mic_levels(float *rms0, float *rms1, uint32_t *errors);

#ifdef __cplusplus
}
#endif
