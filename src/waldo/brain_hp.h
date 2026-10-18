// Two-brain, HP side: the words and stats the M55-HE posts in the mailbox (brain_link.h).
#ifndef BRAIN_HP_H
#define BRAIN_HP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Waits up to timeout_ms for the HE's keyword self-test; prints it and KWS_result.
bool brain_hp_selftest(uint32_t timeout_ms);
// The next word the HE heard (a kws_label index), or -1. Also prints the HE's log.
int brain_hp_word(void);
// One console line: is the HE alive, its mic levels and its NPU/MFCC time.
void brain_hp_print(uint32_t window_ms);
// For the screen: is the HE listening (alive, no error, heartbeat within 3 s), and how busy
// its core was (MFCC + U55 time, %) over the last second or so. Call every frame.
bool brain_hp_status(uint32_t now_ms, float *busy_pct);

#ifdef __cplusplus
}
#endif
#endif
