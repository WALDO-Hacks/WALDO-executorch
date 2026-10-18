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

#ifdef __cplusplus
}
#endif
#endif
