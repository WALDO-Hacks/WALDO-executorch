// "Hey Waldo" debounce, one step per 200 ms window: fire when at least WAKE_K_HITS of the
// last WAKE_N_WINDOW windows have a wake posterior >= the threshold, then ignore further
// events for WAKE_REFRACTORY windows. Same logic as evaluate.py `debounce` (model/wake/),
// which produced the numbers in documentation/wake-word.md; tests/host/test_brain.cpp checks
// the two against the same known-answer sequences. Parameters come from the validation run.
#ifndef WAKE_DEBOUNCE_H
#define WAKE_DEBOUNCE_H

#include <stdbool.h>
#include <stdint.h>

struct wake_debounce {
    uint32_t history;   // bit i = the window i steps ago was a hit (bit 0 = newest)
    int cooldown;       // windows still to ignore after an event
};

static inline int wake_popcount(uint32_t v)
{
    int n = 0;
    for (; v; v &= v - 1) n++;
    return n;
}

// One window; returns true when a wake event fires now.
static inline bool wake_debounce_step(struct wake_debounce *d, bool hit, int n_window, int k_hits, int refractory)
{
    d->history = (d->history << 1) | (hit ? 1u : 0u);
    if (d->cooldown > 0) {
        d->cooldown--;
        return false;
    }
    const uint32_t mask = n_window >= 32 ? 0xFFFFFFFFu : ((1u << n_window) - 1u);
    if (wake_popcount(d->history & mask) < k_hits) return false;
    d->cooldown = refractory;
    return true;
}

#endif
