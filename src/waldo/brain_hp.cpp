// Two-brain, HP side: reads the mailbox the M55-HE writes (brain_link.h). The HP never
// writes it; it drops its cached copy of the mailbox before every read.

#include "brain_hp.h"

#include <stdio.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "brain_link.h"
#include "kws.hpp"

namespace {

const brain_mailbox *const m = BRAIN_MAILBOX;
uint32_t word_tail, wake_tail, log_tail, last_windows, last_mfcc, last_npu, last_heartbeat;
bool tails_set;

const char *const kStage[] = {"-", "boot", "SE services", "SRAM0", "NPU", "model", "self-test", "mics", "listening"};

void refresh() { SCB_InvalidateDCache_by_Addr((void *)BRAIN_MAILBOX, sizeof(brain_mailbox)); }

bool alive() { return m->magic == BRAIN_MAGIC; }

const char *stage_name(uint32_t s) { return s < sizeof kStage / sizeof kStage[0] ? kStage[s] : "?"; }

// The HE's printf output, line by line, prefixed so it stands out on the HP console.
void drain_log()
{
    const uint32_t head = m->log_head;
    if ((int32_t)(head - log_tail) < 0) log_tail = head;   // the HE restarted: start over
    if (head - log_tail > BRAIN_LOG_SIZE) log_tail = head - BRAIN_LOG_SIZE;   // fell behind: newest only
    static char line[128];
    static uint32_t n;
    for (; log_tail != head; log_tail++) {
        const char c = m->log[log_tail % BRAIN_LOG_SIZE];
        if (c == '\n' || n == sizeof line - 1) {
            line[n] = 0;
            printf("HE| %s\n", line);
            n = 0;
        } else {
            line[n++] = c;
        }
    }
}

}  // namespace

bool brain_hp_selftest(uint32_t timeout_ms)
{
    // SysTick isn't running yet at boot: count with the cycle counter.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    const uint32_t per_ms = SystemCoreClock / 1000u;
    uint32_t waited = 0, t = DWT->CYCCNT;
    refresh();
    while ((!alive() || m->selftest < 0) && m->error == 0 && waited < timeout_ms) {
        if (DWT->CYCCNT - t >= per_ms) {
            t += per_ms;
            waited++;
            refresh();
            if (alive()) drain_log();
        }
    }
    refresh();
    if (alive()) drain_log();
    tails_set = true;
    word_tail = m->word_head;   // words from before the HP was listening don't count
    wake_tail = m->wake_head;
    if (!alive()) {
        printf("two-brain: no word from the M55-HE after %lu ms (mailbox at 0x%08lx)\nKWS_result: FAIL\n",
               (unsigned long)waited, (unsigned long)BRAIN_MAILBOX_ADDR);
        return false;
    }
    const bool pass = m->selftest == 1;
    printf("two-brain: M55-HE at %lu MHz reached \"%s\"%s; keyword self-test on its Ethos-U55: %s (%.2f), "
           "MFCC %lu us, NPU %lu us\n",
           (unsigned long)(m->core_hz / 1000000u), stage_name(m->stage), m->error ? " and FAILED there" : "",
           m->selftest < 0 ? "not run" : pass ? "yes" : "wrong word", (double)m->selftest_prob,
           (unsigned long)m->selftest_mfcc_us, (unsigned long)m->selftest_npu_us);
    if (m->sram_ret || m->sram_err)
        printf("two-brain: the HE's SRAM power request returned %lu/%lu\n", (unsigned long)m->sram_ret,
               (unsigned long)m->sram_err);
    printf("KWS_result: %s\n", pass ? "PASS" : "FAIL");
    printf("two-brain: wake word \"Hey Waldo\" %s\n", m->wake_enabled == 1 ? "enabled (HE self-test passed)" : "not available");
    return pass;
}

bool brain_hp_wake_enabled(void)
{
    refresh();
    return alive() && m->wake_enabled == 1;
}

bool brain_hp_wake_poll(uint32_t *score_x1000)
{
    refresh();
    if (!alive()) return false;
    if (!tails_set) {
        tails_set = true;
        word_tail = m->word_head;
        wake_tail = m->wake_head;
    }
    const uint32_t head = m->wake_head;
    if ((int32_t)(head - wake_tail) < 0) wake_tail = head;   // the HE restarted: nothing old to replay
    if (head - wake_tail > BRAIN_WORDS) wake_tail = head - BRAIN_WORDS;   // far behind: the newest ones
    if (wake_tail == head) return false;
    wake_tail++;
    *score_x1000 = m->wake_score_x1000;
    return true;
}

int brain_hp_word(void)
{
    refresh();
    if (!alive()) return -1;
    drain_log();
    if (!tails_set) {
        tails_set = true;
        word_tail = m->word_head;
    }
    const uint32_t head = m->word_head;
    if ((int32_t)(head - word_tail) < 0) word_tail = head;   // the HE restarted: nothing old to replay
    if (word_tail == head) return -1;
    if (head - word_tail > BRAIN_WORDS) word_tail = head - BRAIN_WORDS;   // missed some: oldest still there
    const int w = m->word[word_tail % BRAIN_WORDS];
    word_tail++;
    return w >= 0 && w < KWS_LABELS ? w : -1;
}

void brain_hp_print(uint32_t window_ms)
{
    refresh();
    if (!alive()) {
        printf("two-brain: M55-HE silent (no mailbox)\n");
        return;
    }
    if (m->windows < last_windows || m->heartbeat < last_heartbeat)   // the HE restarted
        last_windows = last_mfcc = last_npu = last_heartbeat = 0;
    const uint32_t windows = m->windows - last_windows, mfcc = m->mfcc_us - last_mfcc, npu = m->npu_us - last_npu;
    const uint32_t beats = m->heartbeat - last_heartbeat;
    last_windows = m->windows;
    last_mfcc = m->mfcc_us;
    last_npu = m->npu_us;
    last_heartbeat = m->heartbeat;
    printf("two-brain: HE %s%s, %lu heartbeats, mic rms %.0f/%.0f (%lu errors), %lu windows, MFCC %lu us, "
           "U55 %lu us over %lu ms (HE core busy %.1f%%)\n",
           stage_name(m->stage), m->error ? " FAILED" : "", (unsigned long)beats, (double)m->rms0, (double)m->rms1,
           (unsigned long)m->mic_errors, (unsigned long)windows, (unsigned long)mfcc, (unsigned long)npu,
           (unsigned long)window_ms, window_ms ? (double)(mfcc + npu) / (window_ms * 10.0) : 0.0);
}
