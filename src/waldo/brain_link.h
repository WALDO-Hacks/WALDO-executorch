// Two-brain link: the M55-HE (always-on ears) talks to the M55-HP (the eyes and the turret).
//
// The HE image (he/) owns the microphones and runs MFCC + MicroNet on its Ethos-U55-128.
// It writes what it hears into this mailbox at the top of SRAM0; the HP image reads it.
// One writer per field: the HE writes everything, the HP only reads. Words go through a
// small ring (head written by the HE, the HP keeps its own tail), so none is lost or torn.
// Both cores cache SRAM0: the HE cleans the mailbox's lines after writing, the HP
// invalidates them before reading.
//
// SRAM0 map in the two-brain build (both linker scripts agree with this):
//   0x02000000  A32 boot stub (ATOC)
//   0x02010000  HP: ExecuTorch pools, LCD frame ...      (board/DevKit-E8/linker_gnu_mram.ld.src)
//   0x02380000  HE: its ExecuTorch pools (508 KB)        (he/linker_gnu_he.ld.src)
//   0x023FF000  this mailbox (4 KB)
#ifndef BRAIN_LINK_H
#define BRAIN_LINK_H

#include <stdint.h>

#define BRAIN_HE_SRAM_BASE 0x02380000u
#define BRAIN_HE_SRAM_SIZE 0x0007F000u
#define BRAIN_MAILBOX_ADDR 0x023FF000u
#define BRAIN_MAGIC        0x32444C57u   // "WLD2"
#define BRAIN_WORDS        8              // ring of heard words
#define BRAIN_LOG_SIZE     2048           // HE printf output, a ring the HP prints

// HE boot stages, in order; a stuck value says where the HE stopped.
enum brain_stage {
    BRAIN_BOOT = 1,      // main() reached
    BRAIN_SE,            // Secure Enclave services up
    BRAIN_SRAM,          // SRAM0 power requested
    BRAIN_NPU,           // Ethos-U55 driver initialised
    BRAIN_MODEL,         // KWS program loaded
    BRAIN_SELFTEST,      // self-test done
    BRAIN_MIC,           // microphones running
    BRAIN_LISTENING,     // main loop
};

struct brain_mailbox {
    volatile uint32_t magic;          // BRAIN_MAGIC once the HE has started
    volatile uint32_t stage;          // enum brain_stage
    volatile uint32_t error;          // nonzero: the stage failed with this code
    volatile uint32_t heartbeat;      // +1 every second while listening
    volatile int32_t selftest;        // -1 not run, 0 fail, 1 pass
    volatile float selftest_prob;
    volatile uint32_t selftest_npu_us;
    volatile uint32_t selftest_mfcc_us;
    volatile uint32_t word_head;      // words written so far
    volatile int32_t word[BRAIN_WORDS];      // label index (kws_label)
    volatile uint32_t windows, mfcc_us, npu_us;   // totals since boot
    volatile float rms0, rms1;        // microphone levels
    volatile uint32_t mic_errors;
    volatile uint32_t core_hz;        // the HE's SystemCoreClock
    volatile uint32_t sram_ret, sram_err;   // the HE's SRAM power request to the SE
    volatile uint32_t log_head;       // bytes written to log so far
    volatile char log[BRAIN_LOG_SIZE];
    // Wake word "Hey Waldo" (model/wake/, he/wake.cpp). Appended after the fields above; the
    // HE writes, the HP reads. Zero until the HE has loaded the wake model and its boot
    // self-test passed.
    volatile uint32_t wake_enabled;   // 1: wake model loaded AND self-test passed
    volatile uint32_t wake_head;      // wake events so far; written after wake_score_x1000
    volatile uint32_t wake_score_x1000;   // posterior (x1000) of the latest event
};

#ifndef BRAIN_MAILBOX   // host tests point it at an ordinary buffer
#define BRAIN_MAILBOX ((struct brain_mailbox *)BRAIN_MAILBOX_ADDR)
#endif

// HE side: publish one heard word. The word first, then the head that makes it visible;
// the caller cleans the mailbox out of its data cache afterwards.
static inline void brain_post_word(struct brain_mailbox *m, int32_t word)
{
    const uint32_t h = m->word_head;
    m->word[h % BRAIN_WORDS] = word;
    __sync_synchronize();   /* a DMB on the M55 */
    m->word_head = h + 1;
}

// HE side: publish one wake event. The score first, then the head that makes it visible;
// the caller cleans the mailbox out of its data cache afterwards.
static inline void brain_post_wake(struct brain_mailbox *m, uint32_t score_x1000)
{
    m->wake_score_x1000 = score_x1000;
    __sync_synchronize();   /* a DMB on the M55 */
    m->wake_head = m->wake_head + 1;
}

#ifdef __cplusplus
static_assert(sizeof(brain_mailbox) <= 0x1000, "mailbox fits its 4 KB");
#endif

#endif
