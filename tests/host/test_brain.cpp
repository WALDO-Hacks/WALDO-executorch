// Host test of the two-brain mailbox: the HE's log writer (he/he_log.c) and word posting
// (brain_link.h) against the HP's reader (src/waldo/brain_hp.cpp), on an ordinary buffer.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

#include "RTE_Components.h"
#include CMSIS_device_header
#include "brain_hp.h"
#include "brain_link.h"

struct host_dwt host_dwt_regs;
struct host_coredebug host_coredebug_regs;
uint32_t SystemCoreClock = 400000000u;
extern "C" {
struct brain_mailbox host_box;
int flushes;
void he_mailbox_flush(void) { flushes++; }
void he_log_enable(void);
int stdout_putchar(int ch);
}

static int failures;
#define CHECK(c, ...)                                   \
    do {                                                \
        if (!(c)) {                                     \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);               \
            fprintf(stderr, "\n");                      \
            failures++;                                 \
        }                                               \
    } while (0)

// Everything the HP prints between two calls, through a temporary file.
static std::string capture(void (*fn)(void *), void *arg)
{
    fflush(stdout);
    FILE *tmp = tmpfile();
    const int saved = dup(fileno(stdout));
    dup2(fileno(tmp), fileno(stdout));
    fn(arg);
    fflush(stdout);
    dup2(saved, fileno(stdout));
    close(saved);
    rewind(tmp);
    std::string out;
    char buf[512];
    size_t n;
    while ((n = fread(buf, 1, sizeof buf, tmp)) > 0) out.append(buf, n);
    fclose(tmp);
    return out;
}

static void he_print(const char *s)
{
    for (; *s; s++) stdout_putchar(*s);
}

static void he_boot()   // what he/main.c does once SRAM0 is on
{
    memset(&host_box, 0, sizeof host_box);
    host_box.selftest = -1;
    host_box.core_hz = 160000000u;
    host_box.stage = BRAIN_SRAM;
    host_box.magic = BRAIN_MAGIC;
    he_log_enable();
}

int main()
{
    // 1. HE not started: no words, "silent".
    memset(&host_box, 0, sizeof host_box);
    CHECK(brain_hp_word() == -1, "word from a dead HE");
    std::string out = capture([](void *) { brain_hp_print(2000); }, nullptr);
    CHECK(out.find("silent") != std::string::npos, "dead HE not reported: %s", out.c_str());

    // 2. Boot, self-test passes; a word posted before the HP listens is dropped.
    he_boot();
    he_print("HE: M55-HE at 160 MHz\nHE: self-test heard \"yes\" (0.98)\n");
    host_box.selftest_prob = 0.98f;
    host_box.selftest = 1;
    host_box.stage = BRAIN_LISTENING;
    brain_post_word(&host_box, 9);
    bool pass = false;
    out = capture([](void *p) { *(bool *)p = brain_hp_selftest(100); }, &pass);
    CHECK(pass, "self-test not seen");
    CHECK(out.find("KWS_result: PASS") != std::string::npos, "no PASS line: %s", out.c_str());
    CHECK(out.find("HE| HE: M55-HE at 160 MHz") != std::string::npos, "HE log not printed: %s", out.c_str());
    CHECK(brain_hp_word() == -1, "stale word delivered");

    // 3. Words in order.
    const int seq[] = {1, 7, 4};   // go, stop, off
    for (int w : seq) brain_post_word(&host_box, w);
    for (int w : seq) CHECK(brain_hp_word() == w, "word out of order");
    CHECK(brain_hp_word() == -1, "extra word");

    // 4. The HP falls behind by 12 words: it gets the newest 8, in order.
    for (int i = 0; i < 12; i++) brain_post_word(&host_box, i % 10);
    for (int i = 4; i < 12; i++) {
        const int w = brain_hp_word();
        CHECK(w == i % 10, "after overflow: expected %d, got %d", i % 10, w);
    }
    CHECK(brain_hp_word() == -1, "extra word after overflow");

    // 5. A bad label index is never passed on.
    brain_post_word(&host_box, 99);
    CHECK(brain_hp_word() == -1, "bad label passed on");

    // 6. The head counter wraps around 2^32: the HP first catches up (8 reads), then
    // four words posted across the wrap arrive in order.
    host_box.word_head = 0xFFFFFFFEu;
    for (int i = 0; i < 8; i++) brain_hp_word();
    CHECK(brain_hp_word() == -1, "not caught up before the wrap");
    for (int i = 0; i < 4; i++) brain_post_word(&host_box, 2 + i);
    int got[4];
    for (int i = 0; i < 4; i++) got[i] = brain_hp_word();
    CHECK(got[0] == 2 && got[1] == 3 && got[2] == 4 && got[3] == 5, "wrap: %d %d %d %d", got[0], got[1], got[2],
          got[3]);
    CHECK(host_box.word_head == 2u, "head did not wrap");

    // 7. The HE logs much more than the ring holds between two reads: the HP prints
    // only recent, whole lines and never runs off the ring.
    for (int i = 0; i < 300; i++) {
        char line[32];
        snprintf(line, sizeof line, "HE: line %03d\n", i);
        he_print(line);
    }
    out = capture([](void *) { brain_hp_word(); }, nullptr);
    CHECK(out.find("HE| HE: line 299") != std::string::npos, "newest log line missing");
    CHECK(out.find("HE| HE: line 100\n") == std::string::npos, "old log line not dropped");

    // 8. Stats line: deltas since the last print.
    host_box.windows = 50;
    host_box.mfcc_us = 10000;
    host_box.npu_us = 5000;
    host_box.heartbeat = 10;
    out = capture([](void *) { brain_hp_print(10000); }, nullptr);
    CHECK(out.find("50 windows") != std::string::npos, "stats: %s", out.c_str());
    host_box.windows = 60;
    out = capture([](void *) { brain_hp_print(2000); }, nullptr);
    CHECK(out.find(" 10 windows") != std::string::npos, "stats delta: %s", out.c_str());

    // 9. HE failed at a stage: the self-test wait ends early and says where.
    he_boot();
    host_box.stage = BRAIN_MODEL;
    host_box.error = 1;
    out = capture([](void *p) { *(bool *)p = brain_hp_selftest(100); }, &pass);
    CHECK(!pass && out.find("model") != std::string::npos && out.find("FAILED") != std::string::npos,
          "failure not reported: %s", out.c_str());

    CHECK(flushes > 0, "the HE log never flushed the mailbox");
    printf("two-brain mailbox: %s\n", failures ? "FAILED" : "all checks passed");
    return failures ? 1 : 0;
}
