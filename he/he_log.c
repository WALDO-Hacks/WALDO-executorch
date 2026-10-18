/* WALDO two-brain, M55-HE: no console of its own. printf() lands in the mailbox's log
 * ring (src/waldo/brain_link.h), and the HP prints it on its console as "HE| ...". */
#include "RTE_Components.h"
#include CMSIS_device_header

#include "he.h"
#include "waldo/brain_link.h"

static int log_ready;

void he_log_enable(void) { log_ready = 1; }

int stdout_putchar(int ch)
{
    if (!log_ready) return ch;   /* SRAM0 not powered yet */
    struct brain_mailbox *m = BRAIN_MAILBOX;
    uint32_t h = m->log_head;
    m->log[h % BRAIN_LOG_SIZE] = (char)ch;
    __DMB();
    m->log_head = h + 1;
    if (ch == '\n') he_mailbox_flush();
    return ch;
}

int stderr_putchar(int ch) { return stdout_putchar(ch); }
