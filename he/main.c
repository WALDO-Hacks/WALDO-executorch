/* WALDO two-brain, M55-HE image: bring-up. The HP image configures the pins (one
 * conductor configuration for the whole chip) and powers SRAM1; the HE powers SRAM0
 * itself, since the mailbox and its pools are there and the two cores start together. */
#include <string.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "he.h"
#include "se_services_port.h"
#include "services_lib_api.h"
#include "waldo/brain_link.h"

static volatile uint32_t ms;
void SysTick_Handler(void) { ms++; }
uint32_t he_millis(void) { return ms; }

void he_mailbox_flush(void)
{
    SCB_CleanDCache_by_Addr((void *)BRAIN_MAILBOX_ADDR, sizeof(struct brain_mailbox));
}

int main(void)
{
    SCB_CleanInvalidateDCache();
    SCB_InvalidateICache();
    SysTick_Config(SystemCoreClock / 1000U);

    se_services_port_init();
    /* The same request as the HP's (board/DevKit-E8/main.c): whichever the SE handles last
       must not leave SRAM1, the HP's YOLO scratch and camera frames, switched off. */
    uint32_t err = 0;
    const uint32_t ret = SERVICES_power_memory_req(se_services_s_handle,
                                                   POWER_MEM_SRAM_0_ENABLE | POWER_MEM_SRAM_1_ENABLE, &err);

    /* Not valid until it's all written: the HP may be reading a mailbox left from before. */
    struct brain_mailbox *m = BRAIN_MAILBOX;
    m->magic = 0;
    __DMB();
    memset((void *)m, 0, sizeof *m);
    m->selftest = -1;
    m->core_hz = SystemCoreClock;
    m->sram_ret = ret;
    m->sram_err = err;
    m->stage = BRAIN_SRAM;
    __DMB();
    m->magic = BRAIN_MAGIC;
    he_mailbox_flush();
    he_log_enable();

    return he_app_main();
}
