/* WALDO two-brain, M55-HE: the HE's own Ethos-U55-128. Derived from the pack's
 * Boards/DevKit-e8/Layers/M55_HE/ethos_setup.c (Apache-2.0, Arm Limited). */
#include <stdio.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "ethosu_driver.h"
#include "sys_utils.h"

static struct ethosu_driver EthosDriver;

void NPU_HE_IRQHandler(void) { ethosu_irq_handler(&EthosDriver); }

/* 0 when the driver is up; the hardware's version goes to the log. */
int ethos_setup(void)
{
    struct ethosu_hw_info hw;
    if (ethosu_init(&EthosDriver, (void *)NPU_HE_BASE, 0, 0, 1, 1) != 0) {
        printf("HE: Ethos-U55 driver init failed\n");
        return -1;
    }
    NVIC_EnableIRQ(NPU_HE_IRQ_IRQn);
    ethosu_get_hw_info(&EthosDriver, &hw);
    printf("HE: Ethos-U55 v%u.%u.%u, %u MACs/cc\n", hw.version.arch_major_rev, hw.version.arch_minor_rev,
           hw.version.arch_patch_rev, (unsigned)(1u << hw.cfg.macs_per_cc));
    return 0;
}

/* As on the HP: the NPU sees the core's TCM only at its global alias. */
uint64_t ethosu_address_remap(uint64_t address, int index)
{
    (void)index;
    return LocalToGlobal((const volatile void *)(uintptr_t)address);
}
