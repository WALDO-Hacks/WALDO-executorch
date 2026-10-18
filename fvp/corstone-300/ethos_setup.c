// Ethos-U55 bring-up on the Corstone-300 FVP (fvp-sim), with the same NPU timing hooks as
// board/Corstone-320/ethos_setup.c: the M55's DWT cycles and the NPU's own PMU cycles.
#include <stdio.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "ethosu_driver.h"
#include "pmu_ethosu.h"

static struct ethosu_driver EthosDriver;

void ETHOS_U55_Handler(void) { ethosu_irq_handler(&EthosDriver); }

int ethos_setup(void)
{
  struct ethosu_hw_info hw_info;
  if (ethosu_init(&EthosDriver, (void *)ETHOS_U55_APB_BASE_S, 0, 0, 1, 1) != 0) {
    printf("Failed to initialize Arm Ethos-U driver\n");
    return -1;
  }
  NVIC_EnableIRQ(ETHOS_U55_IRQn);
  ethosu_get_hw_info(&EthosDriver, &hw_info);
  printf("Ethos-U version info:\n\tArch:       v%u.%u.%u\n\tMACs/cc:    %u\n\tCmd stream: v%u\n",
         hw_info.version.arch_major_rev, hw_info.version.arch_minor_rev,
         hw_info.version.arch_patch_rev, (unsigned)(1u << hw_info.cfg.macs_per_cc),
         hw_info.cfg.cmd_stream_version);
  return 0;
}

static volatile uint32_t npu_t0, npu_cycles;
static volatile uint64_t npu_pmu_cycles;

void ethosu_inference_begin(struct ethosu_driver *drv, void *user_arg)
{
  (void)user_arg;
  ETHOSU_PMU_Enable(drv);
  ETHOSU_PMU_CNTR_Enable(drv, ETHOSU_PMU_CCNT_Msk);
  ETHOSU_PMU_CYCCNT_Reset(drv);
  npu_t0 = DWT->CYCCNT;
}

void ethosu_inference_end(struct ethosu_driver *drv, void *user_arg)
{
  (void)user_arg;
  npu_cycles += DWT->CYCCNT - npu_t0;
  npu_pmu_cycles += ETHOSU_PMU_Get_CCNTR(drv);
  ETHOSU_PMU_Disable(drv);
}

uint64_t ethos_npu_pmu_cycles_take(void)
{
  uint64_t c = npu_pmu_cycles;
  npu_pmu_cycles = 0;
  return c;
}
