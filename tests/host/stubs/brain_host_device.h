// Host stand-in for the CMSIS device header (two-brain mailbox test).
#pragma once
#include <stdint.h>
#define SCB_CleanDCache_by_Addr(addr, size) ((void)(addr), (void)(size))
#define SCB_InvalidateDCache_by_Addr(addr, size) ((void)(addr), (void)(size))
#define __DMB() __sync_synchronize()
struct host_dwt { volatile uint32_t CTRL, CYCCNT; };
struct host_coredebug { volatile uint32_t DEMCR; };
extern struct host_dwt host_dwt_regs;
extern struct host_coredebug host_coredebug_regs;
#define DWT (&host_dwt_regs)
#define CoreDebug (&host_coredebug_regs)
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
extern uint32_t SystemCoreClock;
