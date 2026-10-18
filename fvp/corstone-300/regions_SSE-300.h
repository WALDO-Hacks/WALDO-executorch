// Memory regions of the Corstone-300 FVP for the HE keyword-spotter test (fvp-sim).
// Code, rodata and the KWS program in the 1 MB code SRAM (0x11000000), data, pools, heap
// and stack in the 2 MB internal SRAM (0x31000000): the Ethos-U55 cannot reach the TCMs.
// fvp_config.txt points INITSVTOR at __ROM0_BASE.
#ifndef REGIONS_SSE_300_H
#define REGIONS_SSE_300_H

#define __ROM0_BASE   0x11000000
#define __ROM0_SIZE   0x00100000
#define __ROM1_BASE   0
#define __ROM1_SIZE   0
#define __ROM2_BASE   0
#define __ROM2_SIZE   0
#define __ROM3_BASE   0
#define __ROM3_SIZE   0

#define __RAM0_BASE   0x31000000
#define __RAM0_SIZE   0x00200000
#define __RAM1_BASE   0
#define __RAM1_SIZE   0
#define __RAM2_BASE   0
#define __RAM2_SIZE   0
#define __RAM3_BASE   0
#define __RAM3_SIZE   0

#define __STACK_SIZE  0x00008000
#define __HEAP_SIZE   0x00040000

#endif /* REGIONS_SSE_300_H */
