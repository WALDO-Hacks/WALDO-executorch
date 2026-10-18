// Memory regions of the Corstone-300 FVP for the HE keyword-spotter test (fvp-sim).
// Everything sits in secure DDR4 (0x70000000): the 1 MB code SRAM is too small for the
// ExecuTorch runtime plus the program, and the Ethos-U55 cannot reach the TCMs.
// fvp_config.txt points INITSVTOR at __ROM0_BASE.
#ifndef REGIONS_SSE_300_H
#define REGIONS_SSE_300_H

#define __ROM0_BASE   0x70000000
#define __ROM0_SIZE   0x01000000
#define __ROM1_BASE   0
#define __ROM1_SIZE   0
#define __ROM2_BASE   0
#define __ROM2_SIZE   0
#define __ROM3_BASE   0
#define __ROM3_SIZE   0

#define __RAM0_BASE   0x71000000
#define __RAM0_SIZE   0x02000000
#define __RAM1_BASE   0
#define __RAM1_SIZE   0
#define __RAM2_BASE   0
#define __RAM2_SIZE   0
#define __RAM3_BASE   0
#define __RAM3_SIZE   0

#define __STACK_SIZE  0x00008000
#define __HEAP_SIZE   0x00040000

#endif /* REGIONS_SSE_300_H */
