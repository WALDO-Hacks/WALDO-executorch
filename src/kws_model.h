// Which keyword model both images (HP src/, HE he/) use. One switch for both, so the HE's
// label indices always match the HP's names.
//
// WALDO_KWS_MICRONET: Arm's MicroNet-M (10 commands), src/kws_pte_micronet.c and
// he/kws_pte_he_micronet.c.
// WALDO_KWS_V2: WALDO's own KWS v2 (model/kws/kws_vocab.py: the 10 commands, objects, home,
// patrol, hey_waldo), src/kws_pte.c and he/kws_pte_he.c, with the per-word switches and
// thresholds in src/kws_labels.h.
//
// MicroNet stays the default: KWS v2 missed some of its acceptance gates (model/kws/README.md).
// To try KWS v2, change the default below to WALDO_KWS_V2 (or pass -DWALDO_KWS_V2 to BOTH
// images), then rebuild and reflash both.
#pragma once
#if !defined(WALDO_KWS_MICRONET) && !defined(WALDO_KWS_V2)
#define WALDO_KWS_MICRONET
#endif
#if defined(WALDO_KWS_MICRONET) && defined(WALDO_KWS_V2)
#error "WALDO_KWS_MICRONET and WALDO_KWS_V2 are both set"
#endif
