// Which keyword model both images (HP src/, HE he/) use. One switch for both, so the HE's
// label indices always match the HP's names.
//
// WALDO_KWS_MICRONET: Arm's MicroNet-M (10 commands), src/kws_pte_micronet.c and
// he/kws_pte_he_micronet.c.
// WALDO_KWS_V2: WALDO's own KWS v2 (model/kws/kws_vocab.py: the 10 commands, objects, home,
// patrol, hey_waldo), src/kws_pte.c and he/kws_pte_he.c, with the per-word switches and
// thresholds in src/kws_labels.h.
//
// KWS v2 (run r4) is the default; MicroNet stays as the fallback (WALDO_KWS_MICRONET).
// To go back, change the default below to WALDO_KWS_MICRONET (or pass it to BOTH images),
// then rebuild and reflash both.
#pragma once
#if !defined(WALDO_KWS_MICRONET) && !defined(WALDO_KWS_V2)
#define WALDO_KWS_V2
#endif
#if defined(WALDO_KWS_MICRONET) && defined(WALDO_KWS_V2)
#error "WALDO_KWS_MICRONET and WALDO_KWS_V2 are both set"
#endif
