// Which keyword model both images (HP src/, HE he/) use. One switch for both, so the HE's
// label indices always match the HP's names.
//
// Default: WALDO's own KWS v2 (model/kws/kws_vocab.py: the 10 commands, 10 objects, home,
// patrol, hey_waldo), src/kws_pte.c and he/kws_pte_he.c.
// Fallback: uncomment to go back to Arm's MicroNet-M (10 commands only),
// src/kws_pte_micronet.c and he/kws_pte_he_micronet.c. Then rebuild and reflash both images.
#pragma once
#define WALDO_KWS_MICRONET   // until the KWS v2 program is exported into src/kws_pte.c
