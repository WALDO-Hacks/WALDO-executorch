// Keyword spotting on the Ethos-U through ExecuTorch (model/kws/). The labels come from
// src/kws_labels.h: KWS v2 (commands, objects, home, patrol, hey_waldo) by default, Arm's
// MicroNet-M words with WALDO_KWS_MICRONET (src/kws_model.h). _silence_ and _unknown_ last.
#pragma once
#include <stdint.h>

#include "kws_labels.h"

// Loads the .pte into the ExecuTorch runtime. Call once, after runtime_init().
bool kws_init();
// Classifies 1 s of audio given as 49 x 10 MFCCs (src/waldo/kws_mfcc.h). Writes the KWS_LABELS
// softmax probabilities to probs; returns the most likely label index, -1 on error.
int kws_classify(const float *mfcc, float *probs);
const char *kws_label(int index);
// Microseconds the last NPU inference took (forward() only).
uint32_t kws_last_us();

// Live listening: feeds the microphone into a rolling 1 s MFCC window and classifies it
// every 200 ms. Returns a label index when a command word was heard (confident over
// several windows, not repeated within a second), else -1. Call often.
int kws_listen(uint32_t now_ms);
// MFCC and classification time spent since the last call, and windows classified.
void kws_listen_stats(uint32_t *mfcc_us, uint32_t *npu_us, uint32_t *windows);
