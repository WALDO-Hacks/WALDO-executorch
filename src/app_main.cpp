/*
 * Copyright 2026 Arm Limited and/or its affiliates.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

// WALDO: a pan/tilt turret that points a flashlight at the faces it finds.
// BlazeFace runs on the Ethos-U85 through ExecuTorch (model/model.py, src/face_detector.cpp).
//
// Every target first runs a self-test on a face photo built into the image and prints
// Test_result: PASS when it finds the face (this is what the FVP run shows). On the
// DevKit-E8 it then runs the turret: ESP32 camera frames -> faces on the NPU -> servos.

#include <cstdint>
#include <cstdio>
#include <string_view>

#include <executorch/runtime/platform/runtime.h>

#include "face_detector.hpp"
#include "model_pte.h"
#include "test_face.h"
#ifdef WALDO_HW
#include <cmath>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "kws.hpp"
#include "test_kws.h"
#include "waldo/kws_mfcc.h"
#include "waldo/waldo.h"

// Keyword spotter self-test: MFCCs of a recorded "yes" on the M55 (checked against the
// ones Python computed at export time), then MicroNet on the NPU must say "yes".
static bool kws_self_test() {
  static float feats[KWS_FRAMES * KWS_COEFFS];
  float err = 0;
  const uint32_t t0 = DWT->CYCCNT;
  for (int f = 0; f < KWS_FRAMES; f++) kws_mfcc_frame(test_kws_yes + f * KWS_STRIDE, feats + f * KWS_COEFFS);
  const uint32_t mfcc_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000u);
  for (int i = 0; i < KWS_FRAMES * KWS_COEFFS; i++) err = std::fmax(err, std::fabs(feats[i] - test_kws_yes_mfcc[i]));
  float probs[KWS_LABELS];
  const int best = kws_classify(feats, probs);
  printf("KWS self-test: heard \"%s\" (%.2f), MFCC %lu us for 1 s (max diff to Python %.4f), NPU %lu us\n",
         kws_label(best), (double)(best >= 0 ? probs[best] : 0), static_cast<unsigned long>(mfcc_us),
         (double)err, static_cast<unsigned long>(kws_last_us()));
  const bool pass = best >= 0 && std::string_view(kws_label(best)) == "yes" && err < 0.05f;
  printf("KWS_result: %s\n", pass ? "PASS" : "FAIL");
  return pass;
}
#endif

extern "C" int app_main(void) {
  executorch::runtime::runtime_init();
  printf("WALDO face detector: BlazeFace, %lu byte ExecuTorch program on the Ethos-U85\n",
         static_cast<unsigned long>(model_pte_size));

  if (!face_detector_init()) {
    printf("face detector init failed\n");
    return 1;
  }

  face_box faces[4];
  int n = face_detect_gray(test_face_128, 128, 128, faces, 4);
  printf("Self-test: %d face(s), NPU inference %lu us\n", n,
         static_cast<unsigned long>(face_detector_last_us()));
  for (int i = 0; i < n; i++) {
    printf("  face %d: x %.0f y %.0f w %.0f h %.0f, score %.2f\n", i, (double)faces[i].x,
           (double)faces[i].y, (double)faces[i].w, (double)faces[i].h, (double)faces[i].score);
  }
  printf("Test_result: %s\n", n == 1 ? "PASS" : "FAIL");

#ifdef WALDO_HW
  if (kws_init()) {
    kws_self_test();
  } else {
    printf("keyword spotter init failed\n");
  }
  if (waldo_init() == 0) {
    waldo_servo_test();
    waldo_home();
    while (true) {
      waldo_poll();
    }
  }
#endif
  printf("\x04");  // EOT: ends the FVP run (semihosting exit); a board just sees a 0x04
  fflush(stdout);
  return 0;
}
