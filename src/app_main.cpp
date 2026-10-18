/*
 * Copyright 2026 Arm Limited and/or its affiliates.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 */

// WALDO: a voice-controlled pan/tilt turret that points a flashlight at the people and
// objects it finds.
//
// On the DevKit-E8 the keyword spotter and YOLOv8n check themselves at boot (KWS_result,
// YOLO_result), then the turret runs: ESP32 camera frames -> YOLOv8n on the NPU -> servos,
// steered by voice. The FVP target runs the template's AI layer instead: BlazeFace on a
// built-in face photo, Test_result: PASS when it finds the face.

#include <cstdint>
#include <cstdio>
#include <string_view>

#include <executorch/runtime/platform/runtime.h>

#ifndef WALDO_HW
#include "face_detector.hpp"
#include "model_pte.h"
#include "test_face.h"
#endif
// WALDO_FVP_TURRET (fvp-sim branch): the FVP also runs the turret's two self-tests below,
// YOLOv8n and the U85 KWS, on the Corstone-320's Ethos-U85.
#if defined(WALDO_HW) || defined(WALDO_FVP_TURRET)
#include <cmath>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "kws.hpp"
#include "kws_pte.h"
#include "test_kws.h"
#include "test_yolo.h"
#include "waldo/jpeg_frame.h"
#include "yolo_detector.hpp"
extern "C" const unsigned long yolo_pte_size;
#include "waldo/kws_mfcc.h"
#endif
#ifdef WALDO_FVP_TURRET
#include "waldo/mic.h"
extern "C" uint64_t ethos_npu_pmu_cycles_take(void);   // board/Corstone-320/ethos_setup.c
// No microphone on the FVP: kws_listen() links against this but is never called.
extern "C" uint32_t mic_read(int16_t *, uint32_t) { return 0; }
#endif
#ifdef WALDO_HW
#include "waldo/waldo.h"
#ifdef WALDO_TWO_BRAIN
#include "waldo/brain_hp.h"
#endif
#endif
#if defined(WALDO_HW) || defined(WALDO_FVP_TURRET)

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

// Object detector self-test: a photo of people, through the same JPEG decoder as camera
// frames, then YOLOv8n on the NPU must find at least one person.
static bool yolo_self_test() {
  static uint8_t rgb[240 * 320 * 3] __attribute__((section(".bss.sram1.yolo_test")));
  uint16_t w, h;
  if (jpeg_frame_decode(test_yolo_jpg, sizeof(test_yolo_jpg), rgb, nullptr, &w, &h) != 0) {
    printf("YOLO self-test: JPEG decode failed\nYOLO_result: FAIL\n");
    return false;
  }
  yolo_box b[8];
  const int n = yolo_detect_rgb(rgb, w, h, 0.35f, b, 8);
  uint32_t pre, npu, post;
  yolo_stage_us(&pre, &npu, &post);
  int people = 0;
  printf("YOLO self-test: %d detection(s) in %ux%u, letterbox %lu us, forward %lu us (NPU busy %lu us), decode %lu us\n",
         n, w, h, static_cast<unsigned long>(pre), static_cast<unsigned long>(npu),
         static_cast<unsigned long>(yolo_npu_hw_us()), static_cast<unsigned long>(post));
  for (int i = 0; i < n; i++) {
    printf("  %s %.2f at x %.0f y %.0f w %.0f h %.0f\n", yolo_class_name(b[i].cls), (double)b[i].score,
           (double)b[i].x, (double)b[i].y, (double)b[i].w, (double)b[i].h);
    people += b[i].cls == YOLO_PERSON;
  }
  printf("YOLO_result: %s\n", people > 0 ? "PASS" : "FAIL");
  return people > 0;
}
#endif

extern "C" int app_main(void) {
  executorch::runtime::runtime_init();
#ifdef WALDO_HW
  // The turret: keyword spotting and YOLOv8 on the Ethos-U85, each checked at boot.
#ifdef WALDO_TWO_BRAIN
  printf("WALDO two-brain: YOLOv8n (%lu B) on the HP's Ethos-U85; MicroNet KWS on the M55-HE's Ethos-U55\n",
         static_cast<unsigned long>(yolo_pte_size));
#else
  printf("WALDO: MicroNet KWS (%lu B) and YOLOv8n (%lu B) ExecuTorch programs on the Ethos-U85\n",
         static_cast<unsigned long>(kws_pte_size), static_cast<unsigned long>(yolo_pte_size));
#endif
#ifdef WALDO_TWO_BRAIN
  // Two-brain: the M55-HE listens (he/); here we only wait for its self-test.
  brain_hp_selftest(5000);
#else
  if (kws_init()) {
    kws_self_test();
  } else {
    printf("keyword spotter init failed\n");
  }
#endif
  if (yolo_init()) {
    yolo_self_test();
  } else {
    printf("object detector init failed\n");
  }
  if (waldo_init() == 0) {
    waldo_servo_test();
    waldo_home();
    while (true) {
      waldo_poll();
    }
  }
#else
  // FVP: the template's AI layer, BlazeFace, on a face photo built into the image.
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
#ifdef WALDO_FVP_TURRET
  printf("WALDO turret models on the FVP: MicroNet KWS (%lu B), YOLOv8n (%lu B)\n",
         static_cast<unsigned long>(kws_pte_size), static_cast<unsigned long>(yolo_pte_size));
  ethos_npu_pmu_cycles_take();
  if (kws_init()) {
    kws_self_test();
    printf("KWS NPU PMU cycles: %llu\n", static_cast<unsigned long long>(ethos_npu_pmu_cycles_take()));
  } else {
    printf("keyword spotter init failed\nKWS_result: FAIL\n");
  }
  if (yolo_init()) {
    yolo_self_test();
    printf("YOLO NPU PMU cycles: %llu\n", static_cast<unsigned long long>(ethos_npu_pmu_cycles_take()));
  } else {
    printf("object detector init failed\nYOLO_result: FAIL\n");
  }
#endif
#endif
  printf("\x04");  // EOT: ends the FVP run (semihosting exit); a board just sees a 0x04
  fflush(stdout);
  return 0;
}
