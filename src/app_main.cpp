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

#include <executorch/runtime/platform/runtime.h>

#include "face_detector.hpp"
#include "model_pte.h"
#include "test_face.h"
#ifdef WALDO_HW
#include "waldo/waldo.h"
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
