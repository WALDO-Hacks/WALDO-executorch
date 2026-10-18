// fvp-sim: the M55-HE's keyword-spotter self-test (he/he_app.cpp, src/app_main.cpp) on the
// Corstone-300 FVP's Ethos-U55-128: MFCCs of the recorded "yes" on the M55 (checked against
// the ones Python computed), then the HE's MicroNet-M program must say "yes".
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include <executorch/runtime/platform/runtime.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "kws.hpp"
#include "kws_pte.h"
#include "test_kws.h"
#include "waldo/kws_mfcc.h"
#include "waldo/mic.h"

extern "C" int stdio_init(void);
extern "C" int ethos_setup(void);
extern "C" uint64_t ethos_npu_pmu_cycles_take(void);

// No microphone on the FVP: kws_listen() links against this but is never called.
extern "C" uint32_t mic_read(int16_t *, uint32_t) { return 0; }

// Semihosting SYS_EXIT (ADP_Stopped_ApplicationExit): ends the FVP run.
static void fvp_exit(void)
{
  register int r0 __asm__("r0") = 0x18;
  register int r1 __asm__("r1") = 0x20026;
  __asm__ volatile("bkpt 0xAB" : "+r"(r0) : "r"(r1) : "memory");
}

int main(void)
{
  stdio_init();
  ethos_setup();
  executorch::runtime::runtime_init();
  printf("WALDO HE keyword spotter on the Corstone-300 FVP: MicroNet-M for the Ethos-U55-128 (%lu B)\n",
         static_cast<unsigned long>(kws_pte_size));
  bool pass = false;
  if (kws_init()) {
    static float feats[KWS_FRAMES * KWS_COEFFS];
    float err = 0;
    for (int f = 0; f < KWS_FRAMES; f++) kws_mfcc_frame(test_kws_yes + f * KWS_STRIDE, feats + f * KWS_COEFFS);
    for (int i = 0; i < KWS_FRAMES * KWS_COEFFS; i++) err = std::fmax(err, std::fabs(feats[i] - test_kws_yes_mfcc[i]));
    float probs[KWS_LABELS];
    ethos_npu_pmu_cycles_take();
    const int best = kws_classify(feats, probs);
    printf("KWS self-test (U55-128): heard \"%s\" (%.2f), max MFCC diff to Python %.4f\n", kws_label(best),
           (double)(best >= 0 ? probs[best] : 0), (double)err);
    for (int i = 0; i < KWS_LABELS; i++) printf("  %-10s %.3f\n", kws_label(i), (double)probs[i]);
    printf("KWS NPU PMU cycles: %llu\n", static_cast<unsigned long long>(ethos_npu_pmu_cycles_take()));
    pass = best >= 0 && std::string_view(kws_label(best)) == "yes" && err < 0.05f;
  } else {
    printf("keyword spotter init failed\n");
  }
  printf("KWS_HE_result: %s\n", pass ? "PASS" : "FAIL");
  fflush(stdout);
  fvp_exit();
  while (true) {
  }
}
