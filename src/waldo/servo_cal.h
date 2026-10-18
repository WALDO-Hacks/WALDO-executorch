// Per-axis servo calibration (FIRMWARE_HANDOFF.md, Step 1).
// On the ESP32 build, center_us is overridden at boot by the zero saved with
// the jog sketch (firmware/zero_esp32), if there is one.
// Convention: +pan = turret turns LEFT (counter-clockwise seen from above), +tilt = beam UP.
// Fix `sign` per axis so that holds on the real turret.
#pragma once

typedef struct {
  int center_us;
  float us_per_deg;
  int sign;
  float min_deg, max_deg;
} servo_cal_t;

// ---- Tilt limits: a laser pointer is taped UNDER the flashlight on the tilt head and is
// always on. Tilting down, it reaches the deck before the flashlight does, and a 40 kg.cm
// servo would break it, so tilt never goes below TILT_MIN_DEG.
// How TILT_MIN_DEG is measured: servo power OFF, tilt the head down by hand until the laser
// just touches the deck, read that angle (0 = level, negative = down; the boot log prints
// the commanded tilt), then add 5 degrees of margin (e.g. touches at -50 -> -45).
// PLACEHOLDER until measured.
static const float TILT_MIN_DEG = -15;   // conservative until measured (the head hit at -45)
static const float TILT_MAX_DEG = 30;
static const float PAN_MIN_DEG = -90, PAN_MAX_DEG = 90;
// Park ("off", and power-up): as far down as is safe. With the laser and flashlight always
// on, they light the deck there; parking no longer blacks the beam out.
static const float PARK_PAN = 0, PARK_TILT = TILT_MIN_DEG;

static const servo_cal_t CAL_PAN  = { 1500, 2000.0f / 180, -1, PAN_MIN_DEG, PAN_MAX_DEG };
static const servo_cal_t CAL_TILT = { 1500, 2000.0f / 180, +1, TILT_MIN_DEG, TILT_MAX_DEG };
static const float SLEW_DEG_PER_S = 120;   // handoff: 90-180 deg/s, tune on the real turret
