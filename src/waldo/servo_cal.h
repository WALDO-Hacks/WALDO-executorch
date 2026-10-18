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

static const servo_cal_t CAL_PAN  = { 1500, 2000.0f / 180, -1, -90, 90 };
static const servo_cal_t CAL_TILT = { 1500, 2000.0f / 180, +1, -90, 30 };

static const float PARK_PAN = 0, PARK_TILT = -90;
static const float SLEW_DEG_PER_S = 120;   // handoff: 90-180 deg/s, tune on the real turret
