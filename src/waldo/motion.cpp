#include "motion.h"

#include <stdio.h>

struct AxisState {
  servo_cal_t cal;
  float pos, target;
};

static AxisState ax[2];
static pwm_write_fn write_pwm;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

// Every pulse goes through here: the limits are enforced again at the last step, so no
// caller (park, tracker, voice, slew, a trimmed centre) can drive past them.
static float deg_to_us(const AxisState &a, float deg) {
  const float lim = clampf(deg, a.cal.min_deg, a.cal.max_deg);
  static bool told;
  if (lim != deg && !told) {
    printf("motion: %.1f deg clamped to %.1f (limits %.0f..%.0f)\n", (double)deg, (double)lim,
           (double)a.cal.min_deg, (double)a.cal.max_deg);
    told = true;
  }
  return a.cal.center_us + a.cal.sign * lim * a.cal.us_per_deg;
}

void motion_init(pwm_write_fn write, int pan_center_us, int tilt_center_us) {
  write_pwm = write;
  ax[AXIS_PAN].cal = CAL_PAN;
  ax[AXIS_TILT].cal = CAL_TILT;
  if (pan_center_us > 0) ax[AXIS_PAN].cal.center_us = pan_center_us;
  if (tilt_center_us > 0) ax[AXIS_TILT].cal.center_us = tilt_center_us;
  // Start at the centre, where servo_hw_init() already holds both servos, and ramp to
  // the park pose at the slew rate instead of snapping there.
  for (int i = 0; i < 2; i++) ax[i].pos = clampf(0, ax[i].cal.min_deg, ax[i].cal.max_deg);
  ax[AXIS_PAN].target = PARK_PAN;
  ax[AXIS_TILT].target = PARK_TILT;
  for (int i = 0; i < 2; i++) write_pwm(i, deg_to_us(ax[i], ax[i].pos));
}

void motion_set(int axis, float deg) {
  AxisState &a = ax[axis];
  a.target = clampf(deg, a.cal.min_deg, a.cal.max_deg);   // the ramp's target, so the slew stops at the limit
  static bool told[2];
  if (a.target != deg && !told[axis]) {   // once per axis
    printf("motion: %s command %.1f deg clamped to %.1f\n", axis == AXIS_TILT ? "tilt" : "pan", (double)deg,
           (double)a.target);
    told[axis] = true;
  }
}

void motion_nudge(int axis, float ddeg) { motion_set(axis, ax[axis].target + ddeg); }

void motion_park(void) {
  motion_set(AXIS_PAN, PARK_PAN);
  motion_set(AXIS_TILT, PARK_TILT);
}

void motion_tick(float dt_s) {
  float step = SLEW_DEG_PER_S * dt_s;
  for (int i = 0; i < 2; i++) {
    AxisState &a = ax[i];
    a.pos += clampf(a.target - a.pos, -step, step);
    write_pwm(i, deg_to_us(a, a.pos));
  }
}

float motion_pos(int axis) { return ax[axis].pos; }
float motion_target(int axis) { return ax[axis].target; }
int motion_center_us(int axis) { return ax[axis].cal.center_us; }
void motion_trim(int axis, int dus) { ax[axis].cal.center_us += dus; }
