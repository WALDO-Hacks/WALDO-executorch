#include "motion.h"

struct AxisState {
  servo_cal_t cal;
  float pos, target;
};

static AxisState ax[2];
static pwm_write_fn write_pwm;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

static float deg_to_us(const AxisState &a, float deg) {
  return a.cal.center_us + a.cal.sign * deg * a.cal.us_per_deg;
}

void motion_init(pwm_write_fn write, int pan_center_us, int tilt_center_us) {
  write_pwm = write;
  ax[AXIS_PAN].cal = CAL_PAN;
  ax[AXIS_TILT].cal = CAL_TILT;
  if (pan_center_us > 0) ax[AXIS_PAN].cal.center_us = pan_center_us;
  if (tilt_center_us > 0) ax[AXIS_TILT].cal.center_us = tilt_center_us;
  ax[AXIS_PAN].pos = ax[AXIS_PAN].target = PARK_PAN;
  ax[AXIS_TILT].pos = ax[AXIS_TILT].target = PARK_TILT;
  for (int i = 0; i < 2; i++) write_pwm(i, deg_to_us(ax[i], ax[i].pos));
}

void motion_set(int axis, float deg) {
  AxisState &a = ax[axis];
  a.target = clampf(deg, a.cal.min_deg, a.cal.max_deg);
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
