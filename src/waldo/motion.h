// Motion layer (FIRMWARE_HANDOFF.md, Step 2): limits, calibration, slew.
// Plain C++ with no Arduino calls, so it ports to the E8 as it is: give it a
// function that writes a pulse width and call motion_tick() at a fixed rate.
#pragma once
#include "servo_cal.h"

enum { AXIS_PAN = 0, AXIS_TILT = 1 };

typedef void (*pwm_write_fn)(int axis, float us);

// center_us < 0 keeps the value from servo_cal.h. Outputs the centre (what servo_hw_init
// already holds) and ramps to the park pose at the slew rate. Every pulse is clamped to the
// limits in servo_cal.h (TILT_MIN_DEG keeps the laser off the deck).
void motion_init(pwm_write_fn write, int pan_center_us, int tilt_center_us);
void motion_set(int axis, float deg);     // absolute target, clamped to the axis limits
void motion_nudge(int axis, float ddeg);  // relative to the current target
void motion_park(void);
void motion_tick(float dt_s);             // slew toward the targets and write PWM
// Eased: moves speed up and slow down gently (patrol, boot, looking around); off: straight at
// the slew rate, the most direct response (tracking).
void motion_set_eased(bool on);
float motion_pos(int axis);               // where the servo is being driven now
float motion_target(int axis);
int motion_center_us(int axis);
void motion_trim(int axis, int dus);       // shift the zero by dus microseconds (servo zeroing)
