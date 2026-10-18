// WALDO on the E8 DevKit: servos, motion, (soon) camera link and tracking.
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

// Brings up the servos and parks the turret. 0 on success.
int waldo_init(void);
// Milliseconds since waldo_init (SysTick).
uint32_t waldo_millis(void);
// Advances the slew-limited motion; call often (it steps every 20 ms).
void waldo_poll(void);
// Bench test: pan to +30 and back, tilt to 0 and back, then park. Blocks ~6 s.
void waldo_servo_test(void);
// Pan 0, tilt 0: looking straight ahead, ready to track.
void waldo_home(void);

#ifdef __cplusplus
}
#endif
