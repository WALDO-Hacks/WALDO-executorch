// WALDO servo outputs on the E8 DevKit J10 header (level-shifted to 3.3 V):
//   pan  = J10 pin 27 "C7_4" = P7_4 = UTIMER channel 10, driver A
//   tilt = J10 pin 20 "C6_2" = P6_2 = UTIMER channel 5,  driver A
// 50 Hz PWM: high at the start of each 20 ms period, low at the compare match.
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

int servo_hw_init(void);                       // 0 on success
void servo_hw_write_us(int axis, float us);    // axis 0 = pan, 1 = tilt

#ifdef __cplusplus
}
#endif
