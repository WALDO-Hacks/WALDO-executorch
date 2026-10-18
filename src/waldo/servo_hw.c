#include "servo_hw.h"

#include <stdio.h>
#include "Driver_UTIMER.h"
#include "pinconf.h"

extern ARM_DRIVER_UTIMER Driver_UTIMER0;
static ARM_DRIVER_UTIMER *const ut = &Driver_UTIMER0;

#define UT_COUNTS_PER_US 400U                 /* UTIMER clock: 400 MHz on the E8 */
#define PERIOD_COUNTS    (20000U * UT_COUNTS_PER_US)

static const uint8_t channel[2] = {10, 5};

/* The driver insists on a callback; the PWM needs no events. */
static void ut_event(uint8_t event) { (void)event; }

static int setup_channel(uint8_t ch)
{
    if (ut->Initialize(ch, ut_event) != ARM_DRIVER_OK) return -1;
    if (ut->PowerControl(ch, ARM_POWER_FULL) != ARM_DRIVER_OK) return -1;
    if (ut->ConfigCounter(ch, ARM_UTIMER_MODE_COMPARING, ARM_UTIMER_COUNTER_UP) != ARM_DRIVER_OK) return -1;
    if (ut->SetCount(ch, ARM_UTIMER_CNTR, 0) != ARM_DRIVER_OK) return -1;
    if (ut->SetCount(ch, ARM_UTIMER_CNTR_PTR, PERIOD_COUNTS) != ARM_DRIVER_OK) return -1;
    if (ut->SetCount(ch, ARM_UTIMER_COMPARE_A, 1500U * UT_COUNTS_PER_US) != ARM_DRIVER_OK) return -1;
    if (ut->Start(ch) != ARM_DRIVER_OK) return -1;
    return 0;
}

int servo_hw_init(void)
{
    /* Route the pins to the timers (alternate functions from the E8 pin mux table). */
    pinconf_set(PORT_7, PIN_4, PINMUX_ALTERNATE_FUNCTION_6, PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);
    pinconf_set(PORT_6, PIN_2, PINMUX_ALTERNATE_FUNCTION_5, PADCTRL_OUTPUT_DRIVE_STRENGTH_4MA);

    for (int i = 0; i < 2; i++) {
        if (setup_channel(channel[i]) != 0) {
            printf("servo: UTIMER channel %u setup failed\n", channel[i]);
            return -1;
        }
    }
    return 0;
}

void servo_hw_write_us(int axis, float us)
{
    ut->SetCount(channel[axis], ARM_UTIMER_COMPARE_A, (uint32_t)(us * UT_COUNTS_PER_US + 0.5f));
}
