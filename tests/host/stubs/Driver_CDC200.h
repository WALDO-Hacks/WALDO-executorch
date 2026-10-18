// Host stand-in for Alif's CDC200 display driver: every call succeeds and does nothing.
#pragma once
#include <stdint.h>
#define ARM_DRIVER_OK 0
#define ARM_POWER_FULL 2
#define ARM_CDC_DSI_ERROR_EVENT 1
#define CDC200_CONFIGURE_DISPLAY 1
typedef struct {
    int32_t (*Initialize)(void (*cb)(uint32_t));
    int32_t (*PowerControl)(int state);
    int32_t (*Control)(uint32_t control, uint32_t arg);
    int32_t (*Start)(void);
} ARM_DRIVER_CDC200;
