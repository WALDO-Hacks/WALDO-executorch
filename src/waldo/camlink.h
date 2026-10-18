// Camera link: ESP32-S3-CAM -> E8 over UART. Shared by both ends.
// One frame on the wire:
//   magic "WLDF" | u16 width | u16 height | u8 format | u8 seq | u32 len   (16 bytes, little-endian)
//   payload (len bytes)
//   u16 Fletcher-16 of the payload
// The receiver hunts for the magic, so it can join mid-stream and recover from dropped bytes.
#pragma once
#include <stdint.h>

#define CAMLINK_BAUD        2000000
#define CAMLINK_FMT_GRAY8   1   // raw 8-bit grayscale, width x height bytes
#define CAMLINK_FMT_JPEG    2   // a baseline JPEG (colour), len bytes

typedef struct __attribute__((packed)) {
  char magic[4];   // "WLDF"
  uint16_t width;
  uint16_t height;
  uint8_t format;
  uint8_t seq;
  uint16_t reserved;
  uint32_t len;
} camlink_hdr_t;

static inline uint16_t camlink_fletcher16(const uint8_t *d, uint32_t n) {
  uint32_t a = 0, b = 0;
  while (n) {
    uint32_t chunk = n > 360 ? 360 : n;   // keep the sums from overflowing before the modulo
    n -= chunk;
    while (chunk--) { a += *d++; b += a; }
    a %= 255; b %= 255;
  }
  return (uint16_t)((b << 8) | a);
}
