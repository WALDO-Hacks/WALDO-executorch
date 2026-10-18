// JPEG decoding with ChaN's TJpgDec (tjpgd/, no restrictions on use): the decoder hands
// over 8x8 / 16x16 blocks of RGB888, written straight into the output image.

#include "jpeg_frame.h"

#include <string.h>

#include "tjpgd/tjpgd.h"

typedef struct {
    const uint8_t *src;
    uint32_t len, pos;
    uint8_t *rgb;
    uint16_t w;
} dev_t_;

static uint8_t work[16384] __attribute__((aligned(4)));  // TJpgDec work area (fast mode needs > 8 KB)

static size_t in(JDEC *jd, uint8_t *buf, size_t n)
{
    dev_t_ *d = (dev_t_ *)jd->device;
    if (n > d->len - d->pos) n = d->len - d->pos;
    if (buf) memcpy(buf, d->src + d->pos, n);
    d->pos += n;
    return n;
}

static int out(JDEC *jd, void *bitmap, JRECT *r)
{
    dev_t_ *d = (dev_t_ *)jd->device;
    const uint8_t *p = (const uint8_t *)bitmap;
    const int bw = (r->right - r->left + 1) * 3;
    for (int y = r->top; y <= r->bottom; y++, p += bw)
        memcpy(d->rgb + ((uint32_t)y * d->w + r->left) * 3, p, bw);
    return 1;
}

int jpeg_frame_decode(const uint8_t *jpg, uint32_t len, uint8_t *rgb, uint8_t *gray,
                      uint16_t *w, uint16_t *h)
{
    JDEC jd;
    dev_t_ d = {jpg, len, 0, rgb, 0};
    if (jd_prepare(&jd, in, work, sizeof(work), &d) != JDR_OK) return -1;
    if (jd.width > JPEG_MAX_W || jd.height > JPEG_MAX_H) return -2;
    d.w = jd.width;
    if (jd_decomp(&jd, out, 0) != JDR_OK) return -3;
    *w = jd.width;
    *h = jd.height;
    if (gray) {
        const int gw = jd.width / 2, gh = jd.height / 2, stride = jd.width * 3;
        for (int y = 0; y < gh; y++) {
            const uint8_t *a = rgb + (2 * y) * stride, *b = a + stride;
            for (int x = 0; x < gw; x++, a += 6, b += 6) {
                // luma ~ (R + 2G + B) / 4, summed over the 2x2 block
                unsigned s = a[0] + 2 * a[1] + a[2] + a[3] + 2 * a[4] + a[5] +
                             b[0] + 2 * b[1] + b[2] + b[3] + 2 * b[4] + b[5];
                gray[y * gw + x] = (uint8_t)(s >> 4);
            }
        }
    }
    return 0;
}

void frame_rotate(const uint8_t *src, uint8_t *dst, uint16_t w, uint16_t h, int bpp, int deg,
                  uint16_t *ow, uint16_t *oh)
{
    if (deg == 0) {
        memcpy(dst, src, (size_t)w * h * bpp);
        *ow = w;
        *oh = h;
        return;
    }
    const uint16_t nw = deg == 180 ? w : h, nh = deg == 180 ? h : w;
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            uint32_t nx, ny;
            if (deg == 90) { nx = h - 1 - y; ny = x; }
            else if (deg == 180) { nx = w - 1 - x; ny = h - 1 - y; }
            else { nx = y; ny = w - 1 - x; }   /* 270 */
            memcpy(dst + (ny * nw + nx) * bpp, src + (y * w + x) * bpp, bpp);
        }
    }
    *ow = nw;
    *oh = nh;
}
