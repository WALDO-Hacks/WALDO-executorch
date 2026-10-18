// On-board microphones: the PDM driver fills one half of a double buffer from its FIFO
// interrupt while the other half is copied out, re-armed from the completion callback so
// capture never stops. Mic 0 goes into a ring buffer for keyword spotting (mic_read);
// both mics feed the level meter. Settings follow Alif's ML Embedded Evaluation Kit
// (mic_listener.c), whose KWS models are trained on audio captured this way.

#include "mic.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_PDM.h"
#include "se_services_port.h"
#include "services_lib_api.h"

extern ARM_DRIVER_PDM Driver_PDM;
static ARM_DRIVER_PDM *const pdm = &Driver_PDM;

#define CH0 4
#define CH1 5
#define PDM_PHASE           0x0000001F
#define PDM_GAIN            0x00000800
#define PDM_PEAK_DETECT_TH  0x00060002
#define PDM_PEAK_DETECT_ITV 0x0004002D
#define PDM_IIR_COEF        0x00000004
static const uint32_t ch_fir[18] = {
    0x00000001, 0x00000003, 0x00000003, 0x000007F4, 0x00000004, 0x000007ED,
    0x000007F5, 0x000007F4, 0x000007D3, 0x000007FE, 0x000007BC, 0x000007E5,
    0x000007D9, 0x00000793, 0x00000029, 0x0000072C, 0x00000072, 0x000002FD};

// 2 x 256 stereo frames: a block every 16 ms.
#define BLOCK_FRAMES 256u
static int16_t block[2][BLOCK_FRAMES * 2];
static volatile int filling;

#define RING_SIZE 16384u   // ~1 s of mono, power of two
static int16_t ring[RING_SIZE];
static volatile uint32_t ring_head, ring_tail;

static volatile uint64_t sq0, sq1;
static volatile uint32_t nsq;
static volatile uint32_t errors;

static void on_pdm(uint32_t event)
{
    if (event & ARM_PDM_EVENT_ERROR) errors++;
    if (!(event & ARM_PDM_EVENT_CAPTURE_COMPLETE)) return;

    const int16_t *b = block[filling];
    filling ^= 1;
    pdm->Receive(block[filling], BLOCK_FRAMES * 2);

    uint64_t s0 = 0, s1 = 0;
    uint32_t head = ring_head;
    for (uint32_t i = 0; i < BLOCK_FRAMES; i++) {
        int32_t a = b[2 * i], c = b[2 * i + 1];
        s0 += (uint32_t)(a * a);
        s1 += (uint32_t)(c * c);
        ring[(head + i) & (RING_SIZE - 1)] = (int16_t)a;
    }
    head += BLOCK_FRAMES;
    if (head - ring_tail > RING_SIZE) ring_tail = head - RING_SIZE;   // reader fell behind: keep the newest
    ring_head = head;
    sq0 += s0;
    sq1 += s1;
    nsq += BLOCK_FRAMES;
}

static int config_channel(uint32_t ch)
{
    PDM_CH_CONFIG c;
    if (pdm->Control(ARM_PDM_CHANNEL_PHASE, ch, PDM_PHASE) ||
        pdm->Control(ARM_PDM_CHANNEL_GAIN, ch, PDM_GAIN) ||
        pdm->Control(ARM_PDM_CHANNEL_PEAK_DETECT_TH, ch, PDM_PEAK_DETECT_TH) ||
        pdm->Control(ARM_PDM_CHANNEL_PEAK_DETECT_ITV, ch, PDM_PEAK_DETECT_ITV))
        return -1;
    c.ch_num = ch;
    memcpy(c.ch_fir_coef, ch_fir, sizeof(c.ch_fir_coef));
    c.ch_iir_coef = PDM_IIR_COEF;
    return pdm->Config(&c) == ARM_DRIVER_OK ? 0 : -1;
}

int mic_init(void)
{
    // The PDM block runs from the 76.8 MHz clock, which the Secure Enclave turns on.
    uint32_t err = 0;
    uint32_t ret = SERVICES_clocks_enable_clock(se_services_s_handle, CLKEN_HFOSCx2, true, &err);
    if (ret) printf("mic: HFOSCx2 clock enable failed (%lu, %lu)\n", (unsigned long)ret, (unsigned long)err);

    if (pdm->Initialize(on_pdm) || pdm->PowerControl(ARM_POWER_FULL) ||
        pdm->Control(ARM_PDM_SELECT_CHANNEL, (1u << CH0) | (1u << CH1), 0) ||
        pdm->Control(ARM_PDM_MODE, ARM_PDM_MODE_AUDIOFREQ_16K_DECM_64, 0) ||
        pdm->Control(ARM_PDM_SELECT_RESOLUTION, ARM_PDM_16BIT_RESOLUTION, 0) ||
        pdm->Control(ARM_PDM_BYPASS_IIR_FILTER, 1, 0) ||   // 1 = use the DC-blocking IIR
        config_channel(CH0) || config_channel(CH1)) {
        printf("mic: PDM setup failed\n");
        return -1;
    }
    filling = 0;
    if (pdm->Receive(block[0], BLOCK_FRAMES * 2) != ARM_DRIVER_OK) {
        printf("mic: PDM receive failed\n");
        return -1;
    }
    printf("mic: on-board PDM mics (channels %d, %d) at %d Hz\n", CH0, CH1, MIC_RATE);
    return 0;
}

uint32_t mic_read(int16_t *out, uint32_t max)
{
    uint32_t tail = ring_tail, avail = ring_head - tail;
    if (avail > max) avail = max;   // oldest first; the ring keeps the newest second
    for (uint32_t i = 0; i < avail; i++) out[i] = ring[(tail + i) & (RING_SIZE - 1)];
    ring_tail = tail + avail;
    return avail;
}

void mic_levels(float *rms0, float *rms1, uint32_t *drop)
{
    __disable_irq();
    uint64_t a = sq0, b = sq1;
    uint32_t n = nsq;
    sq0 = sq1 = 0;
    nsq = 0;
    __enable_irq();
    *rms0 = n ? sqrtf((float)a / n) : 0;
    *rms1 = n ? sqrtf((float)b / n) : 0;
    *drop = errors;
}
