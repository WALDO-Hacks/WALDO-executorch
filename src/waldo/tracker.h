// Turns detection boxes into turret motion. The camera rides on the tilt head, so each
// frame nudges the turret a fraction of the way toward centring the target.
#pragma once
#include <stdint.h>
#include "detector.h"

void tracker_update(const struct det_box *boxes, int n, int img_w, int img_h, uint32_t now_ms);
void tracker_print(void);
// Index (in the last boxes passed to tracker_update) of the target being followed, or -1.
int tracker_target_index(void);
