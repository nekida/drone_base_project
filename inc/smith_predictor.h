#ifndef _SMITH_PREDICTOR_H
#define _SMITH_PREDICTOR_H

#include <stdint.h>
#include "filter.h"
#include "utils.h"
#include <stdbool.h>

#define MAX_SMITH_SAMPLES 64

typedef struct {
    bool enabled;
    uint8_t samples;
    uint8_t idx;
    float data[MAX_SMITH_SAMPLES + 1];
    pt1_filter_ts smith_predictor_filter;
    float smith_predictor_strength;
} smith_predictor_ts;

float apply_smith_predictor (uint8_t axis, smith_predictor_ts *predictor, float sample);
void smith_predictor_init   (smith_predictor_ts *predictor, float delay, float strength, uint16_t filter_lpf_hz, uint32_t looptime);

#endif // _SMITH_PREDICTOR_H