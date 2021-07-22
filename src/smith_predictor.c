#include "smith_predictor.h"

float apply_smith_predictor (uint8_t axis, smith_predictor_ts *predictor, float sample) 
{
    UNUSED(axis);

    if (predictor->enabled) {
        predictor->data[predictor->idx] = sample;

        predictor->idx++;
        if (predictor->idx > predictor->samples) {
            predictor->idx = 0;
        }

        // filter the delayed data to help reduce the overall noise this prediction adds
        float delayed = pt1_filter_apply(&predictor->smith_predictor_filter, predictor->data[predictor->idx]);
        float delay_compensated_sample = predictor->smith_predictor_strength * (sample - delayed);

        sample += delay_compensated_sample;
    }
    return sample;
}