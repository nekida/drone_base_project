#include "rc_curves.h"

static const uint16_t dummy_variable = 0; // dummy for motorConfig()->maxthrottle
#define get_throttle_idle_value() 0

static int16_t lookup_throttle_RC[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE
int16_t lookup_throttle_RC_mid;                         // THROTTLE curve mid point

void generate_throttle_curve (const control_rate_config_ts *control_rate_config)
{
    const int min_throttle = get_throttle_idle_value();
    lookup_throttle_RC_mid = min_throttle + (int32_t)(dummy_variable - min_throttle) * control_rate_config->throttle.rc_mid_8 / 100; // [MINTHROTTLE;MAXTHROTTLE]

    for (int i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        const int16_t tmp = 10 * i - control_rate_config->throttle.rc_mid_8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - control_rate_config->throttle.rc_mid_8;
        if (tmp < 0)
            y = control_rate_config->throttle.rc_mid_8;
        lookup_throttle_RC[i] = 10 * control_rate_config->throttle.rc_mid_8 + tmp * (100 - control_rate_config->throttle.rc_expo_8 + (int32_t)control_rate_config->throttle.rc_expo_8 * (tmp * tmp) / (y * y)) / 10;
        lookup_throttle_RC[i] = min_throttle + (int32_t)(dummy_variable - min_throttle) * lookup_throttle_RC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }
}

int16_t rc_lookup (int32_t stick_deflection, uint8_t expo)
{
    float tmpf = stick_deflection / 100.0F;

    return lrintf((2500.0F + (float)expo * (tmpf * tmpf - 25.0F)) * tmpf / 25.0F);
}

uint16_t rc_lookup_throttle (uint16_t absolute_deflection)
{
    if (absolute_deflection > 999)
        return dummy_variable;

    const uint8_t lookup_step = absolute_deflection / 100;
    return lookup_throttle_RC[lookup_step] + (absolute_deflection - lookup_step * 100) * (lookup_throttle_RC[lookup_step + 1] - lookup_throttle_RC[lookup_step]) / 100;
}

int16_t rc_lookup_throttle_mid (void)
{
    return lookup_throttle_RC_mid;
}