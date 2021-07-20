#ifndef _RC_CURVES_H
#define _RC_CURVES_H

#include <stdint.h>
#include "control_rate_profile.h"

#define PITCH_LOOKUP_LENGTH 7
#define YAW_LOOKUP_LENGTH 7
#define THROTTLE_LOOKUP_LENGTH 11

void generate_throttle_curve (const control_rate_config_ts *control_rate_config);
int16_t rc_lookup (int32_t stick_deflection, uint8_t expo);
uint16_t rc_lookup_throttle (uint16_t absolute_deflection);
int16_t rc_lookup_throttle_mid (void);

#endif // _RC_CURVES_H