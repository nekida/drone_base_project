#ifndef _CONTROL_RATE_PROFILE_H
#define _CONTROL_RATE_PROFILE_H

#include <stdint.h>
#include "pid.h"

#define MAX_CONTROL_RATE_PROFILE_COUNT 3

enum {
    SETTING_THR_MID_DEFAULT = 50,
    SETTING_THR_MID = 175,
    SETTING_THR_MID_MIN = 0,
    SETTING_THR_MID_MAX = 100,
    SETTING_THR_EXPO_DEFAULT = 0,
    SETTING_THR_EXPO = 176,
    SETTING_THR_EXPO_MIN = 0,
    SETTING_THR_EXPO_MAX = 100,
    SETTING_TPA_RATE_DEFAULT = 0,
    SETTING_TPA_RATE = 177,
    SETTING_TPA_RATE_MIN = 0,
    SETTING_TPA_RATE_MAX = 100,
    SETTING_TPA_BREAKPOINT_DEFAULT = 1500,
    SETTING_TPA_BREAKPOINT = 178,
    SETTING_TPA_BREAKPOINT_MIN = 1000,
    SETTING_TPA_BREAKPOINT_MAX = 2000,
    SETTING_FW_TPA_TIME_CONSTANT_DEFAULT = 0,
    SETTING_FW_TPA_TIME_CONSTANT = 179,
    SETTING_FW_TPA_TIME_CONSTANT_MIN = 0,
    SETTING_FW_TPA_TIME_CONSTANT_MAX = 5000,
    SETTING_RC_EXPO_DEFAULT = 70,
    SETTING_RC_EXPO = 180,
    SETTING_RC_EXPO_MIN = 0,
    SETTING_RC_EXPO_MAX = 100,
    SETTING_RC_YAW_EXPO_DEFAULT = 20,
    SETTING_RC_YAW_EXPO = 181,
    SETTING_RC_YAW_EXPO_MIN = 0,
    SETTING_RC_YAW_EXPO_MAX = 100,
    SETTING_ROLL_RATE_DEFAULT = 20,
    SETTING_ROLL_RATE = 182,
    SETTING_ROLL_RATE_MIN = 4,
    SETTING_ROLL_RATE_MAX = 180,
    SETTING_PITCH_RATE_DEFAULT = 20,
    SETTING_PITCH_RATE = 183,
    SETTING_PITCH_RATE_MIN = 4,
    SETTING_PITCH_RATE_MAX = 180,
    SETTING_YAW_RATE_DEFAULT = 20,
    SETTING_YAW_RATE = 184,
    SETTING_YAW_RATE_MIN = 1,
    SETTING_YAW_RATE_MAX = 180,
    SETTING_MANUAL_RC_EXPO_DEFAULT = 70,
    SETTING_MANUAL_RC_EXPO = 185,
    SETTING_MANUAL_RC_EXPO_MIN = 0,
    SETTING_MANUAL_RC_EXPO_MAX = 100,
    SETTING_MANUAL_RC_YAW_EXPO_DEFAULT = 20,
    SETTING_MANUAL_RC_YAW_EXPO = 186,
    SETTING_MANUAL_RC_YAW_EXPO_MIN = 0,
    SETTING_MANUAL_RC_YAW_EXPO_MAX = 100,
    SETTING_MANUAL_ROLL_RATE_DEFAULT = 100,
    SETTING_MANUAL_ROLL_RATE = 187,
    SETTING_MANUAL_ROLL_RATE_MIN = 0,
    SETTING_MANUAL_ROLL_RATE_MAX = 100,
    SETTING_MANUAL_PITCH_RATE_DEFAULT = 100,
    SETTING_MANUAL_PITCH_RATE = 188,
    SETTING_MANUAL_PITCH_RATE_MIN = 0,
    SETTING_MANUAL_PITCH_RATE_MAX = 100,
    SETTING_MANUAL_YAW_RATE_DEFAULT = 100,
    SETTING_MANUAL_YAW_RATE = 189,
    SETTING_MANUAL_YAW_RATE_MIN = 0,
    SETTING_MANUAL_YAW_RATE_MAX = 100,
    SETTING_FPV_MIX_DEGREES_DEFAULT = 0,
    SETTING_FPV_MIX_DEGREES = 190,
    SETTING_FPV_MIX_DEGREES_MIN = 0,
    SETTING_FPV_MIX_DEGREES_MAX = 50,
};

typedef struct {
    struct {
        uint8_t rc_mid_8;
        uint8_t rc_expo_8;
        uint8_t dyn_pid;
        uint16_t pa_breakpoint;                // Breakpoint where TPA is activated
        uint16_t fixed_wing_tau_ms;               // Time constant of airplane TPA PT1-filter
    } throttle;

    struct {
        uint8_t rc_expo_8;
        uint8_t rc_yaw_expo_8;
        uint8_t rates[3];
    } stabilized;

    struct {
        uint8_t rc_expo_8;
        uint8_t rc_yaw_expo_8;
        uint8_t rates[3];
    } manual;

    struct {
        uint8_t fpv_cam_angle_degrees;             // Camera angle to treat as "forward" base axis in ACRO (Roll and Yaw sticks will command rotation considering this axis)
    } misc;

} control_rate_config_ts;

extern control_rate_config_ts current_control_rate_profile;

void    set_control_rate_profile            (uint8_t profile_index);
void    change_control_rate_profile         (uint8_t profile_ndex);
void    activate_control_rate_config        (void);
uint8_t get_current_control_rate_profile    (void);

#endif // _CONTROL_RATE_PROFILE_H