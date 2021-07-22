#include "control_rate_profile.h"
#include "rc_curves.h"

control_rate_config_ts control_rate_profiles[MAX_CONTROL_RATE_PROFILE_COUNT];
control_rate_config_ts current_control_rate_profile;

void init_control_rate_profile (void)
{
    for (uint8_t i = 0; i < MAX_CONTROL_RATE_PROFILE_COUNT; i++) {
        control_rate_profiles[i].throttle.rc_mid_8          = SETTING_THR_MID_DEFAULT;
        control_rate_profiles[i].throttle.rc_expo_8         = SETTING_THR_EXPO_DEFAULT;
        control_rate_profiles[i].throttle.dyn_pid           = SETTING_TPA_RATE_DEFAULT;
        control_rate_profiles[i].throttle.pa_breakpoint     = SETTING_TPA_BREAKPOINT_DEFAULT;
        control_rate_profiles[i].throttle.fixed_wing_tau_ms = SETTING_FW_TPA_TIME_CONSTANT_DEFAULT;

        control_rate_profiles[i].stabilized.rc_expo_8       = SETTING_RC_EXPO_DEFAULT;
        control_rate_profiles[i].stabilized.rc_yaw_expo_8   = SETTING_RC_YAW_EXPO_DEFAULT;
        control_rate_profiles[i].stabilized.rates[FD_ROLL]  = SETTING_ROLL_RATE_DEFAULT;
        control_rate_profiles[i].stabilized.rates[FD_PITCH] = SETTING_PITCH_RATE_DEFAULT;
        control_rate_profiles[i].stabilized.rates[FD_YAW]   = SETTING_YAW_RATE_DEFAULT;

        control_rate_profiles[i].manual.rc_expo_8       = SETTING_MANUAL_RC_EXPO_DEFAULT;
        control_rate_profiles[i].manual.rc_yaw_expo_8   = SETTING_MANUAL_RC_YAW_EXPO_DEFAULT;
        control_rate_profiles[i].manual.rates[FD_ROLL]  = SETTING_MANUAL_ROLL_RATE_DEFAULT;
        control_rate_profiles[i].manual.rates[FD_PITCH] = SETTING_MANUAL_PITCH_RATE_DEFAULT;
        control_rate_profiles[i].manual.rates[FD_YAW]   = SETTING_MANUAL_YAW_RATE_DEFAULT;

        control_rate_profiles[i].misc.fpv_cam_angle_degrees = SETTING_FPV_MIX_DEGREES_DEFAULT;
    }
}

void set_control_rate_profile (uint8_t profile_index)
{
    if (profile_index >= MAX_CONTROL_RATE_PROFILE_COUNT) {
        profile_index = 0;
    }
    
    current_control_rate_profile = control_rate_profiles[profile_index];
}

void activate_control_rate_config (void)
{
    generate_throttle_curve(&current_control_rate_profile);
}