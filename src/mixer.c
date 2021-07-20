#include "mixer.h"

uint16_t motor[MAX_SUPPORTED_MOTORS] = {0};
uint16_t motor_disarmed[MAX_SUPPORTED_MOTORS] = {0};
static float motor_mix_range = 0.0;
static float mixer_scale = 1.0f;
static motor_mixer_ts current_mixer[MAX_SUPPORTED_MOTORS];
static uint8_t motor_count = 0;
int mixer_throttle_command = 0;
static int throttle_idle_value = 0;
static int motor_salue_shen_stopped = 0;
static reversible_motors_throttle_state_te reversible_motors_throttle_state = MOTOR_DIRECTION_FORWARD;
static int throttle_deadband_low = 0;
static int throttle_deadband_high = 0;
static int throttle_range_min = 0;
static int throttle_range_max = 0;
static int8_t motor_yaw_multiplier = 1;
int motor_zero_command = 0;

reversible_motors_config_ts reversible_motors_config = {
    .deadband_low   = SETTING_3D_DEADBAND_LOW_DEFAULT,
    .deadband_high  = SETTING_3D_DEADBAND_HIGH_DEFAULT,
    .neutral        = SETTING_3D_NEUTRAL_DEFAULT
};

mixer_config_ts mixer_config = {
    .motor_direction_inverted   = SETTING_MOTOR_DIRECTION_INVERTED_DEFAULT,
    .platform_type              = SETTING_PLATFORM_TYPE_DEFAULT,
    .has_flaps                  = SETTING_HAS_FLAPS_DEFAULT,
    .applied_mixer_preset       = SETTING_MODEL_PREVIEW_TYPE_DEFAULT // Этот флаг недоступен в CLI и используется только Конфигуратором
};

motor_config_ts motor_config = {
    .max_throttle           = SETTING_MAX_THROTTLE_DEFAULT,
    .min_command            = SETTING_MIN_COMMAND_DEFAULT,
    .motor_pwm_rate         = SETTING_MOTOR_PWM_RATE_DEFAULT,
    .motor_pwm_protocol     = SETTING_MOTOR_PWM_PROTOCOL_DEFAULT,
    .motor_accel_time_ms    = SETTING_MOTOR_ACCEL_TIME_DEFAULT,
    .motor_decel_time_ms    = SETTING_MOTOR_DECEL_TIME_DEFAULT,
    .motor_pole_count       = SETTING_MOTOR_POLES_DEFAULT           // Большинство используемых нами бесщеточных двигателей 14-полюсные.
};

motor_mixer_ts primary_motor_mixer[MAX_SUPPORTED_MOTORS];

typedef void (*motor_rate_limiting_apply_fn_ptr) (const float dT);
static motor_rate_limiting_apply_fn_ptr motor_rate_limiting_apply_fn;

static void compute_motor_count (void);

int get_throttle_idle_value (void)
{
    if (!throttle_idle_value)
        throttle_idle_value = motor_config.min_command + (((motor_config.max_throttle - motor_config.min_command) / 100.0F) * current_byttery_profile.motor.throttle_idle);
    
    return throttle_idle_value;
}

static void compute_motor_count (void)
{
    motor_count = 0;

    for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (primary_motor_mixer[i].throttle == 0.0F)
            break;
        motor_count++;
    }
}

uint8_t get_motor_count (void)
{
    return motor_count;
}

float get_motor_mix_range (void)
{
    return motor_mix_range;
}

bool mixer_is_output_saturated (void)
{
    return motor_mix_range >= 1.0F;
}

void mixer_update_state_flags (void)
{

}