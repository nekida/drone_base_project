#include "mixer.h"

uint16_t motor[MAX_SUPPORTED_MOTORS] = {0};
uint16_t motor_disarmed[MAX_SUPPORTED_MOTORS] = {0};
static float motor_mix_range = 0.0;
static float mixer_scale = 1.0f;
static motor_mixer_ts current_mixer[MAX_SUPPORTED_MOTORS];
static uint8_t motor_count = 0;
int mixer_throttle_command = 0;
static int throttle_idle_value = 0;
static int motor_value_when_stopped = 0;
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
static void load_primary_motor_mixer (void);

int get_throttle_idle_value (void)
{
    // if (!throttle_idle_value)
    //     throttle_idle_value = motor_config.min_command + (((motor_config.max_throttle - motor_config.min_command) / 100.0F) * current_byttery_profile.motor.throttle_idle);
    
    // return throttle_idle_value;
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
    DISABLE_STATE(FIXED_WING_LEGACY);
    DISABLE_STATE(MULTIROTOR);
    DISABLE_STATE(ROVER);
    DISABLE_STATE(BOAT);
    DISABLE_STATE(AIRPLANE);
    DISABLE_STATE(MOVE_FORWARD_ONLY);

    switch (mixer_config.platform_type) {
        case PLATFORM_AIRPLANE:
            ENABLE_STATE(FIXED_WING_LEGACY);
            ENABLE_STATE(AIRPLANE);
            ENABLE_STATE(ALTITUDE_CONTROL);
            ENABLE_STATE(MOVE_FORWARD_ONLY);
            break;

        case PLATFORM_ROVER:
            ENABLE_STATE(ROVER);
            ENABLE_STATE(FIXED_WING_LEGACY);
            ENABLE_STATE(MOVE_FORWARD_ONLY);
            break;

        case PLATFORM_BOAT:
            ENABLE_STATE(BOAT);
            ENABLE_STATE(FIXED_WING_LEGACY);
            ENABLE_STATE(MOVE_FORWARD_ONLY);
            break;

        case PLATFORM_MULTIROTOR:
        case PLATFORM_TRICOPTER:
        case PLATFORM_HELICOPTER:
            ENABLE_STATE(MULTIROTOR);
            ENABLE_STATE(ALTITUDE_CONTROL);
            break;
    }

    (mixer_config.has_flaps) ? ENABLE_STATE(FLAPERON_AVAILABLE) : DISABLE_STATE(FLAPERON_AVAILABLE);
}

void null_motor_rate_limiting (const float dT)
{
    UNUSED(dT);
}

void apply_motor_rate_limiting (const float dT)
{
    static float motor_previous[MAX_SUPPORTED_MOTORS] = { 0.0 };

    const uint16_t motor_range = motor_config.max_throttle - throttle_idle_value;
    const float motor_max_inc = (motor_config.motor_accel_time_ms == 0) ? 2000 : motor_range * dT / (motor_config.motor_accel_time_ms * 1E-3F);
    const float motor_max_dec = (motor_config.motor_decel_time_ms == 0) ? 2000 : motor_range * dT / (motor_config.motor_decel_time_ms * 1E-3F);

    for (uint32_t i = 0; i < motor_count; i++) {
        // Применяем ограничение скорости двигателя
        motor_previous[i] = constrainf(motor[i], motor_previous[i] - motor_max_dec, motor_previous[i] + motor_max_inc);

        // Управляем дроссельной заслонкой ниже min_throttle (запуск / остановка двигателя)
        if (motor_previous[i] < throttle_idle_value) {
            if (motor[i] < throttle_idle_value) {
                motor_previous[i] = motor[i];
            } else {
                motor_previous[i] = throttle_idle_value;
            }
        }

    }

 // Обновляем значения мотора
    for (uint32_t i = 0; i < motor_count; i++) {
        motor[i] = motor_previous[i];
    }
}

void mixer_init (void)
{
    compute_motor_count();
    load_primary_motor_mixer();

    //rcControlsConfig()->mid_throttle_deadband меняется в процессе принятия команд с пульта
    // по умолчанию SETTING_3D_DEADBAND_THROTTLE_DEFAULT  = 50
    throttle_deadband_low   = PWM_RANGE_MIDDLE - 50;/*rcControlsConfig()->mid_throttle_deadband;*/
    throttle_deadband_high  = PWM_RANGE_MIDDLE + 50;/*rcControlsConfig()->mid_throttle_deadband;*/

    mixer_reset_disarmed_motors();

    motor_rate_limiting_apply_fn = (motor_config.motor_accel_time_ms || motor_config.motor_decel_time_ms) ? apply_motor_rate_limiting : null_motor_rate_limiting;

    motor_yaw_multiplier = (mixer_config.motor_direction_inverted) ? -1 : 1;
}

static void load_primary_motor_mixer (void) 
{
    for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        current_mixer[i] = primary_motor_mixer[i];
    }
}

void mixer_reset_disarmed_motors (void)
{
    motor_zero_command = motor_config.min_command;
    throttle_range_min = get_throttle_idle_value();
    throttle_range_max = motor_config.max_throttle;
    
    reversible_motors_throttle_state = MOTOR_DIRECTION_FORWARD;
    
    motor_value_when_stopped = get_throttle_idle_value();

    // устанавливаем значения двигателя снятого с охраны
    for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = motor_zero_command;
    }
}

// #ifdef USE_DSHOT
static uint16_t handle_output_scaling(
    int16_t input,          // Input value from the mixer
    int16_t stopThreshold,  // Threshold value to check if motor should be rotating or not
    int16_t onStopValue,    // Value sent to the ESC when min rotation is required - on motor_stop it is STOP command, without motor_stop it's a value that keeps rotation
    int16_t inputScaleMin,  // Input range - min value
    int16_t inputScaleMax,  // Input range - max value
    int16_t outputScaleMin, // Output range - min value
    int16_t outputScaleMax, // Output range - max value
    bool moveForward        // If motor should be rotating FORWARD or BACKWARD
)
{
    int value;
    if (moveForward && input < stopThreshold) {
        //Send motor stop command
        value = onStopValue;
    }
    else if (!moveForward && input > stopThreshold) {
        //Send motor stop command
        value = onStopValue;
    }
    else {
        //Scale input to protocol output values
        value = scaleRangef(input, inputScaleMin, inputScaleMax, outputScaleMin, outputScaleMax);
        value = constrain(value, outputScaleMin, outputScaleMax);
    }
    return value;
}
static void apply_turtle_mode_to_motors (void) 
{
    if (ARMING_FLAG(ARMED)) {
        // currentBatteryProfile->motor.turtleModePowerFactor = SETTING_TURTLE_MODE_POWER_FACTOR_DEFAULT = 55 и нигде не меняется
        const float flip_power_factor = /*((float)currentBatteryProfile->motor.turtleModePowerFactor)*/ 55 / 100.0F;
        const float stick_deflection_pitch_abs = ABS(((float) rc_command[PITCH]) / 500.0f);
        const float stick_deflection_roll_abs = ABS(((float) rc_command[ROLL]) / 500.0f);
        const float stick_deflection_yaw_abs = ABS(((float) rc_command[YAW]) / 500.0f);
        //deflection stick position

        const float stick_deflection_pitch_expo =
                flip_power_factor * stick_deflection_pitch_abs + power3(stick_deflection_pitch_abs) * (1 - flip_power_factor);
        const float stick_deflection_roll_expo =
                flip_power_factor * stick_deflection_roll_abs + power3(stick_deflection_roll_abs) * (1 - flip_power_factor);
        const float stick_deflection_yaw_expo =
                flip_power_factor * stick_deflection_yaw_abs + power3(stick_deflection_yaw_abs) * (1 - flip_power_factor);

        float sign_pitch = rc_command[PITCH] < 0 ? 1 : -1;
        float sign_roll = rc_command[ROLL] < 0 ? 1 : -1;
        float sign_yaw = (float)((rc_command[YAW] < 0 ? 1 : -1) * (mixer_config.motor_direction_inverted ? 1 : -1));

        float stick_deflection_length = fast_fsqrtf(sq(stick_deflection_pitch_abs) + sq(stick_deflection_roll_abs));
        float stick_deflection_expo_length = fast_fsqrtf(sq(stick_deflection_pitch_expo) + sq(stick_deflection_roll_expo));

        if (stick_deflection_yaw_abs > MAX(stick_deflection_pitch_abs, stick_deflection_roll_abs)) {
            // If yaw is the dominant, disable pitch and roll
            stick_deflection_length = stick_deflection_yaw_abs;
            stick_deflection_expo_length = stick_deflection_yaw_expo;
            sign_roll = 0;
            sign_pitch = 0;
        } else {
            // If pitch/roll dominant, disable yaw
            sign_yaw = 0;
        }

        const float cos_phi = (stick_deflection_length > 0) ? (stick_deflection_pitch_abs + stick_deflection_roll_abs) /
                                                           (fast_fsqrtf(2.0f) * stick_deflection_length) : 0;
        const float cos_threshold = fast_fsqrtf(3.0f) / 2.0f; // cos(PI/6.0f)

        if (cos_phi < cos_threshold) {
            // Enforce either roll or pitch exclusively, if not on diagonal
            if (stick_deflection_roll_abs > stick_deflection_pitch_abs) {
                sign_pitch = 0;
            } else {
                sign_roll = 0;
            }
        }

        // Apply a reasonable amount of stick deadband
        const float crash_flip_stick_min_expo =
                flip_power_factor * CRASH_OVER_AFTER_CRASH_FLIP_STICK_MIN + power3(CRASH_OVER_AFTER_CRASH_FLIP_STICK_MIN) * (1 - flip_power_factor);
        const float flip_stick_range = 1.0F - crash_flip_stick_min_expo;
        const float flip_power = MAX(0.0F, stick_deflection_expo_length - crash_flip_stick_min_expo) / flip_stick_range;

        for (uint32_t i = 0; i < motor_count; i++) {
            float motor_output_normalised = sign_pitch  * current_mixer[i].pitch +
                                            sign_roll   * current_mixer[i].roll +
                                            sign_yaw    * current_mixer[i].yaw;

            if (motor_output_normalised < 0) {
                motor_output_normalised = 0;
            }

            motor_output_normalised = MIN(1.0F, flip_power * motor_output_normalised);

            motor[i] = (int16_t)scaleRangef(motor_output_normalised, 0, 1, motor_config.min_command, motor_config.max_throttle);
        }
    } else {
        // Disarmed mode
        //stopMotors();
    }
}
// #endif