#ifndef _MIXER_H
#define _MIXER_H

#include <stdint.h>
#include <stdbool.h>
#include "runtime_config.h"
#include "maths.h"
#include "rx.h"
#include "rc_controls.h"

#define MAX_SUPPORTED_MOTORS 12

// Digital protocol has fixed values
#define DSHOT_DISARM_COMMAND      0
#define DSHOT_MIN_THROTTLE       48
#define DSHOT_MAX_THROTTLE     2047
#define DSHOT_3D_DEADBAND_LOW  1047
#define DSHOT_3D_DEADBAND_HIGH 1048

#define DEFAULT_PWM_PROTOCOL    PWM_TYPE_ONESHOT125
#define DEFAULT_PWM_RATE        400

#define DEFAULT_MAX_THROTTLE    1850

#define CRASH_OVER_AFTER_CRASH_FLIP_STICK_MIN 0.15F

enum {
    SETTING_3D_DEADBAND_LOW_DEFAULT     = 1406,
    SETTING_3D_DEADBAND_LOW             = 165,
    SETTING_3D_DEADBAND_LOW_MIN         = 1000,
    SETTING_3D_DEADBAND_LOW_MAX         = 2000,
    SETTING_3D_DEADBAND_HIGH_DEFAULT    = 1514,
    SETTING_3D_DEADBAND_HIGH            = 166,
    SETTING_3D_DEADBAND_HIGH_MIN        = 1000,
    SETTING_3D_DEADBAND_HIGH_MAX        = 2000,
    SETTING_3D_NEUTRAL_DEFAULT          = 1460,
    SETTING_3D_NEUTRAL                  = 167,
    SETTING_3D_NEUTRAL_MIN              = 1000,
    SETTING_3D_NEUTRAL_MAX              = 2000,
    SETTING_MOTOR_DIRECTION_INVERTED_DEFAULT    = 0,
    SETTING_MOTOR_DIRECTION_INVERTED            = 161,
    SETTING_MOTOR_DIRECTION_INVERTED_MIN        = 0,
    SETTING_MOTOR_DIRECTION_INVERTED_MAX        = 0,
    SETTING_PLATFORM_TYPE_DEFAULT   = 0,
    SETTING_PLATFORM_TYPE           = 162,
    SETTING_PLATFORM_TYPE_MIN       = 0,
    SETTING_PLATFORM_TYPE_MAX       = 0,
    SETTING_HAS_FLAPS_DEFAULT   = 0,
    SETTING_HAS_FLAPS           = 163,
    SETTING_HAS_FLAPS_MIN       = 0,
    SETTING_HAS_FLAPS_MAX       = 0,
    SETTING_MODEL_PREVIEW_TYPE_DEFAULT  = -1,
    SETTING_MODEL_PREVIEW_TYPE          = 164,
    SETTING_MODEL_PREVIEW_TYPE_MIN      = -1,
    SETTING_MODEL_PREVIEW_TYPE_MAX      = 32767,
    SETTING_MAX_THROTTLE_DEFAULT    = 1850,
    SETTING_MAX_THROTTLE            = 99,
    SETTING_MAX_THROTTLE_MIN        = 1000,
    SETTING_MAX_THROTTLE_MAX        = 2000,
    SETTING_MIN_COMMAND_DEFAULT = 1000,
    SETTING_MIN_COMMAND         = 100,
    SETTING_MIN_COMMAND_MIN     = 0,
    SETTING_MIN_COMMAND_MAX     = 2000,
    SETTING_MOTOR_PWM_RATE_DEFAULT  = 400,
    SETTING_MOTOR_PWM_RATE          = 101,
    SETTING_MOTOR_PWM_RATE_MIN      = 50,
    SETTING_MOTOR_PWM_RATE_MAX      = 32000,
    SETTING_MOTOR_PWM_PROTOCOL_DEFAULT  = 1,
    SETTING_MOTOR_PWM_PROTOCOL          = 104,
    SETTING_MOTOR_PWM_PROTOCOL_MIN      = 0,
    SETTING_MOTOR_PWM_PROTOCOL_MAX      = 0,
    SETTING_MOTOR_ACCEL_TIME_DEFAULT    = 0,
    SETTING_MOTOR_ACCEL_TIME            = 102,
    SETTING_MOTOR_ACCEL_TIME_MIN        = 0,
    SETTING_MOTOR_ACCEL_TIME_MAX        = 1000,
    SETTING_MOTOR_DECEL_TIME_DEFAULT    = 0,
    SETTING_MOTOR_DECEL_TIME            = 103,
    SETTING_MOTOR_DECEL_TIME_MIN        = 0,
    SETTING_MOTOR_DECEL_TIME_MAX        = 1000,
    SETTING_MOTOR_POLES_DEFAULT = 14,
    SETTING_MOTOR_POLES         = 105,
    SETTING_MOTOR_POLES_MIN     = 4,
    SETTING_MOTOR_POLES_MAX     = 255,
};

typedef enum {
    PLATFORM_MULTIROTOR     = 0,
    PLATFORM_AIRPLANE       = 1,
    PLATFORM_HELICOPTER     = 2,
    PLATFORM_TRICOPTER      = 3,
    PLATFORM_ROVER          = 4,
    PLATFORM_BOAT           = 5,
    PLATFORM_OTHER          = 6
} flying_platform_type_te;

typedef struct {
    int16_t min;
    int16_t max;
} motor_axis_correction_limits_ts;

// Custom mixer data per motor
typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motor_mixer_ts;

typedef struct {
    int8_t  motor_direction_inverted;
    uint8_t platform_type;
    bool    has_flaps;
    int16_t applied_mixer_preset;
} mixer_config_ts;

typedef struct {
    uint16_t deadband_low;                // min 3d value
    uint16_t deadband_high;               // max 3d value
    uint16_t neutral;                     // center 3d value
} reversible_motors_config_ts;

typedef struct {
    // PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms)
    uint16_t    max_throttle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t    min_command;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t    motor_pwm_rate;                  // The update rate of motor outputs (50-498Hz)
    uint8_t     motor_pwm_protocol;
    uint16_t    motor_accel_time_ms;              // Time limit for motor to accelerate from 0 to 100% throttle [ms]
    uint16_t    motor_decel_time_ms;              // Time limit for motor to decelerate from 0 to 100% throttle [ms]
    uint16_t    digital_idle_offset_value;
    uint8_t     motor_pole_count;                 // Magnetic poles in the motors for calculating actual RPM from eRPM provided by ESC telemetry
} motor_config_ts;

typedef enum {
    MOTOR_STOPPED_USER,
    MOTOR_STOPPED_AUTO,
    MOTOR_RUNNING
} motor_status_te;

typedef enum {
    MOTOR_DIRECTION_FORWARD,
    MOTOR_DIRECTION_BACKWARD,
    MOTOR_DIRECTION_DEADBAND
} reversible_motors_throttle_state_te;

extern uint16_t motor[MAX_SUPPORTED_MOTORS];
extern uint16_t motor_disarmed[MAX_SUPPORTED_MOTORS];
extern int mixer_throttle_command;

extern mixer_config_ts mixer_config;

int             get_throttle_idle_value     (void);
int16_t         get_throttle_percent        (void);
uint8_t         get_motor_count             (void);
float           get_motor_mix_range         (void);
bool            mixer_is_output_saturated   (void);
motor_status_te get_motor_status            (void);

void write_all_motors                   (int16_t mc);
void mixer_init                         (void);
void mixer_update_state_flags           (void);
void mixer_reset_disarmed_motors        (void);
void mix_table                          (const float dT);
void write_motors                       (void);
void process_servo_autotrim             (const float dT);
void process_servo_autotrim_mode        (void);
void process_continuous_servo_autotrim  (const float dT);
void stop_motors                        (void);
void stop_pwm_all_motors                (void);

bool are_motors_running         (void);

#endif // _MIXER_H