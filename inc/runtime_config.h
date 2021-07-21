#ifndef _RUNTIME_CONFIG_H
#define _RUNTIME_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "utils.h"

typedef enum {
    ARMED                                           = (1 << 2),
    WAS_EVER_ARMED                                  = (1 << 3),

    ARMING_DISABLED_FAILSAFE_SYSTEM                 = (1 << 7),
    ARMING_DISABLED_NOT_LEVEL                       = (1 << 8),
    ARMING_DISABLED_SENSORS_CALIBRATING             = (1 << 9),
    ARMING_DISABLED_SYSTEM_OVERLOADED               = (1 << 10),
    ARMING_DISABLED_NAVIGATION_UNSAFE               = (1 << 11),
    ARMING_DISABLED_COMPASS_NOT_CALIBRATED          = (1 << 12),
    ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED    = (1 << 13),
    ARMING_DISABLED_ARM_SWITCH                      = (1 << 14),
    ARMING_DISABLED_HARDWARE_FAILURE                = (1 << 15),
    ARMING_DISABLED_BOXFAILSAFE                     = (1 << 16),
    ARMING_DISABLED_BOXKILLSWITCH                   = (1 << 17),
    ARMING_DISABLED_RC_LINK                         = (1 << 18),
    ARMING_DISABLED_THROTTLE                        = (1 << 19),
    ARMING_DISABLED_CLI                             = (1 << 20),
    ARMING_DISABLED_CMS_MENU                        = (1 << 21),
    ARMING_DISABLED_OSD_MENU                        = (1 << 22),
    ARMING_DISABLED_ROLLPITCH_NOT_CENTERED          = (1 << 23),
    ARMING_DISABLED_SERVO_AUTOTRIM                  = (1 << 24),
    ARMING_DISABLED_OOM                             = (1 << 25),
    ARMING_DISABLED_INVALID_SETTING                 = (1 << 26),
    ARMING_DISABLED_PWM_OUTPUT_ERROR                = (1 << 27),
    ARMING_DISABLED_NO_PREARM                       = (1 << 28),
    ARMING_DISABLED_DSHOT_BEEPER                    = (1 << 29),

    ARMING_DISABLED_ALL_FLAGS                       = (ARMING_DISABLED_FAILSAFE_SYSTEM | ARMING_DISABLED_NOT_LEVEL | ARMING_DISABLED_SENSORS_CALIBRATING | 
                                                       ARMING_DISABLED_SYSTEM_OVERLOADED | ARMING_DISABLED_NAVIGATION_UNSAFE |
                                                       ARMING_DISABLED_COMPASS_NOT_CALIBRATED | ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED |
                                                       ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_HARDWARE_FAILURE | ARMING_DISABLED_BOXFAILSAFE |
                                                       ARMING_DISABLED_BOXKILLSWITCH | ARMING_DISABLED_RC_LINK | ARMING_DISABLED_THROTTLE | ARMING_DISABLED_CLI |
                                                       ARMING_DISABLED_CMS_MENU | ARMING_DISABLED_OSD_MENU | ARMING_DISABLED_ROLLPITCH_NOT_CENTERED |
                                                       ARMING_DISABLED_SERVO_AUTOTRIM | ARMING_DISABLED_OOM | ARMING_DISABLED_INVALID_SETTING |
                                                       ARMING_DISABLED_PWM_OUTPUT_ERROR | ARMING_DISABLED_NO_PREARM | ARMING_DISABLED_DSHOT_BEEPER),
} arming_flag_te;

// Блокираторы постановки на охрану, которые могут быть отменены аварийной постановкой на охрану.
// Имейте в виду, что эта функция предназначена для постановки на охрану в
// ситуации, когда нам может понадобиться просто вращение моторов, поэтому
// самолет может двигаться (даже непредсказуемо) и открепиться (например,
// врезался в высокое дерево).
#define ARMING_DISABLED_EMERGENCY_OVERRIDE  (ARMING_DISABLED_NOT_LEVEL \
                                            | ARMING_DISABLED_NAVIGATION_UNSAFE \
                                            | ARMING_DISABLED_COMPASS_NOT_CALIBRATED \
                                            | ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED \
                                            | ARMING_DISABLED_ARM_SWITCH \
                                            | ARMING_DISABLED_HARDWARE_FAILURE)




extern const char *arming_disable_flag_names[];

#define isArmingDisabled()          (arming_flags & (ARMING_DISABLED_ALL_FLAGS))
#define DISABLE_ARMING_FLAG(mask)   (arming_flags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask)    (arming_flags |= (mask))
#define ARMING_FLAG(mask)           (arming_flags & (mask))

// Возвращает 1-й флаг из ARMING_DISABLED_ALL_FLAGS, который
// предотвращение постановки на охрану или ноль, если постановка на охрану не отключена.
arming_flag_te is_arming_disabled_reason (void);

typedef enum {
    ANGLE_MODE            = (1 << 0),
    HORIZON_MODE          = (1 << 1),
    HEADING_MODE          = (1 << 2),
    NAV_ALTHOLD_MODE      = (1 << 3), // old BARO
    NAV_RTH_MODE          = (1 << 4), // old GPS_HOME
    NAV_POSHOLD_MODE      = (1 << 5), // old GPS_HOLD
    HEADFREE_MODE         = (1 << 6),
    NAV_LAUNCH_MODE       = (1 << 7),
    MANUAL_MODE           = (1 << 8),
    FAILSAFE_MODE         = (1 << 9),
    AUTO_TUNE             = (1 << 10), // old G-Tune
    NAV_WP_MODE           = (1 << 11),
    NAV_COURSE_HOLD_MODE  = (1 << 12),
    FLAPERON              = (1 << 13),
    TURN_ASSISTANT        = (1 << 14),
    TURTLE_MODE           = (1 << 15),
} flight_mode_flags_te;

extern uint32_t arming_flags;
extern uint32_t flight_mode_flags;
extern uint32_t state_flags;

#define DISABLE_FLIGHT_MODE(mask) disable_flight_mode(mask)
#define ENABLE_FLIGHT_MODE(mask) enable_flight_mode(mask)
#define FLIGHT_MODE(mask) (flight_mode_flags & (mask))

typedef enum {
    GPS_FIX_HOME                        = (1 << 0),
    GPS_FIX                             = (1 << 1),
    CALIBRATE_MAG                       = (1 << 2),
    SMALL_ANGLE                         = (1 << 3),
    FIXED_WING_LEGACY                   = (1 << 4),     // No new code should use this state. Use AIRPLANE, MULTIROTOR, ROVER, BOAT, ALTITUDE_CONTROL and MOVE_FORWARD_ONLY states
    ANTI_WINDUP                         = (1 << 5),
    FLAPERON_AVAILABLE                  = (1 << 6),
    NAV_MOTOR_STOP_OR_IDLE              = (1 << 7),     // navigation requests MOTOR_STOP or motor idle regardless of throttle stick, will only activate if MOTOR_STOP feature is available
    COMPASS_CALIBRATED                  = (1 << 8),
    ACCELEROMETER_CALIBRATED            = (1 << 9),
    PWM_DRIVER_AVAILABLE                = (1 << 10),
    NAV_CRUISE_BRAKING                  = (1 << 11),
    NAV_CRUISE_BRAKING_BOOST            = (1 << 12),
    NAV_CRUISE_BRAKING_LOCKED           = (1 << 13),
    NAV_EXTRA_ARMING_SAFETY_BYPASSED    = (1 << 14),    // nav_extra_arming_safey was bypassed. Keep it until power cycle.
    AIRMODE_ACTIVE                      = (1 << 15),
    ESC_SENSOR_ENABLED                  = (1 << 16),
    AIRPLANE                            = (1 << 17),
    MULTIROTOR                          = (1 << 18),
    ROVER                               = (1 << 19),
    BOAT                                = (1 << 20),
    ALTITUDE_CONTROL                    = (1 << 21),    //It means it can fly
    MOVE_FORWARD_ONLY                   = (1 << 22),
    SET_REVERSIBLE_MOTORS_FORWARD       = (1 << 23),
    FW_HEADING_USE_YAW                  = (1 << 24),
    ANTI_WINDUP_DEACTIVATED             = (1 << 25),
} state_flags_te;

#define DISABLE_STATE(mask) (state_flags &= ~(mask))
#define ENABLE_STATE(mask) (state_flags |= (mask))
#define STATE(mask) (state_flags & (mask))

typedef enum {
    FLM_MANUAL,
    FLM_ACRO,
    FLM_ACRO_AIR,
    FLM_ANGLE,
    FLM_HORIZON,
    FLM_ALTITUDE_HOLD,
    FLM_POSITION_HOLD,
    FLM_RTH,
    FLM_MISSION,
    FLM_COURSE_HOLD,
    FLM_CRUISE,
    FLM_LAUNCH,
    FLM_FAILSAFE,
    FLM_COUNT
} flight_mode_for_telemetry_te;

flight_mode_for_telemetry_te get_flight_mode_for_telemetry (void);

uint32_t enable_flight_mode     (flight_mode_flags_te mask);
uint32_t disable_flight_mode    (flight_mode_flags_te mask);

bool sensors            (uint32_t mask);
void sensors_set        (uint32_t mask);
void sensors_clear      (uint32_t mask);
uint32_t sensors_mask   (void);


#endif // _RUNTIME_CONFIG_H`