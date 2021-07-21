#include "runtime_config.h"

uint32_t arming_flags = 0;
uint32_t state_flags = 0;
uint32_t flight_mode_flags = 0;

static  uint32_t enabled_sensors = 0;

#if !defined(CLI_MINIMAL_VERBOSITY)
const char *armingDisableFlagNames[]= {
    "FS", "ANGLE", "CAL", "OVRLD", "NAV", "COMPASS",
    "ACC", "ARMSW", "HWFAIL", "BOXFS", "KILLSW", "RX",
    "THR", "CLI", "CMS", "OSD", "ROLL/PITCH", "AUTOTRIM", "OOM",
    "SETTINGFAIL", "PWMOUT", "NOPREARM", "DSHOTBEEPER"
};
#endif

const arming_flag_te arm_disable_reasons_checklist[] = {
    ARMING_DISABLED_INVALID_SETTING,
    ARMING_DISABLED_HARDWARE_FAILURE,
    ARMING_DISABLED_PWM_OUTPUT_ERROR,
    ARMING_DISABLED_COMPASS_NOT_CALIBRATED,
    ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED,
    ARMING_DISABLED_RC_LINK,
    ARMING_DISABLED_NAVIGATION_UNSAFE,
    ARMING_DISABLED_ARM_SWITCH,
    ARMING_DISABLED_BOXFAILSAFE,
    ARMING_DISABLED_BOXKILLSWITCH,
    ARMING_DISABLED_THROTTLE,
    ARMING_DISABLED_CLI,
    ARMING_DISABLED_CMS_MENU,
    ARMING_DISABLED_OSD_MENU,
    ARMING_DISABLED_ROLLPITCH_NOT_CENTERED,
    ARMING_DISABLED_SERVO_AUTOTRIM,
    ARMING_DISABLED_OOM,
    ARMING_DISABLED_NO_PREARM,
    ARMING_DISABLED_DSHOT_BEEPER
};

static inline uint32_t is_arming_disabled (void);

arming_flag_te is_arming_disabled_reason (void)
{
    arming_flag_te reasons = is_arming_disabled();

    // Ярлык, если мы вообще не блокируем постановку на охрану
    if (!reasons)
        return 0;

    // проверяем наиболее важные флаги
    for (uint32_t i = 0; i < ARRAYLEN(arm_disable_reasons_checklist); i++)  {
        arming_flag_te flag = arm_disable_reasons_checklist[i];
        if (flag & reasons)
            return flag;
    }

    // Откат, мы получили флаг блокировки, не включенный в arm_disable_reasons_checklist[]
    for (uint32_t i = 0; i < sizeof(arming_flag_te) * 8; i++) {
        arming_flag_te flag = (uint32_t)1 << i;
        if (flag & reasons)
            return flag;
    }

    return 0;
}

/**
  * Включает данный режим полета. Раздается звуковой сигнал, если режим полета
  * изменился. Возвращает новое значение flight_mode_flags.
  */
uint32_t enable_flight_mode (flight_mode_flags_te mask)
{
    uint32_t old_val = flight_mode_flags;

    flight_mode_flags |= (mask);
    if (flight_mode_flags != old_val)
        /*beeperConfirmationBeeps(1)*/;

    return flight_mode_flags;
}

/**
  * Отключает данный режим полета. Раздается звуковой сигнал, если режим полета
  * изменился. Возвращает новое значение flight_mode_flags.
  */
uint32_t disable_flight_mode(flight_mode_flags_te mask)
{
    uint32_t old_val = flight_mode_flags;

    flight_mode_flags &= ~(mask);
    if (flight_mode_flags != old_val)
        /*beeperConfirmationBeeps(1)*/;

    return flight_mode_flags;
}

bool sensors (uint32_t mask)
{
    return enabled_sensors & mask;
}

void sensors_set (uint32_t mask)
{
    enabled_sensors |= mask;
}

void sensors_clear (uint32_t mask)
{
    enabled_sensors &= ~(mask);
}

uint32_t sensors_mask (void)
{
    return enabled_sensors;
}

flight_mode_for_telemetry_te get_flight_mode_for_telemetry (void)
{
    if (FLIGHT_MODE(FAILSAFE_MODE))
        return FLM_FAILSAFE;

    if (FLIGHT_MODE(MANUAL_MODE))
        return FLM_MANUAL;

    if (FLIGHT_MODE(NAV_LAUNCH_MODE))
        return FLM_LAUNCH;

    if (FLIGHT_MODE(NAV_RTH_MODE))
        return FLM_RTH;

    if (FLIGHT_MODE(NAV_POSHOLD_MODE))
        return FLM_POSITION_HOLD;

    if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE) && FLIGHT_MODE(NAV_ALTHOLD_MODE))
        return FLM_CRUISE;

    if (FLIGHT_MODE(NAV_COURSE_HOLD_MODE))
        return FLM_COURSE_HOLD;

    if (FLIGHT_MODE(NAV_WP_MODE))
        return FLM_MISSION;

    if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
        return FLM_ALTITUDE_HOLD;

    if (FLIGHT_MODE(ANGLE_MODE))
        return FLM_ANGLE;

    if (FLIGHT_MODE(HORIZON_MODE))
        return FLM_HORIZON;


    return STATE(AIRMODE_ACTIVE) ? FLM_ACRO_AIR : FLM_ACRO;
}

static inline uint32_t is_arming_disabled (void)
{
   return (arming_flags & (ARMING_DISABLED_ALL_FLAGS));
}