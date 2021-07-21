#include "rc_controls.h"

#define AIRMODE_DEADBAND 25
#define MIN_RC_TICK_INTERVAL_MS             20
#define DEFAULT_RC_SWITCH_DISARM_DELAY_MS   250     // Wait at least 250ms before disarming via switch
#define DEFAULT_PREARM_TIMEOUT              10000   // Prearm is invalidated after 10 seconds

stick_positions_te rc_stick_positions;

int16_t rc_command[4];

rc_controls_config_ts rc_controls_config = {
    .deadband                   = SETTING_DEADBAND_DEFAULT,
    .yaw_deadband               = SETTING_YAW_DEADBAND_DEFAULT,
    .pos_hold_deadband          = SETTING_POS_HOLD_DEADBAND_DEFAULT,
    .control_deadband           = SETTING_CONTROL_DEADBAND_DEFAULT,
    .alt_hold_deadband          = SETTING_ALT_HOLD_DEADBAND_DEFAULT,
    .mid_throttle_deadband      = SETTING_3D_DEADBAND_THROTTLE_DEFAULT,
    .airmode_handling_type      = SETTING_AIRMODE_TYPE_DEFAULT,
    .airmode_throttle_threshold = SETTING_AIRMODE_THROTTLE_THRESHOLD_DEFAULT
};

arming_config_ts arming_config = {
    .fixed_wing_auto_arm = SETTING_FIXED_WING_AUTO_ARM_DEFAULT,
    .disarm_kill_switch = SETTING_DISARM_KILL_SWITCH_DEFAULT,
    .switch_disarm_delay_ms = SETTING_SWITCH_DISARM_DELAY_DEFAULT,
    .prearm_timeout_ms = SETTING_PREARM_TIMEOUT_DEFAULT
};

bool are_sticks_in_ap_mode_position (uint16_t ap_mode)
{
    return ABS(rc_command[ROLL]) < ap_mode && ABS(rc_command[PITCH]) < ap_mode;
}

bool are_sticks_deflected (void)
{
    return  (ABS(rc_command[ROLL])  > rc_controls_config.control_deadband)   || 
            (ABS(rc_command[PITCH]) > rc_controls_config.control_deadband)  || 
            (ABS(rc_command[YAW])   > rc_controls_config.control_deadband);
}

bool is_roll_pitch_stick_deflected (void)
{
    return  (ABS(rc_command[ROLL])  > rc_controls_config.control_deadband) || 
            (ABS(rc_command[PITCH]) > rc_controls_config.control_deadband);
}

throttle_status_te calculate_throttle_status (throttle_status_type_te type)
{
    // rxGetChannelValue(THROTTLE) - данные из rc канала, используем временную заглушку
    int16_t dummy_variable = 0xFFAA;

    int16_t value = (type == THROTTLE_STATUS_TYPE_RC) ? /*rxGetChannelValue(THROTTLE)*/ dummy_variable : rc_command[THROTTLE];

    const uint16_t mid_throttle_deadband = rc_controls_config.control_deadband;

    return THROTTLE_HIGH;
}

roll_pitch_status_te calculate_roll_pitch_center_status (void)
{
    // rxGetChannelValue(THROTTLE) - данные из rc канала, используем временную заглушку
    int16_t dummy_variable = 0xFFAA;

    if ((   (/*rxGetChannelValue(PITCH)*/ dummy_variable < (PWM_RANGE_MIDDLE + AIRMODE_DEADBAND)) && 
            (/*rxGetChannelValue(PITCH)*/ dummy_variable > (PWM_RANGE_MIDDLE -AIRMODE_DEADBAND))) 
            && 
        (   (/*rxGetChannelValue(ROLL)*/ dummy_variable < (PWM_RANGE_MIDDLE + AIRMODE_DEADBAND)) && 
            (/*rxGetChannelValue(ROLL)*/ dummy_variable > (PWM_RANGE_MIDDLE -AIRMODE_DEADBAND))))
            return CENTERED;

    return NOT_CENTERED;
}

stick_positions_te get_rc_stick_positions (void)
{
    return rc_stick_positions;
}

bool check_stick_position (stick_positions_te stick_position)
{
    const uint8_t mask[4] = { ROL_LO | ROL_HI, PIT_LO | PIT_HI, YAW_LO | YAW_HI, THR_LO | THR_HI };
    for (uint8_t i = 0; i < 4; i++) {
        if (((stick_position & mask[i]) != 0) && ((stick_position & mask[i]) != (rc_stick_positions & mask[i]))) {
            return false;
        }
    }

    return true;
}

/*
Обновляет положения стиков с учетом принятых данных из канала и конфигурации приемника
*/
static void update_rc_stick_positions (void)
{
//    stick_positions_te tmp = 0;

    // rxConfig - конфигурация приемника
    /* mincheck / maxcheck - это минимальные / максимальные значения (в США), которые, когда канал меньше (min) или больше (max), чем это значение, активируют различные команды RC, 
    такие как постановка на охрану или конфигурация ручки. Обычно каждый RC-канал должен быть установлен так, чтобы min = 1000us, max = 2000us. 
    На большинстве передатчиков это обычно означает 125% конечных точек. Значения проверки по умолчанию на 100 мкс выше / ниже этого значения.
    */
    // tmp |= ((rxGetChannelValue(ROLL) > rxConfig()->mincheck) ? 0x02 : 0x00) << (ROLL * 2);
    // tmp |= ((rxGetChannelValue(ROLL) < rxConfig()->maxcheck) ? 0x01 : 0x00) << (ROLL * 2);

    // tmp |= ((rxGetChannelValue(PITCH) > rxConfig()->mincheck) ? 0x02 : 0x00) << (PITCH * 2);
    // tmp |= ((rxGetChannelValue(PITCH) < rxConfig()->maxcheck) ? 0x01 : 0x00) << (PITCH * 2);

    // tmp |= ((rxGetChannelValue(YAW) > rxConfig()->mincheck) ? 0x02 : 0x00) << (YAW * 2);
    // tmp |= ((rxGetChannelValue(YAW) < rxConfig()->maxcheck) ? 0x01 : 0x00) << (YAW * 2);

    // tmp |= ((rxGetChannelValue(THROTTLE) > rxConfig()->mincheck) ? 0x02 : 0x00) << (THROTTLE * 2);
    // tmp |= ((rxGetChannelValue(THROTTLE) < rxConfig()->maxcheck) ? 0x01 : 0x00) << (THROTTLE * 2);

    // rc_stick_positions = tmp;
}

void process_rc_stick_positions (throttle_status_te throttle_status)
{
    static uint32_t last_tick_time_ms = 0;
    static uint8_t  rc_delay_command;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint32_t rc_sticks;           // this hold sticks position for command combos
    static uint32_t rc_disarm_time_ms;     // this is an extra guard for disarming through switch to prevent that one frame can disarm it
    const uint32_t  current_time_ms = /*millis()*/ 0xAAFF;

    update_rc_stick_positions();

    uint32_t st_tmp = get_rc_stick_positions();
    if (st_tmp == rc_sticks) {
        if (rc_delay_command < 250 && (current_time_ms - last_tick_time_ms) >= MIN_RC_TICK_INTERVAL_MS) {
            last_tick_time_ms = current_time_ms;
            rc_delay_command++;
        }
    } else {
        rc_delay_command = 0;
    }

    rc_sticks = st_tmp;

    // perform actions
    bool arming_switch_is_active = IS_RC_MODE_ACTIVE(BOXARM);
    emergencyArmingUpdate(armingSwitchIsActive);

    if (STATE(AIRPLANE) && feature(FEATURE_MOTOR_STOP) && armingConfig()->fixed_wing_auto_arm) {
        // Auto arm on throttle when using fixedwing and motorstop
        if (throttleStatus != THROTTLE_LOW) {
            tryArm();
            return;
        }
    }
    else {
        if (armingSwitchIsActive) {
            rcDisarmTimeMs = currentTimeMs;
            tryArm();
        } else {
            // Disarming via ARM BOX
            // Don't disarm via switch if failsafe is active or receiver doesn't receive data - we can't trust receiver
            // and can't afford to risk disarming in the air
            if (ARMING_FLAG(ARMED) && !IS_RC_MODE_ACTIVE(BOXFAILSAFE) && rxIsReceivingSignal() && !failsafeIsActive()) {
                const timeMs_t disarmDelay = currentTimeMs - rcDisarmTimeMs;
                if (disarmDelay > armingConfig()->switchDisarmDelayMs) {
                    if (armingConfig()->disarm_kill_switch || (throttleStatus == THROTTLE_LOW)) {
                        disarm(DISARM_SWITCH);
                    }
                }
            }
            else {
                rcDisarmTimeMs = currentTimeMs;
            }
        }

        // KILLSWITCH disarms instantly
        if (IS_RC_MODE_ACTIVE(BOXKILLSWITCH)) {
            disarm(DISARM_KILLSWITCH);
        }
    }

    if (rcDelayCommand != 20) {
        return;
    }

    if (ARMING_FLAG(ARMED)) {
        // actions during armed
        return;
    }

    // actions during not armed

    // GYRO calibration
    if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
        gyroStartCalibration();
        return;
    }


#if defined(NAV_NON_VOLATILE_WAYPOINT_STORAGE)
    // Save waypoint list
    if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) {
        const bool success = saveNonVolatileWaypointList();
        beeper(success ? BEEPER_ACTION_SUCCESS : BEEPER_ACTION_FAIL);
    }

    // Load waypoint list
    if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) {
        const bool success = loadNonVolatileWaypointList(false);
        beeper(success ? BEEPER_ACTION_SUCCESS : BEEPER_ACTION_FAIL);
    }

    if (rcSticks == THR_LO + YAW_CE + PIT_LO + ROL_HI) {
        resetWaypointList();
        beeper(BEEPER_ACTION_FAIL); // The above cannot fail, but traditionally, we play FAIL for not-loading
    }
#endif

    // Multiple configuration profiles
    if (feature(FEATURE_TX_PROF_SEL)) {

        uint8_t i = 0;

        if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO)          // ROLL left  -> Profile 1
            i = 1;
        else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE)     // PITCH up   -> Profile 2
            i = 2;
        else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)     // ROLL right -> Profile 3
            i = 3;

        if (i) {
            setConfigProfileAndWriteEEPROM(i - 1);
            return;
        }

        i = 0;

        // Multiple battery configuration profiles
        if (rcSticks == THR_HI + YAW_LO + PIT_CE + ROL_LO)          // ROLL left  -> Profile 1
            i = 1;
        else if (rcSticks == THR_HI + YAW_LO + PIT_HI + ROL_CE)     // PITCH up   -> Profile 2
            i = 2;
        else if (rcSticks == THR_HI + YAW_LO + PIT_CE + ROL_HI)     // ROLL right -> Profile 3
            i = 3;

        if (i) {
            setConfigBatteryProfileAndWriteEEPROM(i - 1);
            batteryDisableProfileAutoswitch();
            activateBatteryProfile();
            return;
        }

    }

    // Save config
    if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_HI) {
        saveConfigAndNotify();
    }

    // Calibrating Acc
    if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) {
        accStartCalibration();
        return;
    }

    // Calibrating Mag
    if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) {
        ENABLE_STATE(CALIBRATE_MAG);
        return;
    }

    // Accelerometer Trim
    if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
        applyAndSaveBoardAlignmentDelta(0, -2);
        rcDelayCommand = 10;
        return;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
        applyAndSaveBoardAlignmentDelta(0, 2);
        rcDelayCommand = 10;
        return;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
        applyAndSaveBoardAlignmentDelta(-2, 0);
        rcDelayCommand = 10;
        return;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
        applyAndSaveBoardAlignmentDelta(2, 0);
        rcDelayCommand = 10;
        return;
    }
}