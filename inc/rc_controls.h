#ifndef _RC_CONTROLS_H
#define _RC_CONTROLS_H

#include <stdbool.h>
#include <stdint.h>
#include "maths.h"
#include "rx.h"
#include "runtime_config.h"

#define AIRMODE_THROTTLE_THRESHOLD 1300

enum {
    SETTING_DEADBAND_DEFAULT    = 5,
    SETTING_DEADBAND            = 215,
    SETTING_DEADBAND_MIN        = 0,
    SETTING_DEADBAND_MAX        = 32,
    SETTING_YAW_DEADBAND_DEFAULT    = 5,
    SETTING_YAW_DEADBAND            = 216,
    SETTING_YAW_DEADBAND_MIN        = 0,
    SETTING_YAW_DEADBAND_MAX        = 100,
    SETTING_POS_HOLD_DEADBAND_DEFAULT   = 10,
    SETTING_POS_HOLD_DEADBAND           = 217,
    SETTING_POS_HOLD_DEADBAND_MIN       = 2,
    SETTING_POS_HOLD_DEADBAND_MAX       = 250,
    SETTING_CONTROL_DEADBAND_DEFAULT    = 10,
    SETTING_CONTROL_DEADBAND            = 218,
    SETTING_CONTROL_DEADBAND_MIN        = 2,
    SETTING_CONTROL_DEADBAND_MAX        = 250,
    SETTING_ALT_HOLD_DEADBAND_DEFAULT   = 50,
    SETTING_ALT_HOLD_DEADBAND           = 219,
    SETTING_ALT_HOLD_DEADBAND_MIN       = 10,
    SETTING_ALT_HOLD_DEADBAND_MAX       = 250,
    SETTING_3D_DEADBAND_THROTTLE_DEFAULT    = 50,
    SETTING_3D_DEADBAND_THROTTLE            = 220,
    SETTING_3D_DEADBAND_THROTTLE_MIN        = 0,
    SETTING_3D_DEADBAND_THROTTLE_MAX        = 200,
    SETTING_AIRMODE_TYPE_DEFAULT    = 0,
    SETTING_AIRMODE_TYPE            = 221,
    SETTING_AIRMODE_TYPE_MIN        = 0,
    SETTING_AIRMODE_TYPE_MAX        = 0,
    SETTING_AIRMODE_THROTTLE_THRESHOLD_DEFAULT  = 1300,
    SETTING_AIRMODE_THROTTLE_THRESHOLD          = 222,
    SETTING_AIRMODE_THROTTLE_THRESHOLD_MIN      = 1000,
    SETTING_AIRMODE_THROTTLE_THRESHOLD_MAX      = 2000,
    SETTING_FIXED_WING_AUTO_ARM_DEFAULT = 0,
    SETTING_FIXED_WING_AUTO_ARM         = 199,
    SETTING_FIXED_WING_AUTO_ARM_MIN     = 0,
    SETTING_FIXED_WING_AUTO_ARM_MAX     = 0,
    SETTING_DISARM_KILL_SWITCH_DEFAULT  = 1,
    SETTING_DISARM_KILL_SWITCH          = 200,
    SETTING_DISARM_KILL_SWITCH_MIN      = 0,
    SETTING_DISARM_KILL_SWITCH_MAX      = 0,
    SETTING_SWITCH_DISARM_DELAY_DEFAULT = 250,
    SETTING_SWITCH_DISARM_DELAY         = 201,
    SETTING_SWITCH_DISARM_DELAY_MIN     = 0,
    SETTING_SWITCH_DISARM_DELAY_MAX     = 1000,
    SETTING_PREARM_TIMEOUT_DEFAULT  = 10000,
    SETTING_PREARM_TIMEOUT          = 202,
    SETTING_PREARM_TIMEOUT_MIN      = 0,
    SETTING_PREARM_TIMEOUT_MAX      = 10000,
};

typedef enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12
} rc_alias_te;

typedef enum {
    THROTTLE_LOW = 0,
    THROTTLE_HIGH
} throttle_status_te;

typedef enum {
    THROTTLE_STATUS_TYPE_RC = 0,
    THROTTLE_STATUS_TYPE_COMMAND
} throttle_status_type_te;

typedef enum {
    NOT_CENTERED = 0,
    CENTERED
} roll_pitch_status_te;

typedef enum {
    STICK_CENTER = 0,
    THROTTLE_THRESHOLD,
    STICK_CENTER_ONCE
} airmode_handling_type_te;

typedef enum {
    ROL_LO = (1 << (2 * ROLL)),
    ROL_CE = (3 << (2 * ROLL)),
    ROL_HI = (2 << (2 * ROLL)),

    PIT_LO = (1 << (2 * PITCH)),
    PIT_CE = (3 << (2 * PITCH)),
    PIT_HI = (2 << (2 * PITCH)),

    YAW_LO = (1 << (2 * YAW)),
    YAW_CE = (3 << (2 * YAW)),
    YAW_HI = (2 << (2 * YAW)),

    THR_LO = (1 << (2 * THROTTLE)),
    THR_CE = (3 << (2 * THROTTLE)),
    THR_HI = (2 << (2 * THROTTLE))
} stick_positions_te;

extern int16_t rc_command[4];

typedef struct {
    uint8_t     deadband;                   // вводим зону нечувствительности вокруг центра ручки для оси тангажа и крена. Должно быть больше нуля.    
    uint8_t     yaw_deadband;               // вводим зону нечувствительности вокруг центра стика для оси рыскания. Должно быть больше нуля.    
    uint8_t     pos_hold_deadband;          // Зона нечувствительности для удержания позиции    
    uint8_t     control_deadband;           // Общая зона нечувствительности для проверки отклонения стиков с использованием ШИМ.    
    uint8_t     alt_hold_deadband;          // Определяет нейтральную зону ручки газа при удержании высоты    
    uint16_t    mid_throttle_deadband;      // зона нечувствительности дроссельной заслонки по умолчанию из MIDRC    
    uint8_t     airmode_handling_type;      // По умолчанию ANTI_WINDUP запускается при центрировании стиков
    uint16_t    airmode_throttle_threshold; // Порог дроссельной заслонки для начальной активации воздушного режима
} rc_controls_config_ts;

typedef struct {
    uint8_t     fixed_wing_auto_arm;    // Самолет с неподвижным крылом с автоматическим взводом на газу и никогда не снимается с охраны        
    uint8_t     disarm_kill_switch;     // разрешить снятие с охраны с помощью переключателя AUX независимо от значения газа        
    uint16_t    switch_disarm_delay_ms; // дополнительная задержка между срабатыванием коробки ARM и фактическим снятием с охраны
    uint16_t    prearm_timeout_ms;      // продолжительность, в течение которой действует активация Prearm. после этого необходимо сбросить Prearm. 0 означает, что время ожидания не истекает.         
} arming_config_ts;

stick_positions_te  get_rc_stick_positions  (void);
bool                check_stick_position    (stick_positions_te stick_position);

bool                    are_sticks_in_ap_mode_position      (uint16_t ap_mode);
bool                    are_sticks_deflected                (void);
bool                    is_roll_pitch_stick_deflected       (void);
throttle_status_te      calculate_throttle_status           (throttle_status_type_te type);
roll_pitch_status_te    calculate_roll_pitch_center_status  (void);
void                    process_rc_stick_positions          (throttle_status_te throttle_status);

int32_t get_rc_stick_deflection (int32_t axis);

#endif // _RC_CONTROLS_H