#ifndef _PID_AUTOTUNE_H
#define _PID_AUTOTUNE_H

#include <stdint.h>
#include "pid.h"

#define AUTOTUNE_FIXED_WING_MIN_FF              10
#define AUTOTUNE_FIXED_WING_MAX_FF              255
#define AUTOTUNE_FIXED_WING_MIN_ROLL_PITCH_RATE 40
#define AUTOTUNE_FIXED_WING_MIN_YAW_RATE        10
#define AUTOTUNE_FIXED_WING_MAX_RATE            720
#define AUTOTUNE_FIXED_WING_CONVERGENCE_RATE    10
#define AUTOTUNE_FIXED_WING_SAMPLE_INTERVAL     20      // ms
#define AUTOTUNE_FIXED_WING_SAMPLES             1000    // Используем среднее значение за последние 20 секунд жестких маневров
#define AUTOTUNE_FIXED_WING_MIN_SAMPLES         250     // Начинаем обновление мелодии через 5 секунд жестких маневров

#define SETTING_FW_AUTOTUNE_MIN_STICK_DEFAULT 50
#define SETTING_FW_AUTOTUNE_MIN_STICK 313
#define SETTING_FW_AUTOTUNE_MIN_STICK_MIN 0
#define SETTING_FW_AUTOTUNE_MIN_STICK_MAX 100
#define SETTING_FW_AUTOTUNE_FF_TO_P_GAIN_DEFAULT 10
#define SETTING_FW_AUTOTUNE_FF_TO_P_GAIN 314
#define SETTING_FW_AUTOTUNE_FF_TO_P_GAIN_MIN 0
#define SETTING_FW_AUTOTUNE_FF_TO_P_GAIN_MAX 100
#define SETTING_FW_AUTOTUNE_P_TO_D_GAIN_DEFAULT 0
#define SETTING_FW_AUTOTUNE_P_TO_D_GAIN 315
#define SETTING_FW_AUTOTUNE_P_TO_D_GAIN_MIN 0
#define SETTING_FW_AUTOTUNE_P_TO_D_GAIN_MAX 200
#define SETTING_FW_AUTOTUNE_FF_TO_I_TC_DEFAULT 600
#define SETTING_FW_AUTOTUNE_FF_TO_I_TC 316
#define SETTING_FW_AUTOTUNE_FF_TO_I_TC_MIN 100
#define SETTING_FW_AUTOTUNE_FF_TO_I_TC_MAX 5000
#define SETTING_FW_AUTOTUNE_RATE_ADJUSTMENT_DEFAULT 2
#define SETTING_FW_AUTOTUNE_RATE_ADJUSTMENT 317
#define SETTING_FW_AUTOTUNE_RATE_ADJUSTMENT_MIN 0
#define SETTING_FW_AUTOTUNE_RATE_ADJUSTMENT_MAX 0
#define SETTING_FW_AUTOTUNE_MAX_RATE_DEFLECTION_DEFAULT 80
#define SETTING_FW_AUTOTUNE_MAX_RATE_DEFLECTION 318
#define SETTING_FW_AUTOTUNE_MAX_RATE_DEFLECTION_MIN 50
#define SETTING_FW_AUTOTUNE_MAX_RATE_DEFLECTION_MAX 100

typedef struct {
    uint16_t    fw_detect_time;             // Время [мс] для обнаружения продолжительного недорега или перерегулирования
    uint8_t     fw_min_stick;               // Минимальный ввод стика, необходимый для обновления скорости и прироста
    uint8_t     fw_ff_to_p_gain;            // Прирост от FF к P (соотношение сил) [%]
    uint8_t     fw_p_to_d_gain;             // Прирост от P к D (соотношение сил) [%]
    uint16_t    fw_ff_to_i_time_constant;   // Время от FF до I (определяет время для I, чтобы достичь того же уровня ответа, что и FF) [мс]
    uint8_t     fw_rate_adjustment;         // Отрегулировать настройки скорости во время автонастройки?
    uint8_t     fw_max_rate_deflection;     // Процент максимального выхода микшера, используемый для расчета скорости
} pid_autotune_config_ts;

typedef enum {
    DEMAND_TOO_LOW = 0,
    DEMAND_UNDERSHOOT,
    DEMAND_OVERSHOOT,
    TUNE_UPDATED,
} pid_autotune_state_te;

typedef struct {
    float   gain_p;
    float   gain_i;
    float   gain_d;
    float   gain_f_f;
    float   rate;
    float   initial_rate;
    float   abs_desired_rate_accum;
    float   abs_reached_rate_accum;
    float   abs_pid_output_accum;
    uint32_t update_count;
} pid_autotune_data_ts;

typedef enum {
    FIXED,
    LIMIT,
    AUTO,
} fw_autotune_rate_adjustment_te;

void autotune_fixed_wing_update (const flight_dynamics_index_te axis, float desired_rate, float reached_rate, float pid_output);

#endif // _PID_AUTOTUNE_H