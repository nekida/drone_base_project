#include "pid_autotune.h"

static void autotune_update_gains (pid_autotune_data_ts *data);
static void blackbox_log_autotune_event (/*adjustmentFunction_e adjustmentFunction, */int32_t new_value);

pid_autotune_config_ts pid_autotune_config = {
    .fw_min_stick = SETTING_FW_AUTOTUNE_MIN_STICK_DEFAULT,
    .fw_ff_to_p_gain = SETTING_FW_AUTOTUNE_FF_TO_P_GAIN_DEFAULT,
    .fw_ff_to_i_time_constant = SETTING_FW_AUTOTUNE_FF_TO_I_TC_DEFAULT,
    .fw_p_to_d_gain = SETTING_FW_AUTOTUNE_P_TO_D_GAIN_DEFAULT,
    .fw_rate_adjustment = SETTING_FW_AUTOTUNE_RATE_ADJUSTMENT_DEFAULT,
    .fw_max_rate_deflection = SETTING_FW_AUTOTUNE_MAX_RATE_DEFLECTION_DEFAULT
};

static pid_autotune_data_ts     tune_current[XYZ_AXIS_COUNT];
static pid_autotune_data_ts     tune_saved[XYZ_AXIS_COUNT];
static uint32_t                 last_gains_update_time;

void autotune_fixed_wing_update (const flight_dynamics_index_te axis, float desired_rate, float reached_rate, float pid_output)
{
    float max_rate_setting = tune_current[axis].rate;
    float gain_f_f = tune_current[axis].gain_f_f;
    float max_desired_rate = max_rate_setting;

    const float pid_sum_limit = (axis == FD_YAW) ? p_pid_profile->pid_sum_limit_yaw : p_pid_profile->pid_sum_limit;
    const float abs_desired_rate = fabsf(desired_rate);
    const float abs_reached_rate = fabsf(reached_rate);
    const float abs_pid_output = fabsf(pid_output);
    const bool correct_direction = (desired_rate > 0) == (reached_rate > 0);
    float rate_full_stick;

    bool gains_updated = false;
    bool rates_updated = false;

    const uint32_t current_time_ms = /*millis()*/ 0xFFAA;
    static uint32_t previous_sample_time_ms = 0;
    const int32_t time_since_previous_sample = current_time_ms - previous_sample_time_ms;

    // Использование другой максимальной скорости в режиме ANGLE
    if (FLIGHT_MODE(ANGLE_MODE)) {
        float max_desired_rate_in_angle_mode = DECIDEGREES_TO_DEGREES(p_pid_profile->max_angle_inclination[axis] * 1.0F) * get_pid_bank()->pid[PID_LEVEL].P / FP_PID_LEVEL_P_MULTIPLIER;
        max_desired_rate = MIN(max_rate_setting, max_desired_rate_in_angle_mode);
    }

    const float stick_input = abs_desired_rate / max_desired_rate;

    if ((stick_input > (pid_autotune_config.fw_min_stick / 100.0F)) && correct_direction && (time_since_previous_sample >= AUTOTUNE_FIXED_WING_SAMPLE_INTERVAL)) {
        // Записываем значения каждые 20 мс и вычисляем скользящее среднее по выборкам
        tune_current[axis].update_count++;
        tune_current[axis].abs_desired_rate_accum += (abs_desired_rate - tune_current[axis].abs_desired_rate_accum) / MIN(tune_current[axis].update_count, (uint32_t)AUTOTUNE_FIXED_WING_SAMPLES);
        tune_current[axis].abs_reached_rate_accum += (abs_reached_rate - tune_current[axis].abs_reached_rate_accum) / MIN(tune_current[axis].update_count, (uint32_t)AUTOTUNE_FIXED_WING_SAMPLES);
        tune_current[axis].abs_pid_output_accum += (abs_pid_output - tune_current[axis].abs_pid_output_accum) / MIN(tune_current[axis].update_count, (uint32_t)AUTOTUNE_FIXED_WING_SAMPLES);;

        if ((tune_current[axis].update_count & 25) == 0 && tune_current[axis].update_count >= AUTOTUNE_FIXED_WING_MIN_SAMPLES) {
            if (pid_autotune_config.fw_rate_adjustment != FIXED  && !FLIGHT_MODE(ANGLE_MODE)) { // Обнаружение скорости невозможно в режиме ANGLE
                
                // Задача отклонения контрольной поверхности 80%, чтобы оставить место для работы P и I
                float pid_sum_target = (pid_autotune_config.fw_max_rate_deflection / 100.0F) * pid_sum_limit;

                // Теоретически достижимая скорость при целевом отклонении
                rate_full_stick = pid_sum_target / tune_current[axis].abs_pid_output_accum * tune_current[axis].abs_reached_rate_accum;

                // Оценить обновление
                if (rate_full_stick > (max_rate_setting + 10.0F)) {
                    max_rate_setting += 10.0F;
                } else if (rate_full_stick < (max_rate_setting - 10.0F)) {
                    max_rate_setting -= 10.0F;
                }

                // Ограничение безопасными значениями
                uint16_t min_rate = (axis == FD_YAW) ? AUTOTUNE_FIXED_WING_MIN_YAW_RATE : AUTOTUNE_FIXED_WING_MIN_ROLL_PITCH_RATE;
                uint16_t max_rate = (pid_autotune_config.fw_rate_adjustment == AUTO) ? AUTOTUNE_FIXED_WING_MAX_RATE : MAX(tune_current[axis].initial_rate, min_rate);
                tune_current[axis].rate = constrainf(max_rate_setting, min_rate, max_rate);
                rates_updated = true;
            }

            // Обновить FF до значения, необходимого для достижения текущей цели
            gain_f_f += (tune_current[axis].abs_pid_output_accum / tune_current[axis].abs_reached_rate_accum * FP_PID_RATE_FF_MULTIPLIER - gain_f_f) * (AUTOTUNE_FIXED_WING_CONVERGENCE_RATE / 100.0F);
            tune_current[axis].gain_f_f = constrainf(gain_f_f, AUTOTUNE_FIXED_WING_MIN_FF, AUTOTUNE_FIXED_WING_MAX_FF);
            gains_updated = true;
        }

        // Сброс таймера
        previous_sample_time_ms = current_time_ms;
    }

    if (gains_updated) {
        // Устанавливаем P-усиление на 10% от FF-усиления (довольно агрессивно - FIXME)
        tune_current[axis].gain_p = tune_current[axis].gain_f_f * (pid_autotune_config.fw_ff_to_p_gain / 100.0F);
        tune_current[axis].gain_p = constrainf(tune_current[axis].gain_p, 1.0F, 20.0F);

        // Устанавливаем D-усиление относительно P-усиления (на данный момент 0)
        tune_current[axis].gain_d = tune_current[axis].gain_p * (pid_autotune_config.fw_p_to_d_gain / 100.0F);

        // Устанавливаем усиление интегратора для достижения той же реакции, что и усиление FF за 0,667 секунды
        tune_current[axis].gain_i = (tune_current[axis].gain_f_f / FP_PID_RATE_FF_MULTIPLIER) * (1000.0F / pid_autotune_config.fw_ff_to_i_time_constant) * FP_PID_RATE_I_MULTIPLIER;
        tune_current[axis].gain_i = constrainf(tune_current[axis].gain_i, 2.0F, 30.0F);
        autotune_update_gains(tune_current);

        switch (axis) {
            case FD_ROLL:
                blackbox_log_autotune_event(/*ADJUSTMENT_ROLL_FF, */tune_current[axis].gain_f_f);
                break;

            case FD_PITCH:
                blackbox_log_autotune_event(/*ADJUSTMENT_PITCH_FF, */tune_current[axis].gain_f_f);
                break;

            case FD_YAW:
                blackbox_log_autotune_event(/*ADJUSTMENT_YAW_FF, */tune_current[axis].gain_f_f);
                break;
        }
        gains_updated = false;
    }

    if (rates_updated) {
        autotune_update_gains(tune_current);

        switch (axis) {
            case FD_ROLL:
                blackbox_log_autotune_event(/*ADJUSTMENT_ROLL_RATE, */tune_current[axis].rate);
                break;

            case FD_PITCH:
                blackbox_log_autotune_event(/*ADJUSTMENT_PITCH_RATE, */tune_current[axis].rate);
                break;

            case FD_YAW:
                blackbox_log_autotune_event(/*ADJUSTMENT_YAW_RATE, */tune_current[axis].rate);
                break;
        }

        rates_updated = false;
    }
}

static void autotune_update_gains (pid_autotune_data_ts *data)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        get_pid_bank_mutable()->pid[axis].P = lrintf(data[axis].gain_p);
        get_pid_bank_mutable()->pid[axis].I = lrintf(data[axis].gain_i);
        get_pid_bank_mutable()->pid[axis].D = lrintf(data[axis].gain_d);
        get_pid_bank_mutable()->pid[axis].FF = lrintf(data[axis].gain_f_f);
        current_control_rate_profile.stabilized.rates[axis] = lrintf(data[axis].rate / 10.0F);
    }

    schedule_pid_gains_update();
}

/**
  * @brief Запись логов корректировки на черный ящик
  * @param Enum функции корректировки, новое значение
  * @retval Нет
  * @todo Заглушка в теле функции    
  */
static void blackbox_log_autotune_event (/*adjustmentFunction_e adjustmentFunction, */int32_t new_value)
{
// #ifndef USE_BLACKBOX
//     UNUSED(adjustmentFunction);
//     UNUSED(newValue);
// #else
//     if (feature(FEATURE_BLACKBOX)) {
//         flightLogEvent_inflightAdjustment_t eventData;
//         eventData.adjustmentFunction = adjustmentFunction;
//         eventData.newValue = newValue;
//         eventData.floatFlag = false;
//         blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
//     }
// #endif
}