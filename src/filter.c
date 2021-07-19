#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "filter.h"
#include "maths.h"
#include "utils.h"

static void biquad_filter_setup_passthrough (biquad_filter_ts *filter);

// NULL filter
float null_filter_apply (void *filter, float input)
{
    UNUSED(filter);
    return input;
}

float null_filter_apply4 (void *filter, float input, float f_cut, float dt)
{
    UNUSED(filter);
    UNUSED(f_cut);
    UNUSED(dt);
    return input;
}

// PT1 Low Pass filter

static float pt1_Compute_RC (const float f_cut)
{
    return 1.0f / (2.0f * M_PIf * f_cut);
}

// f_cut = cutoff frequency
void pt1_filter_init_RC (pt1_filter_ts *filter, float tau, float dT)
{
    filter->state = 0.0f;
    filter->RC = tau;
    filter->dT = dT;
    filter->alpha = filter->dT / (filter->RC + filter->dT);
}

void pt1_filter_init (pt1_filter_ts *filter, float f_cut, float dT)
{
    pt1_filter_init_RC(filter, pt1_Compute_RC(f_cut), dT);
}

void pt1_filter_set_time_constant (pt1_filter_ts *filter, float tau) {
    filter->RC = tau;
}

float pt1_filter_get_last_output (pt1_filter_ts *filter) {
    return filter->state;
}

void pt1_filter_update_cutoff (pt1_filter_ts *filter, float f_cut)
{
    filter->RC = pt1_Compute_RC(f_cut);
    filter->alpha = filter->dT / (filter->RC + filter->dT);
}

float pt1_filter_apply (pt1_filter_ts *filter, float input)
{
    filter->state = filter->state + filter->alpha * (input - filter->state);
    return filter->state;
}

float pt1_filter_apply3 (pt1_filter_ts *filter, float input, float dT)
{
    filter->dT = dT;
    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);
    return filter->state;
}

float pt1_filter_apply4 (pt1_filter_ts *filter, float input, float f_cut, float dT)
{
    // Pre calculate and store RC
    if (!filter->RC) {
        filter->RC = pt1_Compute_RC(f_cut);
    }

    filter->dT = dT;    // cache latest dT for possible use in pt1FilterApply
    filter->alpha = filter->dT / (filter->RC + filter->dT);
    filter->state = filter->state + filter->alpha * (input - filter->state);

    return filter->state;
}

void pt1_filter_reset (pt1_filter_ts *filter, float input)
{
    filter->state = input;
}

/*
 * PT2 LowPassFilter
 */
float pt2_filter_gain (float f_cut, float dT)
{
    const float order = 2.0f;
    const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f / order) - 1);
    float RC = 1 / (2 * orderCutoffCorrection * M_PIf * f_cut);
    // float RC = 1 / (2 * 1.553773974f * M_PIf * f_cut);
    // where 1.553773974 = 1 / sqrt( (2^(1 / order) - 1) ) and order is 2
    return dT / (RC + dT);
}

void pt2_filter_init (pt2_filter_ts *filter, float k)
{
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->k = k;
}

void pt2_filter_update_cutoff (pt2_filter_ts *filter, float k)
{
    filter->k = k;
}

float pt2_filter_apply (pt2_filter_ts *filter, float input)
{
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state = filter->state + filter->k * (filter->state1 - filter->state);
    return filter->state;
}

/*
 * PT3 LowPassFilter
 */
float pt3_filter_gain (float f_cut, float dT)
{
    const float order = 3.0f;
    const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f / order) - 1);
    float RC = 1 / (2 * orderCutoffCorrection * M_PIf * f_cut);
    // float RC = 1 / (2 * 1.961459177f * M_PIf * f_cut);
    // where 1.961459177 = 1 / sqrt( (2^(1 / order) - 1) ) and order is 3
    return dT / (RC + dT);
}

void pt3_filter_init (pt3_filter_ts *filter, float k)
{
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->state2 = 0.0f;
    filter->k = k;
}

void pt3_filter_update_cutoff (pt3_filter_ts *filter, float k)
{
    filter->k = k;
}

float pt3_filter_apply (pt3_filter_ts *filter, float input)
{
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state2 = filter->state2 + filter->k * (filter->state1 - filter->state2);
    filter->state = filter->state + filter->k * (filter->state2 - filter->state);
    return filter->state;
}

// rate_limit = максимальная скорость изменения выходного значения в единицах в секунду
void rate_limit_filter_init (rate_limit_filter_ts *filter)
{
    filter->state = 0;
}

float rate_limit_filter_apply4 (rate_limit_filter_ts *filter, float input, float rate_limit, float dT)
{
    if (rate_limit > 0) {
        const float rateLimitPerSample = rate_limit * dT;
        filter->state = constrainf(input, filter->state - rateLimitPerSample, filter->state + rateLimitPerSample);
    } else {
        filter->state = input;
    }

    return filter->state;
}

float filter_get_notch_Q (float center_frequency_hz, float cutoff_frequency_hz)
{
    return center_frequency_hz * cutoff_frequency_hz / (center_frequency_hz * center_frequency_hz - cutoff_frequency_hz * cutoff_frequency_hz);
}

void biquad_filter_init_notch (biquad_filter_ts *filter, uint32_t sampling_interval_us, uint16_t filter_freq, uint16_t cutoff_hz)
{
    float Q = filter_get_notch_Q(filter_freq, cutoff_hz);
    biquad_filter_init(filter, filter_freq, sampling_interval_us, Q, FILTER_NOTCH);
}

// настраиваем биквадратный фильтр
void biquad_filter_init_LPF (biquad_filter_ts *filter, uint16_t filter_freq, uint32_t sampling_interval_us)
{
    biquad_filter_init(filter, filter_freq, sampling_interval_us, BIQUAD_Q, FILTER_LPF);
}


static void biquad_filter_setup_passthrough (biquad_filter_ts *filter)
{
    // По умолчанию установлено как сквозное
    filter->b0 = 1.0f;
    filter->b1 = 0.0f;
    filter->b2 = 0.0f;
    filter->a1 = 0.0f;
    filter->a2 = 0.0f;
}

void biquad_filter_init (biquad_filter_ts *filter, uint16_t filter_freq, uint32_t sampling_interval_us, float Q, biquad_filter_type_te filter_type)
{
    // Проверяем частоту Найквиста, и если невозможно инициализировать фильтр по запросу - установить вообще без фильтрации
    if (filter_freq < (1000000 / sampling_interval_us / 2)) {
        const float sample_rate = 1.0f / ((float)sampling_interval_us * 0.000001f);
        const float omega = 2.0f * M_PIf * ((float)filter_freq) / sample_rate;
        const float sn = get_sin_approx(omega);
        const float cs = get_cos_approx(omega);
        const float alpha = sn / (2 * Q);

        float b0, b1, b2;

        switch (filter_type) {
            case FILTER_LPF:
                b0 = (1 - cs) / 2;
                b1 = 1 - cs;
                b2 = (1 - cs) / 2;
                break;

            case FILTER_NOTCH:
                b0 = 1;
                b1 = -2 * cs;
                b2 = 1;
                break;

            default:
                biquad_filter_setup_passthrough(filter);
                return;
        }

        const float a0 =  1 + alpha;
        const float a1 = -2 * cs;
        const float a2 =  1 - alpha;

        // предварительно вычисляем коэффициенты
        filter->b0 = b0 / a0;
        filter->b1 = b1 / a0;
        filter->b2 = b2 / a0;
        filter->a1 = a1 / a0;
        filter->a2 = a2 / a0;
    } else {
        biquad_filter_setup_passthrough(filter);
    }

    // обнуляем сэмплы
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

float biquad_filter_apply_DF1 (biquad_filter_ts *filter, float input)
{
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    // сдвигаем x1 в x2, y1 в y2
    filter->x2 = filter->x1;
    filter->y2 = filter->y1;

    filter->x1 = input;    
    filter->y1 = result;

    return result;
}

// Вычисляет фильтр biquad_t по образцу
float biquad_filter_apply (biquad_filter_ts *filter, float input)
{
    const float result = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;

    return result;
}

float biquad_filter_reset (biquad_filter_ts *filter, float value)
{
    filter->x1 = value - (value * filter->b0);
    filter->x2 = (filter->b2 - filter->a2) * value;

    return value;
}

void biquad_filter_update (biquad_filter_ts *filter, float filter_freq, uint32_t refresh_rate, float Q, biquad_filter_type_te filter_type)
{
    // backup state
    float x1 = filter->x1;
    float x2 = filter->x2;
    float y1 = filter->y1;
    float y2 = filter->y2;

    biquad_filter_init(filter, filter_freq, refresh_rate, Q, filter_type);

    // restore state
    filter->x1 = x1;
    filter->x2 = x2;
    filter->y1 = y1;
    filter->y2 = y2;
}

// #ifdef USE_ALPHA_BETA_GAMMA_FILTER
void alpha_beta_gamma_filter_init(alpha_beta_gamma_filter_ts *filter, float alpha, float boost_gain, float half_life, float dT) 
{
    // beta, gamma, and eta gains all derived from
    // http://yadda.icm.edu.pl/yadda/element/bwmeta1.element.baztech-922ff6cb-e991-417f-93f0-77448f1ef4ec/c/A_Study_Jeong_1_2017.pdf

    const float xi = powf(-alpha + 1.0f, 0.25); // корень четвертой степени из -a + 1

    filter->xk = 0.0f;
    filter->vk = 0.0f;
    filter->ak = 0.0f;
    filter->jk = 0.0f;
    filter->a = alpha;
    filter->b = (1.0f / 6.0f) * powf(1.0f - xi, 2) * (11.0f + 14.0f * xi + 11 * xi * xi);
    filter->g = 2 * powf(1.0f - xi, 3) * (1 + xi);
    filter->e = (1.0f / 6.0f) * powf(1 - xi, 4);
	filter->dT = dT;
	filter->dT2 = dT * dT;
    filter->dT3 = dT * dT * dT;
    pt1_filter_init(&filter->boost_filter, 100, dT);

    const float boost = boost_gain * 100;

    filter->boost = (boost * boost / 10000) * 0.003;
    filter->half_life = half_life != 0 ? powf(0.5f, dT / half_life): 1.0f;
}

float alpha_beta_gamma_filter_apply (alpha_beta_gamma_filter_ts *filter, float input) 
{
    // xk - текущее состояние системы (т.е. положение)
    // vk - производная состояния системы (т.е. скорость)
    // ak - производная от скорости системы (то есть: ускорение)
    // jk - производная от ускорения системы (т.е. рывок)
    float rk;   // остаточная ошибка

    // даем фильтру ограниченную историю
    filter->xk *= filter->half_life;
    filter->vk *= filter->half_life;
    filter->ak *= filter->half_life;
    filter->jk *= filter->half_life;

    // обновляем наше (оценочное) состояние 'x' из системы (т.е. pos = pos + vel (last) .dT)
    filter->xk += filter->dT * filter->vk + (1.0f / 2.0f) * filter->dT2 * filter->ak + (1.0f / 6.0f) * filter->dT3 * filter->jk;
    
    // обновляем (оценочную) скорость (также оцениваемую dterm по результатам измерения)
    filter->vk += filter->dT * filter->ak + 0.5f * filter->dT2 * filter->jk;
    filter->ak += filter->dT * filter->jk;
    
    // какова наша остаточная ошибка (измерено - оценено)
    rk = input - filter->xk;

    // искусственно увеличить ошибку, чтобы увеличить отклик фильтра
    rk += pt1_filter_apply(&filter->boost_filter, fabsf(rk) * rk * filter->boost);
    if ((fabsf(rk * filter->a) > fabsf(input - filter->xk))) {
        rk = (input - filter->xk) / filter->a;
    }
    filter->rk = rk; // для регистрации

    // обновляем наши оценки с учетом остаточной ошибки
    filter->xk += filter->a * rk;
    filter->vk += filter->b / filter->dT * rk;
    filter->ak += filter->g / (2.0f * filter->dT2) * rk;
    filter->jk += filter->e / (6.0f * filter->dT3) * rk;

	return filter->xk;
}