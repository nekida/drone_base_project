#ifndef _FILTER_H
#define _FILTER_H

#define BIQUAD_BANDWIDTH 1.9f           /* полоса пропускания в октавах */
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* фактор качества- butterworth */

typedef struct {
    float state;
} rate_limit_filter_ts;

typedef struct {
    float state;
    float RC;
    float dT;
    float alpha;
} pt1_filter_ts;

typedef struct {
    float state;
    float state1;
    float k;
} pt2_filter_ts;

typedef struct {
    float state;
    float state1;
    float state2;
    float k;
} pt3_filter_ts;

/* содержит данные, необходимые для обновления выборок через фильтр */
typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquad_filter_ts;

typedef union { 
    biquad_filter_ts biquad; 
    pt1_filter_ts pt1; 
} filter_tu;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD
} filterType_te;

typedef enum {
    FILTER_LPF,
    FILTER_NOTCH
} biquad_filter_type_te;

typedef struct {
    float *buf;
    const float *coeffs;
    uint8_t buf_length;
    uint8_t coeffs_length;
} fir_filter_ts;

typedef struct {
    float a, b, g, e;
    float ak; // производная от скорости системы (это: ускорение)
    float vk; // производная состояния системы (то есть: скорость)
    float xk; // текущее состояние системы (например: позиция)
    float jk; // производная от ускорения системы (например, рывок)
    float rk; // остаточная ошибка
    float dT, dT2, dT3;
    float half_life, boost;
    pt1_filter_ts boost_filter;
} alpha_beta_gamma_filter_ts;

typedef float (*filter_apply_fn_ptr)(void *filter, float input);
typedef float (*filter_apply_4_fn_ptr)(void *filter, float input, float f_cut, float dt);

float null_filter_apply     (void *filter, float input);
float null_filter_apply4    (void *filter, float input, float f_cut, float dt);

void    pt1_filter_init               (pt1_filter_ts *filter, float f_cut, float dT);
void    pt1_filter_init_RC            (pt1_filter_ts *filter, float tau, float dT);
void    pt1_filter_set_time_constant  (pt1_filter_ts *filter, float tau);
void    pt1_filter_update_cutoff      (pt1_filter_ts *filter, float f_cut);
float   pt1_filter_get_last_output    (pt1_filter_ts *filter);
float   pt1_filter_apply              (pt1_filter_ts *filter, float input);
float   pt1_filter_apply3             (pt1_filter_ts *filter, float input, float dT);
float   pt1_filter_apply4             (pt1_filter_ts *filter, float input, float f_cut, float dt);
void    pt1_filter_reset              (pt1_filter_ts *filter, float input);

/*
 * PT2 LowPassFilter
 */
float   pt2_filter_gain             (float f_cut, float dT);
void    pt2_filter_init             (pt2_filter_ts *filter, float k);
void    pt2_filter_update_cutoff    (pt2_filter_ts *filter, float k);
float   pt2_filter_apply            (pt2_filter_ts *filter, float input);

/*
 * PT3 LowPassFilter
 */
float   pt3_filter_gain             (float f_cut, float dT);
void    pt3_filter_init             (pt3_filter_ts *filter, float k);
void    pt3_filter_update_cutoff    (pt3_filter_ts *filter, float k);
float   pt3_filter_apply            (pt3_filter_ts *filter, float input);

void    rate_limit_filter_init      (rate_limit_filter_ts *filter);
float   rate_limit_filter_apply4    (rate_limit_filter_ts *filter, float input, float rate_limit, float dT);

void    biquad_filter_init_notch    (biquad_filter_ts *filter, uint32_t sampling_interval_us, uint16_t filter_freq, uint16_t cutoff_hz);
void    biquad_filter_init_LPF      (biquad_filter_ts *filter, uint16_t filter_freq, uint32_t sampling_interval_us);
void    biquad_filter_init          (biquad_filter_ts *filter, uint16_t filter_freq, uint32_t sampling_interval_us, float Q, biquad_filter_type_te filter_type);
float   biquad_filter_apply         (biquad_filter_ts *filter, float sample);
float   biquad_filter_reset         (biquad_filter_ts *filter, float value);
float   biquad_filter_apply_DF1     (biquad_filter_ts *filter, float input);
float   filter_get_notch_Q          (float center_frequency_hz, float cutoff_frequency_hz);
void    biquad_filter_update        (biquad_filter_ts *filter, float filter_freq, uint32_t refresh_rate, float Q, biquad_filter_type_te filter_type);

void    alpha_beta_gamma_filter_init    (alpha_beta_gamma_filter_ts *filter, float alpha, float boost_gain, float half_life, float dT);
float   alpha_beta_gamma_filter_apply   (alpha_beta_gamma_filter_ts *filter, float input);

#endif // _FILTER_H