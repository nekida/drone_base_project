#ifndef _PID_H
#define _PID_H

#include <stdbool.h>
#include "maths.h"
#include <math.h>
#include "filter.h"
#include "smith_predictor.h"
#include "utils.h"
#include "control_rate_profile.h"
#include "scheduler.h"
#include "mixer.h"
#include "rc_controls.h"

#define ANGLE_INDEX_COUNT 2
#define FLIGHT_DYNAMICS_INDEX_COUNT 3
#define XYZ_AXIS_COUNT 3
#define PID_GYRO_RATE_BUF_LENGTH 5
#define TASK_AUX_RATE_HZ   100 //In Hz
#define HEADING_HOLD_ERROR_LPF_FREQ 2
#define GYRO_SATURATION_LIMIT       1800        // 1800dps

#define FP_PID_RATE_FF_MULTIPLIER   31.0f
#define FP_PID_RATE_P_MULTIPLIER    31.0f
#define FP_PID_RATE_I_MULTIPLIER    4.0f
#define FP_PID_RATE_D_MULTIPLIER    1905.0f
#define FP_PID_RATE_D_FF_MULTIPLIER   7270.0f
#define FP_PID_LEVEL_P_MULTIPLIER   6.56f       // Level P gain units is [1/sec] and angle error is [deg] => [deg/s]
#define FP_PID_YAWHOLD_P_MULTIPLIER 80.0f
#define GRAVITY_CMSS    980.665f
#define AXIS_ACCEL_MIN_LIMIT        50
#define MC_ITERM_RELAX_SETPOINT_THRESHOLD 40.0f
#define MC_ITERM_RELAX_CUTOFF_DEFAULT 15

typedef enum {
    X = 0,
    Y,
    Z
} axis_te;

typedef enum {
    FD_ROLL = 0,
    FD_PITCH,
    FD_YAW
} flight_dynamics_index_te;

typedef enum {
    AI_ROLL = 0,
    AI_PITCH,
} angle_index_te;

typedef enum {
    HEADING_HOLD_DISABLED = 0,
    HEADING_HOLD_UPDATE_HEADING,
    HEADING_HOLD_ENABLED
} HEADING_HOLD_STATE_te;

typedef union {
    float v[3];
    struct {
       float x, y, z;
    };
} fp_vector3_tu;

typedef struct {
    float m[3][3];
} fp_mat3_ts;

typedef struct {
    fp_vector3_tu axis;
    float angle;
} fp_axis_angle_ts;

typedef enum {
    /* PID              MC      FW  */
    PID_ROLL,       //   +       +
    PID_PITCH,      //   +       +
    PID_YAW,        //   +       +
    PID_POS_Z,      //   +       +
    PID_POS_XY,     //   +       +
    PID_VEL_XY,     //   +       n/a
    PID_SURFACE,    //   n/a     n/a
    PID_LEVEL,      //   +       +
    PID_HEADING,    //   +       +
    PID_VEL_Z,      //   +       n/a
    PID_POS_HEADING,//   n/a     +
    PID_ITEM_COUNT
} pid_index_te;

// TODO(agh): PIDFF
typedef enum {
    PID_TYPE_NONE = 0,  // Not used in the current platform/mixer/configuration
    PID_TYPE_PID,   // Uses P, I and D terms
    PID_TYPE_PIFF,  // Uses P, I, D and FF
    PID_TYPE_AUTO,  // Autodetect
} pid_type_te;

typedef struct {
    uint16_t P;
    uint16_t I;
    uint16_t D;
    uint16_t FF;
} pid8_ts;

typedef struct {
    pid8_ts  pid[PID_ITEM_COUNT];
} pid_bank_ts;

typedef enum {
    ITERM_RELAX_OFF = 0,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY
} iterm_relax_te;

enum {
    SETTING_MC_P_ROLL_DEFAULT   = 40,
    SETTING_MC_I_ROLL_DEFAULT   = 30,
    SETTING_MC_D_ROLL_DEFAULT   = 23,
    SETTING_MC_CD_ROLL_DEFAULT  = 60,

    SETTING_MC_P_PITCH_DEFAULT  = 40,
    SETTING_MC_I_PITCH_DEFAULT  = 30,
    SETTING_MC_D_PITCH_DEFAULT  = 23,
    SETTING_MC_CD_PITCH_DEFAULT = 60,

    SETTING_MC_P_YAW_DEFAULT    = 85,
    SETTING_MC_I_YAW_DEFAULT    = 45,
    SETTING_MC_D_YAW_DEFAULT    = 0,
    SETTING_MC_CD_YAW_DEFAULT   = 60,

    SETTING_MC_P_LEVEL_DEFAULT  = 20,
    SETTING_MC_I_LEVEL_DEFAULT  = 15,
    SETTING_MC_D_LEVEL_DEFAULT  = 75,
    SETTING_MC_FF_LEVEL_DEFAULT = 0,

    SETTING_NAV_MC_HEADING_P_DEFAULT  = 60,
    SETTING_NAV_MC_HEADING_I_DEFAULT  = 0,
    SETTING_NAV_MC_HEADING_D_DEFAULT  = 0,
    SETTING_NAV_MC_HEADING_FF_DEFAULT = 0,
    
    SETTING_NAV_MC_POS_XY_P_DEFAULT  = 65,
    SETTING_NAV_MC_POS_XY_I_DEFAULT  = 0,
    SETTING_NAV_MC_POS_XY_D_DEFAULT  = 0,
    SETTING_NAV_MC_POS_XY_FF_DEFAULT = 0,

    SETTING_NAV_MC_VEL_XY_P_DEFAULT   = 40,   
    SETTING_NAV_MC_VEL_XY_I_DEFAULT   = 15,   
    SETTING_NAV_MC_VEL_XY_D_DEFAULT   = 100,  
    SETTING_NAV_MC_VEL_XY_FF_DEFAULT  = 40,  

    SETTING_NAV_MC_POS_Z_P_DEFAULT  = 50,
    SETTING_NAV_MC_POS_Z_I_DEFAULT  = 0, 
    SETTING_NAV_MC_POS_Z_D_DEFAULT  = 0,
    SETTING_NAV_MC_POS_Z_FF_DEFAULT = 0,

    SETTING_NAV_MC_VEL_Z_P_DEFAULT = 100,
    SETTING_NAV_MC_VEL_Z_I_DEFAULT = 50,
    SETTING_NAV_MC_VEL_Z_D_DEFAULT = 10,
    SETTING_NAV_MC_VEL_Z_FF_DEFAULT = 0,

    SETTING_NAV_MC_POS_HEADING_P_DEFAULT  = 0,
    SETTING_NAV_MC_POS_HEADING_I_DEFAULT  = 0,
    SETTING_NAV_MC_POS_HEADING_D_DEFAULT  = 0,
    SETTING_NAV_MC_POS_HEADING_FF_DEFAULT = 0
};

enum {
    SETTING_FW_P_ROLL_DEFAULT   = 5,
    SETTING_FW_I_ROLL_DEFAULT   = 7,
    SETTING_FW_D_ROLL_DEFAULT   = 0,
    SETTING_FW_FF_ROLL_DEFAULT  = 50,

    SETTING_FW_P_PITCH_DEFAULT   = 5,
    SETTING_FW_I_PITCH_DEFAULT   = 7,
    SETTING_FW_D_PITCH_DEFAULT   = 0,
    SETTING_FW_FF_PITCH_DEFAULT  = 50,

    SETTING_FW_P_YAW_DEFAULT   = 6,
    SETTING_FW_I_YAW_DEFAULT   = 10,
    SETTING_FW_D_YAW_DEFAULT   = 0,
    SETTING_FW_FF_YAW_DEFAULT  = 60,

    SETTING_FW_P_LEVEL_DEFAULT  = 20,
    SETTING_FW_I_LEVEL_DEFAULT  = 5,
    SETTING_FW_D_LEVEL_DEFAULT  = 75,
    SETTING_FW_FF_LEVEL_DEFAULT = 0,

    SETTING_NAV_FW_HEADING_P_DEFAULT  = 60,
    SETTING_NAV_FW_HEADING_I_DEFAULT  = 0,
    SETTING_NAV_FW_HEADING_D_DEFAULT  = 0,
    SETTING_NAV_FW_HEADING_FF_DEFAULT = 0,

    SETTING_NAV_FW_POS_Z_P_DEFAULT  = 40,
    SETTING_NAV_FW_POS_Z_I_DEFAULT  = 5,
    SETTING_NAV_FW_POS_Z_D_DEFAULT  = 10,
    SETTING_NAV_FW_POS_Z_FF_DEFAULT = 0,

    SETTING_NAV_FW_POS_XY_P_DEFAULT   = 75,     
    SETTING_NAV_FW_POS_XY_I_DEFAULT   = 5,     
    SETTING_NAV_FW_POS_XY_D_DEFAULT   = 8,     
    SETTING_NAV_FW_POS_XY_FF_DEFAULT  = 0,

    SETTING_NAV_FW_POS_HDG_P_DEFAULT  = 30,
    SETTING_NAV_FW_POS_HDG_I_DEFAULT  = 2,
    SETTING_NAV_FW_POS_HDG_D_DEFAULT  = 0,
    SETTING_NAV_FW_POS_HDG_FF_DEFAULT = 0
};

enum {
    HEADING_HOLD_RATE_LIMIT_MIN     = 10,
    HEADING_HOLD_RATE_LIMIT_MAX     = 250,
    HEADING_HOLD_RATE_LIMIT_DEFAULT = 90
};

enum {
    PID_SUM_LIMIT_MIN           = 100,
    PID_SUM_LIMIT_MAX           = 1000,
    PID_SUM_LIMIT_DEFAULT       = 500,
    PID_SUM_LIMIT_YAW_DEFAULT   = 400
};

enum {
    FW_ITERM_THROW_LIMIT_DEFAULT    = 165,
    FW_ITERM_THROW_LIMIT_MIN        = 0,
    FW_ITERM_THROW_LIMIT_MAX        = 500
};

enum {
    NAV_LOITER_RIGHT = 0,                    // Loitering direction right
    NAV_LOITER_LEFT  = 1,                    // Loitering direction left
    NAV_LOITER_YAW   = 2
};

enum {
    OFF = false,
    ON  = !OFF
};

typedef struct {
    uint8_t pid_controller_type;

    pid_bank_ts bank_fw;
    pid_bank_ts bank_mc;

    uint8_t dterm_lpf_type;                 // Dterm LPF type: PT1, BIQUAD
    uint16_t dterm_lpf_hz;                  
    
    uint8_t dterm_lpf2_type;                // Dterm LPF type: PT1, BIQUAD
    uint16_t dterm_lpf2_hz;                 
    
    uint8_t yaw_lpf_hz;

    uint8_t heading_hold_rate_limit;        // Maximum rotation rate HEADING_HOLD mode can feed to yaw rate PID controller

    uint8_t iterm_windup_point_percent;        // Experimental ITerm windup threshold, percent of motor saturation

    uint32_t axis_acceleration_limit_yaw;          // Max rate of change of yaw angular rate setpoint (deg/s^2 = dps/s)
    uint32_t axis_acceleration_limit_roll_pitch;    // Max rate of change of roll/pitch angular rate setpoint (deg/s^2 = dps/s)

    int16_t max_angle_inclination[ANGLE_INDEX_COUNT];       // Max possible inclination (roll and pitch axis separately

    uint16_t pid_sum_limit;
    uint16_t pid_sum_limit_yaw;

    // Airplane-specific parameters
    uint16_t    fixed_wing_iterm_throw_limit;
    float       fixed_wing_reference_airspeed;     // Reference tuning airspeed for the airplane - the speed for which PID gains are tuned
    float       fixed_wing_coordinated_yaw_gain;    // This is the gain of the yaw rate required to keep the yaw rate consistent with the turn rate for a coordinated turn.
    float       fixed_wing_coordinated_pitch_gain;    // This is the gain of the pitch rate to keep the pitch angle constant during coordinated turns.
    float       fixed_wing_iterm_limit_on_stick_position;   //Do not allow Iterm to grow when stick position is above this point
    uint16_t    fixed_wing_yaw_iterm_bank_freeze;       // Freeze yaw Iterm when bank angle is more than this many degrees

    uint8_t loiter_direction;               // Direction of loitering center point on right wing (clockwise - as before), or center point on left wing (counterclockwise)
    float   nav_vel_xy_dterm_lpf_hz;
    uint8_t nav_vel_xy_dterm_attenuation;       // VEL_XY dynamic Dterm scale: Dterm will be attenuatedby this value (in percent) when UAV is traveling with more than navVelXyDtermAttenuationStart percents of max velocity
    uint8_t nav_vel_xy_dterm_attenuation_start;  // VEL_XY dynamic Dterm scale: Dterm attenuation will begin at this percent of max velocity
    uint8_t nav_vel_xy_dterm_attenuation_end;    // VEL_XY dynamic Dterm scale: Dterm will be fully attenuated at this percent of max velocity
    uint8_t iterm_relax_cutoff;             // This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
    uint8_t iterm_relax;                    // Enable iterm suppression during stick input

// USE_D_BOOST
    float d_boost_factor;
    float d_boost_max_at_alleceleration;
    uint8_t d_boost_gyro_delta_lpf_hz;

// USE_ANTIGRAVITY
    float antigravity_gain;
    float antigravity_accelerator;
    uint8_t antigravity_cutoff;

    uint16_t nav_fw_pos_hdg_pid_sum_limit;
    uint8_t control_derivative_lpf_hz;

// USE_GYRO_KALMAN
    uint16_t kalman_q;
    uint16_t kalman_w;
    uint16_t kalman_sharpness;
    bool kalman_enabled;

    float fixed_wing_level_trim;
    float fixed_wing_level_trim_gain;
    float fixed_wing_level_trim_deadband;
// USE_SMITH_PREDICTOR
    float smith_predictor_strength;
    float smith_predictor_delay;
    uint16_t smith_predictor_filter_hz;
} pid_profile_ts;

typedef struct {
    float kP;   // Пропорциональное усиление
    float kI;   // Интегральное усиление
    float kD;   // Дифференциальное усиление
    float kFF;  // Усиление с прямой связью
    float kCD;  // Контрольная производная
    float kT;   // Усиление отслеживания обратных вычислений

    float gyro_rate;
    float rate_target;

    // Буфер для расчета производной
    float gyro_rate_buf[PID_GYRO_RATE_BUF_LENGTH];
    fir_filter_ts gyro_rate_filter;

    // Rate integrator
    float error_gyro_if;
    float error_gyro_if_limit;

    // Используется для УГЛОВОЙ фильтрации (PT1, нам здесь не нужна сверхрезкость)
    pt1_filter_ts angle_filter_state;

    // Rate filtering
    rate_limit_filter_ts axis_accel_filter;
    fir_filter_ts p_term_lpf_state;
    filter_tu d_term_lpf_state;
    filter_tu d_term_lpf2_state;

    float stick_position;

    float previous_rate_target;
    float previous_rate_gyro;

// #ifdef USE_D_BOOST - в common.h
    pt1_filter_ts d_boost_lpf;
    biquad_filter_ts d_boost_gyro_lpf;
//#endif
    uint16_t pid_sum_limit;
    filter_apply_4_fn_ptr p_term_filter_apply_fn;
    bool iterm_limit_active;
    bool iterm_freeze_active;

    pt2_filter_ts rate_target_filter;

    smith_predictor_ts smith_predictor;
} pid_state_ts;

typedef struct {
    float kP;
    float kI;
    float kD;
    float kT;   // Tracking gain (anti-windup)
    float kFF;  // FeedForward Component
} pid_controller_param_ts;

typedef enum {
    PID_DTERM_FROM_ERROR            = 1 << 0,
    PID_ZERO_INTEGRATOR             = 1 << 1,
    PID_SHRINK_INTEGRATOR           = 1 << 2,
    PID_LIMIT_INTEGRATOR            = 1 << 3,
    PID_FREEZE_INTEGRATOR           = 1 << 4,
} pid_controller_flags_te;

typedef struct {
    bool reset;
    pid_controller_param_ts param;
    pt1_filter_ts dterm_filter_state;     // last derivative for low-pass filter
    float dterm_lpf_hz;                   // dTerm low pass filter cutoff frequency
    float integrator;                   // integrator value
    float last_input;                   // last input for derivative

    float integral;                     // used integral value in output
    float proportional;                 // used proportional value in output
    float derivative;                   // used derivative value in output
    float feed_forward;                  // used FeedForward value in output
    float output_constrained;           // controller output constrained
} pid_controller_ts;

void pid_init (void);
void pid_reset_TPA_filter (void);

float nav_pid_apply2 (pid_controller_ts *pid, const float setpoint, const float measurement, const float dt, const float out_min, const float out_max, const pid_controller_flags_te pid_flags);
float nav_pid_apply3 ( 
    pid_controller_ts *pid,
    const float setpoint,
    const float measurement,
    const float dt,
    const float out_min,
    const float out_max,
    const pid_controller_flags_te pid_flags,
    const float gainScaler,
    const float dTermScaler
);

void rotation_matrix_from_angles        (fp_mat3_ts *rmat, const fp_angles_tu *angles);
void rotation_matrix_from_axis_angle    (fp_mat3_ts *rmat, const fp_axis_angle_ts *a);

static inline void set_vector_in_zero (fp_vector3_tu *v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

static inline void rotation_matrix_rotate_vector (fp_vector3_tu *result, const fp_vector3_tu *a, const fp_mat3_ts *rmat)
{
    result->x = rmat->m[0][0] * a->x + rmat->m[1][0] * a->y + rmat->m[2][0] * a->z;
    result->y = rmat->m[0][1] * a->x + rmat->m[1][1] * a->y + rmat->m[2][1] * a->z;
    result->z = rmat->m[0][2] * a->x + rmat->m[1][2] * a->y + rmat->m[2][2] * a->z;
}

static inline void vector_normalize (fp_vector3_tu *result, const fp_vector3_tu *v)
{
    float length = fast_fsqrtf(sq(v->x) + sq(v->y) + sq(v->z));
    
    if (length) {
        result->x = v->x / length;
        result->y = v->y / length;
        result->z = v->z / length;
    } else {
        result->x = 0;
        result->y = 0;
        result->z = 0;
    }
}

static inline void vector_cross_product (fp_vector3_tu *result, const fp_vector3_tu *a, const fp_vector3_tu *b)
{
    result->x = a->y * b->z - a->z * b->y;
    result->y = a->z * b->x - a->x * b->z;
    result->z = a->x * b->y - a->y * b->x;
}

static inline void vector_add (fp_vector3_tu *result, const fp_vector3_tu *a, const fp_vector3_tu *b)
{
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;
}

static inline void vector_scale (fp_vector3_tu *result, const fp_vector3_tu *a, const float b)
{
    result->x = a->x * b;
    result->y = a->y * b;
    result->z = a->z * b;
}

void        pid_init                    (void);
void        pid_controller              (float dT);
void        update_heading_hold_target  (int16_t heading);
void        schedule_pid_gains_update   (void);
const       pid_bank_ts *get_pid_bank   (void);
pid_bank_ts *get_pid_bank_mutable       (void); 
void        nav_pid_init                (pid_controller_ts *pid, float _kP, float _kI, float _kD, float _kFF, float _dterm_lpf_hz);

extern pid_profile_ts *p_pid_profile;

#endif // _PID_H