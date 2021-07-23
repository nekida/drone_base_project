#include "pid.h"

#define FIXED_WING_LEVEL_TRIM_MAX_ANGLE 10.0f // Может потребоваться автоматическая обрезка максимального угла
#define FIXED_WING_LEVEL_TRIM_DIVIDER 500.0f
#define FIXED_WING_LEVEL_TRIM_MULTIPLIER 1.0f / FIXED_WING_LEVEL_TRIM_DIVIDER
#define FIXED_WING_LEVEL_TRIM_CONTROLLER_LIMIT FIXED_WING_LEVEL_TRIM_DIVIDER * FIXED_WING_LEVEL_TRIM_MAX_ANGLE
#define D_BOOST_GYRO_LPF_HZ 80    // Входная отсечка Biquad lowpass до пика D около частот пропитки
#define D_BOOST_LPF_HZ 10         // Отсечка нижних частот PT1 для сглаживания эффекта усиления

static void   pid_apply_fixed_wing_rate_controller (pid_state_ts *pidState, flight_dynamics_index_te axis, float dT);
static void   pid_apply_multicopter_rate_controller (pid_state_ts *pid_state, flight_dynamics_index_te axis, float dT);
static void   null_rate_controller (pid_state_ts *pid_state, flight_dynamics_index_te axis, float dT);
static float  pid_rc_command_to_rate (int16_t stick, uint8_t rate);
static float  pid_heading_hold (float delta_t);
static float  get_calc_horizon_rate_magnitude (void);
static void   pid_level (pid_state_ts *pid_state, flight_dynamics_index_te axis, float horizon_rate_magnitude, float dT);
static float  pid_rc_command_to_angle (int16_t stick, int16_t max_inclination);
static void   pid_turn_assistant (pid_state_ts *pid_state_local, float bank_angle_target_local, float pitch_angle_target_local);
static void   pid_apply_fpv_camera_angle_mix (pid_state_ts *pid_state_local, uint8_t fpv_camera_angle);
static void   pid_apply_setpoint_rate_limiting (pid_state_ts *pid_state_local, flight_dynamics_index_te axis_local, float dT_local);
static bool   is_fixed_wing_iterm_limit_active (float stick_position);
static void   check_iterm_limiting_active (pid_state_ts *pid_state_local);
static void   chech_itrem_freezing_active (pid_state_ts *pid_state_local, flight_dynamics_index_te axis);
static float  p_term_process (pid_state_ts *pid_state_local, float rate_error, float dT);
static float  d_term_process (pid_state_ts *pid_state_local, float dT);
static float  apply_d_boost (pid_state_ts *pid_state_local, float dT);
static float  apply_iterm_relax (const int axis, float current_pid_setpoint, float iterm_error_rate);
static void   apply_iterm_limiting (pid_state_ts *pid_state_local);

// #ifdef USE_BLACKBOX
int32_t axis_pid_p[FLIGHT_DYNAMICS_INDEX_COUNT];
int32_t axis_pid_i[FLIGHT_DYNAMICS_INDEX_COUNT];
int32_t axis_pid_d[FLIGHT_DYNAMICS_INDEX_COUNT];
int32_t axis_pid_setpoint[FLIGHT_DYNAMICS_INDEX_COUNT];

// #ifdef USE_ANTIGRAVITY
static pt1_filter_ts antigravity_throttle_lpf;
static float antigravity_throttle_hpf;
static float antigravity_gain;
static float antigravity_accelerator;

// #ifdef USE_D_BOOST
static float d_boost_factor;
static float d_boost_max_at_alleceleration;

// #ifdef USE_ANTIGRAVITY
static float iTermAntigravityGain;

static bool pid_filters_is_configured = false;
static float heading_hold_cosz_limit;
static int16_t heading_hold_target;
static pt1_filter_ts heading_hold_rate_filter;
static pt1_filter_ts fixed_wing_tpa_filter;

// Коэффициент ослабления ПИД-регулятора тяги. 0,0f означает полное затухание, 1,0f затухание не применяется
static bool pid_gains_update_required;
int16_t axis_pid[FLIGHT_DYNAMICS_INDEX_COUNT];

static pid_state_ts pid_state[FLIGHT_DYNAMICS_INDEX_COUNT];
static pt1_filter_ts windup_lpf[XYZ_AXIS_COUNT];
static uint8_t iterm_relax;

static uint8_t yaw_lpf_hz;
static float motor_iterm_windup_point;
static float anti_windup_scaler;

static uint8_t used_pid_controller_type;

typedef void (*pid_controller_fn_ptr)(pid_state_ts *pidState, flight_dynamics_index_te axis, float dT);
static pid_controller_fn_ptr pid_controller_apply_fn;
static filter_apply_fn_ptr dterm_lpf_filter_apply_fn;
static filter_apply_fn_ptr dterm_lpf2_filter_apply_fn;
static bool leveling_enabled = false;

static float fixed_wing_level_trim;
static pid_controller_ts fixed_wing_level_trim_controller;

pid_profile_ts pid_profile = {
  .bank_mc.pid[PID_ROLL].P    = SETTING_MC_P_ROLL_DEFAULT,
  .bank_mc.pid[PID_ROLL].I    = SETTING_MC_I_ROLL_DEFAULT,
  .bank_mc.pid[PID_ROLL].D    = SETTING_MC_D_ROLL_DEFAULT,
  .bank_mc.pid[PID_ROLL].FF   = SETTING_MC_CD_ROLL_DEFAULT,

  .bank_mc.pid[PID_PITCH].P   = SETTING_MC_P_PITCH_DEFAULT,
  .bank_mc.pid[PID_PITCH].I   = SETTING_MC_I_PITCH_DEFAULT,
  .bank_mc.pid[PID_PITCH].D   = SETTING_MC_D_PITCH_DEFAULT,
  .bank_mc.pid[PID_PITCH].FF  = SETTING_MC_CD_PITCH_DEFAULT,

  .bank_mc.pid[PID_YAW].P     = SETTING_MC_P_YAW_DEFAULT, 
  .bank_mc.pid[PID_YAW].I     = SETTING_MC_I_YAW_DEFAULT,
  .bank_mc.pid[PID_YAW].D     = SETTING_MC_D_YAW_DEFAULT,
  .bank_mc.pid[PID_YAW].FF    = SETTING_MC_CD_YAW_DEFAULT,

  .bank_mc.pid[PID_LEVEL].P   = SETTING_MC_P_LEVEL_DEFAULT,   // Self-level strength
  .bank_mc.pid[PID_LEVEL].I   = SETTING_MC_I_LEVEL_DEFAULT,   // Self-leveing low-pass frequency (0 - disabled)
  .bank_mc.pid[PID_LEVEL].D   = SETTING_MC_D_LEVEL_DEFAULT,   // 75% horizon strength
  .bank_mc.pid[PID_LEVEL].FF  = SETTING_MC_FF_LEVEL_DEFAULT,

  .bank_mc.pid[PID_HEADING].P   = SETTING_NAV_MC_HEADING_P_DEFAULT, 
  .bank_mc.pid[PID_HEADING].I   = SETTING_NAV_MC_HEADING_I_DEFAULT,
  .bank_mc.pid[PID_HEADING].D   = SETTING_NAV_MC_HEADING_D_DEFAULT,
  .bank_mc.pid[PID_HEADING].FF  = SETTING_NAV_MC_HEADING_FF_DEFAULT, 

  .bank_mc.pid[PID_POS_XY].P  = SETTING_NAV_MC_POS_XY_P_DEFAULT,  // NAV_POS_XY_P * 100
  .bank_mc.pid[PID_POS_XY].I  = SETTING_NAV_MC_POS_XY_I_DEFAULT,
  .bank_mc.pid[PID_POS_XY].D  = SETTING_NAV_MC_POS_XY_D_DEFAULT,
  .bank_mc.pid[PID_POS_XY].FF = SETTING_NAV_MC_POS_XY_FF_DEFAULT,

  .bank_mc.pid[PID_VEL_XY].P  = SETTING_NAV_MC_VEL_XY_P_DEFAULT,  // NAV_VEL_XY_P * 20
  .bank_mc.pid[PID_VEL_XY].I  = SETTING_NAV_MC_VEL_XY_I_DEFAULT,  // NAV_VEL_XY_I * 100
  .bank_mc.pid[PID_VEL_XY].D  = SETTING_NAV_MC_VEL_XY_D_DEFAULT,  // NAV_VEL_XY_D * 100
  .bank_mc.pid[PID_VEL_XY].FF = SETTING_NAV_MC_VEL_XY_FF_DEFAULT, // NAV_VEL_XY_D * 100

  .bank_mc.pid[PID_POS_Z].P   = SETTING_NAV_MC_POS_Z_P_DEFAULT,   // NAV_POS_Z_P * 100
  .bank_mc.pid[PID_POS_Z].I   = SETTING_NAV_MC_POS_Z_I_DEFAULT,   // not used
  .bank_mc.pid[PID_POS_Z].D   = SETTING_NAV_MC_POS_Z_D_DEFAULT,   // not used
  .bank_mc.pid[PID_POS_Z].FF  = SETTING_NAV_MC_POS_Z_FF_DEFAULT,

  .bank_mc.pid[PID_VEL_Z].P   = SETTING_NAV_MC_VEL_Z_P_DEFAULT,   // NAV_VEL_Z_P * 66.7
  .bank_mc.pid[PID_VEL_Z].I   = SETTING_NAV_MC_VEL_Z_I_DEFAULT,   // NAV_VEL_Z_I * 20
  .bank_mc.pid[PID_VEL_Z].D   = SETTING_NAV_MC_VEL_Z_D_DEFAULT,   // NAV_VEL_Z_D * 100
  .bank_mc.pid[PID_VEL_Z].FF  = SETTING_NAV_MC_VEL_Z_FF_DEFAULT,

  .bank_mc.pid[PID_POS_HEADING].P   = SETTING_NAV_MC_POS_HEADING_P_DEFAULT,
  .bank_mc.pid[PID_POS_HEADING].I   = SETTING_NAV_MC_POS_HEADING_I_DEFAULT,
  .bank_mc.pid[PID_POS_HEADING].D   = SETTING_NAV_MC_POS_HEADING_D_DEFAULT,
  .bank_mc.pid[PID_POS_HEADING].FF  = SETTING_NAV_MC_POS_HEADING_FF_DEFAULT,

  .bank_fw.pid[PID_ROLL].P  = SETTING_FW_P_ROLL_DEFAULT,    // стабилизация P-составляющей фиксированного крыла для ROLL
  .bank_fw.pid[PID_ROLL].I  = SETTING_FW_I_ROLL_DEFAULT,    // стабилизация I-составляющей фиксированного крыла для ROLL
  .bank_fw.pid[PID_ROLL].D  = SETTING_FW_D_ROLL_DEFAULT,    // стабилизация D-составляющей фиксированного крыла для ROLL
  .bank_fw.pid[PID_ROLL].FF = SETTING_FW_FF_ROLL_DEFAULT,   // стабилизация FF-составляющей фиксированного крыла для ROLL

  .bank_fw.pid[PID_PITCH].P  = SETTING_FW_P_PITCH_DEFAULT,    // стабилизация P-составляющей фиксированного крыла для PITCH
  .bank_fw.pid[PID_PITCH].I  = SETTING_FW_I_PITCH_DEFAULT,    // стабилизация I-составляющей фиксированного крыла для PITCH
  .bank_fw.pid[PID_PITCH].D  = SETTING_FW_D_PITCH_DEFAULT,    // стабилизация D-составляющей фиксированного крыла для PITCH
  .bank_fw.pid[PID_PITCH].FF = SETTING_FW_FF_PITCH_DEFAULT,   // стабилизация FF-составляющей фиксированного крыла для PITCH

  .bank_fw.pid[PID_YAW].P  = SETTING_FW_P_YAW_DEFAULT,    // стабилизация P-составляющей фиксированного крыла для YAW
  .bank_fw.pid[PID_YAW].I  = SETTING_FW_I_YAW_DEFAULT,    // стабилизация I-составляющей фиксированного крыла для YAW
  .bank_fw.pid[PID_YAW].D  = SETTING_FW_D_YAW_DEFAULT,    // стабилизация D-составляющей фиксированного крыла для YAW
  .bank_fw.pid[PID_YAW].FF = SETTING_FW_FF_YAW_DEFAULT,   // стабилизация FF-составляющей фиксированного крыла для YAW

  .bank_fw.pid[PID_LEVEL].P   = SETTING_FW_P_LEVEL_DEFAULT, // стабилизация P-составляющей фиксированного крыла по ориентации, самоуравновешивающаяся сила,
  .bank_fw.pid[PID_LEVEL].I   = SETTING_FW_I_LEVEL_DEFAULT, // Отсечка фильтра нижних частот для стабилизации ориентации неподвижного крыла, Самовыравнивающаяся частота низких частот (0 - отключено)
  .bank_fw.pid[PID_LEVEL].D   = SETTING_FW_D_LEVEL_DEFAULT, // Стабилизация ориентации самолета Точка перехода HORIZON, 75% прочность горизонта
  .bank_fw.pid[PID_LEVEL].FF  = SETTING_FW_FF_LEVEL_DEFAULT,

  .bank_fw.pid[PID_HEADING].P   = SETTING_NAV_FW_HEADING_P_DEFAULT,   // Усиление P контроллера удержания курса (фиксированное крыло)
  .bank_fw.pid[PID_HEADING].I   = SETTING_NAV_FW_HEADING_I_DEFAULT,
  .bank_fw.pid[PID_HEADING].D   = SETTING_NAV_FW_HEADING_D_DEFAULT,
  .bank_fw.pid[PID_HEADING].FF  = SETTING_NAV_FW_HEADING_FF_DEFAULT,

  .bank_fw.pid[PID_POS_Z].P   = SETTING_NAV_FW_POS_Z_P_DEFAULT,   // P усиление высоты ПИД-регулятором (фиксированное крыло), FW_POS_Z_P * 10
  .bank_fw.pid[PID_POS_Z].I   = SETTING_NAV_FW_POS_Z_I_DEFAULT,   // I усиление высоты ПИД-регулятором (фиксированное крыло), FW_POS_Z_I * 10  
  .bank_fw.pid[PID_POS_Z].D   = SETTING_NAV_FW_POS_Z_D_DEFAULT,   // D усиление высоты ПИД-регулятором (фиксированное крыло), FW_POS_Z_D * 10
  .bank_fw.pid[PID_POS_Z].FF  = SETTING_NAV_FW_POS_Z_FF_DEFAULT,

  .bank_fw.pid[PID_POS_XY].P  = SETTING_NAV_FW_POS_XY_P_DEFAULT,  // P-усиление ПИД-регулятора 2D траектории. Поиграйте с этим, чтобы получить прямую линию между путевыми точками или прямой возврат домой,  FW_POS_XY_P * 100
  .bank_fw.pid[PID_POS_XY].I  = SETTING_NAV_FW_POS_XY_I_DEFAULT,  // I-усиление ПИД-регулятора 2D траектории. Слишком большое значение приведет к отклонению траектории. Лучше начинать настройку с нуля,     FW_POS_XY_I * 100
  .bank_fw.pid[PID_POS_XY].D  = SETTING_NAV_FW_POS_XY_D_DEFAULT,  // D-усиление ПИД-регулятора 2D траектории. Слишком большое значение приведет к отклонению траектории. Лучше начинать настройку с нуля,     FW_POS_XY_D * 100
  .bank_fw.pid[PID_POS_XY].FF = SETTING_NAV_FW_POS_XY_FF_DEFAULT,

  .bank_fw.pid[PID_POS_HEADING].P   = SETTING_NAV_FW_POS_HDG_P_DEFAULT,   // P усиление ПИД-регулятора курса. (Фиксированное крыло, вездеходы, лодки)
  .bank_fw.pid[PID_POS_HEADING].I   = SETTING_NAV_FW_POS_HDG_I_DEFAULT,   // I усиление ПИД-регулятора траектории движения. (Фиксированное крыло, вездеходы, лодки)
  .bank_fw.pid[PID_POS_HEADING].D   = SETTING_NAV_FW_POS_HDG_D_DEFAULT,   // D усиление ПИД-регулятора траектории движения. (Фиксированное крыло, вездеходы, лодки)
  .bank_fw.pid[PID_POS_HEADING].FF  = SETTING_NAV_FW_POS_HDG_FF_DEFAULT,

  // Определяет тип D-терминального фильтра LPF ступени 1. Возможные значения: PT1, BIQUAD. «PT1» обеспечивает более быстрый отклик фильтра, в то время как «BIQUAD» лучшее затухание.
  .dterm_lpf_type = FILTER_BIQUAD,  

  // Частота среза фильтра нижних частот Dterm. Настройка по умолчанию очень консервативна, и для небольших мультикоптеров следует использовать более высокое значение в диапазоне от 80 до 100 Гц. 
  // 80 кажется золотым пятном для 7-дюймовых моделей, в то время как 100 лучше всего подходят для 5-дюймовых машин. Если моторы становятся слишком горячими, уменьшите значение 
  // min: 0, max: 500
  .dterm_lpf_hz = 40,      

  // Определяет тип D-терминального фильтра LPF ступени 1. Возможные значения: PT1, BIQUAD. «PT1» обеспечивает более быстрый отклик фильтра, в то время как «BIQUAD» лучшее затухание.       
  .dterm_lpf2_type = FILTER_BIQUAD,

  // Частота среза для ступени 2 D-членного фильтра нижних частот
  // min: 0, max: 500
  .dterm_lpf2_hz = 0,

  // Частота среза фильтра нижних частот рыскания. Должен быть отключен (установлен на «0») на небольших мультикоптерах (7 дюймов и ниже).
  // min: 0, max: 200
  .yaw_lpf_hz = 0,

  // Используется для предотвращения накопления Iterm во время маневров. Iterm будет ослаблен, когда двигатели достигнут своего предела 
  // (когда запрошенный диапазон коррекции двигателя выше процента, указанного этим параметром)
  // min: 0, max: 90
  .iterm_windup_point_percent = 50,

  // Ограничивает ускорение скорости вращения рысканья, которое может быть запрошено с помощью джойстика. В градусах на секунду в квадрате. 
  // Маленький и мощный БПЛА отлично летает с высоким пределом ускорения (> 10000 dps ^ 2). Большие и тяжелые мультикоптеры выиграют от низкого предела ускорения (~ 180 dps ^ 2). 
  // При правильной настройке он значительно улучшает характеристики торможения и общую устойчивость при поворотах по рысканью. Значение 0 отключает ограничение.
  // Макс. значение: 500000
  .axis_acceleration_limit_yaw = 10000,

  // Ограничивает ускорение скорости вращения ROLL / PITCH, которое может быть запрошено с помощью ручки управления. В градусах на секунду в квадрате. 
  // Маленький и мощный БПЛА отлично летает с высоким пределом ускорения (> 5000 dps ^ 2 и даже> 10000 dps ^ 2). Большие и тяжелые мультикоптеры выиграют от низкого предела ускорения (~ 360 dps ^ 2). 
  // При правильной настройке он значительно улучшает эффективность остановки. Значение 0 отключает ограничение.
  // Макс. значение: 500000
  .axis_acceleration_limit_roll_pitch = 0,

  // Этот параметр ограничивает скорость вращения по рысканью, которую контроллер HEADING_HOLD может запрашивать у контроллера внутреннего контура ПИД. 
  // Он не зависит от скорости рыскания вручную и используется только тогда, когда режим полета HEADING_HOLD включен пилотом, режимами RTH или WAYPOINT.
  .heading_hold_rate_limit = HEADING_HOLD_RATE_LIMIT_DEFAULT,

  // Максимальный наклон в горизонтальном (угловом) режиме (ось ROLL). 100 = 10 °
  // min: 100, max: 900
  .max_angle_inclination[FD_ROLL]   = 300,

  // Максимальный наклон в горизонтальном (угловом) режиме (ось PITCH). 100 = 10 °
  // min: 100, max: 900
  .max_angle_inclination[FD_PITCH]  = 300,

  // Ограничение на общую величину коррекции Flight PID может запрашиваться для каждой оси (Roll / Pitch). 
  // Если при выполнении резкого маневра на одной оси машина теряет ориентацию на другой оси - уменьшение этого параметра может помочь.
  .pid_sum_limit = PID_SUM_LIMIT_DEFAULT,

  // Ограничение на общую величину коррекции Flight PID может запрашивать по каждой оси (Yaw). 
  // Если при выполнении резкого маневра на одной оси станок теряет ориентацию на другой оси - уменьшение этого параметра может помочь.
  .pid_sum_limit_yaw = 350,

  // Ограничивает максимальное/минимальное значение I-члена в ПИД-регуляторе стабилизации в случае фиксированного крыла. 
  // Это решает проблему насыщения сервоприводов перед взлетом/подбрасыванием самолета в воздух. 
  // По умолчанию ошибка, накопленная в I-term, не может превышать 1/3 хода сервопривода (около 165 мкс). Установите 0, чтобы полностью отключить.
  .fixed_wing_iterm_throw_limit = FW_ITERM_THROW_LIMIT_DEFAULT,

  // Контрольная воздушная скорость. Установите его на воздушную скорость, на которой были настроены ПИД-регуляторы. 
  // Обычно следует устанавливать крейсерскую скорость. Также используется для расчета согласованного поворота при отсутствии датчика воздушной скорости.
  // min: 300, max: 6000
  .fixed_wing_reference_airspeed = 1500,

  // Усиление, необходимое для поддержания скорости рыскания в соответствии со скоростью поворота для скоординированного поворота (в режиме TURN_ASSIST). 
  // Значение, значительно отличающееся от 1.0, указывает на проблему с калибровкой воздушной скорости (если присутствует) или значением параметра `fw_reference_airspeed`.
  // min: 0, max: 2
  .fixed_wing_coordinated_yaw_gain = 1,

  // Усиление, необходимое для сохранения постоянного угла тангажа во время скоординированных поворотов (в режиме TURN_ASSIST). 
  // Значение, значительно отличающееся от 1.0, указывает на проблему с калибровкой воздушной скорости (если присутствует) или значением параметра `fw_reference_airspeed`.
  // min: 0, max: 2
  .fixed_wing_coordinated_pitch_gain = 1,

  // Iterm не может расти, когда положение ручки превышает пороговое значение. Это решает проблему отскока или продолжения, когда полное отклонение рукояти применяется 
  // к плохо настроенным неподвижным крыльям. Другими словами, стабилизация частично отключается, когда пилот активно управляет самолетом, и активна, когда ручки не 
  // касаются. «0» означает, что ручка находится в центральном положении, «1» означает, что она полностью отклонена в любую сторону.
  // min: 0, max: 1
  .fixed_wing_iterm_limit_on_stick_position = 0.5,

  // Yaw Iterm фиксируется, когда угол крена превышает этот порог [градусы]. Это решает проблему противодействия поворотам руля за счет частичного отключения стабилизации рыскания при выполнении разворотов по крену. 
  // Значение 0 (по умолчанию) отключает эту функцию. Применяется только тогда, когда автопилот не активен и система помощи при повороте отключена.
  // min: 0, max: 90
  .fixed_wing_yaw_iterm_bank_freeze = 0,

  // Направление зависания: центральная точка на правом крыле (по часовой стрелке - по умолчанию) или центральная точка на левом крыле (против часовой стрелки). 
  // При равном рысканье можно изменить в полете с помощью рычага рыскания.
  .loiter_direction = NAV_LOITER_RIGHT,

  // min: 0, max: 100
  .nav_vel_xy_dterm_lpf_hz = 2,

  // Максимальный процент ослабления D-члена для ПИД-регулятора горизонтальной скорости (Multirotor). Это позволяет сгладить PosHold CRUISE, WP и RTH, 
  // когда Multirotor движется на полной скорости. Dterm не затухает на малых скоростях, торможении и разгоне.
  // min: 0, max: 100
  .nav_vel_xy_dterm_attenuation = 90,

  // Точка (в процентах от целевой и текущей горизонтальной скорости), где начинается nav_mc_vel_xy_dterm_attenuation
  // min: 0, max: 100
  .nav_vel_xy_dterm_attenuation_start = 10,

  // Точка (в процентах от целевой и текущей горизонтальной скорости), в которой nav_mc_vel_xy_dterm_attenuation достигает максимума
  // min: 0, max: 100
  .nav_vel_xy_dterm_attenuation_end = 60,

  // min: 0, max: 100
  .iterm_relax_cutoff = 15,
  
  .iterm_relax = ITERM_RELAX_RP,

// ifdef USE_D_BOOST -> if FLASH_MCU > 256
  // min: 1, max: 3
  .d_boost_factor = 1.25,

  // min: 1000, max: 16000
  .d_boost_max_at_alleceleration = 7500,

  // min: 10, max: 250
  .d_boost_gyro_delta_lpf_hz = 80,
// endif

// ifdef USE_ANTIGRAVITY -> if FLASH_MCU > 256
  // Максимальное усиление антигравитации. «1» означает, что антигравитация отключена, «2» означает, 
  // что Iterm может удваиваться при быстром движении дроссельной заслонки.
  // min: 1, max: 20
  .antigravity_gain = 1,

  // min: 1, max: 20
  .antigravity_accelerator = 1,

  // Частота отсечки антигравитационного фильтра дроссельной заслонки. Антигравитация основана на разнице между фактическим и отфильтрованным вводом газа. 
  // Чем больше разница, тем больше прирост Антигравитации.
  // min: 1, max: 30
  .antigravity_cutoff = 15,
// endif

  // Позволяет установить тип ПИД-регулятора, используемого в контуре управления. Возможные значения: NONE, PID, PIFF, AUTO. 
  // Изменяйте только в случае экспериментальных платформ, таких как вертикальный взлет, посадка, марсоходы, лодки и т. Д. 
  // В самолетах всегда следует использовать PIFF и мультикоптеры PID.
  .pid_controller_type = PID_TYPE_AUTO,

  // Выходной предел для ПИД-регулятора траектории движения. (Фиксированное крыло, вездеходы, лодки)
  .nav_fw_pos_hdg_pid_sum_limit = 350,

  // Частота среза для управляющей производной. Более низкое значение более плавная реакция на быстрые движения ручки. 
  // При более высоких значениях реакция будет более агрессивной, отрывистой.
  // min: 0, max: 200
  .control_derivative_lpf_hz = 30,

// ifdef USE_GYRO_KALMAN -> if FLASH_MCU > 256  
  // Добротность уставки фильтра Калмана. Более высокие значения означают меньшую фильтрацию и меньшую фазовую задержку. 
  // На 3-7-дюймовых мультикоптерах обычно можно увеличить до 200-300 или даже выше чистых сборок.
  // min: 1, max: 16000
  .kalman_q = 100,

  // Размер окна для заданного значения фильтра Калмана. Чем шире окно, тем больше выборок используется для вычисления дисперсии. 
  // Как правило, более широкое окно приводит к более плавному отклику фильтра.
  // min: 1, max: 40
  .kalman_w = 4,

  // Коэффициент динамичности для уставки фильтра Калмана. Как правило, чем выше значение, тем более динамичным становится фильтр Калмана.
  // min: 1, max: 16000
  .kalman_sharpness = 100,

  // Включите фильтр Калмана на заданном значении ПИД-регулятора
  .kalman_enabled = OFF,
// endif

  // Триммер тангажа для режимов полета с самовыравниванием. В градусах. +5 означает, что нос самолета должен быть поднят на 5 градусов от уровня
  // min: -10, max: 10
  .fixed_wing_level_trim = 0.0,

  // I-усиление для подстройки угла тангажа в режимах полета с самовыравниванием. Более высокие значения означают, что AUTOTRIM будет работать быстрее, но могут возникнуть колебания.
  // min: 0, max: 20
  .fixed_wing_level_trim_gain = 5.0,

  // Зона нечувствительности для автоматического выравнивания при использовании режима AUTOLEVEL.
  // min: 0, max: 20
  .fixed_wing_level_trim_deadband = 5.0,

// ifdef USE_SMITH_PREDICTOR -> if FLASH_MCU > 256  
  // Фактор силы предсказателя Смита для измерения PID. В процентах
  // min: 0, max: 1
  .smith_predictor_strength = 0.5,

  // Ожидаемая задержка сигнала гироскопа. В миллисекундах
  // min: 0, max: 8
  .smith_predictor_delay = 0,

  // Частота среза фильтра нижних частот Смит-предсказатель
  // min: 1, max: 500
  .smith_predictor_filter_hz = 50
// endif
};  

pid_profile_ts *p_pid_profile = &pid_profile;

/**
  * @brief  Получение состояния удержания курса
  * @param  Нет
  * @retval HEADING_HOLD_STATE_te статус удержания курса
  */
static HEADING_HOLD_STATE_te get_heading_hold_state (void)
{
//     if (calculateCosTiltAngle() < headingHoldCosZLimit) {
//         return HEADING_HOLD_DISABLED;
//     }
// #if defined(USE_NAV) - в common.h
//     int nav_heading_state = navigationGetHeadingControlState();
//     // NAV will prevent MAG_MODE from activating, but require heading control
//     if (nav_heading_state != NAV_HEADING_CONTROL_NONE) {
//         // Apply maghold only if heading control is in auto mode
//         if (nav_heading_state == NAV_HEADING_CONTROL_AUTO) {
//             return HEADING_HOLD_ENABLED;
//         }
//     } else
// //#endif

//     if (ABS(rc_command[YAW]) == 0 && FLIGHT_MODE(HEADING_MODE)) {
//         return HEADING_HOLD_ENABLED;
//     } else {
//         return HEADING_HOLD_UPDATE_HEADING;
//     }

    return HEADING_HOLD_UPDATE_HEADING;
}

void pid_init (void)
{
    // Рассчитываем максимальный общий наклон (максимальный шаг + максимальный крен вместе) как предел удержания курса
    heading_hold_cosz_limit = get_cos_approx(DECIDEGREES_TO_RADIANS(p_pid_profile->max_angle_inclination[FD_ROLL]) * 
                              get_cos_approx(DECIDEGREES_TO_RADIANS(p_pid_profile->max_angle_inclination[FD_PITCH])));

    pid_gains_update_required = false;

    iterm_relax = p_pid_profile->iterm_relax;

    yaw_lpf_hz = p_pid_profile->iterm_relax;

    motor_iterm_windup_point = 1.0F - (p_pid_profile->iterm_windup_point_percent / 100.0F);

    //#ifdef USE_D_BOOST
    d_boost_factor                = p_pid_profile->d_boost_factor;
    d_boost_max_at_alleceleration = p_pid_profile->d_boost_max_at_alleceleration;

    // #ifdef USE_ANTIGRAVITY
    antigravity_gain = p_pid_profile->antigravity_gain;
    antigravity_accelerator = p_pid_profile->antigravity_accelerator;
    
    pid_state[FD_YAW].pid_sum_limit = p_pid_profile->pid_sum_limit_yaw;
    pid_state[FD_YAW].p_term_filter_apply_fn = (yaw_lpf_hz) ? (filter_apply_4_fn_ptr)pt1_filter_apply4 : (filter_apply_4_fn_ptr)null_filter_apply4;
    for (uint8_t axis = FD_ROLL; axis <= FD_PITCH; axis++) {
      pid_state[axis].pid_sum_limit = p_pid_profile->pid_sum_limit;
      pid_state[axis].p_term_filter_apply_fn = (filter_apply_4_fn_ptr)null_filter_apply4;
    }

    // в зависимости от плафтормы меняется и тип ПИДа
    if (p_pid_profile->pid_controller_type == PID_TYPE_AUTO) {
      if (mixer_config.platform_type == PLATFORM_AIRPLANE || mixer_config.platform_type == PLATFORM_BOAT || mixer_config.platform_type == PLATFORM_ROVER) {
        used_pid_controller_type = PID_TYPE_PIFF;
        pid_controller_apply_fn = pid_apply_fixed_wing_rate_controller;
      } else {
        used_pid_controller_type = PID_TYPE_PID;
        pid_controller_apply_fn = pid_apply_multicopter_rate_controller;
      }

    } else {
      used_pid_controller_type = p_pid_profile->pid_controller_type;
      pid_controller_apply_fn = null_rate_controller;
    }

    dterm_lpf_filter_apply_fn   = (p_pid_profile->dterm_lpf_hz)   ? (filter_apply_fn_ptr)pt1_filter_apply : (filter_apply_fn_ptr)biquad_filter_apply;
    dterm_lpf2_filter_apply_fn  = (p_pid_profile->dterm_lpf2_hz)  ? (filter_apply_fn_ptr)pt1_filter_apply : (filter_apply_fn_ptr)biquad_filter_apply;

    pid_reset_TPA_filter();

    fixed_wing_level_trim = p_pid_profile->fixed_wing_level_trim;

    nav_pid_init(&fixed_wing_level_trim_controller, 0.0F, (float)p_pid_profile->fixed_wing_level_trim_gain / 100000.0F, 0.0F, 0.0F, 2.0F);
}

void pid_reset_TPA_filter (void)
{
    if (used_pid_controller_type == PID_TYPE_PIFF && current_control_rate_profile.throttle.fixed_wing_tau_ms > 0) {
        pt1_filter_init_RC(&fixed_wing_tpa_filter, current_control_rate_profile.throttle.fixed_wing_tau_ms * 1e-3f, TASK_PERIOD_HZ(TASK_AUX_RATE_HZ) * 1e-6f);
        pt1_filter_reset(&fixed_wing_tpa_filter, get_throttle_idle_value());
    }
}

static void null_rate_controller (pid_state_ts *pid_state, flight_dynamics_index_te axis, float dT) 
{
    UNUSED(pid_state);
    UNUSED(axis);
    UNUSED(dT);
}

static float p_term_process (pid_state_ts *pid_state_local, float rate_error, float dT) 
{
    float new_p_term = rate_error * pid_state_local->kP;

    return pid_state_local->p_term_filter_apply_fn(&pid_state_local->p_term_lpf_state, new_p_term, yaw_lpf_hz, dT);
}

// #ifdef USE_D_BOOST - в common.h
static float apply_d_boost (pid_state_ts *pid_state_local, float dT) 
{
    float d_boost = 1.0F;

    if (d_boost_factor > 1) {
        const float d_boost_gyro_delta = (pid_state_local->gyro_rate - pid_state_local->previous_rate_gyro) / dT;
        const float d_boost_gyro_acceleration = fabsf(biquad_filter_apply(&pid_state_local->d_boost_gyro_lpf, d_boost_gyro_delta));
        const float d_boost_rate_acceleration = fabsf((pid_state_local->rate_target - pid_state_local->previous_rate_target) / dT);

        const float acceleration = MAX(d_boost_gyro_acceleration, d_boost_rate_acceleration);
        d_boost = scaleRangef(acceleration, 0.0f, d_boost_max_at_alleceleration, 1.0F, d_boost_factor);
        d_boost = pt1_filter_apply4(&pid_state_local->d_boost_lpf, d_boost, D_BOOST_LPF_HZ, dT);
        d_boost = constrainf(d_boost, 1.0F, d_boost_factor);
    }

    return d_boost;
}

static float d_term_process (pid_state_ts *pid_state_local, float dT) 
{
    // Calculate new D-term
    float new_d_term = 0;
    if (pid_state_local->kD == 0) {
        // optimisation for when D is zero, often used by YAW axis
        new_d_term = 0;
    } else {
        float delta = pid_state_local->previous_rate_gyro - pid_state_local->gyro_rate;

        delta = dterm_lpf_filter_apply_fn((filter_tu *) &pid_state_local->d_term_lpf_state, delta);
        delta = dterm_lpf2_filter_apply_fn((filter_tu *) &pid_state_local->d_term_lpf2_state, delta);

        // Calculate derivative
        new_d_term =  delta * (pid_state_local->kD / dT) * apply_d_boost(pid_state_local, dT);
    }
    return (new_d_term);
}

static float apply_iterm_relax (const int axis, float current_pid_setpoint, float iterm_error_rate)
{
  if (iterm_relax && (axis < FD_YAW || iterm_relax == ITERM_RELAX_RPY)) {
    const float setpoint_lpf = pt1_filter_apply(&windup_lpf[axis], current_pid_setpoint);
    const float setpoint_hpf = fabsf(current_pid_setpoint - setpoint_lpf);
    const float iterm_relax_factor = MAX(0, 1 - setpoint_hpf / MC_ITERM_RELAX_SETPOINT_THRESHOLD);

    return iterm_error_rate * iterm_relax_factor;
  }

  return iterm_error_rate;
}

static void pid_apply_multicopter_rate_controller (pid_state_ts *pid_state_local, flight_dynamics_index_te axis, float dT)
{
    const float rate_error = pid_state_local->rate_target - pid_state_local->gyro_rate;
    const float new_p_term = p_term_process(pid_state_local, rate_error, dT);
    const float new_d_term = d_term_process(pid_state_local, dT);

    const float rate_target_delta = pid_state_local->rate_target - pid_state_local->previous_rate_target;
    const float rate_target_delta_filtered = pt2_filter_apply(&pid_state_local->rate_target_filter, rate_target_delta);

    /*
     * Compute Control Derivative
     */
    const float new_c_d_term = rate_target_delta_filtered * (pid_state_local->kCD / dT);

    // TODO: Get feedback from mixer on available correction range for each axis
    const float new_output = new_p_term + new_d_term + pid_state_local->error_gyro_if + new_c_d_term;
    const float new_output_limited = constrainf(new_output, -pid_state_local->pid_sum_limit, +pid_state_local->pid_sum_limit);

    float iterm_error_rate = apply_iterm_relax(axis, pid_state_local->rate_target, rate_error);

// #ifdef USE_ANTIGRAVITY - определенно в common.h
    iterm_error_rate *= iTermAntigravityGain;
// #endif

    pid_state_local->error_gyro_if += (iterm_error_rate * pid_state_local->kI * anti_windup_scaler * dT)
                             + ((new_output_limited - new_output) * pid_state_local->kT * anti_windup_scaler * dT);

    // Don't grow I-term if motors are at their limit
    apply_iterm_limiting(pid_state_local);

    axis_pid[axis] = new_output_limited;

// #ifdef USE_BLACKBOX - определенно в common.h
    axis_pid_p[axis] = new_p_term;
    axis_pid_i[axis] = pid_state_local->error_gyro_if;
    axis_pid_d[axis] = new_d_term;
    axis_pid_setpoint[axis] = pid_state_local->rate_target;
// #endif

    pid_state_local->previous_rate_target = pid_state_local->rate_target;
    pid_state_local->previous_rate_gyro = pid_state_local->gyro_rate;
}

static void apply_iterm_limiting (pid_state_ts *pid_state_local) 
{
  if (pid_state_local->iterm_limit_active) {
      pid_state_local->error_gyro_if = constrainf(pid_state_local->error_gyro_if, -pid_state_local->error_gyro_if_limit, pid_state_local->error_gyro_if_limit);
  } else {
      pid_state_local->error_gyro_if_limit = fabsf(pid_state_local->error_gyro_if);
  }
}

static void pid_apply_fixed_wing_rate_controller (pid_state_ts *pid_state_local, flight_dynamics_index_te axis, float dT)
{
  const float rate_error = pid_state_local->rate_target - pid_state_local->gyro_rate;
  const float new_p_term = p_term_process(pid_state_local, rate_error, dT);
  const float new_d_term = d_term_process(pid_state_local, dT);
  const float new_f_f_term = pid_state_local->rate_target * pid_state_local->kFF;

  /*
    * Интеграл следует обновлять только в том случае, если ось Iterm не заморожена.
    */
  if (!pid_state_local->iterm_freeze_active) {
      pid_state_local->error_gyro_if += rate_error * pid_state_local->kI * dT;
  }

  apply_iterm_limiting(pid_state_local);

  if (p_pid_profile->fixed_wing_iterm_throw_limit) {
      pid_state_local->error_gyro_if = constrainf(pid_state_local->error_gyro_if, -p_pid_profile->fixed_wing_iterm_throw_limit, p_pid_profile->fixed_wing_iterm_throw_limit);
  }

  axis_pid[axis] = constrainf(new_p_term + new_f_f_term + pid_state_local->error_gyro_if + new_d_term, -pid_state_local->pid_sum_limit, +pid_state_local->pid_sum_limit);

// #ifdef USE_AUTOTUNE_FIXED_WING - определенно в common.h
  if (FLIGHT_MODE(AUTO_TUNE) && !FLIGHT_MODE(MANUAL_MODE)) {
      autotune_fixed_wing_update(axis, pid_state_local->rate_target, pid_state_local->gyro_rate, constrainf(new_p_term + new_f_f_term, -pid_state_local->pid_sum_limit, +pid_state_local->pid_sum_limit));
  }
// #endif

// #ifdef USE_BLACKBOX - определенно в common.h
  axis_pid_p[axis] = new_p_term;
  axis_pid_i[axis] = pid_state_local->error_gyro_if;
  axis_pid_d[axis] = new_d_term;
  axis_pid_setpoint[axis] = pid_state_local->rate_target;
// #endif

  pid_state_local->previous_rate_gyro = pid_state_local->gyro_rate;
}

/**
  * @brief  Реализация ПИД контроллера
  * @param  dT Перид дескретизации. Служит для выполнения рассчета и выдачи управляющего сигнала с равной частотой
  * @retval Нет
  */
void pid_controller (float dT)
{
  if (!pid_filters_is_configured)
      return;

  bool can_use_fpv_camera_mix = true;
  uint8_t heading_hold_state = get_heading_hold_state();  // получаем статус удержания курса

  if (heading_hold_state == HEADING_HOLD_UPDATE_HEADING)
    update_heading_hold_target(0); // обновляем значение по рысканию, 0 - заглушка

  for (uint8_t axis = 0; axis < 3; axis++) {
    // Шаг 1. Рассчитываем гироскопические коэффициенты.
    pid_state[axis].gyro_rate = 0; // заглушка, значение с гироскопа по 3-м осям

    // Шаг 2: чтение цели
    float rate_target = (axis = FD_YAW && heading_hold_state == HEADING_HOLD_ENABLED) ? 
                            pid_heading_hold(dT) : pid_rc_command_to_rate(rc_command[axis], current_control_rate_profile.stabilized.rates[axis]);

    // Ограничиваем желаемую скорость тем, что гироскоп может надежно измерить
    pid_state[axis].rate_target = constrainf(rate_target, -GYRO_SATURATION_LIMIT, +GYRO_SATURATION_LIMIT);

// #ifdef USE_GYRO_KALMAN   // определен в common.h
//         gyroKalmanUpdateSetpoint(axis, pidState[axis].rateTarget);
// #endif

//#ifdef USE_SMITH_PREDICTOR
      //DEBUG_SET(DEBUG_SMITH_PREDICTOR, axis, pidState[axis].gyroRate);
      pid_state[axis].gyro_rate = apply_smith_predictor(axis, &pid_state[axis].smith_predictor, pid_state[axis].gyro_rate);
      //DEBUG_SET(DEBUG_SMITH_PREDICTOR, axis + 3, pidState[axis].gyroRate);
//#endif
  }

  // Шаг 3: запустить управление для ANGLE_MODE, HORIZON_MODE и HEADING_LOCK
  if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
    const float horizon_rate_magnitude = get_calc_horizon_rate_magnitude();
    pid_level(&pid_state[FD_ROLL], FD_ROLL, horizon_rate_magnitude, dT);
    pid_level(&pid_state[FD_PITCH], FD_PITCH, horizon_rate_magnitude, dT);
    can_use_fpv_camera_mix = false;     // FPVANGLEMIX is incompatible with ANGLE/HORIZON
    leveling_enabled = true;
  } else {
    leveling_enabled = false;
  }

  if ((FLIGHT_MODE(TURN_ASSISTANT) /*|| navigationRequiresTurnAssistance()*/) && (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) /*|| navigationRequiresTurnAssistance()*/)) {
    float bank_angle_target = DECIDEGREES_TO_RADIANS(pid_rc_command_to_angle(rc_command[FD_ROLL], p_pid_profile->max_angle_inclination[FD_ROLL]));
    float pitch_angle_target = DECIDEGREES_TO_RADIANS(pid_rc_command_to_angle(rc_command[FD_PITCH], p_pid_profile->max_angle_inclination[FD_PITCH]));
    pid_turn_assistant(pid_state, bank_angle_target, pitch_angle_target);
    can_use_fpv_camera_mix = false;
  }

  if (can_use_fpv_camera_mix /*&& IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX)*/ && current_control_rate_profile.misc.fpv_cam_angle_degrees)
    pid_apply_fpv_camera_angle_mix(pid_state, current_control_rate_profile.misc.fpv_cam_angle_degrees);

  // Предотвращаем сильное накопление Iterm при вводе стика
  anti_windup_scaler = constrainf((1.0F - get_motor_mix_range()) / motor_iterm_windup_point, 0.0F, 1.0F);

  for (uint8_t axis = 0; axis < 3; axis++) {
    // Применить заданное значение скорости изменения пределов
    pid_apply_setpoint_rate_limiting(&pid_state[axis], axis, dT);

    // Шаг 4: Запуск управления с гироскопом
    check_iterm_limiting_active(&pid_state[axis]);
    chech_itrem_freezing_active(&pid_state[axis], axis);

    pid_controller_apply_fn(&pid_state[axis], axis, dT);
  }
}

static void chech_itrem_freezing_active (pid_state_ts *pid_state_local, flight_dynamics_index_te axis)
{
  if (used_pid_controller_type == PID_TYPE_PIFF && p_pid_profile->fixed_wing_yaw_iterm_bank_freeze != 0 && axis == FD_YAW) {
    // Не позволять рысканью I-term увеличиваться, когда угол крена слишком велик
    float bank_angle = /*DECIDEGREES_TO_DEGREES(attitude.values.roll)*/ 0.0;

    if (fabs(bank_angle) > p_pid_profile->fixed_wing_yaw_iterm_bank_freeze && !(FLIGHT_MODE(AUTO_TUNE) || FLIGHT_MODE(TURN_ASSISTANT) /*|| navigationRequiresTurnAssistance()*/))
      pid_state_local->iterm_freeze_active = true;
    else
      pid_state_local->iterm_freeze_active = false;
  } else {
    pid_state_local->iterm_freeze_active = false;
  }
}

static void check_iterm_limiting_active (pid_state_ts *pid_state_local)
{
  bool should_activate = (used_pid_controller_type == PID_TYPE_PIFF) ? is_fixed_wing_iterm_limit_active(pid_state_local->stick_position) : mixer_is_output_saturated(); 

  pid_state_local->iterm_limit_active = STATE(ANTI_WINDUP) || should_activate;
}

static bool is_fixed_wing_iterm_limit_active (float stick_position)
{
  /* * Iterm anti-windup должен быть активен только тогда, когда пилот управляет вращением * скорости напрямую, а не когда используются УГОЛ или ГОРИЗОНТ */
  if (leveling_enabled)
    return false;

  return fabs(stick_position) > p_pid_profile->fixed_wing_iterm_limit_on_stick_position;
}

/* Примените ограничение углового ускорения к заданной скорости, чтобы ограничить экстремальные нажатия ручки для соблюдения физических возможностей машины */
static void pid_apply_setpoint_rate_limiting (pid_state_ts *pid_state_local, flight_dynamics_index_te axis_local, float dT_local)
{
  const uint32_t axis_accel_limit = (axis_local == FD_YAW) ? p_pid_profile->axis_acceleration_limit_yaw : p_pid_profile->axis_acceleration_limit_roll_pitch;

  if (axis_accel_limit > AXIS_ACCEL_MIN_LIMIT)
    pid_state_local->rate_target = rate_limit_filter_apply4(&pid_state_local->axis_accel_filter, pid_state_local->rate_target, (float)axis_accel_limit, dT_local);
}

static void pid_apply_fpv_camera_angle_mix (pid_state_ts *pid_state_local, uint8_t fpv_camera_angle)
{
  static uint8_t last_fpv_cam_angle_degrees = 0;
  static float cos_camera_angle = 1.0;
  static float sin_camera_angle = 0.0;

  if (last_fpv_cam_angle_degrees != fpv_camera_angle) {
    last_fpv_cam_angle_degrees = fpv_camera_angle;
    cos_camera_angle = get_cos_approx(DEGREES_TO_RADIANS(fpv_camera_angle));
    sin_camera_angle = get_sin_approx(DEGREES_TO_RADIANS(fpv_camera_angle));
  }

  // Повернуть команду поворота / рыскания из системы координат кадра камеры в систему координат кадра тела
  const float roll_rate = pid_state_local[ROLL].rate_target;
  const float yaw_rate = pid_state_local[YAW].rate_target;

  pid_state_local[ROLL].rate_target = constrainf(roll_rate * cos_camera_angle - yaw_rate * sin_camera_angle, -GYRO_SATURATION_LIMIT, GYRO_SATURATION_LIMIT);
  pid_state_local[YAW].rate_target = constrainf(yaw_rate * cos_camera_angle + roll_rate * sin_camera_angle, -GYRO_SATURATION_LIMIT, GYRO_SATURATION_LIMIT);
}

/*
* Режим TURN ASSISTANT - это вспомогательный режим для выполнения поворота по рысканью на плоскости земли, что позволяет поворачивать одним джойстиком больше RATE
* и сохраняя положение ROLL и PITCH на повороте.
*/
static void pid_turn_assistant (pid_state_ts *pid_state_local, float bank_angle_target_local, float pitch_angle_target_local)
{
  fp_vector3_tu target_rates = {
    .x = 0.0F,
    .y = 0.0F
  };

  if (STATE(AIRPLANE)) {
    if (true) {// calculateCosTiltAngle() >= 0.173648f если косинус наклона больше или равен
    // Идеальный поворот с наклоном следуйте уравнениям:
             // forward_vel ^ 2 / radius = Gravity * tan (roll_angle)
             // yaw_rate = forward_vel / radius
             // Если мы решим угол крена, мы получим:
             // tan (roll_angle) = forward_vel * yaw_rate / Gravity
             // Если мы решим скорость рыскания, мы получим:
             // yaw_rate = tan (roll_angle) * Gravity / forward_vel
//#if defined(USE_PITOT) - определен в common.h
          float airspeed_for_coordinated_turn = sensors(1 /* - заглушка для SENSOR_PITOT*/) ? 0 /* - заглушка для этого: pitot.airSpeed*/ : p_pid_profile->fixed_wing_reference_airspeed;
//#endif
    // Ограничивайтесь разумными пределами - 10 км / ч - 216 км / ч
      airspeed_for_coordinated_turn = constrainf(airspeed_for_coordinated_turn, 300, 6000);

      // Расчет скорости поворота в кадре Земли в соответствии со Справочником пилотов по аэронавигационным знаниям FAA
      bank_angle_target_local = constrainf (bank_angle_target_local, -DEGREES_TO_RADIANS(60), DEGREES_TO_RADIANS(60));
      float turn_rate_pitch_adjustment_factor = get_cos_approx(fabs(pitch_angle_target_local));
      float coordinated_turn_rate_earth_frame = GRAVITY_CMSS * tan_approx(-bank_angle_target_local) / airspeed_for_coordinated_turn * turn_rate_pitch_adjustment_factor;

      target_rates.z = RADIANS_TO_DEGREES(coordinated_turn_rate_earth_frame);
    } else {
      // Не разрешать скоординированный расчет разворота, если самолет находится на крутом крене или крутом подъеме / пикировании
      return;
    }
  } else {
    target_rates.z = pid_state_local[YAW].rate_target;
  }

  // Преобразуем рассчитанные смещения скорости в корпус и применяем
  // imuTransformVectorEarthToBody(&targetRates);

  // Добавляем крен и тангаж
  pid_state_local[ROLL].rate_target = constrainf(pid_state_local[ROLL].rate_target + target_rates.x, -current_control_rate_profile.stabilized.rates[ROLL] * 10.0F, current_control_rate_profile.stabilized.rates[ROLL] * 10.0F);
  pid_state_local[PITCH].rate_target = constrainf(pid_state_local[PITCH].rate_target + target_rates.y * p_pid_profile->fixed_wing_coordinated_pitch_gain, -current_control_rate_profile.stabilized.rates[PITCH] * 10.0F, current_control_rate_profile.stabilized.rates[PITCH] * 10.0F);

  // Заменить рыскание на квадриках - добавить на самолетах
  pid_state_local[YAW].rate_target = (STATE(AIRPLANE)) ? 
      constrainf(pid_state_local[YAW].rate_target + target_rates.z * p_pid_profile->fixed_wing_coordinated_yaw_gain, -current_control_rate_profile.stabilized.rates[YAW] * 10.0F, current_control_rate_profile.stabilized.rates[YAW] * 10.0F) :
      constrainf(target_rates.z, -current_control_rate_profile.stabilized.rates[YAW] * 10.0F, current_control_rate_profile.stabilized.rates[YAW] * 10.0F);
}

static void pid_level (pid_state_ts *pid_state, flight_dynamics_index_te axis, float horizon_rate_magnitude, float dT)
{
  // Это ROLL / PITCH, запускаем контроллеры ANGLE / HORIZON
  float angle_target = pid_rc_command_to_angle(rc_command[axis], p_pid_profile->max_angle_inclination[axis]);

  // Автоматически понижать тангаж, если дроссельная заслонка управляется вручную, и уменьшенная сильфонная дроссельная заслонка
  if ( (axis == FD_PITCH) && STATE(AIRPLANE) && FLIGHT_MODE(ANGLE_MODE) /*&& !navigationIsControllingThrottle() - состояние моторов*/)
    angle_target += 0; /*scaleRange(MAX(0, currentBatteryProfile->nav.fw.cruise_throttle - rcCommand[THROTTLE]), 0, currentBatteryProfile->nav.fw.cruise_throttle - PWM_RANGE_MIN, 0, currentBatteryProfile->fwMinThrottleDownPitchAngle);*/

  // Подстройка PITCH, применяемая в режиме полета AutoLevel и ручная подстройка по тангажу
  if (axis == FD_PITCH && STATE(AIRPLANE)) {
    /*
    * fixed_wing_level_trim имеет знак, противоположный rc_command.
    * Положительное значение rcCommand означает, что нос должен указывать вниз
    * Отрицательный rc_command означает, что нос должен указывать вверх
    * Это противоречит интуиции и естественным образом предполагает, что + должно означать ВВЕРХ.
    * Вот почему fixed_wing_level_trim имеет знак, противоположный rc_command.
    * Положительное значение fixed_wing_level_trim означает, что нос должен указывать вверх
    * Отрицательный параметр fixed_wing_level_trim означает, что нос должен указывать вниз
    */
    angle_target -= DEGREES_TO_DECIDEGREES(fixed_wing_level_trim);
  }
// #ifdef USE_SECONDARY_IMU - в common.h
  float actual;
  // вторичный инерциальный модуль
  // в зависимости от оси получает значение по ролу, питчу или по положению в воздухе ???
  // if (secondaryImuState.active && secondaryImuConfig()->useForStabilized) {
  //   actual = (axis == FD_ROLL) ? secondaryImuState.eulerAngles.values.roll : secondaryImuState.eulerAngles.values.pitch;
  // } else {
  //   actual = attitude.raw[axis];
  // }

  const float angle_error_deg = DECIDEGREES_TO_DEGREES(angle_target - actual);

  float angle_rate_target = constrainf(angle_error_deg * (get_pid_bank()->pid[PID_LEVEL].P / FP_PID_LEVEL_P_MULTIPLIER), -current_control_rate_profile.stabilized.rates[axis] * 10.0F, current_control_rate_profile.stabilized.rates[axis] * 10.0f);

  // Применяем простой LPF к angle_rate_target, чтобы сделать ответ менее резким
  // Идеи, лежащие в основе этого:
  // 1) Отношение обновляется со скоростью гироскопа, rateTarget для режима ANGLE рассчитывается из отношения
  // 2) Если этот параметр rateTarget передается непосредственно в ПИД-регулятор на базе гироскопа, это фактически удваивает ошибку rateError.
  // D-член, рассчитанный на основе ошибки, имеет тенденцию еще больше усиливать это. Более того, это, как правило, откликается на каждое
  // малейшее изменение отношения вызывает нервозность при самовыравнивании
  // 3) Понижение УРОВНЯ P может сделать эффекты (2) менее заметными, но это также замедлит самовыравнивание.
  // 4) Реакция пилота-человека на изменение ориентации в режиме RATE довольно медленная и плавная, пилот-человек - нет.
  // компенсируем каждое малейшее изменение
  // 5) (2) и (4) приводят к простой идее добавления фильтра нижних частот в rateTarget для демпфирования режима ANGLE
  // реакция на быструю смену отношения и сглаживание самовыравнивающейся реакции
  if (get_pid_bank()->pid[PID_LEVEL].I)
    // I8 [PIDLEVEL] - частота среза фильтра (Гц). Практические значения частоты фильтрации 5-10 Гц.
    angle_rate_target = pt1_filter_apply4(&pid_state->angle_filter_state, angle_rate_target, get_pid_bank()->pid[PID_LEVEL].I, dT);

  // P [LEVEL] определяет силу самовыравнивания (как для режимов ANGLE, так и HORIZON)
  pid_state->rate_target = (FLIGHT_MODE(HORIZON_MODE)) ? (1.0F - horizon_rate_magnitude) * angle_rate_target + horizon_rate_magnitude * pid_state->rate_target :
                                                          angle_rate_target;
}

/**
  * @brief Получение значения наклона
  * @param Положение стика и максимальный наклон
  * @retval Float - масштабированное значение наклона в пределах значений стика
  */
static float pid_rc_command_to_angle (int16_t stick, int16_t max_inclination)
{
    stick = constrain(stick, -500, 500);
    return scaleRangef((float)stick, -500.0F, 500.0F, (float)-max_inclination, (float)max_inclination);
}

static float get_calc_horizon_rate_magnitude (void)
{
  // Выясняем исходные позиции стиков
  const int32_t stick_pos_ail = ABS(get_rc_stick_deflection(FD_ROLL));
  const int32_t stick_pos_ele = ABS(get_rc_stick_deflection(FD_PITCH));
  const float most_deflected_stick_pos = constrain(MAX(stick_pos_ail, stick_pos_ele), 0, 500) / 500.0F;
  const float mode_transition_stick_pos = constrain(get_pid_bank()->pid[PID_LEVEL].D, 0, 100) / 100.0F;

  // Рассчитываем точку перехода по отклонению стика
  float horizon_rate_magnitude = (most_deflected_stick_pos <= mode_transition_stick_pos) ? most_deflected_stick_pos / mode_transition_stick_pos : 1.0F;
    
  return horizon_rate_magnitude;
}

void update_heading_hold_target (int16_t heading)
{
    heading_hold_target = heading;
}

float nav_pid_apply3 (pid_controller_ts *pid, const float setpoint, const float measurement,  const float dt, const float out_min, const float out_max,
                      const pid_controller_flags_te pid_flags, const float gain_scaler, const float dTerm_scaler) 
{
  float new_proportional, new_derivative, new_feed_forward;
  float error = setpoint - measurement;
  
  /* P-term */
  new_proportional = error * pid->param.kP * gain_scaler;

  /* D-term */
  if (pid->reset) {
      pid->last_input = (pid_flags & PID_DTERM_FROM_ERROR) ? error : measurement;
      pid->reset = false;
  }

  if (pid_flags & PID_DTERM_FROM_ERROR) {
      /* Error-tracking D-term */
      new_derivative = (error - pid->last_input) / dt;
      pid->last_input = error;
  } else {
      /* Measurement tracking D-term */
      new_derivative = -(measurement - pid->last_input) / dt;
      pid->last_input = measurement;
  }

  new_derivative = pid->param.kD  * ((pid->dterm_lpf_hz > 0) ? pt1_filter_apply4(&pid->dterm_filter_state, new_derivative, pid->dterm_lpf_hz, dt) : new_derivative);

  new_derivative *= new_derivative * dTerm_scaler;

  if (pid_flags & PID_ZERO_INTEGRATOR) {
      pid->integrator = 0.0F;
  }

  /*
    * Compute FeedForward parameter
    */
  new_feed_forward = setpoint * pid->param.kFF * gain_scaler;

  /* Pre-calculate output and limit it if actuator is saturating */
  const float out_val = new_proportional + (pid->integrator * gain_scaler) + new_derivative + new_feed_forward;
  const float out_val_constrained = constrainf(out_val, out_min, out_max);

  pid->proportional = new_proportional;
  pid->integral = pid->integrator;
  pid->derivative = new_derivative;
  pid->feed_forward = new_feed_forward;
  pid->output_constrained = out_val_constrained;

  /* Update I-term */
  if ( !(pid_flags & PID_ZERO_INTEGRATOR) && !(pid_flags & PID_FREEZE_INTEGRATOR) ) {
      const float new_integrator = pid->integrator + (error * pid->param.kI * gain_scaler * dt) + ((out_val_constrained - out_val) * pid->param.kT * dt);

      if (pid_flags & PID_SHRINK_INTEGRATOR) {
          // Only allow integrator to shrink
          if (fabsf(new_integrator) < fabsf(pid->integrator)) {
              pid->integrator = new_integrator;
          }
      } else {
          pid->integrator = new_integrator;
      }
  }
  
  if (pid_flags & PID_LIMIT_INTEGRATOR) {
      pid->integrator = constrainf(pid->integrator, out_min, out_max);
  } 

  return out_val_constrained;
}

float nav_pid_apply2 (pid_controller_ts *pid, const float setpoint, const float measurement, const float dt, const float out_min, const float out_max, const pid_controller_flags_te pid_flags)
{
    return nav_pid_apply3(pid, setpoint, measurement, dt, out_min, out_max, pid_flags, 1.0f, 1.0f);
}

void nav_pid_reset (pid_controller_ts *pid)
{
    pid->reset = true;
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->integrator = 0.0f;
    pid->last_input = 0.0f;
    pid->feed_forward = 0.0f;
    pt1_filter_reset(&pid->dterm_filter_state, 0.0f);
    pid->output_constrained = 0.0f;
}

void nav_pid_init (pid_controller_ts *pid, float _kP, float _kI, float _kD, float _kFF, float _dterm_lpf_hz)
{
    pid->param.kP = _kP;
    pid->param.kI = _kI;
    pid->param.kD = _kD;
    pid->param.kFF = _kFF;

    if (_kI > 1e-6f && _kP > 1e-6f) {
        float Ti = _kP / _kI;
        float Td = _kD / _kP;
        pid->param.kT = 2.0f / (Ti + Td);
    } else if (_kI > 1e-6f) {
        pid->param.kI = _kI;
        pid->param.kT = 0.0f;
    } else {
        pid->param.kI = 0.0;
        pid->param.kT = 0.0;
    }
    
    pid->dterm_lpf_hz = _dterm_lpf_hz;
    nav_pid_reset(pid);
}

// HEADING_HOLD P Контроллер возвращает желаемую скорость вращения в dps для подачи на контроллер скорости
static float pid_heading_hold (float delta_t)
{
  float heading_hold_rate;

  int16_t error = /*DECIDEGREES_TO_DEGREES(attitude.values.yaw) - значение по рысканию*/ - heading_hold_target;

  // Преобразование абсолютной погрешности относительно текущего курса
  if (error <= -180)
    error += 360;
  if (error >= +180)
    error -= 360;

  heading_hold_rate = error * get_pid_bank()->pid[PID_HEADING].P / 30.0F;
  heading_hold_rate = constrainf(heading_hold_rate, -p_pid_profile->heading_hold_rate_limit, p_pid_profile->heading_hold_rate_limit);
  heading_hold_rate = pt1_filter_apply4(&heading_hold_rate_filter, heading_hold_rate, HEADING_HOLD_ERROR_LPF_FREQ, delta_t);

  return heading_hold_rate;
}

/**
  * @brief Возвращает используемый профиль типа ПИДа
  * @param Нет
  * @retval const pid_bank_ts * 
  */
const pid_bank_ts *get_pid_bank (void) 
{
    return used_pid_controller_type == PID_TYPE_PIFF ? &p_pid_profile->bank_fw : &p_pid_profile->bank_mc;
}

/**
  * @brief Возвращает используемый профиль типа ПИДа и позволяет изменить его параметры
  * @param Нет
  * @retval pid_bank_ts * 
  */
pid_bank_ts *get_pid_bank_mutable (void) 
{
    return used_pid_controller_type == PID_TYPE_PIFF ? &p_pid_profile->bank_fw : &p_pid_profile->bank_mc;
}

/**
  * @brief Обновление флага обновления усиления ПИД
  * @param Нет
  * @retval Нет
  */
void schedule_pid_gains_update (void)
{
  pid_gains_update_required = true;
}

static float pid_rc_command_to_rate (int16_t stick, uint8_t rate)
{
    const float max_rate_DPS = rate * 10.0F;
    return scaleRangef((float) stick, -500.0F, 500.0F, -max_rate_DPS, max_rate_DPS);
}