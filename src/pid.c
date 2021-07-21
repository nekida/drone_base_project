#include "pid.h"

#define FIXED_WING_LEVEL_TRIM_MAX_ANGLE 10.0f // Может потребоваться автоматическая обрезка максимального угла
#define FIXED_WING_LEVEL_TRIM_DIVIDER 500.0f
#define FIXED_WING_LEVEL_TRIM_MULTIPLIER 1.0f / FIXED_WING_LEVEL_TRIM_DIVIDER
#define FIXED_WING_LEVEL_TRIM_CONTROLLER_LIMIT FIXED_WING_LEVEL_TRIM_DIVIDER * FIXED_WING_LEVEL_TRIM_MAX_ANGLE
#define D_BOOST_GYRO_LPF_HZ 80    // Входная отсечка Biquad lowpass до пика D около частот пропитки
#define D_BOOST_LPF_HZ 10         // Отсечка нижних частот PT1 для сглаживания эффекта усиления

static void pid_apply_fixed_wing_rate_controller (pid_state_ts *pidState, flight_dynamics_index_te axis, float dT);
static void pid_apply_multicopter_rate_controller (pid_state_ts *pid_state, flight_dynamics_index_te axis, float dT);
static void null_rate_controller (pid_state_ts *pid_state, flight_dynamics_index_te axis, float dT);

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
static filter_apply_fn_ptr dTerm_lpf2_filter_apply_fn;
static bool levelingEnabled = false;

static float fixed_wing_level_trim;
static pid_controller_ts fixed_wing_level_trim_controller;

pid_profile_ts pid_profile = {
  .pid_bank_mc[PID_ROLL].P    = SETTING_MC_P_ROLL_DEFAULT,
  .pid_bank_mc[PID_ROLL].I    = SETTING_MC_I_ROLL_DEFAULT,
  .pid_bank_mc[PID_ROLL].D    = SETTING_MC_D_ROLL_DEFAULT,
  .pid_bank_mc[PID_ROLL].FF   = SETTING_MC_CD_ROLL_DEFAULT,

  .pid_bank_mc[PID_PITCH].P   = SETTING_MC_P_PITCH_DEFAULT,
  .pid_bank_mc[PID_PITCH].I   = SETTING_MC_I_PITCH_DEFAULT,
  .pid_bank_mc[PID_PITCH].D   = SETTING_MC_D_PITCH_DEFAULT,
  .pid_bank_mc[PID_PITCH].FF  = SETTING_MC_CD_PITCH_DEFAULT,

  .pid_bank_mc[PID_YAW].P     = SETTING_MC_P_YAW_DEFAULT, 
  .pid_bank_mc[PID_YAW].I     = SETTING_MC_I_YAW_DEFAULT,
  .pid_bank_mc[PID_YAW].D     = SETTING_MC_D_YAW_DEFAULT,
  .pid_bank_mc[PID_YAW].FF    = SETTING_MC_CD_YAW_DEFAULT,

  .pid_bank_mc[PID_LEVEL].P   = SETTING_MC_P_LEVEL_DEFAULT,   // Self-level strength
  .pid_bank_mc[PID_LEVEL].I   = SETTING_MC_I_LEVEL_DEFAULT,   // Self-leveing low-pass frequency (0 - disabled)
  .pid_bank_mc[PID_LEVEL].D   = SETTING_MC_D_LEVEL_DEFAULT,   // 75% horizon strength
  .pid_bank_mc[PID_LEVEL].FF  = SETTING_MC_FF_LEVEL_DEFAULT,

  .pid_bank_mc[PID_HEADING].P   = SETTING_NAV_MC_HEADING_P_DEFAULT, 
  .pid_bank_mc[PID_HEADING].I   = SETTING_NAV_MC_HEADING_I_DEFAULT,
  .pid_bank_mc[PID_HEADING].D   = SETTING_NAV_MC_HEADING_D_DEFAULT,
  .pid_bank_mc[PID_HEADING].FF  = SETTING_NAV_MC_HEADING_FF_DEFAULT, 

  .pid_bank_mc[PID_POS_XY].P  = SETTING_NAV_MC_POS_XY_P_DEFAULT,  // NAV_POS_XY_P * 100
  .pid_bank_mc[PID_POS_XY].I  = SETTING_NAV_MC_POS_XY_I_DEFAULT,
  .pid_bank_mc[PID_POS_XY].D  = SETTING_NAV_MC_POS_XY_D_DEFAULT,
  .pid_bank_mc[PID_POS_XY].FF = SETTING_NAV_MC_POS_XY_FF_DEFAULT,

  .pid_bank_mc[PID_VEL_XY].P  = SETTING_NAV_MC_VEL_XY_P_DEFAULT,  // NAV_VEL_XY_P * 20
  .pid_bank_mc[PID_VEL_XY].I  = SETTING_NAV_MC_VEL_XY_I_DEFAULT,  // NAV_VEL_XY_I * 100
  .pid_bank_mc[PID_VEL_XY].D  = SETTING_NAV_MC_VEL_XY_D_DEFAULT,  // NAV_VEL_XY_D * 100
  .pid_bank_mc[PID_VEL_XY].FF = SETTING_NAV_MC_VEL_XY_FF_DEFAULT, // NAV_VEL_XY_D * 100

  .pid_bank_mc[PID_POS_Z].P   = SETTING_NAV_MC_POS_Z_P_DEFAULT,   // NAV_POS_Z_P * 100
  .pid_bank_mc[PID_POS_Z].I   = SETTING_NAV_MC_POS_Z_I_DEFAULT,   // not used
  .pid_bank_mc[PID_POS_Z].D   = SETTING_NAV_MC_POS_Z_D_DEFAULT,   // not used
  .pid_bank_mc[PID_POS_Z].FF  = SETTING_NAV_MC_POS_Z_FF_DEFAULT,

  .pid_bank_mc[PID_VEL_Z].P   = SETTING_NAV_MC_VEL_Z_P_DEFAULT,   // NAV_VEL_Z_P * 66.7
  .pid_bank_mc[PID_VEL_Z].I   = SETTING_NAV_MC_VEL_Z_I_DEFAULT,   // NAV_VEL_Z_I * 20
  .pid_bank_mc[PID_VEL_Z].D   = SETTING_NAV_MC_VEL_Z_D_DEFAULT,   // NAV_VEL_Z_D * 100
  .pid_bank_mc[PID_VEL_Z].FF  = SETTING_NAV_MC_VEL_Z_FF_DEFAULT,

  .pid_bank_mc[PID_POS_HEADING].P   = SETTING_NAV_MC_POS_HEADING_P_DEFAULT,
  .pid_bank_mc[PID_POS_HEADING].I   = SETTING_NAV_MC_POS_HEADING_I_DEFAULT,
  .pid_bank_mc[PID_POS_HEADING].D   = SETTING_NAV_MC_POS_HEADING_D_DEFAULT,
  .pid_bank_mc[PID_POS_HEADING].FF  = SETTING_NAV_MC_POS_HEADING_FF_DEFAULT,


  .pid_bank_fw[PID_ROLL].P  = SETTING_FW_P_ROLL_DEFAULT,    // стабилизация P-составляющей фиксированного крыла для ROLL
  .pid_bank_fw[PID_ROLL].I  = SETTING_FW_I_ROLL_DEFAULT,    // стабилизация I-составляющей фиксированного крыла для ROLL
  .pid_bank_fw[PID_ROLL].D  = SETTING_FW_D_ROLL_DEFAULT,    // стабилизация D-составляющей фиксированного крыла для ROLL
  .pid_bank_fw[PID_ROLL].FF = SETTING_FW_FF_ROLL_DEFAULT,   // стабилизация FF-составляющей фиксированного крыла для ROLL

  .pid_bank_fw[PID_PITCH].P  = SETTING_FW_P_PITCH_DEFAULT,    // стабилизация P-составляющей фиксированного крыла для PITCH
  .pid_bank_fw[PID_PITCH].I  = SETTING_FW_I_PITCH_DEFAULT,    // стабилизация I-составляющей фиксированного крыла для PITCH
  .pid_bank_fw[PID_PITCH].D  = SETTING_FW_D_PITCH_DEFAULT,    // стабилизация D-составляющей фиксированного крыла для PITCH
  .pid_bank_fw[PID_PITCH].FF = SETTING_FW_FF_PITCH_DEFAULT,   // стабилизация FF-составляющей фиксированного крыла для PITCH

  .pid_bank_fw[PID_YAW].P  = SETTING_FW_P_YAW_DEFAULT,    // стабилизация P-составляющей фиксированного крыла для YAW
  .pid_bank_fw[PID_YAW].I  = SETTING_FW_I_YAW_DEFAULT,    // стабилизация I-составляющей фиксированного крыла для YAW
  .pid_bank_fw[PID_YAW].D  = SETTING_FW_D_YAW_DEFAULT,    // стабилизация D-составляющей фиксированного крыла для YAW
  .pid_bank_fw[PID_YAW].FF = SETTING_FW_FF_YAW_DEFAULT,   // стабилизация FF-составляющей фиксированного крыла для YAW

  .pid_bank_fw[PID_LEVEL].P   = SETTING_FW_P_LEVEL_DEFAULT, // стабилизация P-составляющей фиксированного крыла по ориентации, самоуравновешивающаяся сила,
  .pid_bank_fw[PID_LEVEL].I   = SETTING_FW_I_LEVEL_DEFAULT, // Отсечка фильтра нижних частот для стабилизации ориентации неподвижного крыла, Самовыравнивающаяся частота низких частот (0 - отключено)
  .pid_bank_fw[PID_LEVEL].D   = SETTING_FW_D_LEVEL_DEFAULT, // Стабилизация ориентации самолета Точка перехода HORIZON, 75% прочность горизонта
  .pid_bank_fw[PID_LEVEL].FF  = SETTING_FW_FF_LEVEL_DEFAULT,

  .pid_bank_fw[PID_HEADING].P   = SETTING_NAV_FW_HEADING_P_DEFAULT,   // Усиление P контроллера удержания курса (фиксированное крыло)
  .pid_bank_fw[PID_HEADING].I   = SETTING_NAV_FW_HEADING_I_DEFAULT,
  .pid_bank_fw[PID_HEADING].D   = SETTING_NAV_FW_HEADING_D_DEFAULT,
  .pid_bank_fw[PID_HEADING].FF  = SETTING_NAV_FW_HEADING_FF_DEFAULT,

  .pid_bank_fw[PID_POS_Z].P   = SETTING_NAV_FW_POS_Z_P_DEFAULT,   // P усиление высоты ПИД-регулятором (фиксированное крыло), FW_POS_Z_P * 10
  .pid_bank_fw[PID_POS_Z].I   = SETTING_NAV_FW_POS_Z_I_DEFAULT,   // I усиление высоты ПИД-регулятором (фиксированное крыло), FW_POS_Z_I * 10  
  .pid_bank_fw[PID_POS_Z].D   = SETTING_NAV_FW_POS_Z_D_DEFAULT,   // D усиление высоты ПИД-регулятором (фиксированное крыло), FW_POS_Z_D * 10
  .pid_bank_fw[PID_POS_Z].FF  = SETTING_NAV_FW_POS_Z_FF_DEFAULT,

  .pid_bank_fw[PID_POS_XY].P  = SETTING_NAV_FW_POS_XY_P_DEFAULT,  // P-усиление ПИД-регулятора 2D траектории. Поиграйте с этим, чтобы получить прямую линию между путевыми точками или прямой возврат домой,  FW_POS_XY_P * 100
  .pid_bank_fw[PID_POS_XY].I  = SETTING_NAV_FW_POS_XY_I_DEFAULT,  // I-усиление ПИД-регулятора 2D траектории. Слишком большое значение приведет к отклонению траектории. Лучше начинать настройку с нуля,     FW_POS_XY_I * 100
  .pid_bank_fw[PID_POS_XY].D  = SETTING_NAV_FW_POS_XY_D_DEFAULT,  // D-усиление ПИД-регулятора 2D траектории. Слишком большое значение приведет к отклонению траектории. Лучше начинать настройку с нуля,     FW_POS_XY_D * 100
  .pid_bank_fw[PID_POS_XY].FF = SETTING_NAV_FW_POS_XY_FF_DEFAULT,

  .pid_bank_fw[PID_POS_HEADING].P   = SETTING_NAV_FW_POS_HDG_P_DEFAULT,   // P усиление ПИД-регулятора курса. (Фиксированное крыло, вездеходы, лодки)
  .pid_bank_fw[PID_POS_HEADING].I   = SETTING_NAV_FW_POS_HDG_I_DEFAULT,   // I усиление ПИД-регулятора траектории движения. (Фиксированное крыло, вездеходы, лодки)
  .pid_bank_fw[PID_POS_HEADING].D   = SETTING_NAV_FW_POS_HDG_D_DEFAULT,   // D усиление ПИД-регулятора траектории движения. (Фиксированное крыло, вездеходы, лодки)
  .pid_bank_fw[PID_POS_HEADING].FF  = SETTING_NAV_FW_POS_HDG_FF_DEFAULT,

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
    pid_state[FD_YAW].pterm_filter_apply_fn = (yaw_lpf_hz) ? (filter_apply_4_fn_ptr)pt1_filter_apply4 : (filter_apply_4_fn_ptr)null_filter_apply4;
    for (uint8_t axis = FD_ROLL; axis <= FD_PITCH; axis++) {
      pid_state[axis].pid_sum_limit = p_pid_profile->pid_sum_limit;
      pid_state[axis].pterm_filter_apply_fn = (filter_apply_4_fn_ptr)null_filter_apply4;
    }

    // в зависимости от плафтормы (типа движков) меняется и тип ПИДа
    if (p_pid_profile->pid_controller_type == PID_TYPE_AUTO) {
      // if (конфиг мотора) {
        used_pid_controller_type = PID_TYPE_PIFF;
        pid_controller_apply_fn = pid_apply_fixed_wing_rate_controller;
      // } else {
        used_pid_controller_type = PID_TYPE_PID;
        pid_controller_apply_fn = pid_apply_multicopter_rate_controller;
      // }

    } else {
      used_pid_controller_type = p_pid_profile->pid_controller_type;
      pid_controller_apply_fn = null_rate_controller;
    }

    dterm_lpf_filter_apply_fn   = (p_pid_profile->dterm_lpf_hz)   ? (filter_apply_fn_ptr)pt1_filter_apply : (filter_apply_fn_ptr)biquad_filter_apply;
    dTerm_lpf2_filter_apply_fn  = (p_pid_profile->dterm_lpf2_hz)  ? (filter_apply_fn_ptr)pt1_filter_apply : (filter_apply_fn_ptr)biquad_filter_apply;

    pid_reset_TPA_filter();

    fixed_wing_level_trim = p_pid_profile->fixed_wing_level_trim;
}

void pid_reset_TPA_filter (void)
{
    if (used_pid_controller_type == PID_TYPE_PIFF && current_control_rate_profile.throttle.fixed_wing_tau_ms > 0) {
        pt1_filter_init_RC(&fixed_wing_tpa_filter, current_control_rate_profile.throttle.fixed_wing_tau_ms * 1e-3f, TASK_PERIOD_HZ(TASK_AUX_RATE_HZ) * 1e-6f);
//        pt1_filter_reset(&fixed_wing_tpa_filter, getThrottleIdleValue());
    }
}

static void null_rate_controller (pid_state_ts *pid_state, flight_dynamics_index_te axis, float dT) 
{
    UNUSED(pid_state);
    UNUSED(axis);
    UNUSED(dT);
}

static void pid_apply_multicopter_rate_controller (pid_state_ts *pid_state, flight_dynamics_index_te axis, float dT)
{
//     const float rateError = pid_state->rate_target - pid_state->gyro_rate;
//     const float newPTerm = pTermProcess(pidState, rateError, dT);
//     const float newDTerm = dTermProcess(pidState, dT);

//     const float rateTargetDelta = pidState->rateTarget - pidState->previousRateTarget;
//     const float rateTargetDeltaFiltered = pt2FilterApply(&pidState->rateTargetFilter, rateTargetDelta);

//     /*
//      * Compute Control Derivative
//      */
//     const float newCDTerm = rateTargetDeltaFiltered * (pidState->kCD / dT);
//     DEBUG_SET(DEBUG_CD, axis, newCDTerm);

//     // TODO: Get feedback from mixer on available correction range for each axis
//     const float newOutput = newPTerm + newDTerm + pidState->errorGyroIf + newCDTerm;
//     const float newOutputLimited = constrainf(newOutput, -pidState->pidSumLimit, +pidState->pidSumLimit);

//     float itermErrorRate = applyItermRelax(axis, pidState->rateTarget, rateError);

// #ifdef USE_ANTIGRAVITY
//     itermErrorRate *= iTermAntigravityGain;
// #endif

//     pidState->errorGyroIf += (itermErrorRate * pidState->kI * antiWindupScaler * dT)
//                              + ((newOutputLimited - newOutput) * pidState->kT * antiWindupScaler * dT);

//     // Don't grow I-term if motors are at their limit
//     applyItermLimiting(pidState);

//     axisPID[axis] = newOutputLimited;

// #ifdef USE_BLACKBOX
//     axisPID_P[axis] = newPTerm;
//     axisPID_I[axis] = pidState->errorGyroIf;
//     axisPID_D[axis] = newDTerm;
//     axisPID_Setpoint[axis] = pidState->rateTarget;
// #endif

//     pidState->previousRateTarget = pidState->rateTarget;
//     pidState->previousRateGyro = pidState->gyroRate;
}

static void pid_apply_fixed_wing_rate_controller (pid_state_ts *pidState, flight_dynamics_index_te axis, float dT)
{
//     const float rateError = pidState->rateTarget - pidState->gyroRate;
//     const float newPTerm = pTermProcess(pidState, rateError, dT);
//     const float newDTerm = dTermProcess(pidState, dT);
//     const float newFFTerm = pidState->rateTarget * pidState->kFF;

//     DEBUG_SET(DEBUG_FW_D, axis, newDTerm);
//     /*
//      * Integral should be updated only if axis Iterm is not frozen
//      */
//     if (!pidState->itermFreezeActive) {
//         pidState->errorGyroIf += rateError * pidState->kI * dT;
//     }

//     applyItermLimiting(pidState);

//     if (pidProfile()->fixedWingItermThrowLimit != 0) {
//         pidState->errorGyroIf = constrainf(pidState->errorGyroIf, -pidProfile()->fixedWingItermThrowLimit, pidProfile()->fixedWingItermThrowLimit);
//     }

//     axisPID[axis] = constrainf(newPTerm + newFFTerm + pidState->errorGyroIf + newDTerm, -pidState->pidSumLimit, +pidState->pidSumLimit);

// #ifdef USE_AUTOTUNE_FIXED_WING
//     if (FLIGHT_MODE(AUTO_TUNE) && !FLIGHT_MODE(MANUAL_MODE)) {
//         autotuneFixedWingUpdate(axis, pidState->rateTarget, pidState->gyroRate, constrainf(newPTerm + newFFTerm, -pidState->pidSumLimit, +pidState->pidSumLimit));
//     }
// #endif

// #ifdef USE_BLACKBOX
//     axisPID_P[axis] = newPTerm;
//     axisPID_I[axis] = pidState->errorGyroIf;
//     axisPID_D[axis] = newDTerm;
//     axisPID_Setpoint[axis] = pidState->rateTarget;
// #endif

//     pidState->previousRateGyro = pidState->gyroRate;

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
      pid->integrator = 0.0f;
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