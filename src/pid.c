#include "pid.h"

static bool pid_filters_is_configured = false;
static float heading_hold_cosz_limit;
static bool pid_gains_update_required;
static uint8_t iterm_relax;

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