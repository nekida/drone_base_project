#ifndef _MAIN_H
#define _MAIN_H

#include "debug.h"
#include <string.h>
#include "fc_init.h"
#include "flight_data.h"
#include "iface.h"

typedef enum {
    MCU_MODEL_0 = 0,
    MCU_MODEL_1,
    MCU_MODEL_2,
    MCU_MODEL_3,
} drone_mcu_model_et;

typedef enum {
    SERVO_MODEL_0 = 0,
    SERVO_MODEL_1,
    SERVO_MODEL_2,
    SERVO_MODEL_3,
} drone_servo_type_et;

typedef struct {
    drone_servo_type_et servo_type;
} drone_peripheral_st;

typedef struct {
    char ch_name[20];
    drone_mcu_model_et mcu_model;
    drone_peripheral_st peripheral;
} drone_model_st;

#endif //_MAIN_H