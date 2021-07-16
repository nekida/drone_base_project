#include "main.h"
#include <stdarg.h>
#include "scheduler.h"

#define TEST_5_ARG_1    5
#define TEST_5_ARG_2    5

#define pt_get_handle(name) (&name##_proto_thead_state)

#define USE_OSD

static const uint8_t U32_ADDR = 123;

#define FIRMWARE_START_ADDRESS ((uint32_t)&U32_ADDR)

typedef struct isrVector_s {
    uint32_t    field_1;
    uint32_t    field_2;
    
} isrVector_ts;

int abc_proto_thead_state = 10;

static drone_model_st drone_model;
static flight_data_st *flight_data;
static int function_test_5 (int n, ...);
static int function_test_6 (int a, int b);
static void function_test_7 (void);
static void function_test_8 (void);

int main (void)
{
    log_print("Test 1 START");

    strcpy(drone_model.ch_name, "Drone 1");
    drone_model.mcu_model = MCU_MODEL_3;
    drone_model.peripheral.servo_type = SERVO_MODEL_2;

    log_print("Test 1 STOP");

    (fc_init() == FC_INIT_STATUS_OK) ? log_print("Test 2 OK") : log_print("Test 2 BAD");

    flight_data = flight_data_get();

    log_print("Parameters of flight:");

    printf("Altitude = %lf\ntilt = %lf\npitch = %lf\nprowl = %lf\nspeed of motor 1 = %lf\nspeed of motor 2 = %lf\nspeed of motor 3 = %lf\nspeed of motor 4 = %lf\ngps coordinate = %lf\n",
    //printf("Altitude = %lf\n",
    flight_data->altitude,
    flight_data->base_parameters->tilt,
    flight_data->base_parameters->pitch,
    flight_data->base_parameters->prowl,
    flight_data->speeds_of_motors->speed_motor_1,
    flight_data->speeds_of_motors->speed_motor_2,
    flight_data->speeds_of_motors->speed_motor_3,
    flight_data->speeds_of_motors->speed_motor_4,
    flight_data->gps_param->gps_coordinate
    );

    log_print("Test 3 - Interfaces: TX");

    (iface_tx(flight_data) == IFACE_RESULT_OK) ? log_print("Test OK") : log_print("Test BAD");

    log_print("Test 4 - Interfaces: TX NULL");

    (iface_tx(NULL) == IFACE_RESULT_OK) ? log_print("Test OK") : log_print("Test BAD");

    log_print("Test 5 - function with variables of parameters");

    (function_test_5(2, TEST_5_ARG_1, TEST_5_ARG_2) == (TEST_5_ARG_1 + TEST_5_ARG_2)) ? log_print("Test OK") : log_print("Test BAD");

    log_print("Test 6 - pointers");

    function_test_6(0, 0);

    int abc = 5;

    printf("%d\n", *pt_get_handle(abc));

    log_print("Test 7 - macros");

    function_test_7();

    log_print("Test 8 - bootloader");

    function_test_8();

    log_print("Test 9 - scheduler");

    scheduler_init();

    reschedule_task(TASK_PRINT_1, 500);
    set_task_enabled(TASK_PRINT_1, true);

    reschedule_task(TASK_PRINT_2, 500);
    set_task_enabled(TASK_PRINT_2, true);

    reschedule_task(TASK_PRINT_3, 500);
    set_task_enabled(TASK_PRINT_3, true);

    reschedule_task(TASK_REALTIME, 500);
    set_task_enabled(TASK_REALTIME, true);

    while (1) {
        scheduler();
    }

    return 0;
}

static int function_test_5 (int n, ...)
{
    int result = 0;
    va_list factor;
    va_start(factor, n);

    for (int i = 0; i < n; i++) {
        result += va_arg(factor, int);
    }
    va_end(factor);

    return result;
}

static int function_test_6 (int a, int b)
{
    char *str1 = "Hop, hey, lalaley - 1!";
    char str2[] = "Hop, hey, lalaley - 2!";

    printf("%s\n", str1);
    printf("%s\n", str2);
}

static void function_test_7 (void)
{
    #if defined(USE_OSD)
        printf("This is #if defined(USE_OSD)\n");
    #endif

    #ifdef USE_OSD
        printf("This is #ifdef USE_OSD\n");
    #endif
}

static void function_test_8 (void)
{
    uint32_t var_1 = 55;
    
    isrVector_ts *bootloaderVector = (isrVector_ts *)&var_1;

    //bootloaderVector = (isrVector_ts *)ptr;
    printf("%d\n", bootloaderVector->field_1);
    printf("%d\n", bootloaderVector->field_2);
}