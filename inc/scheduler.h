#ifndef _SCHEDULER_H
#define _SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

#define SCHEDULER_DELAY_LIMIT 100

#define UNUSED(x) (void)(x)

#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
#define TASK_PERIOD_MS(ms) ((ms) * 1000)
#define TASK_PERIOD_US(us) (us)

typedef enum {
    TASK_PRIORITY_IDLE = 0,
    TASK_PRIORITY_LOW,
    TASK_PRIORITY_MIDDLE,
    TASK_PRIORITY_HIGH,
    TASK_PRIORITY_REALTIME = 18,
    TASK_PRIORITY_MAX = 255
} task_priority_et;

typedef struct {
    uint64_t    max_execution_time;
    uint64_t    total_execution_time;
    uint64_t    average_execution_time;
} check_func_info_st;

typedef struct {
    const char          *name_task;
    bool                is_enabled;
    uint8_t             static_priority;
    int32_t             desire_period;
    check_func_info_st  check_func_info;
    int32_t             latest_delta_time;
} task_info_st;

typedef enum {
    TASK_SYSTEM = 0,
    TASK_PRINT_1,
    TASK_PRINT_2,
    TASK_PRINT_3,
    TASK_REALTIME,
    TASK_COUNT,
    TASK_NONE = TASK_COUNT,
    TASK_SELF
} task_id_et;

typedef struct {
    const char *task_name;                                                              
    bool (*check_func)(uint64_t current_time_in_us, int32_t current_delta_time_in_us);  // функция проверки
    void (*task_func)(uint64_t current_time_in_us);                                     // функция реализации задачи
    int32_t desired_period;                                                             // целевой период исполнения
    const uint8_t static_priority;                                                      // dynamic_priority увеличивается с шагом этого размера, != 0

    /* Scheduling */
    uint16_t dynamic_priority;      // измерение времени последнего выполнения задачи, используется для предотвращения зависания задачи
    uint16_t task_age_cycles;       // возраст задачи в циклах
    uint64_t last_execute_at;       // время последнего вызова
    uint64_t last_signaled_at;       // время вызова события для событийно-управляемых задач
    int32_t task_latest_delta_time; // время с последнего выполнения

    /* Statistics */
    uint64_t moving_sum_execution_time;  // скользящее суммарное время выполнения всех задач
} task_st;

extern task_st tasks[TASK_COUNT];

void task_system (uint64_t currentTimeUs);
void scheduler_init (void);
void scheduler (void);
void reschedule_task (task_id_et task_id, int32_t new_period_in_us);
void set_task_enabled (task_id_et task_id, bool enabled);
int32_t get_task_delta_time (task_id_et task_id);


#endif //_SCHEDULER_H