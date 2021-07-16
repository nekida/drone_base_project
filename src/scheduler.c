#include "scheduler.h"
#include <string.h>
#include "maths.h"
#include "drone_time.h"
#include <stdio.h>
#include <unistd.h>

task_st *current_task = NULL;
int task_queue_pos = 0;
int task_queue_size = 0;

task_st *task_queue_array[TASK_COUNT];

uint32_t total_waiting_tasks;               // общее количество ожидающих задач
uint32_t total_waiting_tasks_samples;       // общее количество ожидающих образцов задач  
uint16_t average_system_load_percent = 0;   // средняя загрузка системы в процентах

static void queue_clear (void)
{
    memset(task_queue_array, 0, sizeof(task_queue_array));
    task_queue_pos = 0;
    task_queue_size = 0;
}

static int queue_get_size (void)
{
    return task_queue_size;
}

static bool queue_is_this_task (task_st *task)
{
    for (uint32_t i = 0; i < task_queue_size; i++)
        if (task_queue_array[i] == task)
            return true;
    
    return false;
}

static bool queue_add_task (task_st *task)
{
    if ((task_queue_size >= TASK_COUNT) || queue_is_this_task(task))
        return false;

    for (uint32_t i = 0; i <= task_queue_size; i++)
        if ((task_queue_array[i] == NULL) || (task_queue_array[i]->static_priority < task->static_priority)) {
            memmove(&task_queue_array[i + 1], &task_queue_array[i], sizeof(task) * (task_queue_size - i));  // переносим (task_queue_size - i) элементов "вниз"
            task_queue_array[i] = task;
            task_queue_size++;

            return true;
        }

    return false;
}

static bool queue_remove_task (task_st *task)
{
    if ((task_queue_size == 0) || !queue_is_this_task(task))
        return false;

    for (uint32_t i = 0; i < task_queue_size; i++)
        if (task_queue_array[i] == task) {
            memmove(&task_queue_array[i], &task_queue_array[i + 1], sizeof(task) * (task_queue_size - i));  // переносим (task_queue_size - i) элементов "вверх"
            task_queue_size--;

            return true;
        }

    return false;
}

static task_st *queue_get_fisrt_task (void)
{
    task_queue_pos = 0;
    
    return task_queue_array[0];
}

static task_st *queue_get_next_task (void)
{
    return task_queue_array[++task_queue_pos]; 
}

void task_system (uint64_t current_time_us)
{
    UNUSED(current_time_us);

    // расчет загруженности системы
    if (total_waiting_tasks_samples > 0) {
        average_system_load_percent = 100 * total_waiting_tasks / total_waiting_tasks_samples;
        total_waiting_tasks_samples = 0;
        total_waiting_tasks = 0;
    }

    printf("System load percent: %d\n", average_system_load_percent);
}

void task_run_realtime_callbacks (uint64_t current_time_us)
{
    UNUSED(current_time_us);
}

/*Задать новый период исполнения*/
/*Задача может сама себе или другой задаче изменить желаемый период выполнения*/
void reschedule_task (task_id_et task_id, int32_t new_period_in_us)
{
    if (TASK_SELF == task_id || task_id < TASK_COUNT) {
        task_st *task = (TASK_SELF == task_id) ? current_task : &tasks[task_id]; 
        task->desired_period = MAX(SCHEDULER_DELAY_LIMIT, new_period_in_us);
    }
}

/*Включить задачу/
/*Задача может сама себя или другую задачу добавить или удалить из удереди выполнения*/
void set_task_enabled (task_id_et task_id, bool enabled)
{
    if (TASK_SELF == task_id || task_id < TASK_COUNT) {
        task_st *task = (TASK_SELF == task_id) ? current_task : &tasks[task_id];
        (enabled && task->task_func) ? queue_add_task(task) : queue_remove_task(task);
    }
}

/*Получить значение времени, прошедшего с последнего выполнения задачи*/
int32_t get_task_delta_time (task_id_et task_id)
{
    if (TASK_SELF == task_id) {
        return current_task->task_latest_delta_time;
    } else if (task_id < TASK_COUNT) {
        return tasks[task_id].task_latest_delta_time;
    } else {
        return 0;
    }
}

void scheduler_init (void)
{
    queue_clear();
    queue_add_task(&tasks[TASK_SYSTEM]);
}

void scheduler (void)
{
    const uint64_t current_time_in_us = drone_time_get_micros();

    task_st *selected_task = NULL;
    uint16_t selected_task_dynamic_priority = 0;
    bool forced_real_time_task = false;

    uint16_t waiting_tasks = 0;

    for (task_st *task = queue_get_fisrt_task(); task != NULL; task = queue_get_next_task()) {
        if (task->check_func) {
            const uint64_t current_time_bebore_check_func_call_in_us = drone_time_get_micros();

            if (task->dynamic_priority > 0) {
                task->task_age_cycles = 1 + ((int32_t)(current_time_in_us - task->last_signaled_at)) / task->desired_period;
                task->dynamic_priority = 1 + task->static_priority * task->task_age_cycles;
                waiting_tasks++;
            } else if (task->check_func(current_time_bebore_check_func_call_in_us, current_time_bebore_check_func_call_in_us - task->last_execute_at)) {
                task->last_signaled_at = current_time_bebore_check_func_call_in_us;
                task->task_age_cycles = 1;
                task->dynamic_priority = 1 + task->static_priority;
                waiting_tasks++;
            } else {
                task->task_age_cycles = 0;
            }
        } else if (task->static_priority == TASK_PRIORITY_REALTIME) {
            if (((int32_t)(current_time_in_us - task->last_execute_at)) > task->desired_period) {
                selected_task_dynamic_priority = task->dynamic_priority;
                selected_task = task;
                waiting_tasks++;
                forced_real_time_task = false;
            }
        } else {
            task->task_age_cycles = ((int32_t)(current_time_in_us - task->last_execute_at)) / task->desired_period;
            if (task->task_age_cycles > 0) {
                task->dynamic_priority = 1 + task->static_priority * task->task_age_cycles;
                waiting_tasks++;
            }
        }

        if (!forced_real_time_task && task->dynamic_priority > selected_task_dynamic_priority) {
            selected_task_dynamic_priority = task->dynamic_priority;
            selected_task = task;
        }
    }

    total_waiting_tasks_samples++;
    total_waiting_tasks += waiting_tasks;

    current_task = selected_task;

    if (selected_task) {
        selected_task->task_latest_delta_time = (int32_t)(current_time_in_us - selected_task->last_execute_at);
        selected_task->last_execute_at = current_time_in_us;
        selected_task->dynamic_priority = 0;

        const uint64_t current_time_before_task_call = drone_time_get_micros();
        selected_task->task_func(current_time_before_task_call);
    } else {
        const uint64_t current_time_before_task_call = drone_time_get_micros();
        task_run_realtime_callbacks(current_time_before_task_call);
    }
}