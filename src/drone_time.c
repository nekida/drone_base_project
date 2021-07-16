#include "drone_time.h"
#include <time.h>

uint64_t drone_time_get_micros (void)
{
    struct timespec tm1;

    clock_gettime(CLOCK_REALTIME, &tm1);

    return (uint64_t)(tm1.tv_nsec / 1000);
}