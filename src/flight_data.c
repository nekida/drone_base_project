#include "flight_data.h"

static flight_data_st flight_data;
static base_parameters_st base_param;
static speeds_of_motors_st speeds_of_motors;
static gps_st gps;

static void flight_data_reset (void);

flight_data_st *flight_data_get (void)
{
    flight_data_reset();
    return &flight_data;
}

static void flight_data_reset (void)
{
    flight_data.altitude = 0.0;

    flight_data.base_parameters = &base_param;

    flight_data.speeds_of_motors = &speeds_of_motors;

    flight_data.gps_param = &gps;
}