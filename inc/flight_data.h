#ifndef _FLIGHT_DATA_H
#define _FLIGHT_DATA_H

typedef struct {
    double tilt;
    double pitch;
    double prowl;
} base_parameters_st;

typedef struct {
    double speed_motor_1;
    double speed_motor_2;
    double speed_motor_3;
    double speed_motor_4;
} speeds_of_motors_st;

typedef struct {
    double gps_coordinate;
} gps_st;

typedef struct {
    double altitude;                        //высота
    base_parameters_st *base_parameters;    //крен, тангаж, рысканье
    speeds_of_motors_st *speeds_of_motors;  //скорость моторов
    gps_st *gps_param;                      //gps координата
} flight_data_st;

flight_data_st *flight_data_get (void);

#endif //_FLIGHT_DATA_H