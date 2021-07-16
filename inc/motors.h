#ifndef _MOTORS_H
#define _MOTORS_H

typedef enum {
    MOTORS_RESULT_OK = 0,
    MOTORS_RESULT_BAD
} motors_result_et;

typedef struct {
    motors_result_et (*api_1)(void *, ...);
    motors_result_et (*api_2)(void *, ...);
    motors_result_et (*api_3)(void *, ...);
} motor;

#endif //_MOTORS_H