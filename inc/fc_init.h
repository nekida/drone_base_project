#ifndef _FC_INIT_H
#define _FC_INIT_H


typedef enum {
    FC_INIT_STATUS_OK = 0,
    FC_INIT_STATUS_BAD
} fc_init_status_et;

typedef enum {
    SYSTEM_STATE_INITIALISING           = 0,
    SYSTEM_STATE_CONFIG_LOADED          = (1 << 0),
    SYSTEM_STATE_SENSORS_READY          = (1 << 1),
    SYSTEM_STATE_MOTORS_READY           = (1 << 2),
    SYSTEM_STATE_TRANSPONDER_ENABLED    = (1 << 3),
    SYSTEM_STATE_READY                  = (1 << 7)
} system_state_et;

extern system_state_et system_state;

fc_init_status_et fc_init (void);

#endif //_FC_INIT_H