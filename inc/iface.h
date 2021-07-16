#ifndef _IFACE_H
#define _IFACE_H

#include <stdint.h>
#include <stddef.h>
#include "flight_data.h"

typedef enum {
    IFACE_RESULT_OK = 0,
    IFACE_RESULT_BAD
} iface_result_et;

iface_result_et iface_tx (flight_data_st *flight_data_for_tx);
iface_result_et iface_rx (flight_data_st *flight_data_for_rx);

#endif //_IFACE_H