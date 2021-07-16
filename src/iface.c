#include "iface.h"

iface_result_et iface_tx (flight_data_st *flight_data_for_tx)
{
    if (flight_data_for_tx == NULL)
        return IFACE_RESULT_BAD;

    //send on iface

    return IFACE_RESULT_OK;
}

iface_result_et iface_rx (flight_data_st *flight_data_for_rx)
{
    //receive on iface

    return IFACE_RESULT_OK;
}