/**
 * WRITE ME
 *
 */

#pragma once

#include "core2forAWS.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct GpsPosition
    {
        float latitude;
        float longitude;
        float altitude;
        uint32_t satellites;
        uint32_t unix_time;
        bool is_valid;
    } GpsPosition;

    extern const char* GPS_TAB_NAME;

    extern lv_obj_t* gps_tab;
    TaskHandle_t gps_handle;

    void display_gps_tab(lv_obj_t*, lv_obj_t*);
    GpsPosition get_latest_gps_position(void);

#ifdef __cplusplus
}
#endif
