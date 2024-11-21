#pragma once

//#include "core2forAWS.h"

#define ENV3_TAB_NAME "ENV_III-SENSOR"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct SensorInfo
    {
        float temperature;
        float humidity;
        float pressure;
    } SensorInfo;

    extern lv_obj_t* env3_tab;
    TaskHandle_t env3_handle;

    void display_env3_tab(lv_obj_t*, lv_obj_t*);

    SensorInfo get_sensor_info(void);

#ifdef __cplusplus
}
#endif
