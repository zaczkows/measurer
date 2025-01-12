#pragma once

#include "core2forAWS.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct SHT3xMeasurement
    {
        float temperature;
        float humidity;
    } SHT3xMeasurement;

    extern const uint8_t SHT3x_DEVICE_ADDRESS;

    I2CDevice_t QMP6988_deviceCheck();

    SHT3xMeasurement SHT3x_get_measurement(I2CDevice_t sht3x_peripheral);

    float QMP6988_calcPressure(I2CDevice_t slave);

#ifdef __cplusplus
}
#endif
