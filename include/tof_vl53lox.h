#pragma once

#include "core2forAWS.h"

#ifdef __cplusplus
extern "C"
{
#endif

    I2CDevice_t TofVl53lox_Init();

    uint16_t TofVl53lox_get_reading(I2CDevice_t device);

#ifdef __cplusplus
}
#endif
