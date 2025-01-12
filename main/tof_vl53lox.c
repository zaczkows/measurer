#include "tof_vl53lox.h"

typedef unsigned char byte;

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID 0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS 0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS 0x14
#define VL53L0X_ADDRESS 0x29 // I2C address

static const char* TAG = "TOF_VL53LOX";

I2CDevice_t
TofVl53lox_Init()
{
    I2CDevice_t device = Core2ForAWS_Port_A_I2C_Begin(VL53L0X_ADDRESS, PORT_A_I2C_STANDARD_BAUD);
    return device;
}

static uint16_t
makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

uint16_t
TofVl53lox_get_reading(I2CDevice_t device)
{
    byte data[12];
    data[0] = 0x01;
    esp_err_t err = Core2ForAWS_Port_A_I2C_Write(device, VL53L0X_REG_SYSRANGE_START, &data[0], 1);
    if (err) {
        ESP_LOGW(TAG, "write error!");
        return 0;
    }

    int cnt = 0;
    while (cnt < 100) { // 1 second waiting time max
        vTaskDelay(pdMS_TO_TICKS(10));
        data[0] = 0;
        err = Core2ForAWS_Port_A_I2C_Read(device, VL53L0X_REG_RESULT_RANGE_STATUS, &data[0], 1);
        if (err) {
            ESP_LOGW(TAG, "initial read error!");
            return 0;
        }
        if (data[0] & 0x01) {
            break;
        }
        cnt++;
    }

    if ((data[0] & 0x01) == 0) {
        ESP_LOGW(TAG, "TOF VL53LOX is not ready");
        return 0;
    }

    err = Core2ForAWS_Port_A_I2C_Read(device, VL53L0X_REG_RESULT_RANGE_STATUS, &data[0], 12);
    if (err) {
        ESP_LOGW(TAG, "status read error!");
        return 0;
    }
    uint16_t acnt = makeuint16(data[7], data[6]);
    uint16_t scnt = makeuint16(data[9], data[8]);
    uint16_t dist = makeuint16(data[11], data[10]);
    byte device_range_status_internal = ((data[0] & 0x78) >> 3);
    /* ESP_LOGI(TAG, */
    /*          "ambient count: %u, signal count: %u, distance: %u, status: %u", */
    /*          acnt, */
    /*          scnt, */
    /*          dist, */
    /*          device_range_status_internal); */
    if (device_range_status_internal == 11) {
        return dist;
    } else {
        return 0;
    }
}
