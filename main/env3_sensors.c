#include "env3_sensors.h"

#include "driver/i2c.h"

typedef unsigned char byte;

static const char* TAG = "ENV_III-SENSORS";

static const uint8_t QMP6988_SLAVE_ADDRESS_L = 0x70;
static const uint8_t QMP6988_SLAVE_ADDRESS_H = 0x56;
static const uint8_t QMP6988_CHIP_ID = 0x5C;
static const uint8_t QMP6988_CHIP_ID_REG = 0xD1;
static const uint8_t QMP6988_RESET_REG = 0xE0; /* Device reset register */

#define QMP6988_U16_t unsigned short
#define QMP6988_S16_t short
#define QMP6988_U32_t unsigned int
#define QMP6988_S32_t int
#define QMP6988_U64_t unsigned long long
#define QMP6988_S64_t long long

#define QMP6988_DEVICE_STAT_REG 0xF3 /* Device state register */
#define QMP6988_CTRLMEAS_REG 0xF4    /* Measurement Condition Control Register */
/* data */
#define QMP6988_PRESSURE_MSB_REG 0xF7    /* Pressure MSB Register */
#define QMP6988_TEMPERATURE_MSB_REG 0xFA /* Temperature MSB Reg */

/* compensation calculation */
#define QMP6988_CALIBRATION_DATA_START 0xA0 /* QMP6988 compensation coefficients */
#define QMP6988_CALIBRATION_DATA_LENGTH 25

#define SHIFT_RIGHT_4_POSITION 4
#define SHIFT_LEFT_2_POSITION 2
#define SHIFT_LEFT_4_POSITION 4
#define SHIFT_LEFT_5_POSITION 5
#define SHIFT_LEFT_8_POSITION 8
#define SHIFT_LEFT_12_POSITION 12
#define SHIFT_LEFT_16_POSITION 16

/* power mode */
#define QMP6988_SLEEP_MODE 0x00
#define QMP6988_FORCED_MODE 0x01
#define QMP6988_NORMAL_MODE 0x03

#define QMP6988_CTRLMEAS_REG_MODE__POS 0
#define QMP6988_CTRLMEAS_REG_MODE__MSK 0x03
#define QMP6988_CTRLMEAS_REG_MODE__LEN 2

/* oversampling */
#define QMP6988_OVERSAMPLING_SKIPPED 0x00
#define QMP6988_OVERSAMPLING_1X 0x01
#define QMP6988_OVERSAMPLING_2X 0x02
#define QMP6988_OVERSAMPLING_4X 0x03
#define QMP6988_OVERSAMPLING_8X 0x04
#define QMP6988_OVERSAMPLING_16X 0x05
#define QMP6988_OVERSAMPLING_32X 0x06
#define QMP6988_OVERSAMPLING_64X 0x07

#define QMP6988_CTRLMEAS_REG_OSRST__POS 5
#define QMP6988_CTRLMEAS_REG_OSRST__MSK 0xE0
#define QMP6988_CTRLMEAS_REG_OSRST__LEN 3

#define QMP6988_CTRLMEAS_REG_OSRSP__POS 2
#define QMP6988_CTRLMEAS_REG_OSRSP__MSK 0x1C
#define QMP6988_CTRLMEAS_REG_OSRSP__LEN 3

/* filter */
#define QMP6988_FILTERCOEFF_OFF 0x00
#define QMP6988_FILTERCOEFF_2 0x01
#define QMP6988_FILTERCOEFF_4 0x02
#define QMP6988_FILTERCOEFF_8 0x03
#define QMP6988_FILTERCOEFF_16 0x04
#define QMP6988_FILTERCOEFF_32 0x05

#define QMP6988_CONFIG_REG 0xF1 /*IIR filter co-efficient setting Register*/
#define QMP6988_CONFIG_REG_FILTER__POS 0
#define QMP6988_CONFIG_REG_FILTER__MSK 0x07
#define QMP6988_CONFIG_REG_FILTER__LEN 3

#define SUBTRACTOR 8388608

typedef struct _qmp6988_cali_data
{
    QMP6988_S32_t COE_a0;
    QMP6988_S16_t COE_a1;
    QMP6988_S16_t COE_a2;
    QMP6988_S32_t COE_b00;
    QMP6988_S16_t COE_bt1;
    QMP6988_S16_t COE_bt2;
    QMP6988_S16_t COE_bp1;
    QMP6988_S16_t COE_b11;
    QMP6988_S16_t COE_bp2;
    QMP6988_S16_t COE_b12;
    QMP6988_S16_t COE_b21;
    QMP6988_S16_t COE_bp3;
} qmp6988_cali_data_t;

typedef struct _qmp6988_fk_data
{
    float a0, b00;
    float a1, a2, bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_fk_data_t;

typedef struct _qmp6988_ik_data
{
    QMP6988_S32_t a0, b00;
    QMP6988_S32_t a1, a2;
    QMP6988_S64_t bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
} qmp6988_ik_data_t;

typedef struct _qmp6988_data
{
    uint8_t slave;
    uint8_t chip_id;
    uint8_t power_mode;
    float temperature;
    float pressure;
    float altitude;
    qmp6988_cali_data_t qmp6988_cali;
    qmp6988_ik_data_t ik;
} qmp6988_data_t;

static qmp6988_data_t qmp6988;

static int
QMP6988_getCalibrationData(I2CDevice_t slave)
{
    esp_err_t err = 0;
    // BITFIELDS temp_COE;
    uint8_t a_data_uint8_tr[QMP6988_CALIBRATION_DATA_LENGTH] = { 0 };
    int len;

    for (len = 0; len < QMP6988_CALIBRATION_DATA_LENGTH; len += 1) {
        err =
          Core2ForAWS_Port_A_I2C_Read(slave, QMP6988_CALIBRATION_DATA_START + len, &a_data_uint8_tr[len], 1);
        if (err) {
            ESP_LOGI(TAG, "qmp6988 read 0xA0 error!");
            return 0;
        }
    }

    qmp6988.qmp6988_cali.COE_a0 =
      (QMP6988_S32_t)(((a_data_uint8_tr[18] << SHIFT_LEFT_12_POSITION) |
                       (a_data_uint8_tr[19] << SHIFT_LEFT_4_POSITION) | (a_data_uint8_tr[24] & 0x0f))
                      << 12);
    qmp6988.qmp6988_cali.COE_a0 = qmp6988.qmp6988_cali.COE_a0 >> 12;

    qmp6988.qmp6988_cali.COE_a1 =
      (QMP6988_S16_t)(((a_data_uint8_tr[20]) << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[21]);
    qmp6988.qmp6988_cali.COE_a2 =
      (QMP6988_S16_t)(((a_data_uint8_tr[22]) << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[23]);

    qmp6988.qmp6988_cali.COE_b00 = (QMP6988_S32_t)(((a_data_uint8_tr[0] << SHIFT_LEFT_12_POSITION) |
                                                    (a_data_uint8_tr[1] << SHIFT_LEFT_4_POSITION) |
                                                    ((a_data_uint8_tr[24] & 0xf0) >> SHIFT_RIGHT_4_POSITION))
                                                   << 12);
    qmp6988.qmp6988_cali.COE_b00 = qmp6988.qmp6988_cali.COE_b00 >> 12;

    qmp6988.qmp6988_cali.COE_bt1 =
      (QMP6988_S16_t)(((a_data_uint8_tr[2]) << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[3]);
    qmp6988.qmp6988_cali.COE_bt2 =
      (QMP6988_S16_t)(((a_data_uint8_tr[4]) << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[5]);
    qmp6988.qmp6988_cali.COE_bp1 =
      (QMP6988_S16_t)(((a_data_uint8_tr[6]) << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[7]);
    qmp6988.qmp6988_cali.COE_b11 =
      (QMP6988_S16_t)(((a_data_uint8_tr[8]) << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[9]);
    qmp6988.qmp6988_cali.COE_bp2 =
      (QMP6988_S16_t)(((a_data_uint8_tr[10]) << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[11]);
    qmp6988.qmp6988_cali.COE_b12 =
      (QMP6988_S16_t)(((a_data_uint8_tr[12]) << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[13]);
    qmp6988.qmp6988_cali.COE_b21 =
      (QMP6988_S16_t)(((a_data_uint8_tr[14]) << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[15]);
    qmp6988.qmp6988_cali.COE_bp3 =
      (QMP6988_S16_t)(((a_data_uint8_tr[16]) << SHIFT_LEFT_8_POSITION) | a_data_uint8_tr[17]);

    ESP_LOGI(TAG, "<-----------calibration data-------------->\r\n");
    ESP_LOGI(TAG,
             "COE_a0[%d]	COE_a1[%d]	COE_a2[%d]	COE_b00[%d]\r\n",
             qmp6988.qmp6988_cali.COE_a0,
             qmp6988.qmp6988_cali.COE_a1,
             qmp6988.qmp6988_cali.COE_a2,
             qmp6988.qmp6988_cali.COE_b00);
    ESP_LOGI(TAG,
             "COE_bt1[%d]	COE_bt2[%d]	COE_bp1[%d]	COE_b11[%d]\r\n",
             qmp6988.qmp6988_cali.COE_bt1,
             qmp6988.qmp6988_cali.COE_bt2,
             qmp6988.qmp6988_cali.COE_bp1,
             qmp6988.qmp6988_cali.COE_b11);
    ESP_LOGI(TAG,
             "COE_bp2[%d]	COE_b12[%d]	COE_b21[%d]	COE_bp3[%d]\r\n",
             qmp6988.qmp6988_cali.COE_bp2,
             qmp6988.qmp6988_cali.COE_b12,
             qmp6988.qmp6988_cali.COE_b21,
             qmp6988.qmp6988_cali.COE_bp3);
    ESP_LOGI(TAG, "<-----------calibration data-------------->\r\n");

    qmp6988.ik.a0 = qmp6988.qmp6988_cali.COE_a0;   // 20Q4
    qmp6988.ik.b00 = qmp6988.qmp6988_cali.COE_b00; // 20Q4

    qmp6988.ik.a1 = 3608L * (QMP6988_S32_t)qmp6988.qmp6988_cali.COE_a1 - 1731677965L; // 31Q23
    qmp6988.ik.a2 = 16889L * (QMP6988_S32_t)qmp6988.qmp6988_cali.COE_a2 - 87619360L;  // 30Q47

    qmp6988.ik.bt1 = 2982L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bt1 + 107370906L;   // 28Q15
    qmp6988.ik.bt2 = 329854L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bt2 + 108083093L; // 34Q38
    qmp6988.ik.bp1 = 19923L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bp1 + 1133836764L; // 31Q20
    qmp6988.ik.b11 = 2406L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_b11 + 118215883L;   // 28Q34
    qmp6988.ik.bp2 = 3079L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bp2 - 181579595L;   // 29Q43
    qmp6988.ik.b12 = 6846L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_b12 + 85590281L;    // 29Q53
    qmp6988.ik.b21 = 13836L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_b21 + 79333336L;   // 29Q60
    qmp6988.ik.bp3 = 2915L * (QMP6988_S64_t)qmp6988.qmp6988_cali.COE_bp3 + 157155561L;   // 28Q65
    ESP_LOGI(TAG, "<----------- int calibration data -------------->\r\n");
    ESP_LOGI(
      TAG, "a0[%d]	a1[%d] a2[%d] b00[%d]\r\n", qmp6988.ik.a0, qmp6988.ik.a1, qmp6988.ik.a2, qmp6988.ik.b00);
    ESP_LOGI(TAG,
             "bt1[%lld]	bt2[%lld]	bp1[%lld]	b11[%lld]\r\n",
             qmp6988.ik.bt1,
             qmp6988.ik.bt2,
             qmp6988.ik.bp1,
             qmp6988.ik.b11);
    ESP_LOGI(TAG,
             "bp2[%lld]	b12[%lld]	b21[%lld]	bp3[%lld]\r\n",
             qmp6988.ik.bp2,
             qmp6988.ik.b12,
             qmp6988.ik.b21,
             qmp6988.ik.bp3);
    ESP_LOGI(TAG, "<----------- int calibration data -------------->\r\n");
    return 1;
}

static void
QMP6988_setpPowermode(I2CDevice_t slave, int power_mode)
{
    uint8_t data;

    ESP_LOGI(TAG, "qmp_set_powermode %d \r\n", power_mode);

    qmp6988.power_mode = power_mode;
    esp_err_t err = Core2ForAWS_Port_A_I2C_Read(slave, QMP6988_CTRLMEAS_REG, &data, 1);
    if (err) {
        ESP_LOGI(TAG, "qmp6988 read 0xA0 error!");
        return;
    }

    data = data & 0xfc;
    if (power_mode == QMP6988_SLEEP_MODE) {
        data |= 0x00;
    } else if (power_mode == QMP6988_FORCED_MODE) {
        data |= 0x01;
    } else if (power_mode == QMP6988_NORMAL_MODE) {
        data |= 0x03;
    }

    err = Core2ForAWS_Port_A_I2C_Write(slave, QMP6988_CTRLMEAS_REG, &data, 1);
    if (err) {
        ESP_LOGI(TAG, "qmp6988 write power mode failed: %d", err);
        return;
    }
    ESP_LOGI(TAG, "qmp_set_powermode 0xf4=0x%x \r\n", data);

    vTaskDelay(pdMS_TO_TICKS(20));
}

static void
QMP6988_setFilter(I2CDevice_t slave, unsigned char filter)
{
    uint8_t data;

    data = (filter & 0x03);
    const esp_err_t err = Core2ForAWS_Port_A_I2C_Write(slave, QMP6988_CONFIG_REG, &data, 1);
    if (err) {
        ESP_LOGI(TAG, "QMP6988_setFilter failed: %d", err);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
}

static void
QMP6988_setOversamplingP(I2CDevice_t slave, unsigned char oversampling_p)
{
    uint8_t data;

    esp_err_t err = Core2ForAWS_Port_A_I2C_Read(slave, QMP6988_CTRLMEAS_REG, &data, 1);
    if (err) {
        ESP_LOGI(TAG, "QMP6988_setOversamplingP read error: %d", err);
        return;
    }
    data &= 0xe3;
    data |= (oversampling_p << 2);
    err = Core2ForAWS_Port_A_I2C_Write(slave, QMP6988_CTRLMEAS_REG, &data, 1);
    if (err) {
        ESP_LOGI(TAG, "QMP6988_setOversamplingP write error: %d", err);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
}

static void
QMP6988_setOversamplingT(I2CDevice_t slave, unsigned char oversampling_t)
{
    uint8_t data;

    esp_err_t err = Core2ForAWS_Port_A_I2C_Read(slave, QMP6988_CTRLMEAS_REG, &data, 1);
    if (err) {
        ESP_LOGI(TAG, "QMP6988_setOversamplingT read error: %d", err);
        return;
    }
    data &= 0x1f;
    data |= (oversampling_t << 5);
    err = Core2ForAWS_Port_A_I2C_Write(slave, QMP6988_CTRLMEAS_REG, &data, 1);
    if (err) {
        ESP_LOGI(TAG, "QMP6988_setOversamplingT write error: %d", err);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
}

I2CDevice_t
QMP6988_deviceCheck()
{
    static byte reset_cmd[] = { 0xe6, 0x00 };

    uint8_t slave_dev_list[2] = { QMP6988_SLAVE_ADDRESS_L, QMP6988_SLAVE_ADDRESS_H };

    for (uint8_t i = 0; i < 2; i++) {
        I2CDevice_t slave = Core2ForAWS_Port_A_I2C_Begin(slave_dev_list[i], PORT_A_I2C_STANDARD_BAUD);
        if (!slave) {
            ESP_LOGI(TAG, "Failed to open QMP6988 chip: %d", slave_dev_list[i]);
            continue;
        }

        uint8_t qmp6988_chip_id = 0;
        esp_err_t err = Core2ForAWS_Port_A_I2C_Read(slave, QMP6988_CHIP_ID_REG, &qmp6988_chip_id, 1);
        if (err) {
            ESP_LOGI(TAG, "%s: read 0xD1 failed", __func__);
            Core2ForAWS_Port_A_I2C_Close(slave);
            continue;
        }
        ESP_LOGI(TAG, "qmp6988 read chip id = 0x%x", qmp6988_chip_id);
        if (qmp6988_chip_id == QMP6988_CHIP_ID) {
            // Execute software reset to prepare chip
            err = Core2ForAWS_Port_A_I2C_Write(slave, QMP6988_RESET_REG, &reset_cmd[0], 1);
            if (err) {
                ESP_LOGI(TAG, "Failed to reset QMP6988 chip (0)");
                Core2ForAWS_Port_A_I2C_Close(slave);
                return NULL;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            err = Core2ForAWS_Port_A_I2C_Write(slave, QMP6988_RESET_REG, &reset_cmd[1], 1);
            if (err) {
                ESP_LOGI(TAG, "Failed to reset QMP6988 chip (1)");
                Core2ForAWS_Port_A_I2C_Close(slave);
                return NULL;
            }

            QMP6988_getCalibrationData(slave);
            QMP6988_setpPowermode(slave, QMP6988_NORMAL_MODE);
            QMP6988_setFilter(slave, QMP6988_FILTERCOEFF_4);
            QMP6988_setOversamplingP(slave, QMP6988_OVERSAMPLING_8X);
            QMP6988_setOversamplingT(slave, QMP6988_OVERSAMPLING_1X);

            return slave;
        } else {
            Core2ForAWS_Port_A_I2C_Close(slave);
        }
    }

    return NULL;
}

QMP6988_S16_t
QMP6988_convTx02e(qmp6988_ik_data_t* ik, QMP6988_S32_t dt)
{
    QMP6988_S16_t ret;
    QMP6988_S64_t wk1, wk2;

    // wk1: 60Q4 // bit size
    wk1 = ((QMP6988_S64_t)ik->a1 * (QMP6988_S64_t)dt);       // 31Q23+24-1=54 (54Q23)
    wk2 = ((QMP6988_S64_t)ik->a2 * (QMP6988_S64_t)dt) >> 14; // 30Q47+24-1=53 (39Q33)
    wk2 = (wk2 * (QMP6988_S64_t)dt) >> 10;                   // 39Q33+24-1=62 (52Q23)
    wk2 = ((wk1 + wk2) / 32767) >> 19;                       // 54,52->55Q23 (20Q04)
    ret = (QMP6988_S16_t)((ik->a0 + wk2) >> 4);              // 21Q4 -> 17Q0
    return ret;
}

QMP6988_S32_t
QMP6988_getPressure02e(qmp6988_ik_data_t* ik, QMP6988_S32_t dp, QMP6988_S16_t tx)
{
    QMP6988_S32_t ret;
    QMP6988_S64_t wk1, wk2, wk3;

    // wk1 = 48Q16 // bit size
    wk1 = ((QMP6988_S64_t)ik->bt1 * (QMP6988_S64_t)tx);       // 28Q15+16-1=43 (43Q15)
    wk2 = ((QMP6988_S64_t)ik->bp1 * (QMP6988_S64_t)dp) >> 5;  // 31Q20+24-1=54 (49Q15)
    wk1 += wk2;                                               // 43,49->50Q15
    wk2 = ((QMP6988_S64_t)ik->bt2 * (QMP6988_S64_t)tx) >> 1;  // 34Q38+16-1=49 (48Q37)
    wk2 = (wk2 * (QMP6988_S64_t)tx) >> 8;                     // 48Q37+16-1=63 (55Q29)
    wk3 = wk2;                                                // 55Q29
    wk2 = ((QMP6988_S64_t)ik->b11 * (QMP6988_S64_t)tx) >> 4;  // 28Q34+16-1=43 (39Q30)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;                     // 39Q30+24-1=62 (61Q29)
    wk3 += wk2;                                               // 55,61->62Q29
    wk2 = ((QMP6988_S64_t)ik->bp2 * (QMP6988_S64_t)dp) >> 13; // 29Q43+24-1=52 (39Q30)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;                     // 39Q30+24-1=62 (61Q29)
    wk3 += wk2;                                               // 62,61->63Q29
    wk1 += wk3 >> 14;                                         // Q29 >> 14 -> Q15
    wk2 = ((QMP6988_S64_t)ik->b12 * (QMP6988_S64_t)tx);       // 29Q53+16-1=45 (45Q53)
    wk2 = (wk2 * (QMP6988_S64_t)tx) >> 22;                    // 45Q53+16-1=61 (39Q31)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;                     // 39Q31+24-1=62 (61Q30)
    wk3 = wk2;                                                // 61Q30
    wk2 = ((QMP6988_S64_t)ik->b21 * (QMP6988_S64_t)tx) >> 6;  // 29Q60+16-1=45 (39Q54)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 23;                    // 39Q54+24-1=62 (39Q31)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 1;                     // 39Q31+24-1=62 (61Q20)
    wk3 += wk2;                                               // 61,61->62Q30
    wk2 = ((QMP6988_S64_t)ik->bp3 * (QMP6988_S64_t)dp) >> 12; // 28Q65+24-1=51 (39Q53)
    wk2 = (wk2 * (QMP6988_S64_t)dp) >> 23;                    // 39Q53+24-1=62 (39Q30)
    wk2 = (wk2 * (QMP6988_S64_t)dp);                          // 39Q30+24-1=62 (62Q30)
    wk3 += wk2;                                               // 62,62->63Q30
    wk1 += wk3 >> 15;                                         // Q30 >> 15 = Q15
    wk1 /= 32767L;
    wk1 >>= 11;     // Q15 >> 7 = Q4
    wk1 += ik->b00; // Q4 + 20Q4
    // wk1 >>= 4; // 28Q4 -> 24Q0
    ret = (QMP6988_S32_t)wk1;
    return ret;
}

float
QMP6988_calcPressure(I2CDevice_t slave)
{
    QMP6988_U32_t P_read, T_read;
    QMP6988_S32_t P_raw, T_raw;
    uint8_t a_data_uint8_tr[6] = { 0 };
    QMP6988_S32_t T_int, P_int;

    // press
    esp_err_t err = Core2ForAWS_Port_A_I2C_Read(slave, QMP6988_PRESSURE_MSB_REG, &a_data_uint8_tr[0], 6);
    if (err) {
        ESP_LOGI(TAG, "QMP6988_calcPressure failed to read: %d", err);
        return 0.0f;
    }
    P_read = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_uint8_tr[0])) << SHIFT_LEFT_16_POSITION) |
                             (((QMP6988_U16_t)(a_data_uint8_tr[1])) << SHIFT_LEFT_8_POSITION) |
                             (a_data_uint8_tr[2]));
    P_raw = (QMP6988_S32_t)(P_read - SUBTRACTOR);

    T_read = (QMP6988_U32_t)((((QMP6988_U32_t)(a_data_uint8_tr[3])) << SHIFT_LEFT_16_POSITION) |
                             (((QMP6988_U16_t)(a_data_uint8_tr[4])) << SHIFT_LEFT_8_POSITION) |
                             (a_data_uint8_tr[5]));
    T_raw = (QMP6988_S32_t)(T_read - SUBTRACTOR);

    T_int = QMP6988_convTx02e(&(qmp6988.ik), T_raw);
    P_int = QMP6988_getPressure02e(&(qmp6988.ik), P_raw, T_int);
    qmp6988.temperature = (float)T_int / 256.0f;
    qmp6988.pressure = (float)P_int / 16.0f;

    return qmp6988.pressure;
}

/**
 *  Handling of SHT3x temperature and humidity sensor
 */
const uint8_t SHT3x_DEVICE_ADDRESS = 0x44;

SHT3xMeasurement
SHT3x_get_measurement(I2CDevice_t sht3x_peripheral)
{
    static byte measurement_command[] = { 0x2C, 0x06 };
    byte temp_data[6] = {};

    SHT3xMeasurement sht3x_measurement = { .temperature = 0.0f, .humidity = 0.0f };
    esp_err_t err = Core2ForAWS_Port_A_I2C_Write(
      sht3x_peripheral, I2C_NO_REG, &measurement_command[0], sizeof(measurement_command));
    if (err) {
        ESP_LOGE(TAG, "Failed to write measurement command — %d", err);
        return sht3x_measurement;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    err = Core2ForAWS_Port_A_I2C_Read(sht3x_peripheral, I2C_NO_REG, &temp_data[0], sizeof(temp_data));
    if (err) {
        ESP_LOGE(TAG, "Failed to read temperature data — %d", err);
        return sht3x_measurement;
    }

    // Read 6 bytes of data and convert
    // cTemp msb, cTemp lsb, cTemp crc, humidity msb, humidity lsb, humidity crc
    sht3x_measurement.temperature = ((((temp_data[0] * 256.0) + temp_data[1]) * 175) / 65535.0) - 45;
    sht3x_measurement.humidity = ((((temp_data[3] * 256.0) + temp_data[4]) * 100) / 65535.0);
    /* ESP_LOGI(TAG, "READ from A for address 0x44 - DONE, temp: %.1f, humidity: %.1f", temperature,
     * humidity); */
    return sht3x_measurement;
}
