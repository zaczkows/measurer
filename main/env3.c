#include "env3.h"
#include "env3_sensors.h"
#include "tof_vl53lox.h"

#include "driver/i2c.h"

static const char* TAG = ENV3_TAB_NAME;
lv_obj_t* temperature_label = NULL;
lv_obj_t* humidity_label = NULL;
lv_obj_t* pressure_label = NULL;
lv_obj_t* tof_label = NULL;

static SensorInfo SENSOR_INFO = { .temperature = 0.0f, .humidity = 0.0f, .pressure = 0.0f };
SemaphoreHandle_t SENSOR_INFO_MUTEX;

SensorInfo
get_sensor_info(void)
{
    xSemaphoreTake(SENSOR_INFO_MUTEX, portMAX_DELAY);
    SensorInfo si = SENSOR_INFO;
    xSemaphoreGive(SENSOR_INFO_MUTEX);
    return si;
}

static void
env3_task(void* pvParameters)
{
    ESP_LOGI(TAG, "Start ENV III task");
    esp_err_t err = Core2ForAWS_Port_PinMode(PORT_A_SDA_PIN, I2C);
    if (err != ESP_OK) {
        const char* I2C_SENSOR_FAILED = "I2C Sensor failed";
        lv_label_set_static_text(temperature_label, I2C_SENSOR_FAILED);
        ESP_LOGE(TAG, "Failed to enable I2C port");
        vTaskDelete(NULL);
        return;
    }

    char text_temp_buffer[32];
    char text_humidity_buffer[32];
    char text_pressure_buffer[32];
    char text_tof_buffer[32];

    I2CDevice_t sht3x_peripheral =
      Core2ForAWS_Port_A_I2C_Begin(SHT3x_DEVICE_ADDRESS, PORT_A_I2C_STANDARD_BAUD);
    I2CDevice_t qmp6988_slave = QMP6988_deviceCheck();
    I2CDevice_t tof_vl53lox = TofVl53lox_Init();

    uint16_t tof_reading = 0;
    while (true) {
        const SHT3xMeasurement sht3x_measurement = SHT3x_get_measurement(sht3x_peripheral);
        float pressure = QMP6988_calcPressure(qmp6988_slave);
        if (pressure > 0.0f) {
            pressure /= 100.0f; // hPa are the norm
        }
        const uint16_t tof_value = TofVl53lox_get_reading(tof_vl53lox);
        if (tof_value > 0) {
            tof_reading = tof_value;
        }

        xSemaphoreTake(SENSOR_INFO_MUTEX, portMAX_DELAY);
        SENSOR_INFO.temperature = sht3x_measurement.temperature;
        SENSOR_INFO.humidity = sht3x_measurement.humidity;
        SENSOR_INFO.pressure = pressure;
        xSemaphoreGive(SENSOR_INFO_MUTEX);

        snprintf(
          text_temp_buffer, sizeof(text_temp_buffer), "Temperature: %.1f°C", sht3x_measurement.temperature);
        snprintf(
          text_humidity_buffer, sizeof(text_humidity_buffer), "Humidity: %.1f%%", sht3x_measurement.humidity);
        snprintf(text_pressure_buffer, sizeof(text_pressure_buffer), "Pressure: %.1f hPa", pressure);
        snprintf(text_tof_buffer, sizeof(text_tof_buffer), "TOF distance: %u mm", tof_reading);

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        lv_label_set_static_text(temperature_label, text_temp_buffer);
        lv_label_set_static_text(humidity_label, text_humidity_buffer);
        lv_label_set_static_text(pressure_label, text_pressure_buffer);
        lv_label_set_static_text(tof_label, text_tof_buffer);
        xSemaphoreGive(xGuiSemaphore);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    Core2ForAWS_Port_A_I2C_Close(qmp6988_slave);
    Core2ForAWS_Port_A_I2C_Close(sht3x_peripheral);
    vTaskDelete(NULL); // Should never get to here...
}

void
display_env3_tab(lv_obj_t* tv, lv_obj_t* core2forAWS_screen_obj)
{
    SENSOR_INFO_MUTEX = xSemaphoreCreateMutex();
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);

    lv_obj_t* env3_tab = lv_tabview_add_tab(tv, ENV3_TAB_NAME); // Create a tab
    /* Create the main body object and set background within the tab*/
    static lv_style_t bg_style;
    lv_obj_t* env3_bg = lv_obj_create(env3_tab, NULL);
    lv_obj_align(env3_bg, NULL, LV_ALIGN_IN_TOP_LEFT, 16, 36);
    lv_obj_set_size(env3_bg, 290, 190);
    lv_obj_set_click(env3_bg, false);
    lv_style_init(&bg_style);
    lv_style_set_bg_color(&bg_style, LV_STATE_DEFAULT, lv_color_make(169, 0, 103));
    lv_obj_add_style(env3_bg, LV_OBJ_PART_MAIN, &bg_style);

    /* Create the title within the main body object */
    static lv_style_t title_style;
    lv_style_init(&title_style);
    lv_style_set_text_font(&title_style, LV_STATE_DEFAULT, LV_THEME_DEFAULT_FONT_TITLE);
    lv_style_set_text_color(&title_style, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_t* tab_title_label = lv_label_create(env3_bg, NULL);
    lv_obj_add_style(tab_title_label, LV_OBJ_PART_MAIN, &title_style);
    lv_label_set_static_text(tab_title_label, "ENV III M5Stack Sensor");
    lv_obj_align(tab_title_label, env3_bg, LV_ALIGN_IN_TOP_MID, 0, 10);

    /* Create the sensor information label object */
    lv_obj_t* body_label = lv_label_create(env3_bg, NULL);
    lv_label_set_long_mode(body_label, LV_LABEL_LONG_BREAK);
    lv_label_set_static_text(body_label,
                             "Combined ENV III sensor, which cosists "
                             "of two sensors: SHT30 and QMP6988");
    lv_obj_set_width(body_label, 252);
    lv_obj_align(body_label, env3_bg, LV_ALIGN_IN_TOP_LEFT, 20, 30);

    static lv_style_t body_style;
    lv_style_init(&body_style);
    lv_style_set_text_color(&body_style, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_add_style(body_label, LV_OBJ_PART_MAIN, &body_style);

    static lv_style_t labels_style;
    lv_style_init(&labels_style);
    lv_style_set_text_color(&labels_style, LV_STATE_DEFAULT, LV_COLOR_YELLOW);

    // Label for temperature
    temperature_label = lv_label_create(env3_bg, NULL);
    lv_label_set_align(temperature_label, LV_LABEL_ALIGN_LEFT);
    lv_obj_add_style(temperature_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_obj_align(temperature_label, body_label, LV_ALIGN_IN_TOP_LEFT, 20, 70);

    // Label for humidity
    humidity_label = lv_label_create(env3_bg, NULL);
    lv_label_set_align(humidity_label, LV_LABEL_ALIGN_LEFT);
    lv_obj_add_style(humidity_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_obj_align(humidity_label, body_label, LV_ALIGN_IN_TOP_LEFT, 20, 90);

    // Label for pressure
    pressure_label = lv_label_create(env3_bg, NULL);
    lv_label_set_align(pressure_label, LV_LABEL_ALIGN_LEFT);
    lv_obj_add_style(pressure_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_obj_align(pressure_label, body_label, LV_ALIGN_IN_TOP_LEFT, 20, 110);

    // Label for TOF (distance)
    tof_label = lv_label_create(env3_bg, NULL);
    lv_label_set_align(tof_label, LV_LABEL_ALIGN_LEFT);
    lv_obj_add_style(tof_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_obj_align(tof_label, body_label, LV_ALIGN_IN_TOP_LEFT, 20, 130);

    xSemaphoreGive(xGuiSemaphore);

    // Start the task
    xTaskCreate(env3_task, "env3Task", 4096, NULL, 0, &env3_handle);
}
