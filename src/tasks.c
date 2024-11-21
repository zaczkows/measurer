#include "tasks.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"

TaskHandle_t clock_handle;
TaskHandle_t power_handle;

void
clock_task(void* pvParameters)
{
    lv_style_t labels_style;
    lv_style_init(&labels_style);
    lv_style_set_text_color(&labels_style, LV_STATE_DEFAULT, LV_COLOR_WHITE);

    lv_obj_t* core2forAWS_screen_obj = (lv_obj_t*)pvParameters;
    lv_obj_t* time_label = lv_label_create(core2forAWS_screen_obj, NULL);
    lv_label_set_static_text(time_label, "01.01.2021, 00:00:00");
    lv_label_set_align(time_label, LV_LABEL_ALIGN_CENTER);
    lv_obj_align(time_label, NULL, LV_ALIGN_IN_TOP_MID, 4, 10);
    lv_obj_add_style(time_label, LV_LABEL_PART_MAIN, &labels_style);

    rtc_date_t datetime;
    char clock_buf[32];
    for (;;) {
        BM8563_GetTime(&datetime);
        snprintf(clock_buf,
                 sizeof(clock_buf),
                 "%02d.%02d.%04d, %02d:%02d:%02d",
                 datetime.day,
                 datetime.month,
                 datetime.year,
                 datetime.hour,
                 datetime.minute,
                 datetime.second);
        lv_label_set_static_text(time_label, clock_buf);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL); // Should never get to here...
}

void
battery_task(void* pvParameters)
{
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t* battery_label = lv_label_create((lv_obj_t*)pvParameters, NULL);
    lv_label_set_static_text(battery_label, LV_SYMBOL_BATTERY_FULL);
    lv_label_set_recolor(battery_label, true);
    lv_label_set_align(battery_label, LV_LABEL_ALIGN_CENTER);
    lv_obj_align(battery_label, (lv_obj_t*)pvParameters, LV_ALIGN_IN_TOP_RIGHT, -20, 10);
    lv_obj_t* charge_label = lv_label_create(battery_label, NULL);
    lv_label_set_recolor(charge_label, true);
    lv_label_set_static_text(charge_label, "");
    lv_obj_align(charge_label, battery_label, LV_ALIGN_CENTER, -4, 0);
    xSemaphoreGive(xGuiSemaphore);

    for (;;) {
        const float battery_voltage = Core2ForAWS_PMU_GetBatVolt();
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        if (battery_voltage >= 4.100) {
            lv_label_set_static_text(battery_label, "#0ab300 " LV_SYMBOL_BATTERY_FULL "#");
        } else if (battery_voltage >= 3.95) {
            lv_label_set_static_text(battery_label, "#0ab300 " LV_SYMBOL_BATTERY_3 "#");
        } else if (battery_voltage >= 3.80) {
            lv_label_set_static_text(battery_label, "#ff9900 " LV_SYMBOL_BATTERY_2 "#");
        } else if (battery_voltage >= 3.25) {
            lv_label_set_static_text(battery_label, "#ff0000 " LV_SYMBOL_BATTERY_1 "#");
        } else {
            lv_label_set_static_text(battery_label, "#ff0000 " LV_SYMBOL_BATTERY_EMPTY "#");
        }

        if (Core2ForAWS_PMU_GetBatCurrent() >= 0.00) {
            lv_label_set_static_text(charge_label, "#0000cc " LV_SYMBOL_CHARGE "#");
        } else {
            lv_label_set_static_text(charge_label, "");
        }
        xSemaphoreGive(xGuiSemaphore);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelete(NULL); // Should never get to here...
}

void
start_backgound_tasks(lv_obj_t* core2forAWS_screen_obj)
{
    xTaskCreatePinnedToCore(clock_task,
                            "clockTask",
                            configMINIMAL_STACK_SIZE * 3,
                            (void*)core2forAWS_screen_obj,
                            0,
                            &clock_handle,
                            1);
    xTaskCreatePinnedToCore(battery_task,
                            "batteryTask",
                            configMINIMAL_STACK_SIZE * 2,
                            (void*)core2forAWS_screen_obj,
                            0,
                            &power_handle,
                            1);
}
