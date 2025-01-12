/*
 * AWS IoT EduKit - Core2 for AWS IoT EduKit
 * Factory Firmware v2.2.0
 * power.c
 *
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "power.h"

static const char* TAG = POWER_TAB_NAME;
lv_obj_t* brightness_label = NULL;
lv_obj_t* brightness_slider = NULL;
lv_obj_t* battery_label = NULL;
lv_obj_t* pir_sensor_label = NULL;

const uint8_t lowest_brightness = 30;
uint8_t the_brightness = 50;

// Caller *must* hold the GUI semaphore
static void
brightness_updater(uint8_t brightness)
{
    static char brightness_text[32];
    Core2ForAWS_Display_SetBrightness(brightness);
    snprintf(brightness_text, sizeof(brightness_text), "Screen brightness: %d", brightness);
    lv_label_set_static_text(brightness_label, brightness_text);
}

static void
brightness_slider_event_cb(lv_obj_t* slider, lv_event_t event)
{
    if (event == LV_EVENT_VALUE_CHANGED) {
        the_brightness = lv_slider_get_value(slider);
        brightness_updater(the_brightness);
    }
}

static void
pir_sensor_task(void* pvParameters)
{
    ESP_LOGI(TAG, "Start ENV III task");
    esp_err_t err = Core2ForAWS_Port_PinMode(GPIO_NUM_36, ADC);
    if (err != ESP_OK) {
        const char* PIR_SENSOR_FAILED = "PIR sensor failed";
        lv_label_set_static_text(pir_sensor_label, PIR_SENSOR_FAILED);
        ESP_LOGE(TAG, "Failed to enable ADC port");
        vTaskDelete(NULL);
        return;
    }

    char pir_sensor_text[32];
    char battery_text[32];

    while (true) {
        const uint32_t adc_sensor_value = Core2ForAWS_Port_B_ADC_ReadRaw();
        const float bat_V = Core2ForAWS_PMU_GetBatVolt();
        const float bat_A = Core2ForAWS_PMU_GetBatCurrent();

        snprintf(battery_text, sizeof(battery_text), "Battery status: %.2fV, %.1fmA", bat_V, bat_A);
        snprintf(pir_sensor_text, sizeof(pir_sensor_text), "PIR sensor: %u", adc_sensor_value);

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        lv_label_set_static_text(pir_sensor_label, pir_sensor_text);
        if (adc_sensor_value < 100 && bat_V < 4.0 && bat_A < 0.0f) {
            lv_slider_set_value(brightness_slider, lowest_brightness, LV_ANIM_OFF);
            brightness_updater(lowest_brightness);
        } else {
            lv_slider_set_value(brightness_slider, the_brightness, LV_ANIM_OFF);
            brightness_updater(the_brightness);
        }

        lv_label_set_static_text(battery_label, battery_text);
        xSemaphoreGive(xGuiSemaphore);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL); // Should never get to here...
}

void
display_power_tab(lv_obj_t* tv)
{
    ESP_LOGI(TAG, "Starting display/battery tab");
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);

    lv_obj_t* power_tab = lv_tabview_add_tab(tv, POWER_TAB_NAME); // Create a tab

    /* Create the main body object and set background within the tab*/
    static lv_style_t bg_style;
    lv_obj_t* power_bg = lv_obj_create(power_tab, NULL);
    lv_obj_align(power_bg, NULL, LV_ALIGN_IN_TOP_LEFT, 16, 36);
    lv_obj_set_size(power_bg, 290, 190);
    lv_obj_set_click(power_bg, false);
    lv_style_init(&bg_style);
    lv_style_set_bg_color(&bg_style, LV_STATE_DEFAULT, lv_color_make(255, 97, 56));
    lv_obj_add_style(power_bg, LV_OBJ_PART_MAIN, &bg_style);

    /* Create the title within the main body object */
    static lv_style_t title_style;
    lv_style_init(&title_style);
    lv_style_set_text_font(&title_style, LV_STATE_DEFAULT, LV_THEME_DEFAULT_FONT_TITLE);
    lv_style_set_text_color(&title_style, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_t* tab_title_label = lv_label_create(power_bg, NULL);
    lv_obj_add_style(tab_title_label, LV_OBJ_PART_MAIN, &title_style);
    lv_label_set_static_text(tab_title_label, "AXP192 Power Mgmt");
    lv_obj_align(tab_title_label, power_bg, LV_ALIGN_IN_TOP_MID, 0, 10);

    /* Create the sensor information label object */
    lv_obj_t* body_label = lv_label_create(power_bg, NULL);
    lv_label_set_long_mode(body_label, LV_LABEL_LONG_BREAK);
    lv_label_set_static_text(body_label,
                             "The AXP192 provides power management for the battery and on-board peripherals");
    lv_obj_set_width(body_label, 252);
    lv_obj_align(body_label, power_bg, LV_ALIGN_IN_TOP_LEFT, 10, 30);

    static lv_style_t body_style;
    lv_style_init(&body_style);
    lv_style_set_text_color(&body_style, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_add_style(body_label, LV_OBJ_PART_MAIN, &body_style);

    static lv_style_t labels_style;
    lv_style_init(&labels_style);
    lv_style_set_text_color(&labels_style, LV_STATE_DEFAULT, LV_COLOR_BLACK);

    // Screen brightness
    brightness_label = lv_label_create(power_bg, NULL);
    lv_obj_align(brightness_label, body_label, LV_ALIGN_OUT_BOTTOM_LEFT, 5, 5);
    lv_obj_add_style(brightness_label, LV_LABEL_PART_MAIN, &labels_style);
    brightness_updater(the_brightness);

    brightness_slider = lv_slider_create(power_bg, NULL);
    lv_obj_set_width(brightness_slider, 180);
    lv_obj_align(brightness_slider, brightness_label, LV_ALIGN_OUT_BOTTOM_LEFT, 5, 5);
    lv_obj_set_event_cb(brightness_slider, brightness_slider_event_cb);
    lv_slider_set_value(brightness_slider, the_brightness, LV_ANIM_OFF);
    lv_slider_set_range(brightness_slider, lowest_brightness, 100);

    // Battery level
    battery_label = lv_label_create(power_bg, NULL);
    lv_obj_align(battery_label, brightness_slider, LV_ALIGN_OUT_BOTTOM_LEFT, -5, 5);
    lv_obj_add_style(battery_label, LV_LABEL_PART_MAIN, &labels_style);
    /* battery_updater(); */

    // PIR sensor
    pir_sensor_label = lv_label_create(power_bg, NULL);
    lv_obj_align(pir_sensor_label, battery_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);
    lv_obj_add_style(pir_sensor_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_label_set_static_text(pir_sensor_label, "Motion sensor (PIR): ?");

    xSemaphoreGive(xGuiSemaphore);

    xTaskCreate(pir_sensor_task, "PirTask", 4096, NULL, 0, &pir_sensor_handle);
}
