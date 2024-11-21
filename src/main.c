/*
 * AWS IoT EduKit - Core2 for AWS IoT EduKit
 * Factory Firmware v2.2.0
 * main.c
 *
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "sdkconfig.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_event.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"

#include "core2forAWS.h"

/* #include "crypto.h" */
/* #include "cta.h" */
#include "env3.h"
#include "gps.h"
#include "home.h"
/* #include "led_bar.h" */
/* #include "mic.h" */
/* #include "mpu.h" */
#include "power.h"
#include "tasks.h"
/* #include "touch.h" */
#include "web.h"
#include "wifi.h"

static const char* TAG = "MAIN";

static void
ui_start(void);
static void
tab_event_cb(lv_obj_t* slider, lv_event_t event);

static lv_obj_t* tab_view;

LV_IMG_DECLARE(powered_by_aws_logo);

void
app_main(void)
{
    ESP_LOGI(TAG,
             "\n***************************************************\n M5Stack "
             "Core2 for AWS IoT EduKit Factory "
             "Firmware\n***************************************************");

    // Initialize NVS for Wi-Fi stack to store data
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_log_level_set("gpio", ESP_LOG_NONE);
    esp_log_level_set("ILI9341", ESP_LOG_NONE);

    Core2ForAWS_Init();
    Core2ForAWS_Display_SetBrightness(50); // Last since the display first needs time to finish initializing.

    ui_start();
}

static void
ui_start(void)
{
    /* Displays the Powered by AWS logo */
    xSemaphoreTake(xGuiSemaphore,
                   portMAX_DELAY);       // Takes (blocks) the xGuiSemaphore mutex from
                                         // being read/written by another task.
    lv_obj_t* opener_scr = lv_scr_act(); // Create a new LVGL "screen". Screens
                                         // can be though of as a window.
    lv_obj_set_style_local_bg_color(opener_scr, LV_OBJ_PART_MAIN, 0, LV_COLOR_BLACK);
    lv_obj_clean(opener_scr);                              // Clear the aws_img_obj and remove from memory
                                                           // space. Currently no objects exist on the screen.
    lv_obj_t* core2forAWS_obj = lv_obj_create(NULL, NULL); // Create a object to draw all with no parent
    lv_scr_load(core2forAWS_obj);
    tab_view = lv_tabview_create(core2forAWS_obj, NULL); // Creates the tab view to display different tabs
                                                         // with different hardware features
    lv_obj_set_style_local_bg_color(tab_view, LV_OBJ_PART_MAIN, 0, LV_COLOR_BLACK);
    lv_obj_set_event_cb(tab_view,
                        tab_event_cb); // Add a callback for whenever there is an event triggered
                                       // on the tab_view object (e.g. a left-to-right swipe)
    lv_tabview_set_btns_pos(tab_view, LV_TABVIEW_TAB_POS_NONE); // Hide the tab buttons so it looks
                                                                // like a clean screen

    xSemaphoreGive(xGuiSemaphore); // Frees the xGuiSemaphore so that another task can use
                                   // it. In this case, the higher priority guiTask will take
                                   // it and then read the values to then display.

    start_backgound_tasks(core2forAWS_obj);
    /*
    Below creates all the display layers for the various peripheral tabs. Some of
    the tabs also starts the concurrent FreeRTOS tasks that read/write to the
    peripheral registers and displays the data from that peripheral.
    */
    display_home_tab(tab_view);
    display_gps_tab(tab_view, core2forAWS_obj);
    display_env3_tab(tab_view, core2forAWS_obj);
    display_power_tab(tab_view);

    initialise_wifi();

    display_web_tab(tab_view);
}

static void
tab_event_cb(lv_obj_t* slider, lv_event_t event)
{
    if (event == LV_EVENT_VALUE_CHANGED) {
        lv_tabview_ext_t* ext = (lv_tabview_ext_t*)lv_obj_get_ext_attr(tab_view);
        const char* tab_name = ext->tab_name_ptr[lv_tabview_get_tab_act(tab_view)];
        ESP_LOGI(TAG, "Current Active Tab: %s\n", tab_name);

        /* vTaskSuspend(MPU_handle); */
        /* vTaskSuspend(mic_handle); */
        /* vTaskSuspend(FFT_handle); */
        // vTaskSuspend(wifi_handle);
        /* vTaskSuspend(touch_handle); */

#if 0
        vTaskSuspend(led_bar_solid_handle);
        if (led_bar_solid_handle != NULL) {
            vTaskSuspend(led_bar_solid_handle);
            /* Delete using the copy of the handle. */
            vTaskDelete(led_bar_solid_handle);
            /* The task is going to be deleted.
            Set the handle to NULL. */
            led_bar_solid_handle = NULL;
        }
        vTaskResume(led_bar_animation_handle);

        /* else if (strcmp(tab_name, MPU_TAB_NAME) == 0) */
        /*   vTaskResume(MPU_handle); */
        if (strcmp(tab_name, MICROPHONE_TAB_NAME) == 0) {
            vTaskResume(mic_handle);
            vTaskResume(FFT_handle);
            /* } else if (strcmp(tab_name, LED_BAR_TAB_NAME) == 0) { */
            /*     vTaskSuspend(led_bar_animation_handle); */
            /*     xTaskCreatePinnedToCore( */
            /*       sk6812_solid_task, "sk6812SolidTask", configMINIMAL_STACK_SIZE * 3, NULL, 0, &led_bar_solid_handle,
             * 1); */
        } else if (strcmp(tab_name, TOUCH_TAB_NAME) == 0) {
            reset_touch_bg();
            vTaskResume(touch_handle);
        }
        // else if (strcmp(tab_name, WIFI_TAB_NAME) == 0)
        //     vTaskResume(wifi_handle);
#endif
    }
}
