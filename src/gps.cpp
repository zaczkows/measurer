/*
 * AWS IoT EduKit - Core2 for AWS IoT EduKit
 * Factory Firmware v2.2.0
 * gps.c
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

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "TinyGPSPlus.h"

#include "esp_log.h"

#include "bm8563.h"
#include "core2forAWS.h"

#include "gps.h"

const char* GPS_TAB_NAME = "AT6558-GPS";
static const char* TAG = GPS_TAB_NAME;

lv_obj_t* gps_tab;

lv_obj_t* latitude_label = nullptr;
lv_obj_t* longitude_label = nullptr;
lv_obj_t* altitude_label = nullptr;
lv_obj_t* satellites_label = nullptr;
lv_obj_t* gps_time_label = nullptr;

static GpsPosition GPS_POSITION = { .latitude = 0.0f,
                                    .longitude = 0.0f,
                                    .altitude = 0.0f,
                                    .satellites = 0,
                                    .unix_time = 0,
                                    .is_valid = false };
SemaphoreHandle_t GPS_POSITION_MUTEX;

GpsPosition
get_latest_gps_position(void)
{
    xSemaphoreTake(GPS_POSITION_MUTEX, portMAX_DELAY);
    GpsPosition gps_pos = GPS_POSITION;
    xSemaphoreGive(GPS_POSITION_MUTEX);
    return gps_pos;
}

static void
gps_task(void* pvParameters)
{
    const esp_err_t err = Core2ForAWS_Port_PinMode(PORT_C_UART_RX_PIN, UART);
    if (err != ESP_OK) {
        const char* SENSOR_FAILED = "UART Sensor failed!";
        lv_label_set_static_text(latitude_label, SENSOR_FAILED);
        ESP_LOGE(TAG, "Failed to enable UART port");
        vTaskDelete(NULL); // Should never get to here...
        return;
    }

    TinyGPSPlus gps;
    Core2ForAWS_Port_C_UART_Begin(9600);
    int rxBytes;
    uint8_t* data = reinterpret_cast<uint8_t*>(
      heap_caps_malloc(UART_RX_BUF_SIZE,
                       MALLOC_CAP_SPIRAM)); // Allocate space for message in external RAM

    struct tm ttc;
    ttc.tm_isdst = 0;

    char text_lat_buffer[20];
    char text_lng_buffer[20];
    char text_alt_buffer[16];
    char text_sat_buffer[16];
    char text_date_buffer[36];
    unsigned long last_time_from_gps_update = 0;
    while (true) {
        rxBytes = Core2ForAWS_Port_C_UART_Receive(data);
        if (rxBytes > 0 && data[0] != '\0') {
            // ESP_LOGI(TAG, "Read %d bytes from UART. Received: '%s'", rxBytes, data);
            for (unsigned i = 0; i < rxBytes; ++i) {
                gps.encode(data[i]);
            }

            if (gps.location.isValid() && gps.altitude.isValid()) {
                xSemaphoreTake(GPS_POSITION_MUTEX, portMAX_DELAY);
                GPS_POSITION.latitude = gps.location.lat();
                GPS_POSITION.longitude = gps.location.lng();
                GPS_POSITION.altitude = gps.altitude.meters();
                GPS_POSITION.satellites = gps.satellites.value();
                GPS_POSITION.is_valid = true;
                xSemaphoreGive(GPS_POSITION_MUTEX);
            } else {
                xSemaphoreTake(GPS_POSITION_MUTEX, portMAX_DELAY);
                GPS_POSITION.latitude = 0.0f;
                GPS_POSITION.longitude = 0.0f;
                GPS_POSITION.altitude = 0.0f;
                GPS_POSITION.satellites = 0;
                GPS_POSITION.is_valid = false;
                xSemaphoreGive(GPS_POSITION_MUTEX);
            }

            if (gps.location.isValid()) {
                snprintf(text_lat_buffer, sizeof(text_lat_buffer), "Latitude: %.6f", gps.location.lat());
                snprintf(text_lng_buffer, sizeof(text_lng_buffer), "Longitude: %.6f", gps.location.lng());
            } else {
                snprintf(text_lat_buffer, sizeof(text_lat_buffer), "Latitude: ?");
                snprintf(text_lng_buffer, sizeof(text_lng_buffer), "Longitude: ?");
            }
            if (gps.altitude.isValid()) {
                snprintf(text_alt_buffer, sizeof(text_alt_buffer), "Altitude: %.1f", gps.altitude.meters());
            } else {
                snprintf(text_alt_buffer, sizeof(text_alt_buffer), "Altitude: ?");
            }
            if (gps.satellites.isValid()) {
                snprintf(text_sat_buffer, sizeof(text_sat_buffer), "Satellites: %u", gps.satellites.value());
            } else {
                snprintf(text_sat_buffer, sizeof(text_sat_buffer), "Satellites: ?");
            }

            time_t conv_time = 0;
            if (gps.time.isValid() && gps.date.isValid()) {
                ttc.tm_year = gps.date.year() - 1900;
                ttc.tm_mon = gps.date.month();
                ttc.tm_mday = gps.date.day();
                ttc.tm_hour = gps.time.hour();
                ttc.tm_min = gps.time.minute();
                ttc.tm_sec = gps.time.second();
                conv_time = mktime(&ttc);

                if (CONFIG_TIMEZONE_MIN != 0) {
                    conv_time += CONFIG_TIMEZONE_MIN * 60;
                    gmtime_r(&conv_time, &ttc);
                }

                const unsigned long now = millis();
                if ((gps.satellites.isValid() && gps.satellites.value() > 4) &&
                    (now < last_time_from_gps_update || now - last_time_from_gps_update < 30 * 60 * 1000)) {
                    rtc_date_t datetime;
                    datetime.year = ttc.tm_year + 1900;
                    datetime.month = ttc.tm_mon;
                    datetime.day = ttc.tm_mday;
                    datetime.hour = ttc.tm_hour;
                    datetime.minute = ttc.tm_min;
                    datetime.second = ttc.tm_sec;
                    BM8563_SetTime(&datetime);
                    last_time_from_gps_update = now;
                }
                snprintf(text_date_buffer,
                         sizeof(text_date_buffer),
                         "Time: %02d.%02d.%04d, %02d:%02d:%02d",
                         ttc.tm_mday,
                         ttc.tm_mon,
                         ttc.tm_year + 1900,
                         ttc.tm_hour,
                         ttc.tm_min,
                         ttc.tm_sec);
            } else {
                snprintf(text_date_buffer, sizeof(text_date_buffer), "Time: ?");
            }
            xSemaphoreTake(GPS_POSITION_MUTEX, portMAX_DELAY);
            GPS_POSITION.unix_time = conv_time;
            xSemaphoreGive(GPS_POSITION_MUTEX);

            xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);

            lv_label_set_static_text(latitude_label, text_lat_buffer);
            lv_label_set_static_text(longitude_label, text_lng_buffer);
            lv_label_set_static_text(altitude_label, text_alt_buffer);
            lv_label_set_static_text(satellites_label, text_sat_buffer);
            lv_label_set_static_text(gps_time_label, text_date_buffer);

            xSemaphoreGive(xGuiSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Read more frequently than transmit to ensure
                                        // the messages are not erased from buffer.
    }
    free(data);        // Free memory from external RAM
    vTaskDelete(NULL); // Should never get to here...
}

extern "C" void
display_gps_tab(lv_obj_t* tv, lv_obj_t* core2forAWS_screen_obj)
{
    GPS_POSITION_MUTEX = xSemaphoreCreateMutex();

    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    gps_tab = lv_tabview_add_tab(tv, GPS_TAB_NAME); // Create a tab

    /* Create the main body object and set background within the tab*/
    static lv_style_t bg_style;
    lv_obj_t* gps_bg = lv_obj_create(gps_tab, NULL);
    lv_obj_align(gps_bg, NULL, LV_ALIGN_IN_TOP_LEFT, 16, 36);
    lv_obj_set_size(gps_bg, 290, 190);
    lv_obj_set_click(gps_bg, false);
    lv_style_init(&bg_style);
    lv_style_set_bg_color(&bg_style, LV_STATE_DEFAULT, lv_color_make(254, 230, 0));
    lv_obj_add_style(gps_bg, LV_OBJ_PART_MAIN, &bg_style);

    /* Create the title within the main body object */
    static lv_style_t title_style;
    lv_style_init(&title_style);
    lv_style_set_text_font(&title_style, LV_STATE_DEFAULT, LV_THEME_DEFAULT_FONT_TITLE);
    lv_style_set_text_color(&title_style, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_t* tab_title_label = lv_label_create(gps_bg, NULL);
    lv_obj_add_style(tab_title_label, LV_OBJ_PART_MAIN, &title_style);
    lv_label_set_static_text(tab_title_label, "AT6558-GPS");
    lv_obj_align(tab_title_label, gps_bg, LV_ALIGN_IN_TOP_MID, 0, 5);

    /* Create the sensor information label object */
    lv_obj_t* body_label = lv_label_create(gps_bg, NULL);
    lv_label_set_long_mode(body_label, LV_LABEL_LONG_BREAK);
    lv_label_set_static_text(body_label, "The BM8563 is an accurate, low powered real-time gps. ▲");
    lv_obj_set_width(body_label, 252);
    lv_obj_align(body_label, gps_bg, LV_ALIGN_IN_TOP_LEFT, 20, 30);

    static lv_style_t body_style;
    lv_style_init(&body_style);
    lv_style_set_text_color(&body_style, LV_STATE_DEFAULT, LV_COLOR_GREEN);
    lv_obj_add_style(body_label, LV_OBJ_PART_MAIN, &body_style);

    static lv_style_t labels_style;
    lv_style_init(&labels_style);
    lv_style_set_text_color(&labels_style, LV_STATE_DEFAULT, LV_COLOR_BLACK);

    // Label for latitude
    latitude_label = lv_label_create(gps_bg, NULL);
    lv_label_set_align(latitude_label, LV_LABEL_ALIGN_LEFT);
    lv_obj_add_style(latitude_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_obj_align(latitude_label, gps_bg, LV_ALIGN_IN_TOP_LEFT, 20, 70);

    // Label for longitude
    longitude_label = lv_label_create(gps_bg, NULL);
    lv_label_set_align(longitude_label, LV_LABEL_ALIGN_LEFT);
    lv_obj_add_style(longitude_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_obj_align(longitude_label, gps_bg, LV_ALIGN_IN_TOP_LEFT, 20, 90);

    // Label for altitude
    altitude_label = lv_label_create(gps_bg, NULL);
    lv_label_set_align(altitude_label, LV_LABEL_ALIGN_LEFT);
    lv_obj_add_style(altitude_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_obj_align(altitude_label, gps_bg, LV_ALIGN_IN_TOP_LEFT, 20, 110);

    // Label for satellites
    satellites_label = lv_label_create(gps_bg, NULL);
    lv_label_set_align(satellites_label, LV_LABEL_ALIGN_LEFT);
    lv_obj_add_style(satellites_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_obj_align(satellites_label, gps_bg, LV_ALIGN_IN_TOP_LEFT, 20, 130);

    // Label for GPS time
    gps_time_label = lv_label_create(gps_bg, NULL);
    lv_label_set_align(gps_time_label, LV_LABEL_ALIGN_LEFT);
    lv_obj_add_style(gps_time_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_obj_align(gps_time_label, gps_bg, LV_ALIGN_IN_TOP_LEFT, 20, 150);

    xSemaphoreGive(xGuiSemaphore);

    xTaskCreate(gps_task, "gpsTask", 4096 * 3, NULL, 0, &gps_handle);
}
