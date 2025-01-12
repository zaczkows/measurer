
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_http_server.h"
#include "esp_log.h"

#include "core2forAWS.h"

#include "env3.h"
#include "gps.h"
#include "web.h"
#include "wifi.h"

static const char* TAG = WEB_TAB_NAME;

static TaskHandle_t web_task_handle;

lv_obj_t* ip_addr_label = NULL;

#define MIN(a, b)                                                                                            \
    ({                                                                                                       \
        __typeof__(a) _a = (a);                                                                              \
        __typeof__(b) _b = (b);                                                                              \
        _a < _b ? _a : _b;                                                                                   \
    })

esp_err_t
get_sensors_handler(httpd_req_t* req)
{
    /* Send a simple response */
    SensorInfo sensor_info = get_sensor_info();
    static char response_buffer[64];
    snprintf(response_buffer,
             sizeof(response_buffer),
             "{ \"temp\": %.2f, \"hum\": %.2f, \"press\": %.2f }",
             sensor_info.temperature,
             sensor_info.humidity,
             sensor_info.pressure);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_buffer, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t
get_gps_handler(httpd_req_t* req)
{
    static char response_buffer[96];

    /* Send a simple response */
    GpsPosition gps_pos = get_latest_gps_position();
    if (!gps_pos.is_valid) {
        /* httpd_resp_set_status(req, HTTPD_204); */
        httpd_resp_set_type(req, "application/json");
        snprintf(response_buffer, sizeof(response_buffer), "{\"ut\":%u}", gps_pos.unix_time);
        httpd_resp_send(req, response_buffer, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    snprintf(response_buffer,
             sizeof(response_buffer),
             "{\"lat\":%.7f,\"lng\":%.7f,\"alt\":%.3f,\"sat\":%u,\"ut\":%u}",
             gps_pos.latitude,
             gps_pos.longitude,
             gps_pos.altitude,
             gps_pos.satellites,
             gps_pos.unix_time);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_buffer, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_uri_t sensors_get = { .uri = "/sensors",
                            .method = HTTP_GET,
                            .handler = get_sensors_handler,
                            .user_ctx = NULL };

httpd_uri_t gps_get = { .uri = "/gps", .method = HTTP_GET, .handler = get_gps_handler, .user_ctx = NULL };

/* Function for starting the webserver */
httpd_handle_t
start_webserver(void)
{
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;

    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &sensors_get);
        httpd_register_uri_handler(server, &gps_get);
    }
    /* If server failed to start, handle will be NULL */
    return server;
}

/* Function for stopping the webserver */
void
stop_webserver(httpd_handle_t server)
{
    if (server) {
        /* Stop the httpd server */
        httpd_stop(server);
    }
}

static void
web_update_task(void* pvParameters)
{
    ESP_LOGI(TAG, "Start WEB update task");
    uint32_t ip_address = 0;
    char ip_address_text[32];
    while (true) {
        const uint32_t ip = get_ip_address();
        if (ip != ip_address) {
            ip_address = ip;
            struct esp_ip4_addr temp = { .addr = ip };
            snprintf(ip_address_text, sizeof(ip_address_text), "IP address: " IPSTR, IP2STR(&temp));
            xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
            lv_label_set_static_text(ip_addr_label, ip_address_text);
            xSemaphoreGive(xGuiSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void
display_web_tab(lv_obj_t* tv)
{
    ESP_LOGI(TAG, "Starting web tab");
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);

    lv_obj_t* web_tab = lv_tabview_add_tab(tv, WEB_TAB_NAME); // Create a tab

    /* Create the main body object and set background within the tab*/
    static lv_style_t bg_style;
    lv_obj_t* web_tab_bg = lv_obj_create(web_tab, NULL);
    lv_obj_align(web_tab_bg, NULL, LV_ALIGN_IN_TOP_LEFT, 16, 36);
    lv_obj_set_size(web_tab_bg, 290, 190);
    lv_obj_set_click(web_tab_bg, false);
    lv_style_init(&bg_style);
    lv_style_set_bg_color(&bg_style, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_obj_add_style(web_tab_bg, LV_OBJ_PART_MAIN, &bg_style);

    /* Create the title within the main body object */
    static lv_style_t title_style;
    lv_style_init(&title_style);
    lv_style_set_text_font(&title_style, LV_STATE_DEFAULT, LV_THEME_DEFAULT_FONT_TITLE);
    lv_style_set_text_color(&title_style, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_t* tab_title_label = lv_label_create(web_tab_bg, NULL);
    lv_obj_add_style(tab_title_label, LV_OBJ_PART_MAIN, &title_style);
    lv_label_set_static_text(tab_title_label, "Webserver and wireless info");
    lv_obj_align(tab_title_label, web_tab_bg, LV_ALIGN_IN_TOP_MID, 0, 10);

    /* Create the sensor information label object */
    lv_obj_t* body_label = lv_label_create(web_tab_bg, NULL);
    lv_label_set_long_mode(body_label, LV_LABEL_LONG_BREAK);
    lv_label_set_static_text(body_label, "Basic information about webserver and wireless info.");
    lv_obj_set_width(body_label, 252);
    lv_obj_align(body_label, web_tab_bg, LV_ALIGN_IN_TOP_LEFT, 10, 30);

    static lv_style_t body_style;
    lv_style_init(&body_style);
    lv_style_set_text_color(&body_style, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_add_style(body_label, LV_OBJ_PART_MAIN, &body_style);

    static lv_style_t labels_style;
    lv_style_init(&labels_style);
    lv_style_set_text_color(&labels_style, LV_STATE_DEFAULT, LV_COLOR_YELLOW);

    ip_addr_label = lv_label_create(web_tab_bg, NULL);
    lv_label_set_align(ip_addr_label, LV_LABEL_ALIGN_LEFT);
    lv_obj_add_style(ip_addr_label, LV_LABEL_PART_MAIN, &labels_style);
    lv_obj_align(ip_addr_label, body_label, LV_ALIGN_OUT_BOTTOM_LEFT, 5, 5);
    lv_label_set_static_text(ip_addr_label, "IP address: 0.0.0.0");

    xSemaphoreGive(xGuiSemaphore);

    start_webserver();
    xTaskCreate(web_update_task, "WebTask", 4096, NULL, 0, &web_task_handle);
}
