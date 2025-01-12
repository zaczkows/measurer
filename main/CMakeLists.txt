set(SOURCES main.c)
idf_component_register(SRC_DIRS "." "images"
                    INCLUDE_DIRS "includes"
                    REQUIRES "core2forAWS" "esp-cryptoauthlib" "fft" "nvs_flash" "TinyGPSPlus" "esp_http_server")
