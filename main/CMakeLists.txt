if(DEFINED WIFI_PWD AND WIFI_SSID)
idf_component_register(SRCS main.c lcd_ili9341.c decode_image.c
                        PRIV_INCLUDE_DIRS .
                        PRIV_REQUIRES nvs_flash esp_timer esp_event esp_lcd)
else()
idf_component_register(SRCS main.c lcd_ili9341.c
                        PRIV_INCLUDE_DIRS .
                        PRIV_REQUIRES nvs_flash esp_timer esp_event esp_lcd esp_netif esp_wifi esp_http_server)
endif()
component_compile_options(-Wno-error=format= -Wno-format)