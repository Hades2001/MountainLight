idf_component_register(SRCS "makerhub.c" "astro_calc.c"
                       INCLUDE_DIRS "."
                       REQUIRES "nvs_flash" "esp_event" "esp_netif" "esp_wifi" "esp_driver_uart" "json")
