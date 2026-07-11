idf_component_register(SRCS "main.c" "my_platform.c" "dfplayer.c"
                    INCLUDE_DIRS "."
                    REQUIRES "bluepad32" "btstack" "driver" "nvs_flash" "esp_timer")
