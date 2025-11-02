idf_component_register(SRCS "button_manager.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver freertos log nvs_flash esp_system)
