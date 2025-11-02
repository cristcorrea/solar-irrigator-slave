idf_component_register(SRCS "pump_controller.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver freertos esp_timer)
