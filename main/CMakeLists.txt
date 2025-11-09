idf_component_register(
    SRCS "solar_irrigator_slave.c"
    INCLUDE_DIRS "."
    REQUIRES 
        led_manager 
        sensor_manager 
        peer_manager 
        time_sync 
        power_manager 
        pump_controller 
        button_manager 
        esp_hw_support
        CJSON
        test_manager
    PRIV_INCLUDE_DIRS "$ENV{IDF_PATH}/components/esp_hw_support/include"
)
