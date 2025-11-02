idf_component_register(SRCS "peer_manager.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES led_manager CJSON time_sync
                       REQUIRES nvs_flash esp_wifi freertos esp_netif esp_event)