idf_component_register(
  SRCS "test_manager.c"
  INCLUDE_DIRS "include"
  REQUIRES sensor_manager power_manager pump_controller led_manager time_sync nvs_flash driver
)
