#pragma once

#include "esp_err.h"

void sensor_manager_init(void);
esp_err_t sensor_manager_read_aht20(float* temperature, float* humidity);
