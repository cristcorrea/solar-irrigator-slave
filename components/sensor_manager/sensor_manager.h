#pragma once

void sensor_manager_init(void);
void sensor_manager_read_aht20(float* temperature, float* humidity);
