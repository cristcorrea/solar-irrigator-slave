#include "sensor_manager.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define TAG "SENSOR_MANAGER"
#define I2C_MASTER_SCL_IO 20
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define AHT20_ADDR 0x38

void sensor_manager_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // Inicialización del sensor (comando 0xBE, 0x08, 0x00)
    uint8_t init_cmd[] = {0xBE, 0x08, 0x00};
    i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_ADDR, init_cmd, sizeof(init_cmd), pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(10));
}

void sensor_manager_read_aht20(float* temperature, float* humidity) {
    uint8_t trigger_cmd[] = {0xAC, 0x33, 0x00};
    uint8_t data[7];

    i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_ADDR, trigger_cmd, sizeof(trigger_cmd), pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(80));
    i2c_master_read_from_device(I2C_MASTER_NUM, AHT20_ADDR, data, 7, pdMS_TO_TICKS(100));

    uint32_t raw_hum = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4));
    uint32_t raw_temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];

    *humidity = ((float)raw_hum / 1048576.0) * 100.0;
    *temperature = ((float)raw_temp / 1048576.0) * 200.0 - 50.0;

    ESP_LOGI(TAG, "Temp: %.2f°C, Hum: %.2f%%", *temperature, *humidity);
}
