#include "sensor_manager.h"
#include <math.h>
#include <stdbool.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define TAG "SENSOR_MANAGER"
#define I2C_MASTER_SCL_IO 20
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define AHT20_ADDR 0x38
#define AHT20_CMD_STATUS 0x71
#define AHT20_CMD_INIT 0xBE
#define AHT20_CMD_TRIGGER 0xAC
#define AHT20_CMD_PARAM_0 0x33
#define AHT20_CMD_PARAM_1 0x00
#define AHT20_INIT_PARAM_0 0x08
#define AHT20_INIT_PARAM_1 0x00
#define AHT20_STATUS_BUSY 0x80
#define AHT20_STATUS_CALIBRATED 0x08
#define AHT20_STATUS_READY_MASK 0x18
#define AHT20_STATUS_READY_VALUE 0x18
#define AHT20_READ_ATTEMPTS 3
#define AHT20_MIN_TEMP_C -40.0f
#define AHT20_MAX_TEMP_C 85.0f
#define AHT20_MIN_HUMIDITY 0.0f
#define AHT20_MAX_HUMIDITY 100.0f

static bool s_i2c_ready = false;

static uint8_t aht20_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;

    for (int i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x80) != 0) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}

static bool sensor_values_are_coherent(float temperature, float humidity)
{
    return isfinite(temperature) &&
           isfinite(humidity) &&
           temperature >= AHT20_MIN_TEMP_C &&
           temperature <= AHT20_MAX_TEMP_C &&
           humidity >= AHT20_MIN_HUMIDITY &&
           humidity <= AHT20_MAX_HUMIDITY;
}

static void aht20_log_status(const char *context, uint8_t status)
{
    ESP_LOGI(TAG,
             "AHT20 %s: status=0x%02X | hoja datos: busy(bit7)=%u cal(bit3)=%u mode(bits6:5)=%u",
             context,
             status,
             (status & AHT20_STATUS_BUSY) ? 1 : 0,
             (status & AHT20_STATUS_CALIBRATED) ? 1 : 0,
             (unsigned)((status >> 5) & 0x03));
}

static esp_err_t aht20_read_status(uint8_t *status)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd = AHT20_CMD_STATUS;
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        AHT20_ADDR,
                                        &cmd,
                                        sizeof(cmd),
                                        status,
                                        sizeof(*status),
                                        pdMS_TO_TICKS(100));
}

void sensor_manager_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config falló: %s", esp_err_to_name(err));
        s_i2c_ready = false;
        return;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C driver install falló: %s", esp_err_to_name(err));
        s_i2c_ready = false;
        return;
    }
    s_i2c_ready = true;

    vTaskDelay(pdMS_TO_TICKS(120));

    uint8_t status = 0;
    err = aht20_read_status(&status);
    bool needs_init = true;
    if (err == ESP_OK) {
        aht20_log_status("status inicial", status);
        needs_init = ((status & AHT20_STATUS_READY_MASK) != AHT20_STATUS_READY_VALUE);
    } else {
        ESP_LOGW(TAG,
                 "AHT20: no se pudo leer status inicial con 0x%02X: %s; se intentará inicializar",
                 AHT20_CMD_STATUS,
                 esp_err_to_name(err));
    }

    if (needs_init) {
        uint8_t init_cmd[] = {AHT20_CMD_INIT, AHT20_INIT_PARAM_0, AHT20_INIT_PARAM_1};
        ESP_LOGI(TAG,
                 "AHT20: enviando inicialización según hoja de datos: %02X %02X %02X",
                 init_cmd[0],
                 init_cmd[1],
                 init_cmd[2]);
        err = i2c_master_write_to_device(I2C_MASTER_NUM,
                                         AHT20_ADDR,
                                         init_cmd,
                                         sizeof(init_cmd),
                                         pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "AHT20: inicialización falló: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(20));

        err = aht20_read_status(&status);
        if (err == ESP_OK) {
            aht20_log_status("status tras init", status);
        } else {
            ESP_LOGW(TAG, "AHT20: no se pudo leer status tras init: %s", esp_err_to_name(err));
        }
    }
}

esp_err_t sensor_manager_read_aht20(float* temperature, float* humidity) {
    if (!temperature || !humidity) {
        return ESP_ERR_INVALID_ARG;
    }

    *temperature = 0.0f;
    *humidity = 0.0f;

    if (!s_i2c_ready) {
        ESP_LOGE(TAG, "AHT20: I2C no inicializado correctamente");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t trigger_cmd[] = {AHT20_CMD_TRIGGER, AHT20_CMD_PARAM_0, AHT20_CMD_PARAM_1};
    esp_err_t last_err = ESP_FAIL;

    for (int attempt = 1; attempt <= AHT20_READ_ATTEMPTS; ++attempt) {
        uint8_t data[7] = {0};

        uint8_t status_before = 0;
        esp_err_t status_err = aht20_read_status(&status_before);
        if (status_err == ESP_OK) {
            aht20_log_status("status antes de medir", status_before);
        } else {
            ESP_LOGW(TAG,
                     "AHT20: no se pudo leer status antes de medir (intento %d/%d): %s",
                     attempt,
                     AHT20_READ_ATTEMPTS,
                     esp_err_to_name(status_err));
        }

        ESP_LOGI(TAG,
                 "AHT20 intento %d/%d: hoja datos espera TX=%02X %02X %02X y RX=7 bytes",
                 attempt,
                 AHT20_READ_ATTEMPTS,
                 trigger_cmd[0],
                 trigger_cmd[1],
                 trigger_cmd[2]);

        last_err = i2c_master_write_to_device(I2C_MASTER_NUM,
                                              AHT20_ADDR,
                                              trigger_cmd,
                                              sizeof(trigger_cmd),
                                              pdMS_TO_TICKS(100));
        if (last_err != ESP_OK) {
            ESP_LOGW(TAG, "AHT20: error iniciando medición (intento %d/%d): %s",
                     attempt,
                     AHT20_READ_ATTEMPTS,
                     esp_err_to_name(last_err));
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(80));

        last_err = i2c_master_read_from_device(I2C_MASTER_NUM,
                                               AHT20_ADDR,
                                               data,
                                               sizeof(data),
                                               pdMS_TO_TICKS(100));
        if (last_err != ESP_OK) {
            ESP_LOGW(TAG, "AHT20: error leyendo RX=7 bytes (intento %d/%d): %s",
                     attempt,
                     AHT20_READ_ATTEMPTS,
                     esp_err_to_name(last_err));
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        ESP_LOGI(TAG,
                 "AHT20 RX bruto: status=%02X hum_msb=%02X hum_mid=%02X hum_temp=%02X temp_mid=%02X temp_lsb=%02X crc=%02X",
                 data[0],
                 data[1],
                 data[2],
                 data[3],
                 data[4],
                 data[5],
                 data[6]);
        aht20_log_status("status recibido en muestra", data[0]);

        if ((data[0] & AHT20_STATUS_BUSY) != 0) {
            ESP_LOGW(TAG, "AHT20: sensor ocupado (status=0x%02X, intento %d/%d)",
                     data[0],
                     attempt,
                     AHT20_READ_ATTEMPTS);
            last_err = ESP_ERR_INVALID_STATE;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        uint8_t expected_crc = aht20_crc8(data, 6);
        ESP_LOGI(TAG,
                 "AHT20 CRC según hoja datos: calculado=0x%02X recibido=0x%02X %s",
                 expected_crc,
                 data[6],
                 (expected_crc == data[6]) ? "OK" : "FAIL");
        if (expected_crc != data[6]) {
            ESP_LOGW(TAG,
                     "AHT20: CRC inválido (intento %d/%d): esperado=0x%02X recibido=0x%02X, raw=%02X %02X %02X %02X %02X %02X %02X",
                     attempt,
                     AHT20_READ_ATTEMPTS,
                     expected_crc,
                     data[6],
                     data[0],
                     data[1],
                     data[2],
                     data[3],
                     data[4],
                     data[5],
                     data[6]);
            last_err = ESP_ERR_INVALID_CRC;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        uint32_t raw_hum = ((uint32_t)data[1] << 12) |
                           ((uint32_t)data[2] << 4) |
                           ((uint32_t)data[3] >> 4);
        uint32_t raw_temp = (((uint32_t)data[3] & 0x0F) << 16) |
                            ((uint32_t)data[4] << 8) |
                            (uint32_t)data[5];

        float read_humidity = ((float)raw_hum / 1048576.0f) * 100.0f;
        float read_temperature = ((float)raw_temp / 1048576.0f) * 200.0f - 50.0f;

        ESP_LOGI(TAG,
                 "AHT20 decodificado según hoja datos: raw_hum=%lu -> %.2f%%RH | raw_temp=%lu -> %.2f°C",
                 (unsigned long)raw_hum,
                 read_humidity,
                 (unsigned long)raw_temp,
                 read_temperature);

        if (!sensor_values_are_coherent(read_temperature, read_humidity)) {
            ESP_LOGW(TAG,
                     "AHT20: lectura incoherente (intento %d/%d): Temp=%.2f°C, Hum=%.2f%%, raw=%02X %02X %02X %02X %02X %02X %02X",
                     attempt,
                     AHT20_READ_ATTEMPTS,
                     read_temperature,
                     read_humidity,
                     data[0],
                     data[1],
                     data[2],
                     data[3],
                     data[4],
                     data[5],
                     data[6]);
            last_err = ESP_ERR_INVALID_RESPONSE;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        *humidity = read_humidity;
        *temperature = read_temperature;

        ESP_LOGI(TAG, "Temp: %.2f°C, Hum: %.2f%%", *temperature, *humidity);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "AHT20: sin lectura válida tras %d intentos", AHT20_READ_ATTEMPTS);
    return last_err;
}
