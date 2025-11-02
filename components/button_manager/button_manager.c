#include "button_manager.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"


#define TAG "BUTTON MANAGER"

void button_init()
{

    gpio_config_t io_config = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_config);
}

void button_erase()
{
    if (gpio_get_level(BUTTON_GPIO) == 0)
    {
        const int hold_time_ms = 8000; 
        const int interval_ms = 100; 
        int elapsed = 0; 

        while (gpio_get_level(BUTTON_GPIO) == 0 && elapsed < hold_time_ms){
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
            elapsed += interval_ms; 
            ESP_LOGI(TAG, "Tiempo trasncurrido: %i", elapsed );
        }

        if (elapsed >= hold_time_ms){
            ESP_LOGI(TAG, "Borrando memoria NVS...");
            ESP_ERROR_CHECK(nvs_flash_erase());
            ESP_LOGI(TAG, "Memoria NVS borrada. Reiniciando...");
            esp_restart();
        }else{
            ESP_LOGI(TAG, "Borrado de momoria NVS interrupido.");
        }
    }
}