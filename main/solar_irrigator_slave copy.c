#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include <string.h>
#include "esp_mac.h"
#include "esp_wifi.h"
#include "sensor_manager.h"
#include "power_manager.h"
#include "led_manager.h"
#include "pump_controller.h"
#include "peer_manager.h"
#include "time_sync.h"
#include "esp_now.h"


#define TAG "MAIN"
#define CHARGER_GPIO GPIO_NUM_10
#define BUTTON_GPIO  GPIO_NUM_0
#define BOOST_GPIO   GPIO_NUM_2



bool response_received = false;
SemaphoreHandle_t response_semaphore = NULL; 

static bool wait_for_response_from_hub_with_retries(int timeout_ms) {
    if (response_semaphore == NULL) {
        response_semaphore = xSemaphoreCreateBinary();
    }

        if (xSemaphoreTake(response_semaphore, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
            led_check();
            return true;
        } 
    
    return false;
}


static void send_data_to_hub(float temp, float hum, float vbat, bool irrigation_done) {
    uint8_t hub_mac[6];
    bool has_mac = false;
    esp_base_mac_addr_get(hub_mac);

    uint8_t target_mac[6];
    peer_manager_load_hub_mac(target_mac);
    if (!(target_mac[0] == 0xFF && target_mac[1] == 0xFF)) {
        has_mac = true;
    } else {
        memset(target_mac, 0xFF, 6);  // broadcast
    }

    char payload[64];
    snprintf(payload, sizeof(payload), "%.1f,%.1f,%.2f,%d %02X%02X%02X%02X%02X%02X",
             hum, temp, vbat, irrigation_done,
             hub_mac[0], hub_mac[1], hub_mac[2],
             hub_mac[3], hub_mac[4], hub_mac[5]);

    if (!esp_now_is_peer_exist(target_mac)) {
        esp_now_peer_info_t peer = {
            .channel = 6,
            .ifidx = WIFI_IF_STA,
            .encrypt = false
        };
        memcpy(peer.peer_addr, target_mac, 6);

        esp_err_t err = esp_now_add_peer(&peer);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "No se pudo agregar peer: %s", esp_err_to_name(err));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_err_t result = esp_now_send(target_mac, (uint8_t *)payload, strlen(payload));

    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Datos enviados a %s: %s", has_mac ? "HUB" : "broadcast", payload);
    } else {
        ESP_LOGE(TAG, "Error al enviar datos: %s", esp_err_to_name(result));
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}



void app_main(void) {
    ESP_LOGI(TAG, "[STATE 1] Inicio");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(peer_manager_on_data_recv));

    sensor_manager_init();
    power_manager_init();
    led_manager_init();
    //pump_controller_init();

    gpio_set_direction(BOOST_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BOOST_GPIO, 0);

    gpio_set_direction(CHARGER_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CHARGER_GPIO, GPIO_PULLDOWN_ONLY);

    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

    if (gpio_get_level(BUTTON_GPIO) == 0) {
        ESP_LOGI(TAG, "Botón presionado. Borrando memoria...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        nvs_flash_erase();
    }


    ESP_LOGI(TAG, "[STATE 3] Sincronizando hora");
    time_sync_request_time();


    int irrigation_day = 1;
    int irrigation_hour = 10;
    int ml_to_irrigate = 50;
    bool irrigation_done = false;

    if (time_sync_check_irrigation_time(irrigation_day, irrigation_hour)) {
        ESP_LOGI(TAG, "[STATE 5] Ejecutando riego");
        pump_controller_irrigate(ml_to_irrigate);
        irrigation_done = true;
    }

    ESP_LOGI(TAG, "[STATE 6] Leyendo sensores");
    float temp = 0, hum = 0;
    sensor_manager_read_aht20(&temp, &hum);
    float vbat = power_manager_get_battery_level();

    ESP_LOGI(TAG, "[STATE 7] Enviando datos al HUB");

    // Formateo a 2 decimales usando strings temporales
    char temp_str[8], hum_str[8], vbat_str[8];
    snprintf(temp_str, sizeof(temp_str), "%.2f", temp);
    snprintf(hum_str, sizeof(hum_str), "%.2f", hum);
    snprintf(vbat_str, sizeof(vbat_str), "%.2f", vbat);

    ESP_LOGI(TAG, "[STATE 7] Enviando datos al HUB");
    ESP_LOGI(TAG, "Datos: Temp=%s°C, Hum=%s%%, VBAT=%sV", temp_str, hum_str, vbat_str);

        response_received = false;
    if (response_semaphore == NULL) {
        response_semaphore = xSemaphoreCreateBinary();
    } else {
        xSemaphoreTake(response_semaphore, 0);
    }

   //vTaskDelay(pdMS_TO_TICKS(1000));

    //send_data_to_hub(atof(temp_str), atof(hum_str), atof(vbat_str), irrigation_done);

    

    int intento = 1; 
    while(!response_received && intento < 5){

        vTaskDelay(pdMS_TO_TICKS(2500));
        ESP_LOGI(TAG, "Intentando enviar msj %i", intento); 
        send_data_to_hub(atof(temp_str), atof(hum_str), atof(vbat_str), irrigation_done);
        intento++; 
    }
    

    wait_for_response_from_hub_with_retries(200);

    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "[STATE 8] Calculando tiempo de sleep");
    uint64_t sleep_time_us = time_sync_get_next_wakeup_time(irrigation_day, irrigation_hour);
    ESP_LOGI(TAG, "Durmiendo por %llu segundos", sleep_time_us / 1000000ULL);
    esp_sleep_enable_timer_wakeup(sleep_time_us);
    esp_deep_sleep_start();
}
