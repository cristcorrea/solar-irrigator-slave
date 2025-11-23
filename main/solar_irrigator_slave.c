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
#include "button_manager.h"
#include <time.h>
#include "test_manager.h"

#define TAG "MAIN"
#define BOOST_GPIO GPIO_NUM_2

#define SOC_PM_SUPPORT_EXT0_WAKEUP 1

bool sleep_mode_active = true;

static void wait_hub_first_link_blocking(void)
{
    uint8_t hub_mac[6];

    // Si ya tenemos un HUB guardado en NVS, no hay nada que hacer
    if (peer_manager_load_hub_mac(hub_mac) == 1)
    {
        ESP_LOGI(TAG,
                 "HUB ya enlazado en NVS: %02X:%02X:%02X:%02X:%02X:%02X",
                 hub_mac[0], hub_mac[1], hub_mac[2],
                 hub_mac[3], hub_mac[4], hub_mac[5]);
        return;
    }
    else
    {
        ESP_LOGW(TAG, "Sin enlace con HUB. Esperando primer mensaje ESP-NOW del HUB...");
    }

    // Indicación visual: LED rojo tenue mientras se espera el primer enlace
    // (ajusta los valores si quieres otra intensidad/color)

    // Bucle bloqueante: se queda aquí hasta que peer_manager_on_data_recv()
    // almacene la MAC del HUB en NVS a través de peer_manager_save_hub_mac()
    while (peer_manager_load_hub_mac(hub_mac) != 1)
    {

        led_manager_set_rgb(16, 0, 0);
        ESP_LOGI(TAG, "Esperando mensaje del HUB...");
        vTaskDelay(pdMS_TO_TICKS(500));
        led_manager_set_rgb(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Apagar LED de espera
    led_manager_set_rgb(0, 0, 0);

    ESP_LOGI(TAG,
             "Primer enlace con HUB establecido: %02X:%02X:%02X:%02X:%02X:%02X",
             hub_mac[0], hub_mac[1], hub_mac[2],
             hub_mac[3], hub_mac[4], hub_mac[5]);
}

static void send_data_to_hub(float temp, float hum, float vbat, bool irrigation_done)
{
    uint8_t hub_mac[6];
    esp_base_mac_addr_get(hub_mac); // MAC propia (esfera)

    uint8_t target_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // default broadcast
    bool has_mac = false;

    // Intentar leer la MAC del hub desde NVS
    if (peer_manager_load_hub_mac(target_mac) == 1)
    {
        has_mac = true;
    }

    char payload[64];
    snprintf(payload, sizeof(payload),
             "%.1f,%.1f,%.2f,%d %02X%02X%02X%02X%02X%02X",
             hum, temp, vbat, irrigation_done,
             hub_mac[0], hub_mac[1], hub_mac[2],
             hub_mac[3], hub_mac[4], hub_mac[5]);

    // Si no existe el peer, lo agregamos
    if (!esp_now_is_peer_exist(target_mac))
    {
        esp_now_peer_info_t peer = {
            .channel = 0,
            .ifidx = WIFI_IF_STA,
            .encrypt = false};
        memcpy(peer.peer_addr, target_mac, 6);

        esp_err_t err = esp_now_add_peer(&peer);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "No se pudo agregar peer: %s", esp_err_to_name(err));
        }
    }

    esp_err_t result = esp_now_send(target_mac, (uint8_t *)payload, strlen(payload));

    if (result == ESP_OK)
    {
        ESP_LOGI(TAG, "Datos enviados a %s: %s", has_mac ? "HUB" : "broadcast", payload);
    }
    else
    {
        ESP_LOGE(TAG, "Error al enviar datos: %s", esp_err_to_name(result));
    }
}

static void enter_deep_sleep(uint64_t sleep_time_us)
{
    ESP_LOGI(TAG, "Durmiendo por %llu segundos",
             (unsigned long long)(sleep_time_us / 1000000ULL));
    esp_sleep_enable_timer_wakeup(sleep_time_us);

    ESP_LOGI(TAG, "Apagando periféricos y Wi-Fi");

    esp_now_deinit(); // Finaliza ESP-NOW (opcional)
    esp_wifi_stop();  // Detiene Wi-Fi (obligatorio para ahorro)

    // i2c_driver_delete(I2C_NUM_0); // Libera I2C si fue usado por sensores
    gpio_set_level(BOOST_GPIO, 0);
    // gpio_hold_en(BOOST_GPIO);  // Mantiene BOOST_GPIO en su último estado
    gpio_deep_sleep_hold_en(); // Habilita retención de GPIOs

    // Despertar por botón y por entrada de alimentación
    esp_sleep_enable_gpio_wakeup();
    esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON_GPIO, 0);

    if (sleep_mode_active)
    {
        esp_deep_sleep_start();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "[STATE 1] Inicio");

    ESP_ERROR_CHECK(nvs_flash_init());

    /*
     * VERIFICACIÓN MODO TEST
     */
#if CONFIG_TEST_MANAGER

#if CONFIG_TEST_MANAGER_START_ON_BOOT
    // Arranca DIRECTO al CLI de TEST. No ejecutes ventana de boot.
    test_manager_start_cli(); // no retorna
    return;
#else
    // Solo si NO forzamos TEST en el arranque, ofrecemos la ventana AT
    test_manager_boot_at_window();

    work_mode_t mode = WORK_MODE_STANDARD;
    test_manager_nvs_get_mode(&mode);
    if (mode == WORK_MODE_TEST)
    {
        test_manager_start_cli(); // no retorna
        return;
    }
#endif // CONFIG_TEST_MANAGER_START_ON_BOOT

#endif // CONFIG_TEST_MANAGER

    /*
     * FIN VERIFICACIÓN MODO TEST
     */

    // Zona horaria por defecto (el HUB la ajustará al fijar el epoch real)
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();

    // Semáforo para notificar que llegó config/ts desde el HUB
    SemaphoreHandle_t cfg_ready_sem = xSemaphoreCreateBinary();
    if (!cfg_ready_sem)
    {
        ESP_LOGE(TAG, "No se pudo crear cfg_ready_sem");
    }
    peer_manager_set_cfg_ready_semaphore(cfg_ready_sem);

    // Cargar configuración de riego (si existe)
    uint8_t cfg_hr = 0, cfg_min = 0, cfg_days = 0;
    uint16_t cfg_ml = 0;
    bool cfg_ok = peer_manager_load_irrigation_config(&cfg_hr, &cfg_min, &cfg_days, &cfg_ml);
    if (cfg_ok)
    {
        ESP_LOGI(TAG, "Config NVS: %02u:%02u | dias=0b%07b | ml=%u",
                 cfg_hr, cfg_min, cfg_days, cfg_ml);
    }
    else
    {
        ESP_LOGI(TAG, "No hay configuración en NVS");
    }
    peer_manager_log_saved_irrigation_config();

    // ¿Ya tenemos HUB enlazado?
    uint8_t hub_mac[6] = {0};
    bool have_hub_mac = (peer_manager_load_hub_mac(hub_mac) == 1);
    bool first_boot = !(cfg_ok && have_hub_mac);

    if (first_boot)
    {
        ESP_LOGI(TAG, "[STATE 2] Primer arranque: sin HUB o sin configuración. Modo emparejamiento.");
    }
    else
    {
        ESP_LOGI(TAG, "[STATE 2] HUB y configuración ya presentes. Ciclo normal.");
    }

    // --- Wi-Fi + ESP-NOW ---
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(peer_manager_on_data_recv));

    // --- Periféricos propios de la esfera ---
    button_init();     // Inicia configuracion del switch
    button_erase();    // Función que realiza borrado memoria NVS
    gpio_power_init(); // Configura GPIOs 10 y 4
    sensor_manager_init();
    power_manager_init();
    led_manager_init();
    pump_controller_init();
    flow_sensor_controller_init();
    gpio_set_direction(BOOST_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BOOST_GPIO, 0);

    /*
     * MODO EMPAREJAMIENTO INICIAL (primer arranque)
     * No se leen sensores ni se envía telemetría: solo se espera al HUB.
     */
    if (first_boot)
    {
        // Esperar a que el HUB nos hable al menos una vez para fijar su MAC en NVS.
        wait_hub_first_link_blocking();

        ESP_LOGI(TAG, "[STATE 2] Esperando configuración inicial del HUB (JSON + ts)...");

        if (cfg_ready_sem &&
            xSemaphoreTake(cfg_ready_sem, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(TAG, "cfg_ready_sem no liberado en primer emparejamiento.");
        }

        if (!peer_manager_load_irrigation_config(&cfg_hr, &cfg_min, &cfg_days, &cfg_ml))
        {
            ESP_LOGE(TAG, "Semáforo recibido pero NVS sin configuración válida. Mantengo la esfera despierta.");
            // Bucle de error visible: LED rojo parpadeando
            for (;;)
            {
                led_manager_set_rgb(16, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(250));
                led_manager_set_rgb(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(750));
            }
        }

        ESP_LOGI(TAG,
                 "Config inicial recibida: %02u:%02u | dias=0b%07b | ml=%u",
                 cfg_hr, cfg_min, cfg_days, cfg_ml);

        // 🔹 NUEVO: hacer el handshake de ACK con el HUB
        peer_manager_perform_cfg_ack_handshake();
        led_manager_start_animation_2(0, 20, 0);
        // En este punto hay:
        //  - MAC del HUB en NVS
        //  - Config de riego válida en NVS
        //  - Epoch fijado por el HUB (campo "ts" del JSON)
        uint64_t sleep_time_us = time_sync_get_next_wakeup_from_mask(cfg_days, cfg_hr, cfg_min);
        ESP_LOGI(TAG, "Primer ciclo: siguiente despertar a %02u:%02u (mask=0x%02X)",
                 cfg_hr, cfg_min, cfg_days);

        enter_deep_sleep(sleep_time_us);
        return; // por seguridad; deep_sleep no debería retornar
    }

    /*
     * CICLO NORMAL
     */

    ESP_LOGI(TAG, "[STATE 3] Sincronizando hora");
    time_sync_request_time(); // en producción, la hora real vendrá del HUB vía 'ts'
    bool irrigation_done = false;

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "[STATE 6] Leyendo sensores");
    float temp = 0, hum = 0;
    sensor_manager_read_aht20(&temp, &hum);
    float vbat = power_manager_get_battery_level();

    // Formateo a 2 decimales usando strings temporales
    char temp_str[8], hum_str[8], vbat_str[8];
    snprintf(temp_str, sizeof(temp_str), "%.2f", temp);
    snprintf(hum_str, sizeof(hum_str), "%.2f", hum);
    snprintf(vbat_str, sizeof(vbat_str), "%.2f", vbat);

    ESP_LOGI(TAG, "[STATE 7] Enviando datos al HUB");
    ESP_LOGI(TAG, "Datos: Temp=%s°C, Hum=%s%%, VBAT=%sV", temp_str, hum_str, vbat_str);

    // [STATE 5] Verificación de riego post-sync (tras recibir 'ts' y guardar config)
    ESP_LOGI(TAG, "[STATE 5] Verificando riego post-sync");

    cfg_ok = peer_manager_load_irrigation_config(&cfg_hr, &cfg_min, &cfg_days, &cfg_ml);

    if (cfg_ok && time_sync_is_valid())
    {
        time_t now_t;
        time(&now_t);
        struct tm now;
        localtime_r(&now_t, &now);

        // 0=Lun..6=Dom  | máscara L=bit6 ... D=bit0
        int idx = (now.tm_wday == 0) ? 6 : (now.tm_wday - 1);
        uint8_t bit_hoy = (uint8_t)(1u << (6 - idx));
        bool hoy_activo = (cfg_days & bit_hoy) != 0;

        // Objetivo HH:MM:00 de HOY (ya con TZ correcta gracias a 'ts')
        struct tm target = now;
        target.tm_hour = cfg_hr;
        target.tm_min = cfg_min;
        target.tm_sec = 0;

        time_t target_t = mktime(&target);
        double delta_s = difftime(now_t, target_t); // ahora - objetivo

        // Ventana de +180 s desde HH:MM:00 para tolerar latencias/derivas
        if (hoy_activo && delta_s >= 0.0 && delta_s <= 180.0)
        {
            ESP_LOGI(TAG, "[STATE 5] Ejecutando riego: %u ml (delta=%.0f s)",
                     (unsigned)cfg_ml, delta_s);
            pump_controller_irrigate((int)cfg_ml);
            // si quiere reportarlo en el próximo envío, active esta marca:
            // irrigation_done = true;
        }
        else
        {
            ESP_LOGI(TAG,
                     "[STATE 5] No corresponde riego ahora. now=%02d:%02d:%02d objetivo=%02u:%02u delta=%.0f s",
                     now.tm_hour, now.tm_min, now.tm_sec, cfg_hr, cfg_min, delta_s);
        }
    }
    else
    {
        ESP_LOGW(TAG, "[STATE 5] Sin config/hora válida tras respuesta; no se riega.");
    }

    // Envío de telemetría protegido por guardia TX
    if (!pm_tx_try_lock(7000))
    {
        ESP_LOGW(TAG, "Omito envío: esperando ACK/timeout previo.");
    }
    else
    {
        send_data_to_hub(atof(temp_str), atof(hum_str), atof(vbat_str), irrigation_done);

        if (cfg_ready_sem &&
            xSemaphoreTake(cfg_ready_sem, pdMS_TO_TICKS(2500)) != pdTRUE)
        {
            ESP_LOGW(TAG, "No llegó config/ts a tiempo; sigo con valores previos.");
        }
    }

    ESP_LOGI(TAG, "[STATE 8] Calculando tiempo de sleep");

    uint64_t sleep_time_us = 3600ULL * 1000000ULL; // fallback

    if (peer_manager_load_irrigation_config(&cfg_hr, &cfg_min, &cfg_days, &cfg_ml))
    {
        sleep_time_us = time_sync_get_next_wakeup_from_mask(cfg_days, cfg_hr, cfg_min);
        ESP_LOGI(TAG, "Siguiente despertar según NVS: %02u:%02u (mask=0x%02X)",
                 cfg_hr, cfg_min, cfg_days);
    }
    else
    {
        ESP_LOGW(TAG, "Sin configuración en NVS; usando fallback de 1 hora.");
    }

    // Antes de dormir, asegurar doble ACK de configuración con el HUB
    peer_manager_perform_cfg_ack_handshake();

    led_manager_start_animation_2(0, 20, 0);
    enter_deep_sleep(sleep_time_us);
}
