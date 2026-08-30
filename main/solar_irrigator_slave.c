#include <stdio.h>
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
#include "esp_now.h"
#include <time.h>
#include "esp_log.h"
#include "esp_log_level.h"
#include "sensor_manager.h"
#include "power_manager.h"
#include "led_manager.h"
#include "pump_controller.h"
#include "peer_manager.h"
#include "time_sync.h"
#include "button_manager.h"
#include "test_manager.h"

#include "sdkconfig.h"

#define TAG "MAIN"
#define BOOST_GPIO GPIO_NUM_2

#define SOC_PM_SUPPORT_EXT0_WAKEUP 1

#define NVS_NAMESPACE "storage"
#define NVS_KEY_CHANNEL "wifi_chan"
#define DEFAULT_WIFI_CHANNEL 1
#define DOCK_POLL_INTERVAL_S 20


#ifndef LOG_LOCAL_LEVEL
    #ifdef CONFIG_LOG_DEFAULT_LEVEL
        #define LOG_LOCAL_LEVEL CONFIG_LOG_MAXIMUN_LEVEL
    #else
        #define LOG_LOCAL_LEVEL 3 // Valor por defecto: INFO
    #endif
#endif

static SemaphoreHandle_t s_tx_done_sem = NULL;
static volatile esp_now_send_status_t s_last_tx_status = ESP_NOW_SEND_FAIL;
bool sleep_mode_active = true;


// Función para obtener el canal actual de la NVS
static uint8_t get_stored_wifi_channel(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    uint8_t channel = DEFAULT_WIFI_CHANNEL; // Valor por defecto

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        if (nvs_get_u8(my_handle, NVS_KEY_CHANNEL, &channel) != ESP_OK) {
            channel = DEFAULT_WIFI_CHANNEL;
        }
        nvs_close(my_handle);
    }
    // Asegurar rango válido (1-13)
    if (channel < 1 || channel > 13) channel = 1;
    return channel;
}

// Función para cambiar el canal y reiniciar
static void switch_channel_and_reboot(uint8_t current_channel)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    
    // Lógica simple de rotación: 1 -> 6 -> 11 -> 1 (Canales no solapados comunes)
    // O puedes hacer simplemente: next = current + 1; if(next>13) next=1;
    uint8_t next_channel = 1;
    if (current_channel < 6) next_channel = 6;
    else if (current_channel < 11) next_channel = 11;
    else next_channel = 1;

    if (err == ESP_OK) {
        nvs_set_u8(my_handle, NVS_KEY_CHANNEL, next_channel);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGW(TAG, "Cambiando canal WiFi de %d a %d y reiniciando...", current_channel, next_channel);
    } else {
        ESP_LOGE(TAG, "Error abriendo NVS para guardar canal");
    }

    // Reinicio forzado para aplicar cambios limpiamente
    esp_restart();
}

static void wait_hub_first_link_blocking(uint8_t current_channel) // <--- Nota: pasamos el canal actual como argumento
{
    uint8_t hub_mac[6];

    if (peer_manager_load_hub_mac(hub_mac) == 1)
    {
        ESP_LOGI(TAG, "HUB ya enlazado en NVS.");
        return;
    }

    ESP_LOGW(TAG, "Sin enlace con HUB. Canal actual: %d. Esperando...", current_channel);

    // Timestamp de inicio
    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t max_wait_ticks = pdMS_TO_TICKS(2000); // 2 segundos

    while (peer_manager_load_hub_mac(hub_mac) != 1)
    {
        // Verificar timeout
        if ((xTaskGetTickCount() - start_tick) > max_wait_ticks) {
            ESP_LOGW(TAG, "Timeout (2s) sin encontrar HUB en canal %d.", current_channel);
            switch_channel_and_reboot(current_channel); // Cambia canal y reinicia
        }

        led_manager_set_rgb(16, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(100)); // Espera más corta para chequear tiempo más seguido
        led_manager_set_rgb(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    led_manager_set_rgb(0, 0, 0);
    ESP_LOGI(TAG, "Enlace establecido en canal %d!", current_channel);
}

/* static void wait_hub_first_link_blocking(void)
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
} */


static void espnow_send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    s_last_tx_status = status;

    if (s_tx_done_sem)
    {
        xSemaphoreGive(s_tx_done_sem);
    }

    // Cambiamos 'target_addr' por 'addr'
    if (tx_info && tx_info->des_addr) 
    {
        const uint8_t *mac_addr = tx_info->des_addr;
        
        ESP_LOGI(TAG,
                 "ESP-NOW TX %s a %02X:%02X:%02X:%02X:%02X:%02X",
                 (status == ESP_NOW_SEND_SUCCESS) ? "OK" : "FAIL",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    }
    else
    {
        ESP_LOGI(TAG, "ESP-NOW TX %s (destino desconocido)",
                 (status == ESP_NOW_SEND_SUCCESS) ? "OK" : "FAIL");
    }
}

static void send_data_to_hub(float temp, float hum, float vbat, bool irrigation_done, uint8_t current_channel)
{
    // La MAC declarada debe ser la misma interfaz Wi-Fi STA usada como remitente ESP-NOW.
    uint8_t self_mac[6] = {0};
    esp_err_t err_mac = esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
    if (err_mac != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo leer MAC propia para telemetría: %s", esp_err_to_name(err_mac));
        return;
    }

    uint8_t target_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
    bool has_mac = (peer_manager_load_hub_mac(target_mac) == 1);

    char payload[64];
    // Formato esperado por el HUB actual: humedad,temperatura,voltaje,riego MAC
    snprintf(payload, sizeof(payload),
             "%.1f,%.1f,%.2f,%d %02X%02X%02X%02X%02X%02X",
             hum, temp, vbat, irrigation_done,
             self_mac[0], self_mac[1], self_mac[2],
             self_mac[3], self_mac[4], self_mac[5]);

    if (!esp_now_is_peer_exist(target_mac)) {
        esp_now_peer_info_t peer = {
            .channel = 0, // 0 usa el canal actual del wifi
            .ifidx   = WIFI_IF_STA,
            .encrypt = false
        };
        memcpy(peer.peer_addr, target_mac, 6);
        esp_now_add_peer(&peer);
    }

    const int max_retries = 3; 
    // Reducimos el delay entre reintentos para no estar mucho tiempo despiertos si falla
    const TickType_t tx_wait_ticks = pdMS_TO_TICKS(200); 
    bool sent_ok = false;

    for (int attempt = 0; attempt < max_retries; ++attempt)
    {
        if (s_tx_done_sem) xSemaphoreTake(s_tx_done_sem, 0);

        esp_err_t result = esp_now_send(target_mac, (uint8_t *)payload, strlen(payload));

        if (result == ESP_OK) {
            if (xSemaphoreTake(s_tx_done_sem, tx_wait_ticks) == pdTRUE) {
                if (s_last_tx_status == ESP_NOW_SEND_SUCCESS) {
                    sent_ok = true;
                    break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (sent_ok) {
        ESP_LOGI(TAG,
                 "Datos enviados a %s: %s",
                 has_mac ? "HUB" : "broadcast",
                 payload);
    } else {
        ESP_LOGE(TAG, "Fallo critico enviando al HUB en canal %u. Payload: %s",
                 (unsigned)current_channel,
                 payload);
        // OPCIONAL: Si falla el envío en modo normal, ¿queremos cambiar de canal?
        // Si el HUB es móvil o cambia de canal dinámicamente, SÍ.
        // Si no, podríamos solo dormir y reintentar luego.
        // Aquí aplico la lógica de cambiar canal si falla totalmente:
        
        // Descomenta la siguiente linea si quieres que cambie de canal al fallar envio
        // switch_channel_and_reboot(current_channel);
    }
}

/* static void send_data_to_hub(float temp, float hum, float vbat, bool irrigation_done)
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
            .ifidx   = WIFI_IF_STA,
            .encrypt = false
        };
        memcpy(peer.peer_addr, target_mac, 6);

        esp_err_t err = esp_now_add_peer(&peer);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "No se pudo agregar peer: %s", esp_err_to_name(err));
        }
    }

    const int max_retries = 3;
    const TickType_t tx_wait_ticks = pdMS_TO_TICKS(300);
    bool sent_ok = false;

    for (int attempt = 0; attempt < max_retries; ++attempt)
    {
        if (attempt > 0)
        {
            ESP_LOGW(TAG, "Reintento de telemetría %d/%d hacia el HUB",
                     attempt + 1, max_retries);
        }

        // Limpiar cualquier señal previo en el semáforo
        if (s_tx_done_sem)
        {
            xSemaphoreTake(s_tx_done_sem, 0);
        }

        esp_err_t result = esp_now_send(target_mac,
                                        (uint8_t *)payload,
                                        strlen(payload));

        if (result != ESP_OK)
        {
            // Error inmediato (no se ha puesto ni en cola)
            ESP_LOGE(TAG, "Error inmediato en esp_now_send (intento %d): %s",
                     attempt + 1, esp_err_to_name(result));
            // Si es un problema permanente (NOT_INIT, etc.), no tiene sentido seguir
            break;
        }

        if (!s_tx_done_sem)
        {
            // Sin semáforo: no podemos esperar al callback. Asumimos éxito si esp_now_send() fue OK
            ESP_LOGW(TAG, "s_tx_done_sem nulo, no se espera ACK de TX. Asumiendo envío OK.");
            sent_ok = true;
            break;
        }

        // Esperar a que el callback de TX nos diga SUCCESS o FAIL
        if (xSemaphoreTake(s_tx_done_sem, tx_wait_ticks) == pdTRUE)
        {
            if (s_last_tx_status == ESP_NOW_SEND_SUCCESS)
            {
                sent_ok = true;
                break;
            }
            else
            {
                ESP_LOGW(TAG,
                         "ESP-NOW TX FAIL al HUB (intento %d)",
                         attempt + 1);
                // se reintenta en siguiente vuelta del bucle
            }
        }
        else
        {
            ESP_LOGW(TAG,
                     "Timeout esperando callback de TX (intento %d)",
                     attempt + 1);
            // se reintenta en siguiente vuelta
        }

        // Pequeña pausa entre reintentos para no saturar el canal
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (sent_ok)
    {
        ESP_LOGI(TAG,
                 "Datos enviados a %s: %s",
                 has_mac ? "HUB" : "broadcast",
                 payload);
    }
    else
    {
        ESP_LOGE(TAG,
                 "No se pudo enviar telemetría al HUB tras %d intentos",
                 max_retries);
    }
}
 */


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

    /* ---------------------------------------------------------
     * 1. NUEVO: Leer el canal guardado en NVS al arrancar
     * --------------------------------------------------------- */
    uint8_t my_channel = get_stored_wifi_channel();
    ESP_LOGI(TAG, "Arrancando en canal WiFi: %d", my_channel);

    /*
     * VERIFICACIÓN MODO TEST
     */
#if CONFIG_TEST_MANAGER

#if CONFIG_TEST_MANAGER_START_ON_BOOT
    test_manager_start_cli(); 
    return;
#else
    test_manager_boot_at_window();

    work_mode_t mode = WORK_MODE_STANDARD;
    test_manager_nvs_get_mode(&mode);
    if (mode == WORK_MODE_TEST)
    {
        test_manager_start_cli();
        return;
    }
#endif 

#endif 

    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();

    SemaphoreHandle_t cfg_ready_sem = xSemaphoreCreateBinary();
    if (!cfg_ready_sem)
    {
        ESP_LOGE(TAG, "No se pudo crear cfg_ready_sem");
    }
    peer_manager_set_cfg_ready_semaphore(cfg_ready_sem);

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
    
    /* ---------------------------------------------------------
     * 2. CORREGIDO: Usar la variable my_channel, NO el numero 6 fijo
     * --------------------------------------------------------- */
    ESP_ERROR_CHECK(esp_wifi_set_channel(my_channel, WIFI_SECOND_CHAN_NONE));
    
    ESP_ERROR_CHECK(esp_now_init());
    
    s_tx_done_sem = xSemaphoreCreateBinary();
    if (!s_tx_done_sem)
    {
        ESP_LOGE(TAG, "No se pudo crear s_tx_done_sem");
    }
    else
    {
        ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    }

    ESP_ERROR_CHECK(esp_now_register_recv_cb(peer_manager_on_data_recv));

    // --- Periféricos propios de la esfera ---
    button_init();     
    button_erase();    
    gpio_power_init(); 
    sensor_manager_init();
    power_manager_init();
    led_manager_init();
    pump_controller_init();
    flow_sensor_controller_init();
    gpio_set_direction(BOOST_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BOOST_GPIO, 0);

    /*
     * MODO EMPAREJAMIENTO INICIAL (primer arranque)
     */
    if (first_boot)
    {
        if (!power_manager_is_hub_connected_stable())
        {
            led_manager_set_rgb(0, 0, 0);
            enter_deep_sleep((uint64_t)DOCK_POLL_INTERVAL_S * 1000000ULL);
            return;
        }

        /* ---------------------------------------------------------
         * 3. CORREGIDO: Pasar my_channel como argumento
         * --------------------------------------------------------- */
        wait_hub_first_link_blocking(my_channel);

        ESP_LOGI(TAG, "[STATE 2] Esperando configuración inicial del HUB (JSON + ts)...");

        bool cfg_received = cfg_ready_sem &&
                            xSemaphoreTake(cfg_ready_sem, pdMS_TO_TICKS(30000)) == pdTRUE;
        if (!cfg_received)
        {
            ESP_LOGW(TAG, "Timeout esperando configuracion inicial del HUB.");
            led_manager_set_rgb(0, 0, 0);
            enter_deep_sleep((uint64_t)DOCK_POLL_INTERVAL_S * 1000000ULL);
            return;
        }

        if (!peer_manager_load_irrigation_config(&cfg_hr, &cfg_min, &cfg_days, &cfg_ml))
        {
            ESP_LOGE(TAG, "Semáforo recibido pero NVS sin configuración válida.");
            for (;;)
            {
                led_manager_set_rgb(16, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(250));
                led_manager_set_rgb(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(750));
            }
        }

        ESP_LOGI(TAG, "Config inicial recibida.");
        peer_manager_perform_cfg_ack_handshake();

        /* Enviar una primera telemetria antes de dormir tras el emparejamiento. */
        float first_temp = 0.0f, first_hum = 0.0f;
        float first_vbat = power_manager_get_battery_level();
        esp_err_t first_sensor_err = sensor_manager_read_aht20(&first_temp, &first_hum);

        if (first_sensor_err == ESP_OK)
        {
            ESP_LOGI(TAG, "Enviando primera telemetria al HUB.");
            send_data_to_hub(first_temp, first_hum, first_vbat, false, my_channel);
        }
        else
        {
            ESP_LOGW(TAG,
                     "No se pudo leer AHT20 (%s); se omite la primera telemetria.",
                     esp_err_to_name(first_sensor_err));
        }

        led_manager_start_animation_2(0, 20, 0);

        uint64_t sleep_time_us = time_sync_get_next_wakeup_from_mask(cfg_days, cfg_hr, cfg_min);
        ESP_LOGI(TAG, "Primer ciclo: sleep time calculado.");

        enter_deep_sleep(sleep_time_us);
        return; 
    }

    /*
     * CICLO NORMAL
     */

    ESP_LOGI(TAG, "[STATE 3] Sincronizando hora");
    time_sync_request_time(); 
    bool irrigation_done = false;

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "[STATE 6] Leyendo sensores");
    float temp = 0, hum = 0;
    esp_err_t sensor_err = sensor_manager_read_aht20(&temp, &hum);
    bool sensors_ok = (sensor_err == ESP_OK);
    float vbat = power_manager_get_battery_level();

    char temp_str[8], hum_str[8], vbat_str[8];
    if (sensors_ok)
    {
        snprintf(temp_str, sizeof(temp_str), "%.2f", temp);
        snprintf(hum_str, sizeof(hum_str), "%.2f", hum);
    }
    else
    {
        snprintf(temp_str, sizeof(temp_str), "INVALID");
        snprintf(hum_str, sizeof(hum_str), "INVALID");
    }
    snprintf(vbat_str, sizeof(vbat_str), "%.2f", vbat);

    ESP_LOGI(TAG, "[STATE 7] Enviando datos al HUB");
    if (sensors_ok)
    {
        ESP_LOGI(TAG, "Datos: Temp=%s°C, Hum=%s%%, VBAT=%sV", temp_str, hum_str, vbat_str);
    }
    else
    {
        ESP_LOGW(TAG,
                 "Lectura AHT20 inválida (%s). No se enviará telemetría de temperatura/humedad.",
                 esp_err_to_name(sensor_err));
        ESP_LOGI(TAG, "Datos: Temp=%s, Hum=%s, VBAT=%sV", temp_str, hum_str, vbat_str);
    }

    // [STATE 5] Verificación de riego post-sync
    ESP_LOGI(TAG, "[STATE 5] Verificando riego post-sync");

    cfg_ok = peer_manager_load_irrigation_config(&cfg_hr, &cfg_min, &cfg_days, &cfg_ml);

    if (cfg_ok && time_sync_is_valid())
    {
        time_t now_t;
        time(&now_t);
        struct tm now;
        localtime_r(&now_t, &now);

        int idx = (now.tm_wday == 0) ? 6 : (now.tm_wday - 1);
        uint8_t bit_hoy = (uint8_t)(1u << (6 - idx));
        bool hoy_activo = (cfg_days & bit_hoy) != 0;

        struct tm target = now;
        target.tm_hour = cfg_hr;
        target.tm_min = cfg_min;
        target.tm_sec = 0;

        time_t target_t = mktime(&target);
        double delta_s = difftime(now_t, target_t); 

        if (hoy_activo && delta_s >= 0.0 && delta_s <= 180.0)
        {
            ESP_LOGI(TAG, "[STATE 5] Ejecutando riego: %u ml", (unsigned)cfg_ml);
            pump_controller_irrigate((int)cfg_ml);
        }
        else
        {
            ESP_LOGI(TAG, "[STATE 5] No corresponde riego ahora.");
        }
    }
    else
    {
        ESP_LOGW(TAG, "[STATE 5] Sin config/hora válida tras respuesta; no se riega.");
    }

    if (!sensors_ok)
    {
        ESP_LOGW(TAG, "Omito envío de telemetría: temperatura/humedad fuera de rango o lectura I2C inválida.");
    }
    else if (!pm_tx_try_lock(7000))
    {
        ESP_LOGW(TAG, "Omito envío: esperando ACK/timeout previo.");
    }
    else
    {
        /* ---------------------------------------------------------
         * 4. CORREGIDO: Pasar my_channel como 5to argumento
         * --------------------------------------------------------- */
        send_data_to_hub(temp, hum, vbat, irrigation_done, my_channel);

        if (cfg_ready_sem &&
            xSemaphoreTake(cfg_ready_sem, pdMS_TO_TICKS(2500)) != pdTRUE)
        {
            ESP_LOGW(TAG, "No llegó config/ts a tiempo; sigo con valores previos.");
        }
    }

    ESP_LOGI(TAG, "[STATE 8] Calculando tiempo de sleep");

    uint64_t sleep_time_us = 3600ULL * 1000000ULL; 

    if (peer_manager_load_irrigation_config(&cfg_hr, &cfg_min, &cfg_days, &cfg_ml))
    {
        sleep_time_us = time_sync_get_next_wakeup_from_mask(cfg_days, cfg_hr, cfg_min);
        ESP_LOGI(TAG, "Siguiente despertar: %02u:%02u", cfg_hr, cfg_min);
    }
    else
    {
        ESP_LOGW(TAG, "Sin configuración en NVS; usando fallback de 1 hora.");
    }

    peer_manager_perform_cfg_ack_handshake();
    led_manager_start_animation_2(0, 20, 0);
    enter_deep_sleep(sleep_time_us);
}
