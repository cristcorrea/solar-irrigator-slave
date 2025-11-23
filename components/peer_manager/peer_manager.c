#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "peer_manager.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "led_manager.h"
#include "cJSON.h"
#include "time.h"
#include "time_sync.h"
#include "esp_mac.h"

#define TAG "PEER_MANAGER"
#define NVS_NAMESPACE "storage"
#define NVS_KEY_HUB_MAC "hub_mac"

#define NVS_KEY_CFG_HR "cfg_hr"
#define NVS_KEY_CFG_MIN "cfg_min"
#define NVS_KEY_CFG_DAYS "cfg_days"
#define NVS_KEY_CFG_ML "cfg_ml"
#define NVS_KEY_CFG_SRC "cfg_src"     // opcional: quién envió la config
#define NVS_KEY_CFG_VALID "cfg_valid" // 1 cuando hay config válida

#define CFG_ACK_MAX_RETRIES 5         // número máximo de reintentos
#define CFG_ACK_TIMEOUT_MS 300        // espera por ACK del HUB en cada intento
#define CFG_ACK_STR "CFG_OK"          // mensaje de ACK hacia el HUB
#define CFG_ACK_BACK_STR "ACK_END" // mensaje de ACK de vuelta desde el HUB

// NUEVO: semáforo + estado para el ACK del HUB
static SemaphoreHandle_t s_cfg_ack_sem = NULL;
static bool s_cfg_ack_pending = false;
static uint8_t s_cfg_ack_hub_mac[6] = {0};

static SemaphoreHandle_t s_cfg_ready_sem = NULL;

// --- TX guard mínimal ---
static volatile bool s_tx_busy = false;
static TimerHandle_t s_tx_guard = NULL;

void peer_manager_set_cfg_ready_semaphore(SemaphoreHandle_t sem)
{
    s_cfg_ready_sem = sem;
}

static void s_tx_guard_cb(TimerHandle_t xTimer)
{
    s_tx_busy = false; // libera por timeout
}

bool pm_tx_try_lock(int timeout_ms)
{
    if (s_tx_busy)
        return false; // ya hay uno en vuelo
    s_tx_busy = true; // tomo el lock
    if (!s_tx_guard)
    {
        s_tx_guard = xTimerCreate("pm_tx_guard",
                                  pdMS_TO_TICKS(timeout_ms),
                                  pdFALSE, NULL, s_tx_guard_cb);
    }
    xTimerChangePeriod(s_tx_guard, pdMS_TO_TICKS(timeout_ms), 0);
    xTimerStart(s_tx_guard, 0);
    return true;
}

void pm_tx_unlock(void)
{
    s_tx_busy = false; // libera por ACK
    if (s_tx_guard)
        xTimerStop(s_tx_guard, 0);
}
// --- fin TX guard ---

int peer_manager_load_hub_mac(uint8_t *hub_mac)
{
    int result = 1;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK)
    {
        size_t mac_size = 6;
        if (nvs_get_blob(handle, NVS_KEY_HUB_MAC, hub_mac, &mac_size) == ESP_OK)
        {
            ESP_LOGI(TAG, "MAC del hub cargada desde NVS: %02X:%02X:%02X:%02X:%02X:%02X",
                     hub_mac[0], hub_mac[1], hub_mac[2], hub_mac[3], hub_mac[4], hub_mac[5]);
        }
        else
        {
            ESP_LOGW(TAG, "MAC del hub no encontrada en NVS");
            result = 0;
        }
        nvs_close(handle);
    }
    else
    {
        ESP_LOGE(TAG, "No se pudo abrir NVS para leer MAC");
        result = 0;
    }

    return result;
}

void peer_manager_save_hub_mac(const uint8_t *mac)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK)
    {
        nvs_set_blob(handle, NVS_KEY_HUB_MAC, mac, 6);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "MAC del HUB guardada en NVS");
    }
    else
    {
        ESP_LOGE(TAG, "Error guardando MAC en NVS: %s", esp_err_to_name(err));
    }
}

static esp_err_t peer_manager_save_irrigation_config(const peer_data_t *d, const uint8_t *hub_mac)
{
    // Validación mínima: rangos
    if (d->hora_riego > 23 || d->minuto_riego > 59)
        return ESP_ERR_INVALID_ARG;
    // dias_riego_bin: 7 bits (0..127), ml: >=0 (usamos uint16 más abajo)

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;

    // Guardamos todo y confirmamos al final
    if ((err = nvs_set_u8(h, NVS_KEY_CFG_HR, (uint8_t)d->hora_riego)) != ESP_OK)
        goto out;
    if ((err = nvs_set_u8(h, NVS_KEY_CFG_MIN, (uint8_t)d->minuto_riego)) != ESP_OK)
        goto out;
    if ((err = nvs_set_u8(h, NVS_KEY_CFG_DAYS, (uint8_t)d->dias_riego_bin)) != ESP_OK)
        goto out;
    if ((err = nvs_set_u16(h, NVS_KEY_CFG_ML, (uint16_t)((d->mililitros < 0) ? 0 : d->mililitros))) != ESP_OK)
        goto out;

    if (hub_mac)
    {
        // Guardamos quién envió la config (no crítico si falla)
        (void)nvs_set_blob(h, NVS_KEY_CFG_SRC, hub_mac, 6);
    }

    // Marcamos config válida SOLO si todo lo anterior fue bien
    if ((err = nvs_set_u8(h, NVS_KEY_CFG_VALID, 1)) != ESP_OK)
        goto out;

    err = nvs_commit(h);

out:
    nvs_close(h);
    return err;
}

static bool hay_que_regar_hoy(const peer_data_t *data)
{
    if (data->dia_actual < 0 || data->dia_actual > 6)
        return false;

    uint8_t bit = (uint8_t)(1u << (6 - data->dia_actual));
    return (data->dias_riego_bin & bit) != 0;
}

static void handle_non_json_payload(const esp_now_recv_info_t *recv_info,
                                    const char *payload)
{
    if (!recv_info || !payload)
        return;

    /* 0) ACK del HUB para nuestro CFG_OK */
    if (strncmp(payload, CFG_ACK_BACK_STR, strlen(CFG_ACK_BACK_STR)) == 0)
    {
        ESP_LOGI(TAG, "ACK_CFG_OK recibido desde el HUB.");
        if (s_cfg_ack_sem)
        {
            xSemaphoreGive(s_cfg_ack_sem);
        }
        return;
    }

    /* 1) Handshake: HELLO del HUB -> respondemos con nuestra MAC */
    if (strncmp(payload, "HELLO_ESFERA", strlen("HELLO_ESFERA")) == 0)
    {
        ESP_LOGI(TAG, "HELLO_ESFERA recibido; enviando respuesta al HUB.");

        // Leer MAC propia (Wi-Fi STA)
        uint8_t self_mac[6] = {0};
        esp_err_t err_mac = esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
        if (err_mac != ESP_OK)
        {
            ESP_LOGW(TAG, "No se pudo leer MAC propia para HELLO_HUB: %s",
                     esp_err_to_name(err_mac));
            return;
        }

        // Formato: HELLO_HUB,AABBCCDDEEFF
        char ack[64] = {0};
        int len = snprintf(ack, sizeof(ack),
                           "HELLO_HUB,%02X%02X%02X%02X%02X%02X",
                           self_mac[0], self_mac[1], self_mac[2],
                           self_mac[3], self_mac[4], self_mac[5]);
        if (len <= 0)
            return;

        // Asegurar peer unicast al HUB (canal 0 = canal actual del Wi-Fi)
        if (!esp_now_is_peer_exist(recv_info->src_addr))
        {
            esp_now_peer_info_t p = (esp_now_peer_info_t){0};
            p.channel = 0; // ⚠️ IMPORTANTE: no fijar canal distinto
            p.ifidx = WIFI_IF_STA;
            p.encrypt = false;
            memcpy(p.peer_addr, recv_info->src_addr, 6);
            (void)esp_now_add_peer(&p);
        }

        esp_err_t es = esp_now_send(recv_info->src_addr,
                                    (const uint8_t *)ack,
                                    (size_t)len);
        if (es != ESP_OK)
        {
            ESP_LOGW(TAG, "Falló envío de HELLO_HUB: %s", esp_err_to_name(es));
        }
        else
        {
            ESP_LOGI(TAG, "HELLO_HUB enviado al HUB.");
        }

        return;
    }

    /* 2) Otros mensajes no-JSON: se ignoran por ahora */
    ESP_LOGW(TAG, "Payload no-JSON recibido: '%s' (no se procesa).", payload);
}

void peer_manager_on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
{
    if (!recv_info || !data || data_len <= 0)
    {
        ESP_LOGW(TAG, "on_data_recv: argumentos inválidos");
        return;
    }
    pm_tx_unlock();
    // 1) Guardar MAC del HUB si no existe o cambió
    uint8_t stored[6];
    bool have = (peer_manager_load_hub_mac(stored) == 1);
    if (!have || memcmp(stored, recv_info->src_addr, 6) != 0)
    {
        peer_manager_save_hub_mac(recv_info->src_addr);

        // Asegurar peer unicast al HUB
        if (!esp_now_is_peer_exist(recv_info->src_addr))
        {
            esp_now_peer_info_t p = {
                .channel = 0,
                .ifidx = WIFI_IF_STA,
                .encrypt = false};
            memcpy(p.peer_addr, recv_info->src_addr, 6);
            (void)esp_now_add_peer(&p);
        }
    }

    // 2) Copia segura a buffer C-string
    char buffer[256] = {0};
    int tocpy = MIN(data_len, (int)sizeof(buffer) - 1);
    memcpy(buffer, data, (size_t)tocpy);
    buffer[tocpy] = '\0';

    ESP_LOGI(TAG, "Mensaje recibido de %02X:%02X:%02X:%02X:%02X:%02X: %s",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5],
             buffer);

    // 3) Si no es JSON, puede ser un mensaje de control (HELLO, etc.)
    const char *p = buffer;
    while (*p == ' ' || *p == '\t')
        p++;

    if (*p != '{')
    {
        handle_non_json_payload(recv_info, p);
        return;
    }

    cJSON *root = cJSON_Parse(buffer);
    if (!root)
    {
        ESP_LOGW(TAG, "JSON inválido; se ignora.");

        return;
    }

    /* ===== (A) PONER EN HORA SI VIENE "ts" DEL HUB ===== */
    cJSON *jTs = cJSON_GetObjectItemCaseSensitive(root, "ts");
    if (cJSON_IsNumber(jTs) && jTs->valuedouble > 1600000000.0)
    { // umbral razonable
        time_sync_set_epoch((uint32_t)jTs->valuedouble);
        time_sync_log_now("SYNC from HUB");
    }
    else
    {
        ESP_LOGW(TAG, "Config sin 'ts' o inválido; dependerá del reloj local.");
    }
    /* ===== FIN (A) ===== */

    // --- Parseo campos de configuración ---
    // diasRiego: número de 7 dígitos (ej: 1111111) -> máscara L=bit6 ... D=bit0
    uint8_t mask = 0;
    cJSON *jDays = cJSON_GetObjectItemCaseSensitive(root, "diasRiego");
    if (cJSON_IsNumber(jDays))
    {
        char seven[16];
        snprintf(seven, sizeof(seven), "%07d", jDays->valueint);
        for (int i = 0; i < 7; ++i)
        {
            if (seven[i] == '1')
                mask |= (uint8_t)(1u << (6 - i));
        }
    }

    // horaRiego: "HH:MM"
    uint8_t hr = 0, mn = 0;
    cJSON *jHora = cJSON_GetObjectItemCaseSensitive(root, "horaRiego");
    if (cJSON_IsString(jHora) && jHora->valuestring)
    {
        unsigned int H = 0, M = 0;
        if (sscanf(jHora->valuestring, "%u:%u", &H, &M) == 2 && H <= 23 && M <= 59)
        {
            hr = (uint8_t)H;
            mn = (uint8_t)M;
        }
    }

    // ml: entero (0..65535)
    int ml = 0;
    cJSON *jMl = cJSON_GetObjectItemCaseSensitive(root, "ml");
    if (cJSON_IsNumber(jMl))
    {
        ml = jMl->valueint;
        if (ml < 0)
            ml = 0;
        if (ml > 65535)
            ml = 65535;
    }

    // Construir config
    peer_data_t cfg = (peer_data_t){0};
    cfg.hora_riego = hr;
    cfg.minuto_riego = mn;
    cfg.dias_riego_bin = mask;
    cfg.mililitros = ml;

    // 4) **USO de hay_que_regar_hoy** con día actual local (0=lun...6=dom)
    time_t now;
    time(&now);
    struct tm ti;
    localtime_r(&now, &ti);
    int dia_0_lun_6_dom = (ti.tm_wday == 0) ? 6 : (ti.tm_wday - 1);
    cfg.dia_actual = dia_0_lun_6_dom;

    if (hay_que_regar_hoy(&cfg))
    {
        ESP_LOGI(TAG, "Hoy corresponde riego (dia_actual=%d, mask=0x%02X).",
                 cfg.dia_actual, cfg.dias_riego_bin);
    }
    else
    {
        ESP_LOGI(TAG, "Hoy NO corresponde riego (dia_actual=%d, mask=0x%02X).",
                 cfg.dia_actual, cfg.dias_riego_bin);
    }

    // 5) Guardar en NVS y marcar ACK pendiente
    esp_err_t se = peer_manager_save_irrigation_config(&cfg, recv_info->src_addr);
    if (se == ESP_OK)
    {
        ESP_LOGI(TAG, "Configuración (JSON) guardada en NVS. Marcando ACK pendiente...");

        // Asegurar peer unicast al HUB (por si acaso)
        if (!esp_now_is_peer_exist(recv_info->src_addr))
        {
            esp_now_peer_info_t p = {0};
            p.channel = 0; // usar canal actual del Wi-Fi
            p.ifidx = WIFI_IF_STA;
            p.encrypt = false;
            memcpy(p.peer_addr, recv_info->src_addr, 6);
            (void)esp_now_add_peer(&p);
        }

        // Guardar MAC del HUB para el handshake de ACK
        memcpy(s_cfg_ack_hub_mac, recv_info->src_addr, 6);
        s_cfg_ack_pending = true;

        // Despertar a app_main para que procese la nueva config y haga el handshake
        if (s_cfg_ready_sem)
            xSemaphoreGive(s_cfg_ready_sem);
    }
    else
    {
        ESP_LOGE(TAG, "Error guardando configuración (JSON): %s", esp_err_to_name(se));
    }

    cJSON_Delete(root);
}

bool peer_manager_load_irrigation_config(uint8_t *hr, uint8_t *mn, uint8_t *days, uint16_t *ml)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
    {
        return false;
    }

    uint8_t valid = 0;
    if (nvs_get_u8(h, NVS_KEY_CFG_VALID, &valid) != ESP_OK || valid != 1)
    {
        nvs_close(h);
        return false;
    }

    uint8_t _hr = 0, _mn = 0, _days = 0;
    uint16_t _ml = 0;

    if (nvs_get_u8(h, NVS_KEY_CFG_HR, &_hr) != ESP_OK ||
        nvs_get_u8(h, NVS_KEY_CFG_MIN, &_mn) != ESP_OK ||
        nvs_get_u8(h, NVS_KEY_CFG_DAYS, &_days) != ESP_OK ||
        nvs_get_u16(h, NVS_KEY_CFG_ML, &_ml) != ESP_OK)
    {
        nvs_close(h);
        return false;
    }

    nvs_close(h);

    if (hr)
        *hr = _hr;
    if (mn)
        *mn = _mn;
    if (days)
        *days = _days;
    if (ml)
        *ml = _ml;

    return true;
}

void peer_manager_log_saved_irrigation_config(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "NVS no disponible (%s).", esp_err_to_name(err));
        return;
    }

    uint8_t valid = 0;
    if (nvs_get_u8(h, NVS_KEY_CFG_VALID, &valid) != ESP_OK || valid != 1)
    {
        ESP_LOGI(TAG, "No hay configuración de riego válida en NVS (cfg_valid != 1).");
        nvs_close(h);
        return;
    }

    uint8_t hr = 0, mn = 0, days = 0;
    uint16_t ml = 0;
    bool ok = (nvs_get_u8(h, NVS_KEY_CFG_HR, &hr) == ESP_OK) && (nvs_get_u8(h, NVS_KEY_CFG_MIN, &mn) == ESP_OK) && (nvs_get_u8(h, NVS_KEY_CFG_DAYS, &days) == ESP_OK) && (nvs_get_u16(h, NVS_KEY_CFG_ML, &ml) == ESP_OK);

    uint8_t src_mac[6] = {0};
    size_t mac_sz = sizeof(src_mac);
    bool have_src = (nvs_get_blob(h, NVS_KEY_CFG_SRC, src_mac, &mac_sz) == ESP_OK && mac_sz == 6);

    nvs_close(h);

    if (!ok)
    {
        ESP_LOGW(TAG, "Config marcada como válida pero faltan claves en NVS.");
        return;
    }

    // armo cadena binaria "LMXJVSD" -> "1/0"
    char bin[8]; // 7 + '\0'
    for (int i = 0; i < 7; ++i)
    {
        bin[i] = (days & (1u << (6 - i))) ? '1' : '0';
    }
    bin[7] = '\0';

    // lista legible de días
    const char *nombres[7] = {"Lun", "Mar", "Mi\u00E9", "Jue", "Vie", "S\u00E1b", "Dom"};
    char lista[64] = {0};
    size_t used = 0;
    for (int i = 0; i < 7; ++i)
    {
        if (days & (1u << (6 - i)))
        {
            int n = snprintf(lista + used, sizeof(lista) - used, "%s%s",
                             (used ? ", " : ""), nombres[i]);
            if (n > 0)
                used += (size_t)n;
            if (used >= sizeof(lista))
                break;
        }
    }
    if (used == 0)
        strncpy(lista, "ninguno", sizeof(lista) - 1);

    if (have_src)
    {
        ESP_LOGI(TAG,
                 "Config NVS -> hora=%02u:%02u, dias=0b%s (%s), ml=%u, src=%02X:%02X:%02X:%02X:%02X:%02X",
                 hr, mn, bin, lista, ml,
                 src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
    }
    else
    {
        ESP_LOGI(TAG,
                 "Config NVS -> hora=%02u:%02u, dias=0b%s (%s), ml=%u",
                 hr, mn, bin, lista, ml);
    }
}


void peer_manager_perform_cfg_ack_handshake(void)
{
    if (!s_cfg_ack_pending)
    {
        ESP_LOGI(TAG, "Sale de la función peer_manager_perform_cfg_ack_handshake %i", s_cfg_ack_pending);
        return;
    }

    ESP_LOGI(TAG, "Iniciando handshake de ACK de configuración con el HUB.");

    // Asegurar peer unicast al HUB
    if (!esp_now_is_peer_exist(s_cfg_ack_hub_mac))
    {
        esp_now_peer_info_t p = {0};
        p.channel = 0;              // usar canal actual del Wi-Fi
        p.ifidx   = WIFI_IF_STA;
        p.encrypt = false;
        memcpy(p.peer_addr, s_cfg_ack_hub_mac, 6);
        (void)esp_now_add_peer(&p);
    }

    // Crear semáforo de ACK si aún no existe
    if (!s_cfg_ack_sem)
    {
        s_cfg_ack_sem = xSemaphoreCreateBinary();
        if (!s_cfg_ack_sem)
        {
            ESP_LOGE(TAG, "No se pudo crear s_cfg_ack_sem");
            return;
        }
    }

    // Limpiar posibles señales antiguas
    while (xSemaphoreTake(s_cfg_ack_sem, 0) == pdTRUE)
    {
        // vaciar cola
    }

    const char *ack_str = CFG_ACK_STR;

    for (int attempt = 1; attempt <= CFG_ACK_MAX_RETRIES; ++attempt)
    {
        esp_err_t es = esp_now_send(s_cfg_ack_hub_mac,
                                    (const uint8_t *)ack_str,
                                    strlen(ack_str));
        if (es != ESP_OK)
        {
            ESP_LOGW(TAG,
                     "Falló envío de CFG_OK (intento %d): %s",
                     attempt, esp_err_to_name(es));
        }
        else
        {
            ESP_LOGI(TAG,
                     "CFG_OK enviado (intento %d). Esperando ACK_CFG_OK...",
                     attempt);
        }

        // Esperar ACK_CFG_OK del HUB
        if (xSemaphoreTake(s_cfg_ack_sem,
                           pdMS_TO_TICKS(CFG_ACK_TIMEOUT_MS)) == pdTRUE)
        {
            ESP_LOGI(TAG, "ACK_CFG_OK recibido desde el HUB.");
            s_cfg_ack_pending = false;

            return;
        }

        ESP_LOGW(TAG,
                 "Timeout esperando ACK_CFG_OK (intento %d). Reintentando...",
                 attempt);
    }

    ESP_LOGW(TAG,
             "No se recibió ACK_CFG_OK tras %d intentos. "
             "Continuando ejecución.",
             CFG_ACK_MAX_RETRIES);

    // Si prefieres reintentar en el próximo ciclo, podrías dejarlo en true.
    s_cfg_ack_pending = false;
}
