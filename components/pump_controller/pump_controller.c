#include "pump_controller.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <math.h>

#define TAG "PUMP_CTRL"
#define PUMP_GPIO GPIO_NUM_6
#define FLOW_GPIO GPIO_NUM_1

#define ML_PER_PULSE   0.00545f       // ~0.005449...
#define PULSES_PER_ML  183.5f


static volatile bool s_stop_requested = false;  // <-- NUEVO
static volatile int pulse_count = 0;
static int flow_timeout_ms = 5000; // tiempo maximo sin pulsos antes de parar la bomba

void pump_controller_init(void)
{

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);

    gpio_set_level(PUMP_GPIO, 1);

    ESP_LOGI(TAG, "Pump controller initialized");
}

void IRAM_ATTR flow_sensor_isr_handler(void *arg)
{
    pulse_count++;
}

void flow_sensor_controller_init()
{

    gpio_config_t io_config = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << FLOW_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE};

    gpio_config(&io_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(FLOW_GPIO, flow_sensor_isr_handler, NULL);
    ESP_LOGI(TAG, "Flow sensor initialized");
}

uint32_t irrigation_get_pulse_target(uint32_t volume_ml)
{
    // pulses = volume_ml * PULSES_PER_ML, con redondeo al entero más cercano
    float pulses_f = (float)volume_ml * PULSES_PER_ML;
    if (pulses_f < 1.0f) {
        pulses_f = 1.0f;
    }
    return (uint32_t)(pulses_f + 0.5f);
}

float irrigation_pulses_to_ml(uint32_t pulses)
{
    return (float)pulses * ML_PER_PULSE;
}


void pump_controller_irrigate(int ml)
{
    s_stop_requested = false;

    if (ml <= 0)
    {
        ESP_LOGW(TAG, "ml <= 0; nada que regar");
        return;
    }
    if (ML_PER_PULSE <= 0.0f)
    {
        ESP_LOGE(TAG, "ML_PER_PULSE inválido (%.3f)", (double)ML_PER_PULSE);
        return;
    }

    // Pulsos necesarios (redondeo hacia arriba: al menos 1 pulso si ml>0)
    int pulses_needed = irrigation_get_pulse_target(ml);
    if (pulses_needed < 1)
        pulses_needed = 1;

    // Contadores (pulse_count lo actualiza la ISR)
    pulse_count = 0;
    int last_pulse_count = 0;
    int no_pulse_cycles = 0; // <-- LOCAL, se acumula correctamente

    const int check_period_ms = 100;             // ventana de muestreo
    const int max_no_pulse_ms = flow_timeout_ms; // p.ej. 5000 ms

    // (Opcional) timeout absoluto de seguridad (10 min)
#ifndef MAX_IRRIGATION_MSat
#define MAX_IRRIGATION_MS 600000
#endif
    int64_t t_start_us = esp_timer_get_time();

    // Enciende bomba (tu hardware es activo-bajo: 0 = ON)
    gpio_set_level(PUMP_GPIO, 0);

    while (pulse_count < pulses_needed)
    {
        vTaskDelay(pdMS_TO_TICKS(check_period_ms));
        if (s_stop_requested)
        {
            ESP_LOGW(TAG, "Manual STOP");
            break;
        }

        int current_pulse_count = pulse_count;
        int pulse_diff = current_pulse_count - last_pulse_count;
        last_pulse_count = current_pulse_count;

        // Frecuencia media en la última ventana
        float freq_hz = (float)pulse_diff / ((float)check_period_ms / 1000.0f);
        ESP_LOGI(TAG, "Pulses %d/%d (+%d) ~ %.2f Hz",
                 current_pulse_count, pulses_needed, pulse_diff, freq_hz);

        if (pulse_diff == 0)
        {
            // No llegaron pulsos en esta ventana: acumulamos
            no_pulse_cycles++;
            if (no_pulse_cycles * check_period_ms >= max_no_pulse_ms)
            {
                ESP_LOGE(TAG, "Flow timeout (%d ms sin pulsos). Parando bomba.", max_no_pulse_ms);
                break;
            }
        }
        else
        {
            // Hubo flujo: reiniciamos el contador de “sin flujo”
            no_pulse_cycles = 0;
        }

        // Timeout absoluto de seguridad
        int64_t elapsed_ms = (esp_timer_get_time() - t_start_us) / 1000;
        if (elapsed_ms > MAX_IRRIGATION_MS)
        {
            ESP_LOGE(TAG, "Tiempo máximo de riego excedido (%d ms). Parando bomba.", MAX_IRRIGATION_MS);
            break;
        }
    }

    // Apaga bomba (activo-bajo: 1 = OFF)
    gpio_set_level(PUMP_GPIO, 1);

    float ml_entregados = irrigation_pulses_to_ml(pulse_count);
    ESP_LOGI(TAG, "Irrigation finished. Pulses=%d/%d -> %.1f ml",
             pulse_count, pulses_needed, ml_entregados);
}

void pump_controller_stop(void)
{
    s_stop_requested = true;
    gpio_set_level(PUMP_GPIO, 1); // OFF inmediato
}
