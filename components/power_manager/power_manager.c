#include "power_manager.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "POWER_MANAGER"
#define VBAT_ADC_CHANNEL ADC_CHANNEL_3 // GPIO3 = ADC1_CH3
#define VOLTAGE_DIVIDER_RATIO (100.0 / (100.0 + 68.0))


static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;
static bool s_gpio_power_initialized = false;
static bool s_hub_connected_stable = false;

void gpio_power_init()
{

    gpio_config_t io10_config = {
        .pin_bit_mask = 1ULL << GPIO_POWER_CONNECTED,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

        gpio_config_t io4_config = {
        .pin_bit_mask = 1ULL << GPIO_CHARGER_STATUS,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io10_config);
    gpio_config(&io4_config);
    s_gpio_power_initialized = true;
}

bool power_manager_is_hub_connected(void)
{
    if (!s_gpio_power_initialized)
    {
        gpio_power_init();
    }

    return gpio_get_level(GPIO_POWER_CONNECTED) == 1;
}

bool power_manager_is_hub_connected_stable(void)
{
    bool sample = power_manager_is_hub_connected();

    for (int i = 1; i < 5; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        if (power_manager_is_hub_connected() != sample)
        {
            return s_hub_connected_stable;
        }
    }

    s_hub_connected_stable = sample;
    return s_hub_connected_stable;
}

void power_manager_init(void) {
    // Inicializar ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, VBAT_ADC_CHANNEL, &chan_config));

    // Configurar calibración (line fitting)
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));
    
    ESP_LOGI(TAG, "ADC y calibración inicializados correctamente");
}

float power_manager_get_battery_level(void) {
    int raw = 0;
    int voltage_mv = 0;

    esp_err_t result = adc_oneshot_read(adc_handle, VBAT_ADC_CHANNEL, &raw);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "ADC raw: %d", raw);
    } else {
        ESP_LOGE(TAG, "Error leyendo ADC");
        return -1.0;
    }

    if (adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv) == ESP_OK) {
        float voltage = (voltage_mv / 1000.0f) / VOLTAGE_DIVIDER_RATIO;
        ESP_LOGI(TAG, "Voltaje calibrado: %dmV (%.2fV real)", voltage_mv, voltage);
        return voltage;
    } else {
        ESP_LOGE(TAG, "Error en calibración ADC");
        return -1.0;
    }
}
