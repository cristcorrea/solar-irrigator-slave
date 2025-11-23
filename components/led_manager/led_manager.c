#include "led_manager.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define TAG "LED_MANAGER"
#define LED_GPIO 7
#define RMT_RESOLUTION_HZ 10000000 // 10 MHz

static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

// Duraciones para el protocolo WS2812B
static const rmt_symbol_word_t ws2812_zero = {
    .level0 = 1,
    .duration0 = 3,  // 0.3us
    .level1 = 0,
    .duration1 = 9   // 0.9us
};

static const rmt_symbol_word_t ws2812_one = {
    .level0 = 1,
    .duration0 = 9,  // 0.9us
    .level1 = 0,
    .duration1 = 3   // 0.3us
};

static const rmt_symbol_word_t ws2812_reset = {
    .level0 = 1,
    .duration0 = RMT_RESOLUTION_HZ / 1000000 * 50 / 2,
    .level1 = 0,
    .duration1 = RMT_RESOLUTION_HZ / 1000000 * 50 / 2,
};

static size_t ws2812_encoder_cb(const void *data, size_t data_size, size_t symbols_written,
                                size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    if (symbols_free < 8) return 0;
    size_t byte_index = symbols_written / 8;
    uint8_t *bytes = (uint8_t *)data;

    if (byte_index < data_size) {
        for (int i = 0; i < 8; i++) {
            symbols[i] = (bytes[byte_index] & (1 << (7 - i))) ? ws2812_one : ws2812_zero;
        }
        return 8;
    } else {
        symbols[0] = ws2812_reset;
        *done = true;
        return 1;
    }
}

void led_manager_init(void)
{
    ESP_LOGI(TAG, "Inicializando canal RMT para LED WS2812B");

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 1,
        .flags.with_dma = false
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &led_chan));

    const rmt_simple_encoder_config_t enc_cfg = {
        .callback = ws2812_encoder_cb,
    };

    ESP_ERROR_CHECK(rmt_new_simple_encoder(&enc_cfg, &led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_chan));
}

static void ws2812_send_color(uint8_t red, uint8_t green, uint8_t blue)
{
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0
    };

    uint8_t grb[3] = {green, red, blue}; // Formato WS2812B = GRB

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, grb, sizeof(grb), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

void led_manager_start_animation(void)
{
    ESP_LOGI(TAG, "LED ON (Magenta)");
    ws2812_send_color(16, 0, 16); // Magenta
}

void led_manager_stop_animation(void)
{
    ESP_LOGI(TAG, "LED OFF");
    ws2812_send_color(0, 0, 0); // Apagar LED
}


void led_check(){

    gpio_set_level(GPIO_NUM_0, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    led_manager_start_animation();
    vTaskDelay(pdMS_TO_TICKS(1000));
    led_manager_stop_animation();
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_NUM_0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void led_manager_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    // Si tu función interna se llama distinto, usa la correcta.
    ws2812_send_color(r, g, b);
}


void led_manager_start_animation_2(uint8_t r, uint8_t g, uint8_t b)
{   
    gpio_set_level(GPIO_NUM_0, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "Empieza Animación Led");     
    for (int i = 0; i < 5; i++)
    {
        led_manager_set_rgb(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(250));
        led_manager_set_rgb(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    gpio_set_level(GPIO_NUM_0, 0);
}