#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "driver/usb_serial_jtag.h"
#include "test_manager.h"
#include "sensor_manager.h"
#include "power_manager.h"
#include "pump_controller.h"
#include "led_manager.h"
#include "time_sync.h"

#define TM_NVS_NS      "cfg"
#define TM_NVS_KEYMODE "work_mode"

/* ---------- Utilidades I/O ---------- */

// --- Inicialización perezosa del driver USB-Serial-JTAG ---
static void tm_usb_init_once(void)
{
    static bool s_inited = false;
    if (s_inited) return;

    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };
    if (usb_serial_jtag_driver_install(&cfg) == ESP_OK) {
        s_inited = true;
    }
}

// Drena cualquier byte pendiente en RX (evita "ERROR" por basura inicial)
static void tm_rx_drain(void) {
    uint8_t ch;
    while (usb_serial_jtag_read_bytes(&ch, 1, 0) == 1) { /* descarta */ }
}

static void tm_put(const char *s) { usb_serial_jtag_write_bytes(s, strlen(s), 0); }
static void tm_putln(const char *s){ tm_put(s); usb_serial_jtag_write_bytes("\r\n",2,0); }

static void tm_printf(const char *fmt, ...) {
    char b[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n > 0) usb_serial_jtag_write_bytes(b, n, 0);
}

// No se usa, pero queda si quieres un readline bloqueante
static int tm_readline(char *buf, size_t cap, TickType_t wait)
{
    size_t i = 0;
    for (;;) {
        uint8_t ch;
        int n = usb_serial_jtag_read_bytes(&ch, 1, wait);
        if (n <= 0) continue;
        if (ch == '\r' || ch == '\n') break;  // acepta CR o LF
        if (i < cap - 1) buf[i++] = (char)ch;
    }
    buf[i] = 0;
    return (int)i;
}

/* ---------- Persistencia de modo en NVS ---------- */
esp_err_t test_manager_nvs_get_mode(work_mode_t *out_mode)
{
    if (!out_mode) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open(TM_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) { *out_mode = WORK_MODE_STANDARD; return ESP_OK; }

    uint8_t v = 0;
    esp_err_t err2 = nvs_get_u8(h, TM_NVS_KEYMODE, &v);
    nvs_close(h);

    if (err2 == ESP_OK && v == 1) *out_mode = WORK_MODE_TEST;
    else                          *out_mode = WORK_MODE_STANDARD;

    return ESP_OK;
}

esp_err_t test_manager_nvs_set_mode(work_mode_t mode)
{
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(TM_NVS_NS, NVS_READWRITE, &h));
    ESP_ERROR_CHECK(nvs_set_u8(h, TM_NVS_KEYMODE, (mode == WORK_MODE_TEST) ? 1 : 0));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
    return ESP_OK;
}

/* ---------- Ventana AT de arranque ---------- */
void test_manager_boot_at_window(void)
{
#if CONFIG_TEST_MANAGER_BOOT_AT_WINDOW
    tm_usb_init_once();
    tm_rx_drain();

    tm_putln("BOOT-AT: AT+MODE=TEST | AT+MODE=STD | AT+MODE?");
    usb_serial_jtag_write_bytes("> ", 2, 0);  // prompt

    const TickType_t tick = pdMS_TO_TICKS(10);
    int elapsed = 0;
    char line[96];
    int len = 0;

    while (elapsed < CONFIG_TEST_MANAGER_BOOT_AT_MS) {
        uint8_t ch;
        int n = usb_serial_jtag_read_bytes(&ch, 1, tick);
        if (n == 1) {
            // --- ECO LOCAL + edición básica ---
            if (ch == 0x08 || ch == 0x7F) {                 // backspace
                if (len > 0) { len--; usb_serial_jtag_write_bytes("\b \b", 3, 0); }
                continue;
            }
            if (ch == '\r' || ch == '\n') {                 // fin de línea: CR o LF
                usb_serial_jtag_write_bytes("\r\n", 2, 0);  // eco de nueva línea
                if (len == 0) { usb_serial_jtag_write_bytes("> ", 2, 0); continue; }

                line[len] = 0;

                // --- Parseo mínimo ---
                char up[96]; strncpy(up, line, sizeof(up)); up[sizeof(up)-1] = 0;
                for (char *p = up; *p; ++p) *p = (char)toupper((unsigned)*p);

                if (!strcmp(up, "AT") || !strcmp(up, "ATZ")) {
                    tm_putln("OK");
                } else if (!strcmp(up, "AT+MODE?")) {
                    work_mode_t m; test_manager_nvs_get_mode(&m);
                    tm_putln((m == WORK_MODE_TEST) ? "MODE:TEST" : "MODE:STD");
                    tm_putln("OK");
                } else if (!strncmp(up, "AT+MODE=", 8)) {
                    bool to_test = strstr(up, "=TEST") != NULL;
                    test_manager_nvs_set_mode(to_test ? WORK_MODE_TEST : WORK_MODE_STANDARD);
                    tm_putln("OK");
                    if (to_test) {
                        tm_putln("REBOOTING TO TEST...");
                        vTaskDelay(pdMS_TO_TICKS(150));
                        esp_restart(); // no retorna
                    } else {
                        break; // seguir flujo estándar
                    }
                } else {
                    tm_putln("ERROR");
                }

                len = 0;
                usb_serial_jtag_write_bytes("> ", 2, 0);
                continue;
            }

            // eco de caracteres imprimibles
            usb_serial_jtag_write_bytes((const char *)&ch, 1, 0);
            if (len < (int)sizeof(line) - 1) line[len++] = (char)ch;
        }
        elapsed += 10;
    }

    // Si expiró el tiempo y quedó una línea parcialmente escrita, procésala
    if (len > 0) {
        line[len] = 0;
        char up[96]; strncpy(up, line, sizeof(up)); up[sizeof(up)-1] = 0;
        for (char *p = up; *p; ++p) *p = (char)toupper((unsigned)*p);
        if (!strncmp(up, "AT+MODE=", 8)) {
            bool to_test = strstr(up, "=TEST") != NULL;
            test_manager_nvs_set_mode(to_test ? WORK_MODE_TEST : WORK_MODE_STANDARD);
            tm_putln("OK");
            if (to_test) {
                tm_putln("REBOOTING TO TEST...");
                vTaskDelay(pdMS_TO_TICKS(150));
                esp_restart();
            }
        }
    }
#endif
}

/* ---------- CLI AT completo (modo TEST) ---------- */
static void tm_help(void)
{
    tm_putln("OK");
    tm_putln("AT                -> OK");
    tm_putln("AT+HELP           -> this help");
    tm_putln("AT+MODE?          -> STD|TEST (NVS)");
    tm_putln("AT+MODE=STD|TEST  -> save to NVS (reboot if STD)");
    tm_putln("AT+SENS?          -> T,H,VBAT");
    tm_putln("AT+IRR=<ml>       -> irrigate <ml> (blocking)");
    tm_putln("AT+STOP           -> abort irrigation");
    tm_putln("AT+LED=<r,g,b>    -> set LED (0..255)");
    tm_putln("AT+TIME?          -> log current time");
    tm_putln("AT+DEEPSLEEP=<s>  -> deep sleep s seconds");
}

void test_manager_start_cli(void)
{
    tm_usb_init_once();
    tm_rx_drain();

    // Inicializa solo lo necesario para pruebas
    sensor_manager_init();
    power_manager_init();
    pump_controller_init();
    led_manager_init();

    tm_putln("\r\n=== TEST MANAGER (AT) ===");
    usb_serial_jtag_write_bytes("> ", 2, 0);

    char line[160];
    int len = 0;

    for (;;) {
        uint8_t ch;
        int n = usb_serial_jtag_read_bytes(&ch, 1, portMAX_DELAY);
        if (n != 1) continue;

        // --- ECO LOCAL + edición básica ---
        if (ch == 0x08 || ch == 0x7F) {                          // backspace
            if (len > 0) { len--; usb_serial_jtag_write_bytes("\b \b", 3, 0); }
            continue;
        }
        if (ch == '\r' || ch == '\n') {                          // fin de línea: CR o LF
            usb_serial_jtag_write_bytes("\r\n", 2, 0);
            if (len == 0) { usb_serial_jtag_write_bytes("> ", 2, 0); continue; }

            line[len] = 0;

            // UPPERCASE de la línea para parseo
            char up[160]; strncpy(up, line, sizeof(up)); up[sizeof(up)-1] = 0;
            for (char *p = up; *p; ++p) *p = (char)toupper((unsigned)*p);

            // --- comandos ---
            if (!strcmp(up, "AT") || !strcmp(up, "ATZ")) {
                tm_putln("OK");
            }
            else if (!strcmp(up, "AT+HELP")) {
                tm_help();
            }
            else if (!strcmp(up, "AT+MODE?")) {
                work_mode_t m; test_manager_nvs_get_mode(&m);
                tm_putln((m == WORK_MODE_TEST) ? "MODE:TEST" : "MODE:STD");
                tm_putln("OK");
            }
            else if (!strncmp(up, "AT+MODE=", 8)) {
                bool to_test = strstr(up, "=TEST") != NULL;
                work_mode_t cur; test_manager_nvs_get_mode(&cur);
                work_mode_t tgt = to_test ? WORK_MODE_TEST : WORK_MODE_STANDARD;
                test_manager_nvs_set_mode(tgt); tm_putln("OK");
                if (tgt == WORK_MODE_STANDARD && cur != tgt) {
                    tm_putln("REBOOTING TO STD...");
                    vTaskDelay(pdMS_TO_TICKS(200));
                    esp_restart();
                }
            }
            else if (!strcmp(up, "AT+SENS?")) {
                float t=0,h=0; sensor_manager_read_aht20(&t,&h);
                float v = power_manager_get_battery_level();
                char b[96]; int m = snprintf(b, sizeof(b), "T=%.2f,H=%.2f,VBAT=%.2f\r\n", t,h,v);
                if (m > 0) usb_serial_jtag_write_bytes(b, m, 0);
                tm_putln("OK");
            }
            else if (!strncmp(up, "AT+IRR=", 7)) {
                int ml = atoi(line + 7);
                if (ml <= 0) tm_putln("ERROR");
                else {
                    pump_controller_stop(); vTaskDelay(pdMS_TO_TICKS(50));
                    pump_controller_irrigate(ml);
                    tm_putln("OK");
                }
            }
            else if (!strcmp(up, "AT+STOP")) {
                pump_controller_stop(); tm_putln("OK");
            }
            else if (!strncmp(up, "AT+LED=", 7)) {
                int r=0,g=0,bv=0;
                if (sscanf(line + 7, "%d,%d,%d", &r, &g, &bv) == 3 &&
                    r>=0&&r<=255 && g>=0&&g<=255 && bv>=0&&bv<=255) {
                    led_manager_set_rgb((uint8_t)r,(uint8_t)g,(uint8_t)bv);
                    tm_putln("OK");
                } else tm_putln("ERROR");
            }
            else if (!strcmp(up, "AT+TIME?")) {
                time_sync_log_now("AT"); tm_putln("OK");
            }
            else if (!strncmp(up, "AT+DEEPSLEEP=", 13)) {
                int s = atoi(line + 13);
                if (s <= 0) tm_putln("ERROR");
                else {
                    tm_putln("OK"); tm_putln("ENTERING DEEP SLEEP...");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_sleep_enable_timer_wakeup((uint64_t)s * 1000000ULL);
                    esp_deep_sleep_start();
                }
            }
            else {
                tm_putln("ERROR");
            }

            // listo para la siguiente línea
            len = 0;
            usb_serial_jtag_write_bytes("> ", 2, 0);
            continue;
        }

        // eco de caracteres imprimibles
        usb_serial_jtag_write_bytes((const char *)&ch, 1, 0);
        if (len < (int)sizeof(line) - 1) line[len++] = (char)ch;
    }
}

/* ===================================================================== */
/* Compatibilidad hacia atrás: solo si aún llamas test_mode_* en el repo */
/* ===================================================================== */
esp_err_t test_mode_nvs_get(work_mode_t *o)  { return test_manager_nvs_get_mode(o); }
esp_err_t test_mode_nvs_set(work_mode_t m)   { return test_manager_nvs_set_mode(m); }
void test_mode_start_cli(void)               { test_manager_start_cli(); }
