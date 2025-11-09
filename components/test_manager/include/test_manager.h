#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { WORK_MODE_STANDARD = 0, WORK_MODE_TEST = 1 } work_mode_t;

esp_err_t test_manager_nvs_get_mode(work_mode_t *out_mode);
esp_err_t test_manager_nvs_set_mode(work_mode_t mode);
void test_manager_boot_at_window(void);   // retorna siempre
void test_manager_start_cli(void);        // no retorna (modo TEST)

#ifdef __cplusplus
}
#endif
