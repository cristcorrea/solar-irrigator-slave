#ifndef PEER_MANAGER_H
#define PEER_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_now.h"
// peer_manager.h
#include "freertos/semphr.h"





#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

// otras declaraciones...

typedef struct {
    // Tiempo actual
    int hora_actual;
    int minuto_actual;
    int dia_actual;         // 0 = lunes, 6 = domingo

    // Configuración de riego (opcional)
    int hora_riego;
    int minuto_riego;
    uint8_t dias_riego_bin; // bits: L=bit6 ... D=bit0
    int mililitros;
} peer_data_t;

// Registra el semáforo que se dará cuando la config quede lista (ts + NVS OK)
void peer_manager_set_cfg_ready_semaphore(SemaphoreHandle_t sem);
void peer_manager_receive_hub_mac(void);
int  peer_manager_load_hub_mac(uint8_t *hub_mac); // hoy retorna 1/0 en tu .c
void peer_manager_save_hub_mac(const uint8_t *mac);
void peer_manager_on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len);
bool peer_manager_load_irrigation_config(uint8_t *hr, uint8_t *mn, uint8_t *days, uint16_t *ml);
void peer_manager_log_saved_irrigation_config(void);
bool pm_tx_try_lock(int timeout_ms);
void pm_tx_unlock(void);
void peer_manager_perform_cfg_ack_handshake(void);


#endif // PEER_MANAGER_H
