#include "time_sync.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include <time.h>
#include <sys/time.h>

#define TAG "TIME_SYNC"

static struct tm current_time;

void time_sync_request_time(void)
{
    // Simula recepción de hora del hub
    time_t rawtime;
    time(&rawtime);
    localtime_r(&rawtime, &current_time);
    ESP_LOGI(TAG, "Hora simulada sincronizada: %02d:%02d, día %d",
             current_time.tm_hour, current_time.tm_min, current_time.tm_wday);
}

bool time_sync_check_irrigation_time(int irrigation_day, int irrigation_hour)
{
    time_t rawtime;
    time(&rawtime);
    localtime_r(&rawtime, &current_time);
    return (current_time.tm_wday == irrigation_day && current_time.tm_hour == irrigation_hour);
}

uint64_t time_sync_get_next_wakeup_time(int irrigation_day, int irrigation_hour)
{
    time_t rawtime;
    time(&rawtime);
    struct tm now_tm;
    localtime_r(&rawtime, &now_tm);

    struct tm irrigation_tm = now_tm;
    irrigation_tm.tm_sec = 0;
    irrigation_tm.tm_min = 0;
    irrigation_tm.tm_hour = irrigation_hour;
    irrigation_tm.tm_mday += (irrigation_day - now_tm.tm_wday + 7) % 7;

    time_t target_time = mktime(&irrigation_tm);
    time_t delta = target_time - rawtime;

    if (delta < 3600)
    {
        return delta * 1000000ULL;
    }
    else
    {
        return 3600000000ULL; // 1 hora en microsegundos
    }
}

uint64_t time_sync_get_next_wakeup_from_mask(uint8_t days_mask, uint8_t hour, uint8_t minute)
{
    time_t now_t;
    time(&now_t);
    struct tm now;
    localtime_r(&now_t, &now);

    // 0=Lun..6=Dom desde tm_wday (0=Dom..6=Sáb)
    int today_idx = (now.tm_wday == 0) ? 6 : (now.tm_wday - 1);

    for (int off = 0; off < 7; ++off) {
        int di = (today_idx + off) % 7;
        uint8_t bit = (uint8_t)(1u << (6 - di));  // L=bit6 ... D=bit0
        if ((days_mask & bit) == 0) continue;

        struct tm target = now;
        target.tm_sec  = 0;
        target.tm_min  = minute;
        target.tm_hour = hour;

        // Si es hoy pero ya pasó la hora:minuto, salta al próximo día válido
        if (off == 0) {
            int now_min = now.tm_hour * 60 + now.tm_min;
            int trg_min = hour * 60 + minute;
            if (trg_min <= now_min) continue;
        }
        target.tm_mday += off;

        time_t target_t = mktime(&target);
        time_t delta = target_t - now_t;

        // Seguridad y tope de 1 hora
        if (delta <= 0) delta = 1;                 // evita 0 us
        if (delta > 3600) delta = 3600;            // nunca dormir más de 1 h

        return (uint64_t)delta * 1000000ULL;       // a microsegundos
    }

    // Si la máscara no tiene días activos: dormir 1 h
    return 3600ULL * 1000000ULL;
}


bool time_sync_is_valid(void)
{
    time_t now = 0;
    time(&now);
    return now >= 1672531200; // 2023-01-01
}

void time_sync_set_epoch(uint32_t epoch_sec)
{
    struct timeval tv = {.tv_sec = (time_t)epoch_sec, .tv_usec = 0};
    settimeofday(&tv, NULL);

    // Zona horaria Europa/Roma (CET/CEST)
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();

    time_t now;
    time(&now);
    struct tm t;
    localtime_r(&now, &t);
    ESP_LOGI("TIME_SYNC", "Hora fijada por hub: %02d:%02d:%02d wday=%d", t.tm_hour, t.tm_min, t.tm_sec, t.tm_wday);
}

void time_sync_log_now(const char *who)
{
    time_t now;
    time(&now);
    struct tm t;
    localtime_r(&now, &t);
    ESP_LOGI("TIME_SYNC", "[%s] Ahora: %02d:%02d:%02d wday=%d epoch=%ld",
             who ? who : "", t.tm_hour, t.tm_min, t.tm_sec, t.tm_wday, (long)now);
}
