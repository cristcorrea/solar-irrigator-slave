
#include <stdint.h>
#include <stdbool.h>

void time_sync_request_time(void);
bool time_sync_check_irrigation_time(int irrigation_day, int irrigation_hour);
uint64_t time_sync_get_next_wakeup_time(int irrigation_day, int irrigation_hour);
uint64_t time_sync_get_next_wakeup_from_mask(uint8_t days_mask, uint8_t hour, uint8_t minute);

bool time_sync_is_valid(void);
void time_sync_set_epoch(uint32_t epoch_sec);
void time_sync_log_now(const char *who);