#include <stdbool.h>

#define GPIO_CHARGER_STATUS GPIO_NUM_4
#define GPIO_POWER_CONNECTED GPIO_NUM_10

void power_manager_init(void);

float power_manager_get_battery_level(void);

void gpio_power_init(void);

bool power_manager_is_hub_connected(void);

bool power_manager_is_hub_connected_stable(void);
