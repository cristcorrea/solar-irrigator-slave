#include <stdbool.h>
#include <stdint.h>

void pump_controller_init(void);
void pump_controller_stop(void);  
uint16_t pump_controller_irrigate(int ml, bool *cut_by_flow);
void flow_sensor_controller_init(void);
