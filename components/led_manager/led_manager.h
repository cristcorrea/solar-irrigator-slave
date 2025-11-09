
#include <stdint.h> 

void led_manager_init(void);
void led_manager_start_animation(void);
void led_manager_stop_animation(void);
void led_check(); 
void led_manager_set_rgb(uint8_t r, uint8_t g, uint8_t b);  // <-- NUEVO

