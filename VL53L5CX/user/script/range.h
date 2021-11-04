#define NUM_DEVICES 3


int initialize_sensors(VL53L5CX_Configuration Dev_list[], uint8_t use_8x8, uint8_t freq);
int stop_ranging(VL53L5CX_Configuration Dev_list[]);
int get_range();

