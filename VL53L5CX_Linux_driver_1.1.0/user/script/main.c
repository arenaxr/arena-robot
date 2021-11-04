#include <unistd.h>
#include <signal.h>
#include <dlfcn.h>

#include <stdio.h>
#include <string.h>

#include "vl53l5cx_api.h"
#include "range.h"

int main()
{
    int status = 0;
    VL53L5CX_Configuration 	Dev_list[NUM_DEVICES];
    
    initialize_sensors(Dev_list, 1, 15);

	int loop = 0;
	while(loop < 30)
	{
		get_range();
		loop++;
	}
	stop_ranging(Dev_list);
	return 1;

}