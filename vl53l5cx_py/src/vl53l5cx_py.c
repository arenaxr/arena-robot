/*
 * vl53l5cx_py.c
 * Created by Perry Naseck and Vikram Bhat on 11/4/21.
 * 
 * Copyright (c) 2021, The CONIX Research Center
 * All rights reserved.
 * 
 * This file also contains source code originating from the VL53L5CX Ultra
 * Lite Driver licensed under the BSD 3-clause 'New' or 'Revised' License.
 * Copyright (c) 2020, STMicroelectronics
 * All rights reserved.
 * 
 * This source code is licensed under the BSD-3-Clause license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "vl53l5cx_api.h"
#include "math.h"

#define VL53L5CX_PI_COMMS_ERROR		-2

#include "vl53l5cx_py.h"

int32_t vl53l5cx_py_comms_init(VL53L5CX_Platform* p_platform,
							   const char *dev_path) {
	p_platform->fd = open(dev_path, O_RDONLY);
	if (p_platform->fd == -1) {
		printf("VL53L5CX_PY Failed to open %s\n", dev_path);
		return VL53L5CX_PI_COMMS_ERROR;
	}

	if (ioctl(p_platform->fd, I2C_SLAVE, 0x29) <0) {
		printf("VL53L5CX_PY Could not speak to the device on the i2c bus\n");
		return VL53L5CX_PI_COMMS_ERROR;
	}

	printf("VL53L5CX_PY Opened ST TOF Dev = %d\n", p_platform->fd);

	return 0;
}

int32_t vl53l5cx_py_comms_close(VL53L5CX_Platform* p_platform) {
	close(p_platform->fd);
	return 0;
}

int32_t vl53l5cx_py_init(VL53L5CX_Configuration* dev_conf,
                         const char* dev_path, uint16_t target_addr,
						 uint8_t freq) {
	if (freq > 15) {
		printf("VL53L5CX_PY Ranging Frequency too high for 8x8 (<=15hz)\n");
		return -1;
	}

	int32_t status = 0;
	(&(dev_conf->platform))->address = VL53L5CX_DEFAULT_I2C_ADDRESS;
	status |= vl53l5cx_py_comms_init(&(dev_conf)->platform, dev_path);
    if (status) {
        printf("VL53L5CX_PY comms init failed\n");
        return -1;
    }
	
	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/
	/* (Optional) Check if there is a VL53L5CX sensor connected */
	/*
	uint8_t isAlive = 0;
	status = vl53l5cx_is_alive(dev_conf, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L5CX not detected at requested address\n");
		return status;
	}
	*/

	status = vl53l5cx_set_i2c_address(dev_conf, target_addr);
	if (status) {
		printf("VL53L5CX_PY Set I2C address Failed \n");
		vl53l5cx_py_comms_close(&(dev_conf)->platform);
		return status;
	}
	printf("VL53L5CX_PY Set I2C address success 0x%x\n",
	       (&(dev_conf->platform))->address);

	status = vl53l5cx_init(dev_conf);
	if (status) {
		printf("VL53L5CX_PY ULD Loading failed\n");
		vl53l5cx_py_comms_close(&(dev_conf)->platform);
		return status;
	}
	printf("VL53L5CX_PY Initialized\n");
	
	status = vl53l5cx_set_resolution(dev_conf, VL53L5CX_RESOLUTION_8X8);
	if (status) {
		printf("VL53L5CX_PY set resolution failed\n");
		vl53l5cx_py_comms_close(&(dev_conf)->platform);
		return status;
	}
	printf("VL53L5CX_PY set resolution success\n");

	status = vl53l5cx_set_ranging_frequency_hz(dev_conf, freq);
	if (status) {
		printf("VL53L5CX_PY set ranging frequency failed\n");
		vl53l5cx_py_comms_close(&(dev_conf)->platform);
		return status;
	}
	printf("VL53L5CX_PY set ranging frequency success\n");

	printf("VL53L5CX_PY ULD ready ! (Version : %s)\n",
			VL53L5CX_API_REVISION);
	return 0;
}

int32_t vl53l5cx_py_close(VL53L5CX_Configuration* dev_conf) {
	int32_t status = 0;
	status = vl53l5cx_py_comms_close(&(dev_conf)->platform);
	if (status == 0) printf("VL53L5CX_PY closed\n");

	return status;
}

int32_t vl53l5cx_py_start_ranging(VL53L5CX_Configuration* dev_conf) {
	int32_t status = 0;
	status = vl53l5cx_start_ranging(dev_conf);
	if (status == 0) printf("VL53L5CX_PY started ranging\n");

	return status;
}

int32_t vl53l5cx_py_stop_ranging(VL53L5CX_Configuration* dev_conf) {
	int32_t status = 0;
	status = vl53l5cx_stop_ranging(dev_conf);
	if (status == 0) printf("VL53L5CX_PY stopped ranging\n");
	return status;
}

int32_t vl53l5cx_py_get_range(VL53L5CX_Configuration* dev_conf,
                              VL53L5CX_ResultsData* results) {
	int32_t status = 0;
	uint8_t isReady = 0;

	isReady = 0;
	while (!isReady) {	
		status = vl53l5cx_check_data_ready(dev_conf, &isReady); //Wait for device to be ready
	}
	vl53l5cx_get_ranging_data(dev_conf, results); //Get ranging data and store in results

	return status;
}
