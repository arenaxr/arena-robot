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
 
//sin(pi/2) = 1
//sin(-pi/2) = -1
//cos(pi/2) = 0
//cos(-pi/2) = 0 
const double transform_forward[4][4] = {
	{1, 0, 0, 0},
	{0, 1, 0, 0},
	{0, 0, 1, 0},
	{0, 0, 0, 1}

};
//-pi/2
const double transform_left[4][4] = {
	{0, 0, -1, 0},
	{0, 1, 0, 0},
	{1, 0, 0, 0},
	{0, 0, 0, 1}

};
//pi/2
const double transform_right[4][4] = {
	{0, 0, 1, 0},
	{0, 1, 0, 0},
	{-1, 0, 0, 0},
	{0, 0, 0, 1}

};

static double SinOfPitch[64], CosOfPitch[64], SinOfYaw[64], CosOfYaw[64];
const double VL53L5_Zone_Pitch8x8[64] = {
		59.00,64.00,67.50,70.00,70.00,67.50,64.00,59.00,
		64.00,70.00,72.90,74.90,74.90,72.90,70.00,64.00,
		67.50,72.90,77.40,80.50,80.50,77.40,72.90,67.50,
		70.00,74.90,80.50,85.75,85.75,80.50,74.90,70.00,
		70.00,74.90,80.50,85.75,85.75,80.50,74.90,70.00,
		67.50,72.90,77.40,80.50,80.50,77.40,72.90,67.50,
		64.00,70.00,72.90,74.90,74.90,72.90,70.00,64.00,
		59.00,64.00,67.50,70.00,70.00,67.50,64.00,59.00
};
 
const double VL53L5_Zone_Yaw8x8[64] = {
		135.00,125.40,113.20, 98.13, 81.87, 66.80, 54.60, 45.00,
		144.60,135.00,120.96,101.31, 78.69, 59.04, 45.00, 35.40,
		156.80,149.04,135.00,108.45, 71.55, 45.00, 30.96, 23.20,
		171.87,168.69,161.55,135.00, 45.00, 18.45, 11.31,  8.13,
		188.13,191.31,198.45,225.00,315.00,341.55,348.69,351.87,
		203.20,210.96,225.00,251.55,288.45,315.00,329.04,336.80,
		203.20,225.00,239.04,258.69,281.31,300.96,315.00,324.60,
		225.00,234.60,246.80,261.87,278.13,293.20,305.40,315.00
};

uint8_t ComputeSinCosTables8x8() {
	static bool computed_sin_cos_tables_8x8 = false;
	//This function will save the math processing time of the code.  If the user wishes to not
	//perform this function, these tables can be generated and saved as a constant.
	if (!computed_sin_cos_tables_8x8) {
		uint8_t ZoneNum;
		for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
		{
			SinOfPitch[ZoneNum] = sin((VL53L5_Zone_Pitch8x8[ZoneNum])*M_PI/180);
			CosOfPitch[ZoneNum] = cos((VL53L5_Zone_Pitch8x8[ZoneNum])*M_PI/180);
			SinOfYaw[ZoneNum] = sin(VL53L5_Zone_Yaw8x8[ZoneNum]*M_PI/180);
			CosOfYaw[ZoneNum] = cos(VL53L5_Zone_Yaw8x8[ZoneNum]*M_PI/180);
		}
	
		computed_sin_cos_tables_8x8 = true;
	}
	return 0;
}

uint8_t PrintXYZCoords(XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates) {
	uint8_t i, j, idx;
	printf("XYZ Coordinates for the target in each zone\n");
	for (i = 0; i < 8; i++) \
	{
		for (j = 0; j < 8; j++)
		{
			idx = (i * 8 + j);
			printf("%5.0f, %5.0f, %5.0f", XYZ_ZoneCoordinates->Xpos[idx],
			       XYZ_ZoneCoordinates->Ypos[idx],
				   XYZ_ZoneCoordinates->Zpos[idx]);
			printf("\n");
		}
		
	}
	printf("\n");
 
	return 0;
}
 
uint8_t ConvertDist2XYZCoords8x8(VL53L5CX_ResultsData *ResultsData,
                                 XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates,
                                 uint8_t transform_type) {
	uint8_t ZoneNum;
	float Hyp;
	double transform[4][4];

	switch(transform_type) {
		case XYZ_COORDINATES_TRANSFORM_FORWARD:
			memcpy(&transform[0][0], &transform_forward[0][0],
			       4*4*sizeof(transform_forward[0][0]));
			break;
		case XYZ_COORDINATES_TRANSFORM_LEFT:
		 	memcpy(&transform[0][0], &transform_left[0][0],
			       4*4*sizeof(transform_left[0][0]));
			break;
		case XYZ_COORDINATES_TRANSFORM_RIGHT:
			memcpy(&transform[0][0], &transform_right[0][0],
			       4*4*sizeof(transform_right[0][0]));
			break;
	}
	printf("transform %0.5f %0.5f %0.5f %0.5f\n", transform[0][0],
	       transform[0][1],transform[0][2],transform[0][3]);

	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++) {
		if ((ResultsData->nb_target_detected[ZoneNum] > 0) &&
		    (ResultsData->distance_mm[ZoneNum] > 0) &&
			((ResultsData->target_status[ZoneNum] == 5) ||
			 (ResultsData->target_status[ZoneNum] == 6) ||
			 (ResultsData->target_status[ZoneNum] == 9))) {
			Hyp = ResultsData->distance_mm[ZoneNum]/SinOfPitch[ZoneNum];
			double x = CosOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp;
			double y = SinOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp;
			double z = ResultsData->distance_mm[ZoneNum];
			XYZ_ZoneCoordinates->Xpos[ZoneNum] = (transform[0][0]*x +
			                                      transform[0][1]*y +
												  transform[0][2]*z +
												  transform[0][3]);
			XYZ_ZoneCoordinates->Ypos[ZoneNum] = (transform[1][0]*x +
											      transform[1][1]*y +
												  transform[1][2]*z +
												  transform[1][3]);
			XYZ_ZoneCoordinates->Zpos[ZoneNum] = (transform[2][0]*x +
												  transform[2][1]*y +
												  transform[2][2]*z +
												  transform[2][3]);
		} else {
			XYZ_ZoneCoordinates->Xpos[ZoneNum] = 0;
			XYZ_ZoneCoordinates->Ypos[ZoneNum] = 0;
			XYZ_ZoneCoordinates->Zpos[ZoneNum] = 0;
		}
	}
	return 0;
}

int32_t vl53l5cx_py_comms_init(VL53L5CX_Platform* p_platform,
							   const char *dev_path) {
	p_platform->fd = open(dev_path, O_RDONLY);
	if (p_platform->fd == -1) {
		printf("Failed to open %s\n", dev_path);
		return VL53L5CX_PI_COMMS_ERROR;
	}

	if (ioctl(p_platform->fd, I2C_SLAVE, 0x29) <0) {
		printf("Could not speak to the device on the i2c bus\n");
		return VL53L5CX_PI_COMMS_ERROR;
	}

	printf("Opened ST TOF Dev = %d\n", p_platform->fd);

	return 0;
}

int32_t vl53l5cx_py_comms_close(VL53L5CX_Platform* p_platform) {
	close(p_platform->fd);
	return 0;
}

int32_t vl53l5cx_py_init(VL53L5CX_Configuration* dev_conf, const char* dev_path,
                         uint16_t target_addr, uint8_t freq) {
	if (freq > 15) {
		printf("VL53L5CX Ranging Frequency too high for 8x8 (<=15hz)\n");
		return -1;
	}

	int32_t status = 0;
	(&(dev_conf->platform))->address = VL53L5CX_DEFAULT_I2C_ADDRESS;
	status |= vl53l5cx_py_comms_init(&(dev_conf)->platform, dev_path);
    if (status) {
        printf("VL53L5CX comms init failed\n");
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
		printf("VL53L5CX Set i2c Failed \n");
		vl53l5cx_py_comms_close(&(dev_conf)->platform);
		return status;
	}
	printf("set addresses success\n");

	status = vl53l5cx_init(dev_conf);
	if (status) {
		printf("VL53L5CX ULD Loading failed\n");
		vl53l5cx_py_comms_close(&(dev_conf)->platform);
		return status;
	}
	printf("Initialized\n");	
	
	vl53l5cx_set_resolution(dev_conf, VL53L5CX_RESOLUTION_8X8);

	vl53l5cx_set_ranging_frequency_hz(dev_conf, freq);


	printf("VL53L5CX ULD ready ! (Version : %s)\n",
			VL53L5CX_API_REVISION);

	ComputeSinCosTables8x8(); //initialize sin + cos tables (Can be hardcoded)
}

int32_t vl53l5cx_py_close(VL53L5CX_Configuration* dev_conf) {
	int32_t status = 0;
	status = vl53l5cx_py_comms_close(&(dev_conf)->platform);

	return status;
}

int32_t vl53l5cx_py_start_ranging(VL53L5CX_Configuration* dev_conf) {
	int32_t status = 0;
	status = vl53l5cx_start_ranging(dev_conf);

	return status;
}

int32_t vl53l5cx_py_stop_ranging(VL53L5CX_Configuration* dev_conf) {
	int32_t status = 0;
	status = vl53l5cx_stop_ranging(dev_conf);
	status |= vl53l5cx_py_comms_close(&(dev_conf)->platform);
	return status;
}

int32_t vl53l5cx_py_get_range(VL53L5CX_Configuration* dev_conf,
                              uint8_t transform,
							  XYZ_ZoneCoordinates_t XYZ_Coordinates) {	
	int32_t status = 0;
	VL53L5CX_ResultsData Results;
	uint8_t isReady = 0;

	isReady = 0;
	printf("Waiting\n");
	while (!isReady) {	
		status = vl53l5cx_check_data_ready(dev_conf, &isReady); //Wait for device to be ready
	}
	vl53l5cx_get_ranging_data(dev_conf, &Results); //Get ranging data and store in results

	memset(&XYZ_Coordinates, 0, sizeof(XYZ_ZoneCoordinates_t));
	ConvertDist2XYZCoords8x8(&Results, &XYZ_Coordinates, transform); //Get XYZ Coordinates in XYZ_Coordinates variable
	// printf("DEVICE: %d\n", j);
	// PrintXYZCoords(&XYZ_Coordinates);	//Print coordinates
	return status;
}
