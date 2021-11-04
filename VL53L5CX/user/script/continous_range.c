

/***********************************/
/*   VL53L5CX ULD basic example    */
/***********************************/
/*
* This example is the most basic. It initializes the VL53L5CX ULD, and starts
* a ranging to capture 10 frames.
*
* By default, ULD is configured to have the following settings :
* - Resolution 4x4
* - Ranging period 1Hz
*
* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l5cx_api.h"
#include <wiringPi.h>
#include "math.h"

#include "range.h"
#define NUM_DEVICES 3
typedef struct
{
	double Xpos[64];
	double Ypos[64];
	double Zpos[64];
}XYZ_ZoneCoordinates_t;


XYZ_ZoneCoordinates_t XYZ_Coordinates;
 
//sin(pi/2) = 1
//sin(-pi/2) = -1
//cos(pi/2) = 0
//cos(-pi/2) = 0 
double transform_forward[4][4] =
{
	{1, 0, 0, 0},
	{0, 1, 0, 0},
	{0, 0, 1, 0},
	{0, 0, 0, 1}

};
//-pi/2
double transform_left[4][4] =
{
	{0, 0, -1, 0},
	{0, 1, 0, 0},
	{1, 0, 0, 0},
	{0, 0, 0, 1}

};
//pi/2
double transform_right[4][4] =
{
	{0, 0, 1, 0},
	{0, 1, 0, 0},
	{-1, 0, 0, 0},
	{0, 0, 0, 1}

};

double SinOfPitch[64], CosOfPitch[64], SinOfYaw[64], CosOfYaw[64];
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
uint8_t ComputeSinCosTables(void)
{
	//This function will save the math processing time of the code.  If the user wishes to not
	//perform this function, these tables can be generated and saved as a constant.
	uint8_t ZoneNum;
	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
	{
		SinOfPitch[ZoneNum] = sin((VL53L5_Zone_Pitch8x8[ZoneNum])*M_PI/180);
		CosOfPitch[ZoneNum] = cos((VL53L5_Zone_Pitch8x8[ZoneNum])*M_PI/180);
		SinOfYaw[ZoneNum] = sin(VL53L5_Zone_Yaw8x8[ZoneNum]*M_PI/180);
		CosOfYaw[ZoneNum] = cos(VL53L5_Zone_Yaw8x8[ZoneNum]*M_PI/180);
	}
 
	return 0;
}

uint8_t PrintXYZCoords(XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates)
{
	uint8_t i, j, idx;
	printf("XYZ Coordinates for the target in each zone\n");
	for (i = 0; i < 8; i++) \
	{
		for (j = 0; j < 8; j++)
		{
			idx = (i * 8 + j);
			printf("%5.0f, %5.0f, %5.0f",XYZ_ZoneCoordinates->Xpos[idx],XYZ_ZoneCoordinates->Ypos[idx],XYZ_ZoneCoordinates->Zpos[idx]);
			printf("\n");
		}
		
	}
	printf("\n");
 
	return 0;
}
 
uint8_t ConvertDist2XYZCoords8x8(VL53L5CX_ResultsData *ResultsData, XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates, uint8_t transform_num)
{
	uint8_t ZoneNum;
	float Hyp;
	double transform[4][4];

	switch(transform_num)
	{
		case 0:
			memcpy(&transform[0][0], &transform_forward[0][0], 4*4*sizeof(transform_forward[0][0]));
			break;
		case 1:
		 	memcpy(&transform[0][0], &transform_left[0][0], 4*4*sizeof(transform_left[0][0]));
			break;
		case 2:
			memcpy(&transform[0][0], &transform_right[0][0], 4*4*sizeof(transform_right[0][0]));
			break;
	}
	printf("transform %0.5f %0.5f %0.5f %0.5f\n", transform[0][0], transform[0][1],transform[0][2],transform[0][3]);

	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
	{
		if ((ResultsData->nb_target_detected[ZoneNum] > 0) && (ResultsData->distance_mm[ZoneNum] > 0) && ((ResultsData->target_status[ZoneNum] == 5) || (ResultsData->target_status[ZoneNum] == 6) || (ResultsData->target_status[ZoneNum] == 9)) )
		{
			Hyp = ResultsData->distance_mm[ZoneNum]/SinOfPitch[ZoneNum];
			double x = CosOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp;
			double y = SinOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp;
			double z = ResultsData->distance_mm[ZoneNum];
			XYZ_ZoneCoordinates->Xpos[ZoneNum] = transform[0][0]*x + transform[0][1]*y + transform[0][2]*z + transform[0][3];
			XYZ_ZoneCoordinates->Ypos[ZoneNum] = transform[1][0]*x + transform[1][1]*y + transform[1][2]*z + transform[1][3];
			XYZ_ZoneCoordinates->Zpos[ZoneNum] = transform[2][0]*x + transform[2][1]*y + transform[2][2]*z + transform[2][3];

		}
		else
		{
			XYZ_ZoneCoordinates->Xpos[ZoneNum] = 0;
			XYZ_ZoneCoordinates->Ypos[ZoneNum] = 0;
			XYZ_ZoneCoordinates->Zpos[ZoneNum] = 0;
		}
	}
	return 0;
}

VL53L5CX_Configuration *p_devs[NUM_DEVICES];
int initialize_sensors(VL53L5CX_Configuration Dev_list[], uint8_t use_8x8, uint8_t freq)
{
	
	int status = 0;
    for (int i = 0; i < NUM_DEVICES; i++)
    {
        (&(Dev_list[i].platform))->address = 0x52;
        status |= vl53l5cx_comms_init(&(Dev_list[i]).platform);
    }
    if(status)
    {
        printf("VL53L5CX comms init failed\n");
        return -1;
    }


	wiringPiSetup();
	pinMode(7, OUTPUT); //0x62 LPn
	pinMode(0, OUTPUT); //0x52 LPn
	pinMode(2, OUTPUT); //0x72 LPn 
	pullUpDnControl(2, PUD_UP);
	pullUpDnControl(0, PUD_UP);
	pullUpDnControl(7 ,PUD_UP);
	digitalWrite(2,1);
	digitalWrite(0,1);
	digitalWrite(7,1); 

	pinMode(3, OUTPUT);
	digitalWrite(3, LOW);
	pinMode(12, OUTPUT);
	digitalWrite(12, LOW);
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/
	status = 0;
	/* (Optional) Check if there is a VL53L5CX sensor connected */
	/*
	status = vl53l5cx_is_alive(p_dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L5CX not detected at requested address\n");
		return status;
	}
	*/
	
	/* (Mandatory) Init VL53L5CX sensor */
	for(int i = 0; i < NUM_DEVICES; i++)
	{
		p_devs[i] = &(Dev_list[i]); //Order - 0x62, 0x72, 0x52
	}

	digitalWrite(0,0); //Sleep 0x52	
	digitalWrite(2,0); //Sleep 0x72
	
	status = vl53l5cx_set_i2c_address(p_devs[0], 0x62); //set 0x62
	digitalWrite(7,0); //Sleep 0x62
	digitalWrite(2,1); //Wake 0x72
	
	status |= vl53l5cx_set_i2c_address(p_devs[1], 0x72); //set 0x72
	digitalWrite(7, 1); //Wake 0x62
	digitalWrite(0, 1); //Wake 0x52
	
	if (status)
	{
		printf("VL53L5CX Set i2c Failed \n");
		return status;
	}
	printf("set addresses success\n");
	status = 0;
	for (int i = 0; i < NUM_DEVICES; i++) status |= vl53l5cx_init(p_devs[i]);
	if(status)
	{
		printf("VL53L5CX ULD Loading failed\n");
		return status;
	}
	printf("Initialized\n");	
	
	if(use_8x8)
	{
		for(int i = 0; i < NUM_DEVICES; i++) vl53l5cx_set_resolution(p_devs[i], VL53L5CX_RESOLUTION_8X8);
	}
	else
	{
		for(int i = 0; i < NUM_DEVICES; i++) vl53l5cx_set_resolution(p_devs[i], VL53L5CX_RESOLUTION_4X4);
	}

	if ((freq > 15) && use_8x8)
	{
		printf("VL53L5CX Ranging Frequency too high for 8x8 (<=15hz)\n");
		return -1;
	}
	else if ( (freq > 60) && !use_8x8)
	{
		printf("VL53L5CX Ranging Frequency too high for 4x4 (<=60hz)\n");
		return -1;
	}
	for(int i = 0; i < NUM_DEVICES; i++) vl53l5cx_set_ranging_frequency_hz(p_devs[i], freq);


	printf("VL53L5CX ULD ready ! (Version : %s)\n",
			VL53L5CX_API_REVISION);

	/*********************************/
	/*         Ranging loop          */
	/*********************************/
	ComputeSinCosTables(); //initialize sin + cos tables (Can be hardcoded)

	status = 0;
	//Start ranging on all devices
	for(int i = 0; i < NUM_DEVICES; i++) 
	{
		status |= vl53l5cx_start_ranging(p_devs[i]);
	}
	return status;
}

int stop_ranging(VL53L5CX_Configuration Dev_list[])
{
	int status = 0;
	for(int i = 0; i < NUM_DEVICES; i++) status |= vl53l5cx_stop_ranging(p_devs[i]);

	for (int i = 0; i < NUM_DEVICES; i++)
    {
        status |= vl53l5cx_comms_close(&(Dev_list[i]).platform);
    }

	return status;
}

int get_range()
{	
	int status = 0;
	VL53L5CX_ResultsData 	Results;
	uint8_t isReady = 0;
	for (int j = 0; j < NUM_DEVICES; j++)
	{

		VL53L5CX_Configuration *p_dev = p_devs[j];
		isReady = 0;
		printf("Waiting\n");
		while (!isReady)
		{	
			status = vl53l5cx_check_data_ready(p_dev, &isReady); //Wait for device to be ready
		}

			

			vl53l5cx_get_ranging_data(p_dev, &Results); //Get ranging data and store in results

			memset(&XYZ_Coordinates, 0, sizeof(XYZ_ZoneCoordinates_t));
			ConvertDist2XYZCoords8x8(&Results, &XYZ_Coordinates, j); //Get XYZ Coordinates in XYZ_Coordinates variable
			printf("DEVICE: %d\n", j);
			PrintXYZCoords(&XYZ_Coordinates);	//Print coordinates
	}
	return status;
}


