/*
 * vl53l5cx_py.h
 * Created by Perry Naseck and Vikram Bhat on 11/4/21.
 * 
 * Copyright (c) 2021, The CONIX Research Center
 * All rights reserved.
 * 
 * This source code is licensed under the BSD-3-Clause license found in the
 * LICENSE file in the root directory of this source tree.
 */

#ifndef _VL53L5CX_PY_H
#define _VL53L5CX_PY_H

#include <stdint.h>
#include "vl53l5cx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	double Xpos[64];
	double Ypos[64];
	double Zpos[64];
} XYZ_ZoneCoordinates_t;

typedef enum XYZ_Coordinates_Transform_t {
	XYZ_COORDINATES_TRANSFORM_FORWARD = 0,
	XYZ_COORDINATES_TRANSFORM_LEFT = 1,
	XYZ_COORDINATES_TRANSFORM_RIGHT = 2,
} XYZ_Coordinates_Transform_t;

int32_t vl53l5cx_py_init(VL53L5CX_Configuration* dev_conf, const char *dev_path,
                         uint16_t target_addr, uint8_t freq);
int32_t vl53l5cx_py_close(VL53L5CX_Configuration* dev_conf);
int32_t vl53l5cx_py_start_ranging(VL53L5CX_Configuration* dev_conf);
int32_t vl53l5cx_py_stop_ranging(VL53L5CX_Configuration* dev_conf);
int32_t vl53l5cx_py_get_range(VL53L5CX_Configuration* dev_conf,
                              uint8_t transform,
							  XYZ_ZoneCoordinates_t XYZ_Coordinates);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
