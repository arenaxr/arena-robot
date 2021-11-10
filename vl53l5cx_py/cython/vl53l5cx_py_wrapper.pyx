"""
vl53l5cx_py_wrapper.pyx
Created by Perry Naseck on 11/4/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from libc.stdint cimport uint8_t, uint16_t, int32_t
import numpy as np
cimport numpy as np

cdef extern from "../VL53L5CX_Linux_driver_1.1.2/user/uld-driver/inc/vl53l5cx_api.h":
    ctypedef struct VL53L5CX_Configuration:
        pass
    cdef enum:
        VL53L5CX_DEFAULT_I2C_ADDRESS

cdef extern from "../include/vl53l5cx_py.h":
    ctypedef struct XYZ_ZoneCoordinates_t:
        double Xpos[64]
        double Ypos[64]
        double Zpos[64]

    int32_t vl53l5cx_py_init(VL53L5CX_Configuration* dev_conf,
                             const char *dev_path, uint16_t target_addr,
                             uint8_t freq)
    int32_t vl53l5cx_py_close(VL53L5CX_Configuration* dev_conf)
    int32_t vl53l5cx_py_start_ranging(VL53L5CX_Configuration* dev_conf)
    int32_t vl53l5cx_py_stop_ranging(VL53L5CX_Configuration* dev_conf)
    int32_t vl53l5cx_py_get_range(VL53L5CX_Configuration* dev_conf,
                                  uint8_t transform,
                                  XYZ_ZoneCoordinates_t XYZ_Coordinates)

cpdef VL53L5CX_PY_DEFAULT_ADDR = VL53L5CX_DEFAULT_I2C_ADDRESS
cpdef VL53L5CX_PY_DEFAULT_FREQ = 15
cpdef VL53L5CX_PY_DEFAULT_TRANSFORM = 0

cdef class VL53L5CX:
    cdef VL53L5CX_Configuration dev_conf
    cdef uint8_t transform

    def __cinit__(self, dev_path: str,
                  target_addr: np.uint16_t = VL53L5CX_PY_DEFAULT_ADDR,
                  freq: np.uint8_t = VL53L5CX_PY_DEFAULT_FREQ,
                  transform: np.uint8_t = VL53L5CX_PY_DEFAULT_TRANSFORM) -> None:
        self.transform = transform
        cdef int32_t status = vl53l5cx_py_init(&self.dev_conf, dev_path.encode(), target_addr, freq)
        if status != 0:
            raise RuntimeError

    def __dealloc__(self) -> None:
        cdef int32_t status = vl53l5cx_py_close(&self.dev_conf)
        if status != 0:
            raise RuntimeError

    cpdef void start_ranging(self):
        cdef int32_t status = vl53l5cx_py_start_ranging(&self.dev_conf)
        if status != 0:
            raise RuntimeError

    cpdef void stop_ranging(self):
        cdef int32_t status = vl53l5cx_py_stop_ranging(&self.dev_conf)
        if status != 0:
            raise RuntimeError

    cpdef tuple get_range(self):
        cdef XYZ_ZoneCoordinates_t XYZ_Coordinates
        cdef int32_t status = vl53l5cx_py_get_range(&self.dev_conf, self.transform, XYZ_Coordinates)
        if status != 0:
            raise RuntimeError
        return (XYZ_Coordinates.Xpos, XYZ_Coordinates.Ypos, XYZ_Coordinates.Zpos)
