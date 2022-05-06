"""
vl53l5cx_py_wrapper.pyx
Created by Perry Naseck on 11/4/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""


import atexit
from libc.stdint cimport int32_t, uint8_t, uint16_t
import numpy as np
cimport numpy as np

from vl53l5cx_py.helpers import VL53L5CXSensorData

cdef extern from "../VL53L5CX_Linux_driver_1.1.2/user/uld-driver/inc/vl53l5cx_api.h":
    enum: VL53L5CX_DEFAULT_I2C_ADDRESS
    enum: VL53L5CX_RESOLUTION_8X8
    enum: VL53L5CX_NB_TARGET_PER_ZONE
    ctypedef struct VL53L5CX_Configuration:
        pass
    # ctypedef struct motion_indicator_t:
    #     uint32_t global_indicator_1
    #     uint32_t global_indicator_2
    #     uint8_t	 status
    #     uint8_t	 nb_of_detected_aggregates
    #     uint8_t	 nb_of_aggregates
    #     uint8_t	 spare
    #     uint32_t motion[32]
    ctypedef struct VL53L5CX_ResultsData:
        #ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
        np.uint32_t ambient_per_spad[VL53L5CX_RESOLUTION_8X8]
        #endif

        #ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
        np.uint8_t nb_target_detected[VL53L5CX_RESOLUTION_8X8]
        #endif

        #ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
        np.uint32_t nb_spads_enabled[VL53L5CX_RESOLUTION_8X8]
        #endif

        #ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
        np.uint32_t signal_per_spad[(VL53L5CX_RESOLUTION_8X8
                        *VL53L5CX_NB_TARGET_PER_ZONE)]
        #endif

        #ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
        np.uint16_t range_sigma_mm[(VL53L5CX_RESOLUTION_8X8
                        *VL53L5CX_NB_TARGET_PER_ZONE)]
        #endif

        #ifndef VL53L5CX_DISABLE_DISTANCE_MM
        np.int16_t distance_mm[(VL53L5CX_RESOLUTION_8X8
                        *VL53L5CX_NB_TARGET_PER_ZONE)]
        #endif

        #ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
        np.uint8_t reflectance[(VL53L5CX_RESOLUTION_8X8
                        *VL53L5CX_NB_TARGET_PER_ZONE)]
        #endif

        #ifndef VL53L5CX_DISABLE_TARGET_STATUS
        np.uint8_t target_status[(VL53L5CX_RESOLUTION_8X8
                        *VL53L5CX_NB_TARGET_PER_ZONE)]
        #endif

        #ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
        # motion_indicator_t motion_indicator
        #endif

cdef extern from "../include/vl53l5cx_py.h":
    int32_t vl53l5cx_py_init(VL53L5CX_Configuration* dev_conf,
                             const char *dev_path, uint16_t target_addr,
                             uint8_t freq)
    int32_t vl53l5cx_py_close(VL53L5CX_Configuration* dev_conf)
    int32_t vl53l5cx_py_start_ranging(VL53L5CX_Configuration* dev_conf)
    int32_t vl53l5cx_py_stop_ranging(VL53L5CX_Configuration* dev_conf)
    int32_t vl53l5cx_py_get_range(VL53L5CX_Configuration* dev_conf,
                                  VL53L5CX_ResultsData* results)

cdef VL53L5CX_PY_DEFAULT_ADDR = VL53L5CX_DEFAULT_I2C_ADDRESS
cdef VL53L5CX_PY_DEFAULT_FREQ = 15


cdef class VL53L5CX:
    cdef VL53L5CX_Configuration dev_conf
    cdef bint ranging

    DEFAULT_ADDR = np.uint16(VL53L5CX_PY_DEFAULT_ADDR)
    DEFAULT_FREQ = np.uint8(VL53L5CX_PY_DEFAULT_FREQ)

    def __cinit__(self, dev_path: str,
                  target_addr: np.uint16_t = VL53L5CX_PY_DEFAULT_ADDR,
                  freq: np.uint8_t = VL53L5CX_PY_DEFAULT_FREQ) -> None:
        cdef int32_t status = vl53l5cx_py_init(&self.dev_conf, dev_path.encode(), target_addr, freq)
        if status != 0:
            raise RuntimeError
        self.ranging = False
        atexit.register(self.__del__)

    def __del__(self) -> None:
        if self.ranging:
            self.stop_ranging()
        cdef int32_t status = vl53l5cx_py_close(&self.dev_conf)
        if status != 0:
            raise RuntimeError
        atexit.unregister(self.__del__)

    cpdef void start_ranging(self):
        cdef int32_t status = vl53l5cx_py_start_ranging(&self.dev_conf)
        if status != 0:
            raise RuntimeError
        self.ranging = True

    cpdef void stop_ranging(self):
        cdef int32_t status = vl53l5cx_py_stop_ranging(&self.dev_conf)
        if status != 0:
            raise RuntimeError
        self.ranging = False

    cpdef get_range(self):
        cdef VL53L5CX_ResultsData results
        cdef int32_t status = vl53l5cx_py_get_range(&self.dev_conf, &results)
        if status != 0:
            raise RuntimeError

        return VL53L5CXSensorData(
            distance_mm=results.distance_mm,
            target_status=np.frombuffer(results.target_status, dtype='S1')
        )
