"""
vl53l5cx_py_wrapper_disable.pyx
Created by Perry Naseck on 2/1/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""


cdef class VL53L5CX:
    DEFAULT_ADDR = None
    DEFAULT_FREQ = None

    def __cinit__(self, dev_path: str,
                  target_addr = None,
                  freq = None) -> None:
        raise RuntimeError("vl53l5cx_py.driver not available on this platform!")
