#!/usr/bin/env python
"""
vl53l5cx_py_basic.py: Get data from 1 VL53L5CX sensor.

Created by Perry Naseck on 11/10/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import argparse

from vl53l5cx_py import VL53L5CX

DEV_PATH = "/dev/i2c-1"

def main():
    parser = argparse.ArgumentParser(
        description='Get data from 1 VL53L5CX sensor.')
    parser.add_argument('dev_path', metavar='/dev/i2c-XX', type=str,
                        help='i2c device path')
    args = parser.parse_args()

    sensor = VL53L5CX(args.dev_path)
    sensor.start_ranging()

    while True:
        results = sensor.get_range()
        dist = results["distance_mm"][0]
        print(f"py  results distance_mm[0]: {dist}")

if __name__=="__main__":
   res = main()
   exit(res)
