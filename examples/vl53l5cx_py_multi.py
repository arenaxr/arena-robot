#!/usr/bin/env python
"""
vl53l5cx_py_basic.py: Get data from multiple VL53L5CX sensors.

Created by Perry Naseck on 12/7/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import argparse
import numpy as np
import time

from periphery import GPIO
from vl53l5cx_py.driver import VL53L5CX

def main():
    parser = argparse.ArgumentParser(
        description='Get data from 1 VL53L5CX sensor.')
    parser.add_argument('dev_path', metavar='/dev/i2c-XX', type=str,
                        help='i2c device path')
    parser.add_argument('gpio_path', metavar='/dev/gpiochipXX', type=str,
                        help='gpio device path')
    parser.add_argument('rst_pin', type=int, help='i2c rst pin')
    parser.add_argument('sensor_pins', type=int, nargs='+',
                        help='LPn GPIO pins')
    args = parser.parse_args()

    sensors_data = []
    sensors = [None] * len(args.sensor_pins)

    rst_pin = GPIO(args.gpio_path, args.rst_pin, "out")
    rst_pin.write(False)

    for i in range(len(args.sensor_pins)):
        pin_num = args.sensor_pins[i]
        pin = GPIO(args.gpio_path, pin_num, "out")
        pin.write(False)
        addr = VL53L5CX.DEFAULT_ADDR + np.uint16(2 * i)
        sensors_data.append((i, pin, addr,))
        print(f'Will setup sensor {i}, addr {addr:02x} on pin {pin_num}')

    rst_pin.write(True)
    time.sleep(1)
    rst_pin.write(False)
    time.sleep(1)

    for data in sensors_data:
        i, pin, addr = data
        pin.write(True)
        time.sleep(1)
        sensors[i] = VL53L5CX(args.dev_path, target_addr=addr)
        pin.write(False)
        time.sleep(1)

    for data in sensors_data:
        i, pin, addr = data
        pin.write(True)
        time.sleep(1)
        sensors[i].start_ranging()

    while True:
        out = ""
        for i in range(len(sensors)):
            results = sensors[i].get_range()
            out += f"{i}:\n"
            for i in range(8):
                for j in range(8):
                    out += '{:4d}'.format(results["distance_mm"][(i*8) + j])
                    out += " "
                out += "\n"
        print(out)

if __name__=="__main__":
   res = main()
   exit(res)
