"""
vl53l5cx.py: Definition for VL53L5CX sensors.

Created by Perry Naseck on 1/21/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import time
from json import dumps, loads
from typing import Sequence

import numpy as np
from periphery import GPIO

from arenarobot.service.sensor import ArenaRobotServiceSensor
from vl53l5cx_py.helpers import VL53L5CXJSONEncoder

# Optional imports allow for constants from this module to be referenced
# and for the service to be recognized without all of the required
# dependencies installed need for starting this service.
imports_missing = []
try:
    from vl53l5cx_py.driver import VL53L5CX
except ImportError as err:
    VL53L5CX = err
    imports_missing.append(err)


class ArenaRobotServiceSensorVL53L5CX(ArenaRobotServiceSensor):
    """VL53L5CX sensor class for the ARENA."""

    DEVICE_INSTANCE_SENSOR_TYPE = "vl53l5cx"

    def __init__(self, dev_path: str, gpio_path: str, rst_pin: int,
                 lpn_pins: Sequence[int], **kwargs):
        """Initialize the VL53L5CX sensor class."""
        # Check for missing imports
        if len(imports_missing) > 0:
            raise Exception(imports_missing)

        self.dev_path = dev_path
        self.gpio_path = gpio_path
        self.rst_pin = rst_pin
        self.lpn_pins = lpn_pins
        self.sensors_setup = [None] * len(self.lpn_pins)
        self.sensors = [None] * len(self.lpn_pins)

        sensor_type = (ArenaRobotServiceSensorVL53L5CX
                       .DEVICE_INSTANCE_SENSOR_TYPE)
        super().__init__(
            device_instance_sensor_type=sensor_type,
            device_instance_prefix="vl53l5cx_",
            **kwargs
        )

    def setup(self):
        """Set up VL53L5CX sensor."""
        rst_pin = GPIO(self.gpio_path, self.rst_pin, "out")
        rst_pin.write(False)

        for i, pin_num in enumerate(self.lpn_pins):
            pin = GPIO(self.gpio_path, pin_num, "out")
            pin.write(False)
            addr = VL53L5CX.DEFAULT_ADDR + np.uint16(2 * i)
            self.sensors_setup[i] = ((i, pin, addr,))
            print(f'Will setup sensor {i}, addr {addr:02x} on pin {pin_num}')

        rst_pin.write(True)
        time.sleep(1)
        rst_pin.write(False)
        time.sleep(1)

        for sensor_setup in self.sensors_setup:
            i, pin, addr = sensor_setup
            pin.write(True)
            time.sleep(1)
            self.sensors[i] = VL53L5CX(self.dev_path, target_addr=addr)
            pin.write(False)
            time.sleep(1)

        for sensor_setup in self.sensors_setup:
            i, pin, addr = sensor_setup
            pin.write(True)
            time.sleep(1)
            self.sensors[i].start_ranging()

        super().setup()

    def fetch(self):
        """Fetch VL53L5CX data."""
        data = []
        for sensor in self.sensors:
            ranges = sensor.get_range()
            data.append(ranges)
        data = loads(dumps(data, cls=VL53L5CXJSONEncoder))
        self.publish({"data": data})
