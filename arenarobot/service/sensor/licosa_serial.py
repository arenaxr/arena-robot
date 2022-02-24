"""
licosa_serial.py: Definition for LiCosa IMU and lidar modules over serial.

Created by Perry Naseck on 2/22/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import time
from dataclasses import asdict
from json import dumps, loads

# from aioserial import AioSerial
from serial import Serial

from arenarobot.service.sensor import ArenaRobotServiceSensor
from licosa_py import LiCosaJSONEncoder, LiCosaPacketParser


# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceSensorLiCosaSerial(ArenaRobotServiceSensor):
    """LiCosa serial sensor class for the ARENA."""

    DEVICE_INSTANCE_SENSOR_TYPE = "licosa_serial"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self, dev_path: str, baudrate: int = 115200, **kwargs):
        """Initialize the LiCosa serial sensor class."""
        self.dev_path = dev_path
        self.baudrate = baudrate

        # self.aioserial_instance = None
        self.serial_instance = None

        sensor_type = (ArenaRobotServiceSensorLiCosaSerial
                       .DEVICE_INSTANCE_SENSOR_TYPE)
        super().__init__(
            device_instance_sensor_type=sensor_type,
            device_instance_prefix="licosa_serial_",
            async_service=True, **kwargs
        )

    async def async_setup(self):
        """Set up LiCosa serial sensor."""
        # self.aioserial_instance = AioSerial(port=self.dev_path,
        #                                     baudrate=self.baudrate)
        self.serial_instance = Serial(port=self.dev_path,
                                      baudrate=self.baudrate)
        time.sleep(1)
        await super().async_setup()

    async def async_fetch(self):
        """Fetch LiCosa serial data."""
        pkt = LiCosaPacketParser()
        while True:
            # msg_byte = await self.aioserial_instance.read_async(size=1)
            msg_byte = self.serial_instance.read(size=1)

            pkt_out = pkt.next_byte(msg_byte)
            if pkt_out is not False:
                data = asdict(pkt_out)
                del data['data_buf']
                data = loads(dumps(data, cls=LiCosaJSONEncoder))
                print(data)
                self.publish({"data": data})
