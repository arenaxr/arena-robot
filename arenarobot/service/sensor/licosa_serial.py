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
from struct import unpack

from serial import Serial

from arenarobot.service.sensor import ArenaRobotServiceSensor
from licosa_py.packets import LiCosaJSONEncoder
from licosa_py.parser import LiCosaPacketParser


# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceSensorLiCosaSerial(ArenaRobotServiceSensor):
    """LiCosa serial sensor class for the ARENA."""

    DEVICE_INSTANCE_SENSOR_TYPE = "licosa_serial"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self, dev_path: str, baudrate: int = 230400, **kwargs):
        """Initialize the LiCosa serial sensor class."""
        self.dev_path = dev_path
        self.baudrate = baudrate

        self.serial_instance = None
        self.pkt = None

        sensor_type = (ArenaRobotServiceSensorLiCosaSerial
                       .DEVICE_INSTANCE_SENSOR_TYPE)
        super().__init__(
            device_instance_sensor_type=sensor_type,
            device_instance_prefix="licosa_serial_",
            **kwargs
        )

    def setup(self):
        """Set up LiCosa serial sensor."""
        self.serial_instance = Serial(port=self.dev_path,
                                      baudrate=self.baudrate,
                                      timeout=0)
        self.pkt = LiCosaPacketParser()
        time.sleep(1)
        super().setup()

    def fetch(self):
        """Fetch LiCosa serial data."""
        msg_bytes = self.serial_instance.read(size=32)

        # See thread here, this is the fastest way to iterate over bytes
        # https://stackoverflow.com/questions/14267452/iterate-over-individual-bytes-in-python-3
        for msg_byte in list(unpack(str(len(msg_bytes)) + 'c', msg_bytes)):
            pkt_out = self.pkt.next_byte(msg_byte)
            if pkt_out is not False:
                data = asdict(pkt_out)
                del data['data_buf']
                data = loads(dumps(data, cls=LiCosaJSONEncoder))
                self.publish({"data": data})
