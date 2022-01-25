"""
__init__.py: Definitions for sensors.

Created by Perry Naseck on 1/21/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from arenarobot.service.service import ArenaRobotService


class ArenaRobotServiceSensor(ArenaRobotService):
    """Sensor class for the ARENA."""

    DEVICE_INSTANCE_TYPE = "sensor"
    DEVICE_INSTANCE_SENSOR_TYPE = "unknown"

    def __init__(self, device_instance_prefix="",
                 device_instance_sensor_type="unknown",
                 subtopic="",
                 **kwargs):
        """Initialize the sensor class."""
        self.device_instance_sensor_type = device_instance_sensor_type
        super().__init__(
            device_instance_type=ArenaRobotServiceSensor.DEVICE_INSTANCE_TYPE,
            device_instance_prefix=f"sensor_{device_instance_prefix}",
            subtopic=f"sensors/{subtopic}",
            **kwargs
        )

    def publish_msg_base(self):
        """Publish message base."""
        payload = super().publish_msg_base()
        payload.update({
            "device_instance_sensor_type": self.device_instance_sensor_type
        })
        return payload
