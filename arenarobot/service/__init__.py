"""
__init__.py: Definitions for services.

Created by Perry Naseck on 1/25/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from .sensor import ArenaRobotServiceSensor
from .sensor.beluga_serial import ArenaRobotServiceSensorBelugaSerial
from .sensor.licosa_serial import ArenaRobotServiceSensorLiCosaSerial
from .sensor.vl53l5cx import ArenaRobotServiceSensorVL53L5CX
from .service import ArenaRobotService

SERVICE_TYPE_MAP = {
    "unknown": ArenaRobotService,
    "sensor": ArenaRobotServiceSensor,
    "sensor_beluga_serial": ArenaRobotServiceSensorBelugaSerial,
    "sensor_licosa_serial": ArenaRobotServiceSensorLiCosaSerial,
    "sensor_vl53l5cx": ArenaRobotServiceSensorVL53L5CX
}
