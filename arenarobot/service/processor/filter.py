"""
filter.py: filters various sensor ouptut

Created by John Pedraza on July 3rd, 2022.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from json import JSONEncoder, dumps, loads
from datetime import datetime
from scipy.spatial.transform import Rotation as R
from arena import Device
from paho.mqtt.client import MQTTMessage
from arenarobot.service.processor import ArenaRobotServiceProcessor
from filterpy.kalman import predict, update, KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

class ArenaRobotServiceProcessorFilter(ArenaRobotServiceProcessor):
    DEVICE_INSTANCE_PROCESSOR_TYPE = "filter"
    def __init__(self, **kwargs):
        processor_type = (ArenaRobotServiceProcessorFilter
                          .DEVICE_INSTANCE_PROCESSOR_TYPE)
        super().__init__(
            device_instance_processor_type=processor_type,
            device_instance_prefix="filter_",
            **kwargs
            )
    def setup(self):
        super().setup()

    def fetch(self):
        pass
