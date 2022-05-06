"""
__init__.py: Definitions for VL53L5CX helpers.

Created by Perry Naseck on 2/24/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from dataclasses import dataclass
from json import JSONEncoder
from typing import List

import numpy as np

@dataclass
class VL53L5CXSensorData():
    """VL53L5CX Lidar sensor data."""
    distance_mm: List[np.int16]
    target_status: List[np.uint8]

# VL53L5_Zone_Pitch8x8 = [
# 		59.00,64.00,67.50,70.00,70.00,67.50,64.00,59.00,
# 		64.00,70.00,72.90,74.90,74.90,72.90,70.00,64.00,
# 		67.50,72.90,77.40,80.50,80.50,77.40,72.90,67.50,
# 		70.00,74.90,80.50,85.75,85.75,80.50,74.90,70.00,
# 		70.00,74.90,80.50,85.75,85.75,80.50,74.90,70.00,
# 		67.50,72.90,77.40,80.50,80.50,77.40,72.90,67.50,
# 		64.00,70.00,72.90,74.90,74.90,72.90,70.00,64.00,
# 		59.00,64.00,67.50,70.00,70.00,67.50,64.00,59.00
# ]

# VL53L5_Zone_Yaw8x8 = [
# 		135.00,125.40,113.20, 98.13, 81.87, 66.80, 54.60, 45.00,
# 		144.60,135.00,120.96,101.31, 78.69, 59.04, 45.00, 35.40,
# 		156.80,149.04,135.00,108.45, 71.55, 45.00, 30.96, 23.20,
# 		171.87,168.69,161.55,135.00, 45.00, 18.45, 11.31,  8.13,
# 		188.13,191.31,198.45,225.00,315.00,341.55,348.69,351.87,
# 		203.20,210.96,225.00,251.55,288.45,315.00,329.04,336.80,
# 		215.40,225.00,239.04,258.69,281.31,300.96,315.00,324.60,
# 		225.00,234.60,246.80,261.87,278.13,293.20,305.40,315.00]

class VL53L5CXJSONEncoder(JSONEncoder):
    """JSON Encoder helper for VL53L5CX packets."""

    def default(self, obj):
        """JSON Encoder helper function for VL53L5CX packets."""
        if isinstance(obj, np.ndarray):
            return obj.tolist()

        elif isinstance(obj, bytes):
            return int.from_bytes(obj, "little")

        elif isinstance(obj, VL53L5CXSensorData):
            return vars(obj)

        return JSONEncoder.default(self, obj)
