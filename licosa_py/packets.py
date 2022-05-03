"""
packets.py: Definitions for LiCosa packets and helpers.

Created by Perry Naseck on 2/24/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from enum import Enum
from dataclasses import dataclass
from json import JSONEncoder
from typing import ClassVar, List

import numpy as np

from vl53l5cx_py.helpers import VL53L5CXSensorData

class LiCosaPacketType(Enum):
    """LiCosa serial sensor packet type class."""
    LICOSA_PKT_TYPE_ERROR = b'\x01'
    LICOSA_PKT_TYPE_IMU = b'\x02'
    LICOSA_PKT_TYPE_IMU_FULL = b'\x03'
    LICOSA_PKT_TYPE_LIDAR = b'\x04'

LICOSA_PKT_TYPE_VALUES = set(item.value for item in LiCosaPacketType)

# Excludes start/type/checksum
LICOSA_PKT_LENGTHS = {
    LiCosaPacketType.LICOSA_PKT_TYPE_ERROR: 1,
    LiCosaPacketType.LICOSA_PKT_TYPE_IMU: 16,

    # 22 bytes of data, 2 bytes of padding as per C struct-padding
    LiCosaPacketType.LICOSA_PKT_TYPE_IMU_FULL: 24,
    LiCosaPacketType.LICOSA_PKT_TYPE_LIDAR: 640
}

@dataclass
class _LiCosaPacketBase:
    """Base dataclass for LiCosa packets."""
    data_buf: bytes


@dataclass
class LiCosaPacket(_LiCosaPacketBase):
    """Dataclass for LiCosa packets."""
    pkt_type: LiCosaPacketType


@dataclass
class _LiCosaErrorPacketBase(_LiCosaPacketBase):
    """Error LiCosa packet base."""
    err: int


@dataclass
class LiCosaErrorPacket(LiCosaPacket, _LiCosaErrorPacketBase):
    """Error LiCosa packet."""
    PKT_TYPE: ClassVar[LiCosaPacketType] = LiCosaPacketType.LICOSA_PKT_TYPE_ERROR
    pkt_type: LiCosaPacketType = LiCosaPacketType.LICOSA_PKT_TYPE_ERROR


@dataclass
class _LiCosaIMUPacketBase(_LiCosaPacketBase):
    """IMU LiCosa packet base."""
    ts: np.uint64
    yaw: np.float64
    roll: np.float64
    pitch: np.float64


@dataclass
class LiCosaIMUPacket(LiCosaPacket, _LiCosaIMUPacketBase):
    """IMU LiCosa packet."""
    PKT_TYPE: ClassVar[LiCosaPacketType] = LiCosaPacketType.LICOSA_PKT_TYPE_IMU
    pkt_type: LiCosaPacketType = LiCosaPacketType.LICOSA_PKT_TYPE_IMU


@dataclass
class _LiCosaIMUFullPacketBase(_LiCosaIMUPacketBase):
    """Full IMU LiCosa packet base."""
    sys_calib: np.uint8
    gyro_calib: np.uint8
    accel_calib: np.uint8
    mag_calib: np.uint8
    sys_status: np.uint8
    sys_error: np.uint8


@dataclass
class LiCosaIMUFullPacket(LiCosaIMUPacket, _LiCosaIMUFullPacketBase):
    """Full IMU LiCosa packet."""
    PKT_TYPE: ClassVar[LiCosaPacketType] = LiCosaPacketType.LICOSA_PKT_TYPE_IMU_FULL
    pkt_type: LiCosaPacketType = LiCosaPacketType.LICOSA_PKT_TYPE_IMU_FULL


@dataclass
class _LiCosaLidarPacketBase(_LiCosaPacketBase):
    """Lidar LiCosa packet base."""
    sensors: List[VL53L5CXSensorData]


@dataclass
class LiCosaLidarPacket(LiCosaPacket, _LiCosaLidarPacketBase):
    """Lidar LiCosa packet."""
    PKT_TYPE: ClassVar[LiCosaPacketType] = LiCosaPacketType.LICOSA_PKT_TYPE_LIDAR
    pkt_type: LiCosaPacketType = LiCosaPacketType.LICOSA_PKT_TYPE_LIDAR

class LiCosaJSONEncoder(JSONEncoder):
    """JSON Encoder helper for LiCosa packets."""

    def default(self, obj):
        """JSON Encoder helper function for LiCosa packets."""
        if isinstance(obj, (np.int_, np.int16, np.uint8, np.uint16, np.uint64)):
            return int(obj)

        elif isinstance(obj, (np.float_, np.float64)):
            return float(obj)

        elif isinstance(obj, (LiCosaPacketType)):
            return obj.name

        return JSONEncoder.default(self, obj)
