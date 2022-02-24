"""
__init__.py: Definitions for LiCosa packet parser.

Created by Perry Naseck on 2/22/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from json import JSONEncoder
from struct import unpack
from typing import ClassVar, List

import numpy as np

LICOSA_PKT_HEADER = b'\xCC'


class LiCosaState(Enum):
    """LiCosa serial sensor state class."""
    LICOSA_PKT_STATE_START = auto()
    LICOSA_PKT_STATE_TYPE = auto()
    LICOSA_PKT_STATE_DATA = auto()
    LICOSA_PKT_STATE_CHECKSUM = auto()


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
class LiCosaLidarSensor():
    """Lidar LiCosa sensor data."""
    raw_values: List[np.uint16]
    ranges: List[np.uint16]
    statuses: List[np.uint8]


@dataclass
class _LiCosaLidarPacketBase(_LiCosaPacketBase):
    """Lidar LiCosa packet base."""
    sensors: List[LiCosaLidarSensor]


@dataclass
class LiCosaLidarPacket(LiCosaPacket, _LiCosaLidarPacketBase):
    """Lidar LiCosa packet."""
    PKT_TYPE: ClassVar[LiCosaPacketType] = LiCosaPacketType.LICOSA_PKT_TYPE_LIDAR
    pkt_type: LiCosaPacketType = LiCosaPacketType.LICOSA_PKT_TYPE_LIDAR


class LiCosaPacketParser():
    """LiCosa packet parser."""

    def __init__(self):
        """Initialize the LiCosa packet parser class."""
        self.state = LiCosaState.LICOSA_PKT_STATE_START
        self.pkt_type = None
        self.pkt_data_len = None
        self.pkt_data_index = 0
        self.pkt_buf_list = []
        self.pkt_data_buf_list = []

    @staticmethod
    def calculate_checksum(pkt_type, pkt_data_buf_list):
        """Calculate the LiCosa packet checksum."""
        return (
            (
                sum(int.from_bytes(x, 'big') for x in pkt_data_buf_list) +
                int.from_bytes(LICOSA_PKT_HEADER, 'big') +
                int.from_bytes(pkt_type.value, 'big')
            ) & 0xFF
        ).to_bytes(1, 'big')

    @staticmethod
    def parse_error_pkt_data(pkt_data_buf):
        """Parse LiCosa error packet data."""
        err_msg = int(pkt_data_buf)
        return LiCosaErrorPacket(
            data_buf=pkt_data_buf,
            err=err_msg
        )

    @staticmethod
    def parse_imu_pkt_data(pkt_data_buf):
        """Parse LiCosa IMU packet data."""
        ts, yaw, roll, pitch = unpack("<Lfff", pkt_data_buf)
        return LiCosaIMUPacket(
            data_buf=pkt_data_buf,
            ts=np.uint64(ts),
            yaw=np.float64(yaw),
            roll=np.float64(roll),
            pitch=np.float64(pitch)
        )

    @staticmethod
    def parse_imu_full_pkt_data(pkt_data_buf):
        """Parse LiCosa full IMU packet data."""
        (ts, yaw, roll, pitch, sys_calib, gyro_calib, accel_calib, mag_calib,
         sys_status, sys_error) = unpack("<LfffBBBBBBxx", pkt_data_buf)
        return LiCosaIMUFullPacket(
            data_buf=pkt_data_buf,
            ts=np.uint64(ts),
            yaw=np.float64(yaw),
            roll=np.float64(roll),
            pitch=np.float64(pitch),
            sys_calib=np.float64(sys_calib),
            gyro_calib=np.uint8(gyro_calib),
            accel_calib=np.uint8(accel_calib),
            mag_calib=np.uint8(mag_calib),
            sys_status=np.uint8(sys_status),
            sys_error=np.uint8(sys_error)
        )

    @staticmethod
    def parse_lidar_pkt_data(pkt_data_buf):
        """Parse LiCosa lidar packet data."""
        num_sensors = len(pkt_data_buf) // 128
        sensors = [None] * num_sensors
        for i in range(num_sensors):
            buf = pkt_data_buf[(i*128):((i+1)*128)]
            raw_values = unpack("<" + ("H" * 64), buf)
            ranges = []
            statuses = []
            for j in range(64):
                ranges.append(np.uint16(raw_values[j]) & 0xFFF)
                statuses.append(np.uint8((np.uint16(raw_values[j]) >> 12) & 0xF))
            sensors[i] = LiCosaLidarSensor(
                raw_values=raw_values,
                ranges=ranges,
                statuses=statuses
            )

        return LiCosaLidarPacket(
            data_buf=pkt_data_buf,
            sensors=sensors
        )

    @staticmethod
    def parse_pkt_data(pkt_type, pkt_data_buf_list):
        """Parse LiCosa packet data."""
        pkt_data_buf = b''.join(pkt_data_buf_list)
        if pkt_type == LiCosaPacketType.LICOSA_PKT_TYPE_ERROR:
            return LiCosaPacketParser.parse_error_pkt_data(pkt_data_buf)
        elif pkt_type == LiCosaPacketType.LICOSA_PKT_TYPE_IMU:
            return LiCosaPacketParser.parse_imu_pkt_data(pkt_data_buf)
        elif pkt_type == LiCosaPacketType.LICOSA_PKT_TYPE_IMU_FULL:
            return LiCosaPacketParser.parse_imu_full_pkt_data(pkt_data_buf)
        elif pkt_type == LiCosaPacketType.LICOSA_PKT_TYPE_LIDAR:
            return LiCosaPacketParser.parse_lidar_pkt_data(pkt_data_buf)

        return LiCosaPacket(
            pkt_type=pkt_type,
            data_buf=pkt_data_buf
        )

    def next_byte(self, msg_byte):
        """Parse LiCosa packets."""
        self.pkt_buf_list.append(msg_byte)
        if self.state == LiCosaState.LICOSA_PKT_STATE_START:
            if msg_byte == LICOSA_PKT_HEADER:
                self.pkt_buf_list = [ msg_byte ]
                self.state = LiCosaState.LICOSA_PKT_STATE_TYPE
            else:
                print(f"LiCosa: non-start byte: {msg_byte}, bytes: {len(self.pkt_buf_list)}")

        elif self.state == LiCosaState.LICOSA_PKT_STATE_TYPE:
            if msg_byte in LICOSA_PKT_TYPE_VALUES:
                self.pkt_type = LiCosaPacketType(msg_byte)
                self.pkt_data_len = LICOSA_PKT_LENGTHS[self.pkt_type]
                self.pkt_data_index = 0
                self.pkt_data_buf_list = [None] * self.pkt_data_len
                self.state = LiCosaState.LICOSA_PKT_STATE_DATA
            else:
                print(f"LiCosa: invalid message type: {msg_byte}, bytes: {self.pkt_buf_list}")
                self.state = LiCosaState.LICOSA_PKT_STATE_START

        elif self.state == LiCosaState.LICOSA_PKT_STATE_DATA:
            self.pkt_data_buf_list[self.pkt_data_index] = msg_byte
            self.pkt_data_index += 1
            if self.pkt_data_index == self.pkt_data_len:
                self.state = LiCosaState.LICOSA_PKT_STATE_CHECKSUM

        elif self.state == LiCosaState.LICOSA_PKT_STATE_CHECKSUM:
            msg_chk_cal = self.calculate_checksum(self.pkt_type, self.pkt_data_buf_list)
            if msg_chk_cal != msg_byte:
                print(f"LiCosa: invalid checksum! type: {self.pkt_type}, computed: {msg_chk_cal}, received: {msg_byte}, bytes: {self.pkt_buf_list}")
                self.state = LiCosaState.LICOSA_PKT_STATE_START
            else:
                self.state = LiCosaState.LICOSA_PKT_STATE_START
                return self.parse_pkt_data(self.pkt_type, self.pkt_data_buf_list)

        return False


class LiCosaJSONEncoder(JSONEncoder):
    """JSON Encoder helper for LiCosa packets."""

    def default(self, obj):
        """JSON Encoder helper function for LiCosa packets."""
        if isinstance(obj, (np.int_, np.uint8, np.uint16, np.uint64)):
            return int(obj)

        elif isinstance(obj, (np.float_, np.float64)):
            return float(obj)

        elif isinstance(obj, (LiCosaPacketType)):
            return obj.name

        return JSONEncoder.default(self, obj)
