"""
__init__.py: Definitions for LiCosa packet parser.

Created by Perry Naseck on 2/22/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from dataclasses import dataclass
from enum import Enum, auto
from struct import unpack
from typing import ClassVar, List

import numpy as np

LICOSA_PKT_HEADER = 0xCC


class LiCosaState(Enum):
    """LiCosa serial sensor state class."""
    LICOSA_PKT_STATE_START = auto()
    LICOSA_PKT_STATE_TYPE = auto()
    LICOSA_PKT_STATE_DATA = auto()
    LICOSA_PKT_STATE_CHECKSUM = auto()


class LiCosaPacketType(Enum):
    """LiCosa serial sensor packet type class."""
    LICOSA_PKT_TYPE_ERROR = 0x01
    LICOSA_PKT_TYPE_IMU = 0x02
    LICOSA_PKT_TYPE_IMU_FULL = 0x03
    LICOSA_PKT_TYPE_LIDAR = 0x04
    LICOSA_PKT_TYPE_UNKNOWN = auto()

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
class LiCosaPacket:
    """Base dataclass for LiCosa packets."""
    TYPE: int
    data_buf: bytes


@dataclass
class LiCosaErrorPacket(LiCosaPacket):
    """Error LiCosa packet."""
    TYPE: ClassVar[int] = LiCosaPacketType.LICOSA_PKT_TYPE_ERROR
    err: int


@dataclass
class LiCosaIMUPacket(LiCosaPacket):
    """IMU LiCosa packet."""
    TYPE: ClassVar[int] = LiCosaPacketType.LICOSA_PKT_TYPE_IMU
    ts: np.uint64
    yaw: np.float64
    roll: np.float64
    pitch: np.float64


@dataclass
class LiCosaIMUFullPacket(LiCosaIMUPacket):
    """Full IMU LiCosa packet."""
    TYPE: ClassVar[int] = LiCosaPacketType.LICOSA_PKT_TYPE_IMU_FULL
    sys_calib: np.unint8
    gyro_calib: np.unint8
    accel_calib: np.unint8
    mag_calib: np.unint8
    sys_status: np.unint8
    sys_error: np.unint8


@dataclass
class LiCosaLidarSensor():
    """Lidar LiCosa sensor data."""
    raw_values: List[np.uint16]
    ranges: List[np.uint16]
    statuses: List[np.uint8]


@dataclass
class LiCosaLidarPacket(LiCosaPacket):
    """Lidar LiCosa packet."""
    TYPE: ClassVar[int] = LiCosaPacketType.LICOSA_PKT_TYPE_LIDAR
    sensors: List[LiCosaLidarSensor]


class LiCosaPacketParser():
    """LiCosa packet parser."""

    def __init__(self):
        """Initialize the LiCosa packet parser class."""
        self.state = LiCosaState.LICOSA_PKT_STATE_START
        self.pkt_type = None
        self.pkt_data_len = None
        self.pkt_data_index = 0
        self.pkt_data_buf_list = []

    @staticmethod
    def calculate_checksum(pkt_type, pkt_data_buf_list):
        """Calculate the LiCosa packet checksum."""
        return (sum(pkt_data_buf_list) + LICOSA_PKT_HEADER + pkt_type) & 0xFF

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
            gyro_calib=np.unint8(gyro_calib),
            accel_calib=np.unint8(accel_calib),
            mag_calib=np.unint8(mag_calib),
            sys_status=np.unint8(sys_status),
            sys_error=np.unint8(sys_error)
        )

    @staticmethod
    def parse_lidar_pkt_data(pkt_data_buf):
        """Parse LiCosa lidar packet data."""
        num_sensors = len(pkt_data_buf) / 128
        sensors = [None] * num_sensors
        for i in enumerate(num_sensors):
            buf = pkt_data_buf[(i*128):((i+1)*128)]
            raw_values = unpack("<" + ("H" * 64), buf)
            ranges = []
            statuses = []
            for i in range(64):
                ranges.append(np.uint16(raw_values[i]) & 0xFFF)
                statuses.append(np.uint8((np.uint16(raw_values[i]) >> 12) & 0xF))
            sensors.append(LiCosaLidarSensor(
                raw_values=raw_values,
                ranges=ranges,
                statuses=statuses
            ))

        return LiCosaLidarPacket(
            data_buf=pkt_data_buf,
            sensors=sensors
        )

    @staticmethod
    def parse_pkt_data(pkt_type, pkt_data_buf_list):
        """Parse LiCosa packet data."""
        pkt_data_buf = bytes(pkt_data_buf_list)
        if pkt_type == LiCosaPacketType.LICOSA_PKT_TYPE_ERROR:
            return LiCosaPacketParser.parse_error_pkt_data(pkt_data_buf)
        elif pkt_type == LiCosaPacketType.LICOSA_PKT_TYPE_IMU:
            return LiCosaPacketParser.parse_imu_pkt_data(pkt_data_buf)
        elif pkt_type == LiCosaPacketType.LICOSA_PKT_TYPE_IMU_FULL:
            return LiCosaPacketParser.parse_imu_full_pkt_data(pkt_data_buf)
        elif pkt_type == LiCosaPacketType.LICOSA_PKT_TYPE_LIDAR:
            return LiCosaPacketParser.parse_lidar_pkt_data(pkt_data_buf)

        return LiCosaPacket(
            type=pkt_type,
            data_buf=pkt_data_buf
        )

    async def next_byte(self, msg_byte):
        """Parse LiCosa packets."""
        if self.state == LiCosaState.LICOSA_PKT_STATE_START:
            if msg_byte == LICOSA_PKT_HEADER:
                self.state = LiCosaState.LICOSA_PKT_STATE_TYPE

        elif self.state == LiCosaState.LICOSA_PKT_STATE_TYPE:
            if msg_byte in LICOSA_PKT_TYPE_VALUES:
                self.pkt_type = msg_byte
                self.pkt_data_len = LICOSA_PKT_LENGTHS[self.pkt_type]
                self.pkt_data_index = 0
                self.pkt_data_buf_list = [None] * self.pkt_data_len
                self.state = LiCosaState.LICOSA_PKT_STATE_DATA
            else:
                print("LiCosa: invalid message type")
                self.state = LiCosaState.LICOSA_PKT_STATE_START

        elif self.state == LiCosaState.LICOSA_PKT_STATE_DATA:
            self.pkt_data_buf_list[self.pkt_data_index] = msg_byte
            self.pkt_data_index += 1
            if self.pkt_data_index == self.pkt_data_len:
                self.state = LiCosaState.LICOSA_PKT_STATE_CHECKSUM

        elif self.state == LiCosaState.LICOSA_PKT_STATE_CHECKSUM:
            msg_chk_cal = self.calculate_checksum(self.pkt_type, self.pkt_data_buf_list)
            if msg_chk_cal != msg_byte:
                    print("LiCosa: invalid checksum")
                    self.state = LiCosaState.LICOSA_PKT_STATE_START
            else:
                return self.parse_pkt_data(self.pkt_type, self.pkt_data_buf_list)

        return False
