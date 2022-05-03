"""
parser.py: Definitions for LiCosa packet parser.

Created by Perry Naseck on 2/24/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from enum import Enum, auto
from struct import unpack

import numpy as np

from .packets import (LICOSA_PKT_LENGTHS, LICOSA_PKT_TYPE_VALUES,
                      LiCosaErrorPacket, LiCosaIMUFullPacket, LiCosaIMUPacket,
                      LiCosaLidarPacket, LiCosaPacket, LiCosaPacketType)
from vl53l5cx_py.helpers import VL53L5CXSensorData

LICOSA_PKT_HEADER = b'\xCC'


class LiCosaState(Enum):
    """LiCosa serial sensor state class."""
    LICOSA_PKT_STATE_START = auto()
    LICOSA_PKT_STATE_TYPE = auto()
    LICOSA_PKT_STATE_DATA = auto()
    LICOSA_PKT_STATE_CHECKSUM = auto()

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
            distance_mm = []
            target_status = []
            for j in range(64):
                distance_mm.append(np.int16(raw_values[j]) & 0xFFF)
                target_status.append(np.uint8((np.uint16(raw_values[j]) >> 12) & 0xF))
            sensors[i] = VL53L5CXSensorData(
                distance_mm=distance_mm,
                target_status=target_status
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
