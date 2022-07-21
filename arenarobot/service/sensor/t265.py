"""
t265.py: Definition for Intel T265 sensors.

Created by Perry Naseck on 5/6/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import time
from json import JSONEncoder, dumps, loads

from arenarobot.service.sensor import ArenaRobotServiceSensor

# Optional imports allow for constants from this module to be referenced
# and for the service to be recognized without all of the required
# dependencies installed need for starting this service.
imports_missing = []
try:
    import pyrealsense2.pyrealsense2 as rs
except ImportError as err:
    rs = err
    imports_missing.append(err)


# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceSensorT265(ArenaRobotServiceSensor):
    """T265 sensor class for the ARENA."""

    DEVICE_INSTANCE_SENSOR_TYPE = "t265"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self, timeout_ms: int = 500,
                 enable_pose_jumping: bool = False,
                 enable_relocalization: bool = False,
                 enable_map_preservation: bool = False,
                 enable_mapping: bool = False,
                 enable_dynamic_calibration: bool = True, **kwargs):
        """Initialize the T265 sensor class."""
        # Check for missing imports
        if len(imports_missing) > 0:
            raise Exception(imports_missing)

        self.timeout_ms = timeout_ms
        self.enable_pose_jumping = enable_pose_jumping
        self.enable_relocalization = enable_relocalization
        self.enable_map_preservation = enable_map_preservation
        self.enable_mapping = enable_mapping
        self.enable_dynamic_calibration = enable_dynamic_calibration

        self.rs_pipe = None
        self.rs_cfg = None
        self.rs_device = None
        self.pose_sensor = None
        self.rs_ctx = None

        sensor_type = (ArenaRobotServiceSensorT265
                       .DEVICE_INSTANCE_SENSOR_TYPE)
        super().__init__(
            device_instance_sensor_type=sensor_type,
            device_instance_prefix="t265_",
            **kwargs
        )

    def setup(self):
        """Set up T265 sensor."""
        rs.log_to_console(rs.log_severity.warn)
        self.rs_pipe = rs.pipeline()
        self.rs_cfg = rs.config()
        self.rs_cfg.enable_stream(rs.stream.pose)

        self.rs_device = self.rs_cfg.resolve(self.rs_pipe).get_device()
        self.pose_sensor = self.rs_device.first_pose_sensor()
        self.pose_sensor.set_option(rs.option.enable_pose_jumping,
                                    int(self.enable_pose_jumping))
        self.pose_sensor.set_option(rs.option.enable_relocalization,
                                    int(self.enable_relocalization))
        self.pose_sensor.set_option(rs.option.enable_map_preservation,
                                    int(self.enable_map_preservation))
        self.pose_sensor.set_option(rs.option.enable_mapping,
                                    int(self.enable_mapping))
        self.pose_sensor.set_option(rs.option.enable_dynamic_calibration,
                                    int(self.enable_dynamic_calibration))

        self.rs_ctx = rs.context()

        def rs_notification_callback(notif):
            print("INFO: T265 event: " + notif)
            self.publish({"data": {"notif": notif}})

        self.pose_sensor.set_notifications_callback(rs_notification_callback)
        self.rs_pipe.start(self.rs_cfg)

        time.sleep(1)
        super().setup()

    def fetch(self):
        """Fetch T265 data."""
        status, frames = self.rs_pipe.try_wait_for_frames(self.timeout_ms)
        if not status:
            print("REALSENSE DATA BAD!")
            self.publish({"data": {"error": "rs_data_bad"}})
            return

        # Fetch pose frame
        pose = frames.get_pose_frame()
        if not pose:
            print("No pose!")
            self.publish({"data": {"error": "rs_no_pose"}})
            return

        pose_data = pose.get_pose_data()
        serializable_pose_data = {
            "acceleration": pose_data.acceleration,
            "angular_acceleration": pose_data.angular_acceleration,
            "angular_velocity": pose_data.angular_velocity,
            "mapper_confidence": pose_data.mapper_confidence,
            "rotation": pose_data.rotation,
            "tracker_confidence": pose_data.tracker_confidence,
            "translation": pose_data.translation,
            "velocity": pose_data.velocity
        }
        serializable_pose_data = loads(dumps(
            serializable_pose_data,
            cls=T265PoseJSONEncoder
        ))

        self.publish({"data": {"pose": serializable_pose_data}})


class T265PoseJSONEncoder(JSONEncoder):
    """JSON Encoder helper for T265 pose attributes."""

    def default(self, o):
        """JSON Encoder helper function for T265 pose attributes."""
        if isinstance(o, rs.vector):
            return {"x": o.x, "y": o.y, "z": o.z}

        if isinstance(o, rs.quaternion):
            return {"w": o.w, "x": o.x, "y": o.y, "z": o.z}

        return JSONEncoder.default(self, o)
