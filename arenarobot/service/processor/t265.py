"""
t265.py: Transformations for T265 input.

Created by Perry Naseck on 6/1/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from json import JSONEncoder, dumps, loads

import numpy as np
from paho.mqtt.client import MQTTMessage
from transformations import euler_matrix, quaternion_matrix

from arenarobot.service.processor import ArenaRobotServiceProcessor
from arenarobot.service.sensor.t265 import ArenaRobotServiceSensorT265


# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceProcessorT265(ArenaRobotServiceProcessor):
    """T265 processor class for the ARENA."""

    DEVICE_INSTANCE_PROCESSOR_TYPE = "t265"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self, sensor_t265_topic: str,
                 sensor_t265_instance_name: str = None,
                 camera_orientation: int = 0,
                 scale_factor: int = 1,
                 **kwargs):
        """Initialize the Y265 processor class."""
        self.sensor_t265_topic = sensor_t265_topic
        self.sensor_t265_instance_name = sensor_t265_instance_name
        self.camera_orientation = camera_orientation
        self.scale_factor = scale_factor

        if (not isinstance(self.camera_orientation, int) or
                not 0 <= self.camera_orientation <= 3):
            raise ValueError('Unknown camera orientation!')

        self.fetched_once = False
        self.prev_data = False
        self.h_aeroref_t265ref = None
        self.h_t265body_aerobody = None

        processor_type = (ArenaRobotServiceProcessorT265
                          .DEVICE_INSTANCE_PROCESSOR_TYPE)
        super().__init__(
            device_instance_processor_type=processor_type,
            device_instance_prefix="t265_",
            **kwargs
        )

    def setup(self):
        """Set up T265 processor."""
        # Forward, USB port to the right
        if self.camera_orientation == 0:
            self.h_aeroref_t265ref = np.array(
                [[0, 0, -1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
            self.h_t265body_aerobody = np.linalg.inv(self.h_aeroref_t265ref)
        # Downfacing, USB port to the right
        elif self.camera_orientation == 1:
            self.h_aeroref_t265ref = np.array(
                [[0, 0, -1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
            self.h_t265body_aerobody = np.array(
                [[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        # 45degree forward
        elif self.camera_orientation == 2:
            self.h_aeroref_t265ref = np.array(
                [[0, 0, -1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
            self.h_t265body_aerobody = (
                euler_matrix(np.pi/4, 0, 0)).dot(
                    np.linalg.inv(self.h_aeroref_t265ref))
        # Upward facing, USB port to the right
        elif self.camera_orientation == 3:
            self.h_aeroref_t265ref = np.array(
                [[0, 0, -1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
            self.h_t265body_aerobody = np.array(
                [[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        super().setup()

    def fetch(self):
        """Fetch T265 data."""
        # Ensure not run more than once (sets up callbacks)
        if self.fetched_once:
            print("T265 processor should have interval of -1!")
            return
        self.fetched_once = True

        # pylint: disable=unused-argument
        def process_t265(client, userdata, msg: MQTTMessage):
            """Process a T265 sensor message."""
            payload = self.decode_payload(msg)
            if payload is None:
                return

            # Ignore messages from other sensor types
            if (payload["device_instance_sensor_type"] !=
                    ArenaRobotServiceSensorT265.DEVICE_INSTANCE_SENSOR_TYPE):
                return

            # If set, ignore other instances of T265 sensors
            if (self.sensor_t265_instance_name is not None and
                    payload["device_instance_name"] !=
                    self.sensor_t265_instance_name):
                return

            data = payload["msg"]["data"]

            # Skip errors
            if "error" in data:
                print("Skipping errored data for",
                      payload["device_instance_name"])
                return
            if "pose" not in data:
                print("Pose missing from data for",
                      payload["device_instance_name"])
                return

            pose = data["pose"]
            rotation = pose["rotation"]
            translation = pose["translation"]
            velocity = pose["velocity"]

            # In transformations, Quaternions w+ix+jy+kz are represented as
            # [w, x, y, z]!
            h_t265_t265body = quaternion_matrix([
                rotation["w"],
                rotation["x"],
                rotation["y"],
                rotation["z"]])
            h_t265_t265body[0][3] = translation["x"] * self.scale_factor
            h_t265_t265body[1][3] = translation["y"] * self.scale_factor
            h_t265_t265body[2][3] = translation["z"] * self.scale_factor

            # Transform to aeronautic coordinates (body AND reference frame!)
            h_aeroref_aerobody = self.h_aeroref_t265ref.dot(
                h_t265_t265body.dot(self.h_t265body_aerobody))

            # Calculate GLOBAL XYZ speed (speed from T265 is already GLOBAL)
            v_aeroref_aerobody = quaternion_matrix([1, 0, 0, 0])
            v_aeroref_aerobody[0][3] = velocity["x"]
            v_aeroref_aerobody[1][3] = velocity["y"]
            v_aeroref_aerobody[2][3] = velocity["z"]
            v_aeroref_aerobody = self.h_aeroref_t265ref.dot(v_aeroref_aerobody)

            out = {
                "h_t265_t265body": h_t265_t265body,
                "h_aeroref_aerobody": h_aeroref_aerobody,
                "v_aeroref_aerobody": v_aeroref_aerobody
            }
            self.prev_data = out
            serializable_out = loads(dumps(
                out,
                cls=TransformedT265PoseJSONEncoder
            ))
            self.publish({"data": serializable_out})

        self.device.message_callback_add(self.sensor_t265_topic, process_t265)


class TransformedT265PoseJSONEncoder(JSONEncoder):
    """JSON Encoder helper for T265 transformed attributes."""

    def default(self, o):
        """JSON Encoder helper function for T265 pose attributes."""
        if isinstance(o, (np.ndarray)):
            return list(o)

        return JSONEncoder.default(self, o)
