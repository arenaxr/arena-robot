"""
apriltag_detector.py: reads frames, detects apriltags, and returns pose.

Created by John Pedraza on 6/7/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from json import JSONEncoder, dumps, loads
from typing import List

import numpy as np

from arenarobot.service.processor import ArenaRobotServiceProcessor

# Optional imports allow for constants from this module to be referenced
# and for the service to be recognized without all of the required
# dependencies installed need for starting this service.
imports_missing = []
try:
    import cv2
except ImportError as err:
    cv2 = err
    imports_missing.append(err)
try:
    from dt_apriltags import Detector
except ImportError as err:
    Detector = err
    imports_missing.append(err)

OPENCV_RES_INDEX_HORIZ = 3
OPENCV_RES_INDEX_VERT = 4

FLIP_MATRIX = np.array([[1,  0,  0, 0],
                        [0, -1,  0, 0],
                        [0,  0, -1, 0],
                        [0,  0,  0, 1]], dtype=float)


def detection2matrix(translation, rotation):
    """
    Reformat detected translation and rotation into 4x4 matrix.

    upper-left 3x3 matrix is the rotation matrix.
    rightmost column is translation vector (x, y, z, top to bottom).
    bottom row is [0, 0, 0, 1] to help with transformations.
    """
    translation = np.array([[t[0] for t in translation]])
    temp_mtx = np.concatenate((rotation, translation.T),
                              axis=1)
    temp_mtx = np.concatenate((temp_mtx, [np.array([0, 0, 0, 1])]),
                              axis=0)
    return temp_mtx


# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceProcessorApriltagDetector(ArenaRobotServiceProcessor):
    """Apriltag Detector processor class for the ARENA."""

    DEVICE_INSTANCE_PROCESSOR_TYPE = "apriltag_detector"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self,
                 video_file: str,
                 camera_resolution: List[int],
                 camera_params: List[int],
                 dist_params: List[int],
                 apriltag_locations: dict,
                 apriltag_family: str = 'tag36h11',
                 num_detector_threads: int = 1,
                 quad_decimate: float = 2.0,
                 quad_sigma: float = 0.0,
                 refine_edges: int = 1,
                 decode_sharpening: float = 0.25,
                 tag_size: float = 0.15,
                 **kwargs):
        """Initialize the apriltag detector processor class."""
        # Check for missing imports
        if len(imports_missing) > 0:
            raise Exception(imports_missing)

        # video_capture
        self.cap = video_file

        # camera resolution [width, height] in pixels
        self.camera_resolution = camera_resolution

        # camera matrix parameters fx, fy, cx, cy
        self.params = camera_params

        # distortion parameters k1, k2, p1, p2, k3
        self.dist_params = np.array(dist_params)

        # known apriltag poses (world coordinates in meters)
        self.apriltags = apriltag_locations

        # documentation for below parameters can be found at
        # https://github.com/duckietown/lib-dt-apriltags
        self.apriltag_family = apriltag_family
        self.num_detector_threads = num_detector_threads
        self.quad_decimate = quad_decimate
        self.quad_sigma = quad_sigma
        self.refine_edges = refine_edges
        self.decode_sharpening = decode_sharpening
        self.tag_size = tag_size

        self.at_detector = None
        self.mtx = None

        processor_type = (ArenaRobotServiceProcessorApriltagDetector
                          .DEVICE_INSTANCE_PROCESSOR_TYPE)

        super().__init__(
            device_instance_processor_type=processor_type,
            device_instance_prefix="apriltag_detector_",
            **kwargs
        )

    def setup(self):
        """Set up video capture and Apriltag Detector object."""
        self.cap = cv2.VideoCapture(self.cap)

        # subsample to increasing processing speed
        # note: camera parameters depend on which resolution you're using
        self.cap.set(OPENCV_RES_INDEX_HORIZ, self.camera_resolution[0])
        self.cap.set(OPENCV_RES_INDEX_VERT, self.camera_resolution[1])

        # camera matrix
        self.mtx = np.array([[self.params[0], 0.000000000000, self.params[2]],
                             [0.000000000000, self.params[1], self.params[3]],
                             [0.000000000000, 0.000000000000, 1.000000000000]])

        # apriltag detector
        # set quad_decimate to 2.0 for faster detections, worse accuracy
        # setting debug=1 saves images - don't use, runs very slowly
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families=self.apriltag_family,
                                    nthreads=self.num_detector_threads,
                                    quad_decimate=self.quad_decimate,
                                    quad_sigma=self.quad_sigma,
                                    refine_edges=self.refine_edges,
                                    decode_sharpening=self.decode_sharpening,
                                    debug=0)

        super().setup()

    # Planned: figure out how this works with different users/devices
    #     (not just jpedraza/john-pi)
    #     - so the device for the topic is just specified when
    #       running arena-robot-service
    # Planned: handle getting wrong representation
    #     (apriltag solver has two possible solutions)
    # Planned: handle multiple apriltags in frame at same time
    # Planned: cap.release()?
    def fetch(self):
        """
        Capture a camera frame, detect apriltags, and calculate pose.

        Publish pose information to MQTT topic:
            realm/d/<user>/<device>/processors/apriltag_pose
        """
        # get grayscale frame for apriltag detection
        ret, frame = self.cap.read()
        if not ret:
            print('read() failed')
            return

        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # undistort frame
        height, width = gray_image.shape[:2]
        new_camera_mtx, _ = cv2.getOptimalNewCameraMatrix(self.mtx,
                                                          self.dist_params,
                                                          (width, height), 1,
                                                          (width, height))
        dst = cv2.undistort(gray_image, self.mtx,
                            self.dist_params, None, new_camera_mtx)

        # detect apriltags
        tags = self.at_detector.detect(dst, estimate_tag_pose=True,
                                       camera_params=self.params,
                                       tag_size=self.tag_size)

        pose = None
        if len(tags) > 0:
            temp_mtx = detection2matrix(tags[0].pose_t, tags[0].pose_R)
            if str(tags[0].tag_id) in self.apriltags:
                # given tag pose in world coordinates and tag pose
                # in camera coordinates, perform transformation and return
                # pose of camera in world coordinates
                tag_mtx = self.apriltags[str(tags[0].tag_id)]
                temp_mtx = FLIP_MATRIX @ temp_mtx @ FLIP_MATRIX
                pose = tag_mtx @ np.linalg.inv(temp_mtx)
            else:
                print('Unknown Apriltag')

        out = {
            "pose": pose,
        }
        serializable_out = loads(dumps(
            out,
            cls=TransformedApriltagDetectorJSONEncoder
        ))

        # this publishes to subtopic (self.topic)
        self.publish({"data": serializable_out})


class TransformedApriltagDetectorJSONEncoder(JSONEncoder):
    """JSON Encoder helper for Apriltag Detector transformed attributes."""

    def default(self, o):
        """JSON Encoder helper for Apriltag Detector pose attributes."""
        if isinstance(o, (np.ndarray)):
            return list(o)

        return JSONEncoder.default(self, o)
