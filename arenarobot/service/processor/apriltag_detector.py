"""
apriltag_detector.py: reads frames, detects apriltags, and returns pose.

Created by John Pedraza on 6/7/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from json import JSONEncoder, dumps, loads
import numpy as np
import cv2
from paho.mqtt.client import MQTTMessage
from transformations import euler_matrix, quaternion_matrix
from dt_apriltags import Detector
from arenarobot.service.processor import ArenaRobotServiceProcessor
from scipy.spatial.transform import Rotation as Ro
from filterpy.kalman import predict, update
from filterpy.common import Q_discrete_white_noise
import time

HORIZONTAL_RESOLUTION = 3
VERTICAL_RESOLUTION = 4
        
FLIP_MATRIX = np.array([[1,  0,  0, 0],
                        [0, -1,  0, 0],
                        [0,  0, -1, 0],
                        [0,  0,  0, 1]], dtype=float)


# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceProcessorApriltagDetector(ArenaRobotServiceProcessor):
    """Apriltag Detector processor class for the ARENA."""

    DEVICE_INSTANCE_PROCESSOR_TYPE = "apriltag_detector"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self,
                 camera_resolution:  list[int] = None,
                 camera_params:      list[int] = None,
                 dist_params:        list[int] = None,
                 apriltag_family:    str       = None,
                 apriltag_locations: dict      = None,
                 tagSize:            float     = 0.15,
                 cap = cv2.VideoCapture(0),
                 **kwargs):
        """Initialize the apriltag detector processor class."""
        
        self.x                 = [0, 0, 0, 0, 0, 0]    # filter state estimate
        self.P = np.diag([500, 100, 500, 100, 500, 100]) # filter state covariances
        self.prev_translation  = [0, 0, 0]
        self.prev_time         = time.time()           # previous time measurement
        self.cap               = cap            # video capture
        self.camera_resolution = camera_resolution     # camera resolution [width, height] (pixels)
        self.mtx               = None                  # camera matrix
        self.params            = camera_params         # camera matrix parameters fx, fy, cx, cy
        self.dist_params       = np.array(dist_params) # distortion parameters
        self.at_detector       = None                  # apriltag detector object
        self.apriltag_family   = apriltag_family       # tag family
        self.apriltags         = apriltag_locations    # known apriltag poses (world coordinates in meters)
        self.tagSize = tagSize
        
        processor_type = (ArenaRobotServiceProcessorApriltagDetector
                          .DEVICE_INSTANCE_PROCESSOR_TYPE)

        super().__init__(
            device_instance_processor_type=processor_type,
            device_instance_prefix="apriltag_detector_",
            **kwargs
        )
    '''
    Set up Apriltag Detector object and define a bunch of stuff
    '''
    def setup(self):
        # Initialize camera using /dev/video0 (works for USB cameras 
        # but not Arducam-type cameras because the Raspberry Pi
        # Foundation is sooooo silly)
        # self.cap = cv2.VideoCapture(0)

        # subsample to increasing processing speed
        # note: selected resolution affects camera parameters
        self.cap.set(HORIZONTAL_RESOLUTION, self.camera_resolution[0])
        self.cap.set(VERTICAL_RESOLUTION, self.camera_resolution[1])

        # camera matrix
        self.mtx = np.array([[self.params[0], 0.000000000000, self.params[2]],
                             [0.000000000000, self.params[1], self.params[3]],
                             [0.000000000000, 0.000000000000, 1.000000000000]])

        # apriltag detector
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families=self.apriltag_family,
                                    nthreads=4,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        super().setup()

    '''
    Capture a camera frame, detect apriltags, calculate pose, and
    publish pose information to MQTT topic: realm/d/<user>/<device>/processors/pose_transformed
    TODO: Implement Kalman Filter
    TODO: figure out how this works with different users/devices (not just jpedraza/john-pi)
          - so the device for the topic is just specified when running arena-robot-service
    TODO: handle getting wrong representation (apriltag solver has two possible solutions)
    TODO: handle multiple apriltags in frame at same time
    TODO: cap.release()?
    TODO: add among us support
    '''
    def fetch(self):
        # get grayscale frame for apriltag detection
        ret, frame = self.cap.read()
        grayImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # undistort frame
        h, w = grayImage.shape[:2]
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist_params, (w,h), 1, (w,h))
        dst = cv2.undistort(grayImage, self.mtx, self.dist_params, None, newCameraMatrix)
        
        # detect apriltags
        tags = self.at_detector.detect(dst, estimate_tag_pose=True, camera_params=self.params, tag_size=self.tagSize)
        pose = None
        # TODO: handle multiple apriltags
        if len(tags) > 0:
            Rot = tags[0].pose_R
            T = tags[0].pose_t

            # TODO: find a nicer way to notate/document this
            # M = [Rot,     T.T]
            #     [0, 0, 0, 1  ]
            
            T = [t[0] for t in T] 
            T = np.array([T])
            M = np.concatenate((Rot, T.T), axis=1)
            bottom_row = np.array([0, 0, 0, 1], dtype=float)
            M = np.concatenate((M, [bottom_row]), axis=0)
            
            if str(tags[0].tag_id) in self.apriltags: 
                tag_M = self.apriltags[str(tags[0].tag_id)]
                M = FLIP_MATRIX @ M @ FLIP_MATRIX
                pose = tag_M @ np.linalg.inv(M)
                # rotation = Ro.from_matrix(rotation)
                # rotation = rotation.as_quat()
            else:
                print('Unknown Apriltag')
            
        out = {
            "pose":        pose
        }
        print(out)

        serializable_out = loads(dumps(
            out,
            cls=TransformedApriltagDetectorJSONEncoder
        ))

        # this publishes to subtopic (self.topic)
        self.publish({"data": serializable_out})

class TransformedApriltagDetectorJSONEncoder(JSONEncoder):
    """JSON Encoder helper for Apriltag Detector transformed attributes."""

    def default(self, o):
        """JSON Encoder helper function for Apriltag Detector pose attributes."""
        if isinstance(o, (np.ndarray)):
            return list(o)

        return JSONEncoder.default(self, o)
