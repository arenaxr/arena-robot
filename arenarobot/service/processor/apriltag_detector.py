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
from scipy.spatial.transform import Rotation as R

# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceProcessorApriltagDetector(ArenaRobotServiceProcessor):
    """Apriltag Detector processor class for the ARENA."""

    DEVICE_INSTANCE_PROCESSOR_TYPE = "apriltag_detector"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self, processor_apriltag_detector_topic: str,
                 processor_apriltag_detector_instance_name: str = None,
                 scale_factor: int = 1,
                 camera_resolution = None,
                 camera_params = None,
                 dist_params = None,
                 apriltag_family = None,
                 apriltag_locations = None,
                 **kwargs):
        """Initialize the apriltag detector processor class."""
        self.processor_apriltag_detector_topic = processor_apriltag_detector_topic
        self.processor_apriltag_detector_instance_name = processor_apriltag_detector_instance_name

        self.prev_data         = None                  # saving data from previous frame
        self.cap               = None                  # video capture
        self.camera_resolution = camera_resolution     # camera resolution [width, height] (pixels)
        self.mtx               = None                  # camera matrix
        self.params            = camera_params         # camera matrix parameters fx, fy, cx, cy
        self.dist_params       = np.array(dist_params) # distortion parameters
        self.at_detector       = None                  # apriltag detector object
        self.flip_matrix       = None                  # flip matrix
        self.apriltag_family   = apriltag_family       # tag family
        self.apriltags         = apriltag_locations    # known apriltag poses (world coordinates in meters)

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
        self.cap = cv2.VideoCapture(0)

        # subsample to increasing processing speed
        # note: selected resolution affects camera parameters
        self.cap.set(3, self.camera_resolution[0])
        self.cap.set(4, self.camera_resolution[1])

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

        self.flip_matrix = np.array([[1,  0,  0, 0],
                                     [0, -1,  0, 0],
                                     [0,  0, -1, 0],
                                     [0,  0,  0, 1]], dtype=float)

        super().setup()

    '''
    Capture a camera frame, detect apriltags, calculate pose, and
    publish pose information to MQTT topic: realm/d/<user>/<device>/processors/pose_transformed
    TODO: Implement Kalman Filter
    TODO: figure out how this works with different users/devices (not just jpedraza/john-pi)
    TODO: handle getting wrong representation (apriltag solver has two possible solutions)
    TODO: handle multiple apriltags in frame at same time
    '''
    def fetch(self):
        # get grayscale frame for apriltag detection
        ret, frame = self.cap.read()
        im_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # undistort frame
        h, w = im_gray.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist_params, (w,h), 1, (w,h))
        dst = cv2.undistort(im_gray, self.mtx, self.dist_params, None, newcameramtx)
        
        # detect apriltags
        tags = self.at_detector.detect(dst, estimate_tag_pose=True, camera_params=self.params, tag_size=0.15)
        pose        = None
        translation = None
        rotation    = None
        
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
                M = self.flip_matrix @ M @ self.flip_matrix
                pose = tag_M @ np.linalg.inv(M)
                # print(pose)
                translation = [pose[i][3] for i in range(len(pose))]
                rotation = np.array([[pose[0][0], pose[0][1], pose[0][2]],
                                     [pose[1][0], pose[1][1], pose[1][2]],
                                     [pose[2][0], pose[2][1], pose[2][2]]], dtype=float)
                rotation = R.from_matrix(rotation)
                rotation = rotation.as_quat()
            else:
                pose        = None
                translation = None
                rotation    = None
                # print('Unknown Apriltag') 
        out = {
            "pose":        pose,
            "translation": translation,
            "rotation":    rotation
        }
        self.prev_data = out

        serializable_out = loads(dumps(
            out,
            cls=TransformedApriltagDetectorPoseJSONEncoder
        ))

        # this publishes to subtopic (self.topic)
        self.publish({"data": serializable_out})

        # print(self.topic)

class TransformedApriltagDetectorPoseJSONEncoder(JSONEncoder):
    """JSON Encoder helper for Apriltag Detector transformed attributes."""

    def default(self, o):
        """JSON Encoder helper function for Apriltag Detector pose attributes."""
        if isinstance(o, (np.ndarray)):
            return list(o)

        return JSONEncoder.default(self, o)
