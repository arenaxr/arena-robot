#!/usr/bin/env python
"""
uwb-vfio-demo2.py
Created by Alex Strasser on 6/3/21.
Modified by Perry Naseck on 6/28/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import serial
import io
import time
import math
import json
from datetime import date, datetime
from particlefilter import ParticleFilterLoc
from arena import *

import pyrealsense2.pyrealsense2 as rs

UWB_BIAS = 0.2
UWB_STD = 0.1

IN_TO_METER = 0.0254
RAD_TO_DEG = 180 / math.pi

# Code taken from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
  # Convert a quaternion into euler angles (roll, pitch, yaw)
  # roll is rotation around x  (counterclockwise)
  # pitch is rotation around y (counterclockwise)
  # yaw is rotation around z   (counterclockwise)
    
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)*RAD_TO_DEG

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)*RAD_TO_DEG

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)*RAD_TO_DEG

  return roll_x, pitch_y, yaw_z

scene = Scene(host="mqtt.arenaxr.org", scene="test0", realm="realm", namespace="astrasse")

box = Icosahedron(object_id="my_box", position=Position(0, 0, 0), scale=Scale(0.5, 0.5, 0.5), color=(255, 255, 0))
text = Text(text="User", position=Position(0, 2, 0), scale=Scale(3,3,3), parent=box)

pf = ParticleFilterLoc()
ser = serial.Serial('/dev/serial/by-id/usb-SEGGER_J-Link_000760047741-if00', 115200, timeout=1)

locations = {
  7: (166.75, 39.5, 270.75),
  8: (0, 65, 34),
  9: (132.25, 57.675, 135.75),
  11: (41.25, 27.375, 0)
}

for k in locations:
  x, y, z = locations[k]
  locations[k] = (x*IN_TO_METER, y*IN_TO_METER, z*IN_TO_METER)

print(locations)

@scene.run_once
def init():
  scene.add_object(box)
  scene.add_object(text)

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)

ser.readline()
try:
  data = None
  while True:
    line = ser.readline()
    if (len(line) > 0 and line[0] != ord('#')):
      line = line[0:-2].replace(b' ', b'').decode('utf-8')
      #ID, RANGE, RSSI, TIMESTAMP
      curData = line.split(',')
      if (len(curData) < 4):
         print(curData)
         continue
      if (int(curData[0]) in locations.keys()):
        x, y, z = locations[int(curData[0])]
        pf.depositRange(x, y, z, float(curData[1]), UWB_STD)

#      box.update_attributes(position=Position(x.value, y.value, z.value), rotation=Rotation(0, theta.value*RAD_TO_DEG, 0))
#      scene.update_object(box)

    frames = pipe.wait_for_frames()
    pose = frames.get_pose_frame()
    if pose:
      # Print some of the pose data to the terminal
      data = pose.get_pose_data()
#      print("Frame #{}".format(pose.frame_number))
#      print("Velocity: {}".format(data.velocity))
#      print("Acceleration: {}\n".format(data.acceleration))

    if data is not None:
      pf.depositVio(time.time(), data.translation.x, data.translation.z * (-1), data.translation.y, 0.0)
      # print("625_X:{: 3f} 625_Y:{: 3f} 625_Z:{: 3f}".format(data.translation.x,data.translation.y,data.translation.z))
    pfStatus, pfT, pfX, pfY, pfZ, pfTheta = pf.getTagLoc()

    rotation = euler_from_quaternion(data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z)

    if pfStatus:
      box.update_attributes(position=Position(pfX, pfY, pfZ), rotation=Rotation(-rotation[2], -rotation[1], -rotation[0]))
      scene.update_object(box)

      print('PAR_X:{: 3f}'.format(pfX), 'PAR_Y:{: 3f}'.format(pfY), 'PAR_Z:{: 3f}'.format(pfZ), 'PAR_T:{: 3f}'.format(pfTheta))
finally:
  pipe.stop()
