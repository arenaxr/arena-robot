#!/usr/bin/env python
# Alex Strasser Cylab 6/3/202

import serial
import io
import time
import numpy as np
from math import pi
import json
from datetime import date, datetime
from particlefilter import ParticleFilterLoc
#from arena import *
import pyrealsense2.pyrealsense2 as rs
#from scipy.spatial.transform import Rotation
UWB_BIAS = 0.2
UWB_STD = 0.1
IN_TO_METER = 0.0254
RAD_TO_DEG = 180 / pi
#scene = Scene(host="arenaxr.org", scene="test0", realm="realm", namespace="astrasse")
#box = Box(object_id="my_box", position=Position(0, 0, 0), scale=Scale(0.3, 1.8, 0.3), color=(255, 0, 0))
#text = Text(text="User", position=Position(0, 2, 0), scale=Scale(3,3,3), parent=box)
pf = ParticleFilterLoc()
ser = serial.Serial('/dev/serial/by-id/usb-SEGGER_J-Link_000760025484-if00', 115200, timeout=1)
locations = {
  7: (166.75, 39.5, 270.75),
  8: (-7.5, 91.25, 104.5),
  9: (132.25, 57.675, 135.75),
  11: (41.25, 27.375, 0)
}
locations = {
  7: (1.828, 1.749, 2.558),
  8: (2.806, 0.799, -1.901),
  9: (-1.82, 0.168, 1.955),
  10: (-2.94, 2.804, -2.126),
  11: (1.334, 2.052, -3.678)
}
#for k in locations:
#  x, y, z = locations[k]
#  locations[k] = (x*IN_TO_METER, y*IN_TO_METER, z*IN_TO_METER)
print(locations)
#@scene.run_once
#def init():
#  scene.add_object(box)
#  scene.add_object(text)
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
    if (line[0] != ord('#')):
      line = line[0:-2].replace(b' ', b'').decode('utf-8')
      #ID, RANGE, RSSI, TIMESTAMP
      curData = line.split(',')
      if (len(curData) < 4):
         print(curData)
         continue
      if int(curData[0]) in locations:
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
      pf.depositVio(np.float64(time.time()), data.translation.x, data.translation.z * (-1), data.translation.y, 0.0)
      print("625_X:{: 3f} 625_Y:{: 3f} 625_Z:{: 3f}".format(data.translation.x,data.translation.y,data.translation.z))
    pf_data = pf.getTagLoc()
    print('PAR_X:{: 3f}'.format(pf_data[2]), 'PAR_Y:{: 3f}'.format(pf_data[3]), 'PAR_Z:{: 3f}'.format(pf_data[4]), 'PAR_T:{: 3f}'.format(pf_data[5]))
#    print(data)
finally:
  pipe.stop()