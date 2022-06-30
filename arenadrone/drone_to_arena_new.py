#!/usr/bin/env python3

#####################################################
##          librealsense T265 to MAVLink           ##
#####################################################
# This script assumes pyrealsense2.[].so file is found under the same directory as this script
# Install required packages: 
#   pip3 install pyrealsense2
#   pip3 install transformations
#   pip3 install pymavlink
#   pip3 install apscheduler
#   pip3 install pyserial
#   pip3 install slam3d
#   pip3 install arena-py

# Set the path for IDLE
import sys
sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Import the libraries
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import transformations as tf
import math as m
import time
import argparse
import threading
import signal
import serial
# from arena.device import Device

from time import sleep
from apscheduler.schedulers.background import BackgroundScheduler
from pymavlink import mavutil

from particlefilter import ParticleFilterLoc
from arena import *

from vl53l5cx_py.driver import VL53L5CX


UWB_BIAS = 0.2
UWB_STD = 0.1

GPS_FIX_TYPE_PPP = 8
UINT16_MAX = 65535
UINT8_MAX = 255

IN_TO_METER = 0.0254
RAD_TO_DEG = 180 / m.pi
METER_TO_LATLNG = 10000000 / 111139

# Replacement of the standard print() function to flush the output
def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()

#######################################
# Parameters
#######################################

# Default configurations for connection to the FCU
connection_string_default = '/dev/ttyS0'
connection_baudrate_default = 115200

# Transformation to convert different camera orientations to NED convention. Replace camera_orientation_default for your configuration.
#   0: Forward, USB port to the right
#   1: Downfacing, USB port to the right 
#   2: Forward, 45 degree tilted down
# Important note for downfacing camera: you need to tilt the vehicle's nose up a little - not flat - before you run the script, otherwise the initial yaw will be randomized, read here for more details: https://github.com/IntelRealSense/librealsense/issues/4080. Tilt the vehicle to any other sides and the yaw might not be as stable.
camera_orientation_default = 3

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
enable_msg_vision_position_estimate = True
vision_position_estimate_msg_hz_default = 30.0

# https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
enable_msg_vision_speed_estimate = True
vision_speed_estimate_msg_hz_default = 30.0

# Monitor user's online input via keyboard, can only be used when runs from terminal
enable_user_keyboard_input = True

# Default global position for EKF home/ origin
enable_auto_set_ekf_home = True
home_lat = 0     # Somewhere random
home_lon = 0     # Somewhere random
home_alt = 0        # Somewhere random

# TODO: Taken care of by ArduPilot, so can be removed (once the handling on AP side is confirmed stable)
# In NED frame, offset from the IMU or the center of gravity to the camera's origin point
body_offset_enabled = 0
body_offset_x = 0  # In meters (m)
body_offset_y = 0  # In meters (m)
body_offset_z = 0  # In meters (m)

# Global scale factor, position x y z will be scaled up/down by this factor
scale_factor = 0.85

# Enable using yaw from compass to align north (zero degree is facing north)
compass_enabled = 1

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High 
pose_data_confidence_level = ('FAILED', 'Low', 'Medium', 'High')

# lock for thread synchronization
drone_lock = threading.Lock()
threads_should_exit = False

# default exit code is failure - a graceful termination with a
# terminate signal is possible.
exit_code = 1


#######################################
# Global variables
#######################################

# File variables
arduAttFile = open("./data/arduPilotAtt.csv", "w")
arduGlobalFile = open("./data/arduPilotGps.csv", "w")
cameraFile = open("./data/t265Pos.csv", "w")
imuFile = open("./data/imuData.csv", "w")
pfOutFile = open("./data/pfOut.csv", "w")
pfLocFile = open("./data/pfLoc.csv", "w")

# FCU connection variables

# Camera-related variables
pipe = None
pose_sensor = None
rs_cfg = None
linear_accel_cov = 0.0001
angular_vel_cov  = 0.0001

# Data variables
data = None
prev_data = None
H_aeroRef_aeroBody = None
V_aeroRef_aeroBody = None
heading_north_yaw = 0
current_global_pos = [0, 0, 0]
current_confidence_level = None
current_time_us = 0
home_msg_received = False
drone_ready = False
realsense_resetting = False
pfStatus = 0
alt_offset = 0

#######################################
#Lidar Variables
#######################################

sensor = None
flying_box = False
close_counter = 0

VL53L5_Zone_Pitch8x8 = [
        59.00,64.00,67.50,70.00,70.00,67.50,64.00,59.00,
        64.00,70.00,72.90,74.90,74.90,72.90,70.00,64.00,
        67.50,72.90,77.40,80.50,80.50,77.40,72.90,67.50,
        70.00,74.90,80.50,85.75,85.75,80.50,74.90,70.00,
        70.00,74.90,80.50,85.75,85.75,80.50,74.90,70.00,
        67.50,72.90,77.40,80.50,80.50,77.40,72.90,67.50,
        64.00,70.00,72.90,74.90,74.90,72.90,70.00,64.00,
        59.00,64.00,67.50,70.00,70.00,67.50,64.00,59.00
]
 
VL53L5_Zone_Yaw8x8 = [
        135.00,125.40,113.20, 98.13, 81.87, 66.80, 54.60, 45.00,
        144.60,135.00,120.96,101.31, 78.69, 59.04, 45.00, 35.40,
        156.80,149.04,135.00,108.45, 71.55, 45.00, 30.96, 23.20,
        171.87,168.69,161.55,135.00, 45.00, 18.45, 11.31,  8.13,
        188.13,191.31,198.45,225.00,315.00,341.55,348.69,351.87,
        203.20,210.96,225.00,251.55,288.45,315.00,329.04,336.80,
        215.40,225.00,239.04,258.69,281.31,300.96,315.00,324.60,
        225.00,234.60,246.80,261.87,278.13,293.20,305.40,315.00]

#Compute Sin and Cos
SinOfPitch = [m.sin(m.radians(i)) for i in VL53L5_Zone_Pitch8x8]
CosOfPitch = [m.cos(m.radians(i)) for i in VL53L5_Zone_Pitch8x8]
SinOfYaw = [m.sin(m.radians(i)) for i in VL53L5_Zone_Yaw8x8]
CosOfYaw = [m.cos(m.radians(i)) for i in VL53L5_Zone_Yaw8x8]


#######################################
#Lidar Helper Functions
#######################################

#Convert VL53L5CX distances to local xyz
#Optional transform parameter 

def convertDistToXYZ(ranges, statuses, transform=None):
    
    xyzArr = np.empty((0,3))

    for zoneNum in range(0,64):
        
        coorNew = np.empty((0,3))
        
        if statuses[zoneNum] != 255 and ranges[zoneNum] > 0:


            Hyp = ranges[zoneNum]/SinOfPitch[zoneNum]
            x = CosOfYaw[zoneNum]*CosOfPitch[zoneNum]*Hyp
            y = SinOfYaw[zoneNum]*CosOfPitch[zoneNum]*Hyp
            z = ranges[zoneNum]

            coor = np.array([x,y,z]) #get coordinates as a row matrix in numpy

            if transform is not None:
                coor = np.append(coor,1)
                #coor = np.transpose(coor) #for multiplication
                coorNew = np.dot(transform, coor) #mutltiply transform by coor

                coorNew = np.transpose(coorNew)[:-1] #leave off 1 
            else:
                coorNew = np.transpose(coor)

        else:
            coorNew = np.array([0,0,0])

        xyzArr = np.append(xyzArr, [coorNew], axis=0)

    return xyzArr
        



#Convert yaw, pitch and roll into rotation and xyz offsets into a 4x4 transform
def yprOffsetToTrans(yaw, pitch, roll, xOff, yOff, zOff):

    yaw *= m.pi/180
    pitch *= m.pi/180
    roll *= m.pi/180
    su = m.sin(roll)
    cu = m.cos(roll)
    sv = m.sin(pitch)
    cv = m.cos(pitch)
    sw = m.sin(yaw)
    cw = m.cos(yaw)
    #print(su,cu,sv,cv,sw,cw)
    trans = np.array([[0,0,0,0],
                      [0,0,0,0],
                      [0,0,0,0],
                      [0,0,0,1]]).astype(float)

    #Rotation
    trans[0][0] = 1
    trans[0][0] = cv*cw
    trans[0][1] = su*sv*cw - cu*sw
    trans[0][2] = su*sw + cu*sv*cw
    trans[1][0] = cv*sw
    trans[1][1] = cu*cw + su*sv*sw
    trans[1][2] = cu*sv*sw - su*cw
    trans[2][0] = -sv
    trans[2][1] = su*cv
    trans[2][2] = cu*cv  

    #Translation
    trans[0][3] = xOff
    trans[1][3] = yOff
    trans[2][3] = zOff


    return trans
#######################################
#Lidar Transforms
#######################################


#                       yaw, pitch, roll , x,y,z offsets to transforms
t_back = yprOffsetToTrans(90,0,70,      0,0,0)
t_left = yprOffsetToTrans(0,0,70,       0,0,0)
t_down = yprOffsetToTrans(0,0,0,        0,0,0)
t_forward = yprOffsetToTrans(-90,0,70,  0,0,0)
t_right = yprOffsetToTrans(180,0,70,    0,0,0)

flip = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,-1,0],
                [0,0,0,1]])

transforms = [t_back, t_left, t_forward, t_down, t_right]

flip_transforms = [np.dot(i, flip) for i in transforms]




# Increment everytime pose_jumping or relocalization happens
# See here: https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#are-there-any-t265-specific-options
# For AP, a non-zero "reset_counter" would mean that we could be sure that the user's setup was using mavlink2
reset_counter = 1

#######################################
# Parsing user' inputs
#######################################

parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('--connect', default=connection_string_default,
                    help="Vehicle connection target string. If not specified, a default string will be used.")
parser.add_argument('--baudrate', type=float, default=connection_baudrate_default,
                    help="Vehicle connection baudrate. If not specified, a default value will be used.")
parser.add_argument('--vision_position_estimate_msg_hz', type=float, default=vision_position_estimate_msg_hz_default,
                    help="Update frequency for VISION_POSITION_ESTIMATE message. If not specified, a default value will be used.")
parser.add_argument('--vision_speed_estimate_msg_hz', type=float, default=vision_speed_estimate_msg_hz_default,
                    help="Update frequency for VISION_SPEED_DELTA message. If not specified, a default value will be used.")
parser.add_argument('--scale_calib_enable', default=False, action='store_true',
                    help="Scale calibration. Only run while NOT in flight")
parser.add_argument('--camera_orientation', type=int, default=camera_orientation_default,
                    help="Configuration for camera orientation. Currently supported: forward, usb port to the right - 0; downward, usb port to the right - 1, 2: forward tilted down 45deg")
parser.add_argument('--arena_disable', action='store_true',
                    help="Disable streaming data to ARENA.")
parser.add_argument('--update_arena_hz', type=float, default=5,
                    help="Update frequency for position updates to ARENA. If not specified, a default value will be used.")
parser.add_argument('--uwb_disable', action='store_true',
                    help="Disable UWB")
parser.add_argument('--debug_enable', action='store_true',
                    help="Enable debug messages on terminal")
parser.add_argument('--flight_disable', action='store_true',
                    help="Disable Flying Boxes")

args = parser.parse_args()

connection_string = args.connect
connection_baudrate = args.baudrate
vision_position_estimate_msg_hz = args.vision_position_estimate_msg_hz
vision_speed_estimate_msg_hz = args.vision_speed_estimate_msg_hz
scale_calib_enable = args.scale_calib_enable
camera_orientation = args.camera_orientation
debug_enable = args.debug_enable
enable_arena = not args.arena_disable
update_arena_hz = args.update_arena_hz
enable_uwb = not args.uwb_disable
enable_flying = not args.flight_disable


if body_offset_enabled == 1:
    progress("INFO: Using camera position offset: Enabled, x y z is %s %s %s" % (body_offset_x, body_offset_y, body_offset_z))
else:
    progress("INFO: Using camera position offset: Disabled")

if compass_enabled == 1:
    progress("INFO: Using compass: Enabled. Heading will be aligned to north.")
else:
    progress("INFO: Using compass: Disabled")

if scale_calib_enable == True:
    progress("\nINFO: SCALE CALIBRATION PROCESS. DO NOT RUN DURING FLIGHT.\nINFO: TYPE IN NEW SCALE IN FLOATING POINT FORMAT\n")
else:
    if scale_factor == 1.0:
        progress("INFO: Using default scale factor %s" % scale_factor)
    else:
        progress("INFO: Using scale factor %s" % scale_factor)

if camera_orientation == 0:     # Forward, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)
elif camera_orientation == 1:   # Downfacing, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
elif camera_orientation == 2:   # 45degree forward
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = (tf.euler_matrix(m.pi/4, 0, 0)).dot(np.linalg.inv(H_aeroRef_T265Ref))
elif camera_orientation == 3:   # Upward facing, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.array([[0,1,0,0],[-1,0,0,0],[0,0,-1,0],[0,0,0,1]])
# elif camera_orientation == 4:   # Upward facing
#     H_aeroRef_T265Ref   = np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]])
#     H_T265body_aeroBody = np.array([[0,1,0,0],[-1,0,0,0],[0,0,-1,0],[0,0,0,1]])
else:                           # Default is facing forward, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

if not debug_enable:
    debug_enable = 0
else:
    debug_enable = 1
    np.set_printoptions(precision=4, suppress=True) # Format output on terminal 
    progress("INFO: Debug messages enabled.")

#######################################
# ARENA, Particle Filter, & UWB
#######################################

arena_attitude = [0, 0, 0]
arena_batt = 0
if enable_arena:
    scene = Scene(host="mqtt.arenaxr.org", scene="ARENA-drone", realm="realm", namespace="pnaseck")
    arena_drone = Box(object_id="arena_drone", depth=0.4, width=0.4, height=0.07, position=Position(0, 0, 0), scale=Scale(1, 1, 1), color=(128, 0, 172), material={"opacity":0.5})
    scene.add_object(arena_drone)

# device = Device(host="mqtt.arenaxr.org", device="drone02", debug=False)
# CUSTOM_TOPIC_lidar = f"{device.realm}/d/{device.namespace}/{device.device}/sensors/lidar"
# CUSTOM_TOPIC_waypoints = f"{device.realm}/d/{device.namespace}/{device.device}/waypoints"

def drone_sensor_message(client, userdata, msg):
    global flying_box

    # print(len(msg.payload))
    payload_str = msg.payload.decode("utf-8", "ignore").replace("\\", "")[1:-1]
    payload = json.loads(payload_str)
    lidar_msg = payload["msg"]["data"][0]

    dist = lidar_msg["distance_mm"]
    status = lidar_msg["target_status"]

    #print(dist)
    s = 0
    c = 0
    for j in range(len(dist)):
        if status[j] == 5:
            s += dist[j]
            c += 1
    if c != 0:
        a = s / c
        if a < 500 and flying_box:
            print("FOR THE LOVE OF GOD STOP")
            with drone_lock:
                set_mode("LOITER")
            flying_box = False

        if flying_box == False and a >= 300:
            print("OK CONTINUE")
            with drone_lock:
                set_mode("GUIDED")
            flying_box = True



    #     print("------------------------")
    #     # for i in range(len(payload["msg"]["data"]["sensors"])):
    #     i = 0
    #     sensor = payload["msg"]["data"]["sensors"][i]
    #     ranges = sensor["ranges"]
    #     statuses = sensor["statuses"]
    #     s = 0
    #     c = 0
    #     for j in range(len(ranges)):
    #       if statuses[j] == 5:
    #         s += ranges[j]
    #         c += 1

    #     if (c > 0):
    #       print(i, c, s/c)

# device.message_callback_add(CUSTOM_TOPIC_lidar, drone_sensor_message)

def device_target_location_update(client, userdata, msg):
    payload_str = msg.payload.decode("utf-8", "ignore").replace("\\", "")
    payload = json.loads(payload_str)
    pos = payload["target"]["position"]
    # yaw = msg.target.rotation.y
    print("***************************************************")
    # print("DEVICE TARGET UPDATE:", pos, yaw)
    print("DEVICE TARGET UPDATE:", pos)
    if (pos["x"] == 0 and pos["y"] == 0 and pos["z"] == 0):
        land()
    else:
        # set_target(pos.x, pos.z, -pos.y, yaw=yaw)
        set_target(pos["x"], pos["z"], -pos["y"])

# device.message_callback_add(CUSTOM_TOPIC_waypoints, device_target_location_update)

landed = False
def update_arena_pos():
    global arena_drone, arena_attitude, arena_batt, pfStatus, landed
    # with pf_lock:
    #     if pfStatus < 3:
    #         return
    #     pf_data = pf.getTagLoc()
    # print("ARENA", pf_data[2], pf_data[4], -pf_data[3], pf_data[5])
    # pfOutFile.write(f"{str(time.time())},{str(pf_data[1])},{str(pf_data[2])},{str(pf_data[3])},{str(pf_data[4])},{str(pf_data[5])}\n")
    if not landed:
        arena_drone.update_attributes(position=Position(current_global_pos[0], current_global_pos[2], current_global_pos[1]), rotation=Rotation(arena_attitude[0], arena_attitude[1], arena_attitude[2]))#, battery=Attribute(battery=arena_batt))
        scene.update_object(arena_drone)

uwb_locations = {}

def arena_uwb_location_update(tag):
    global uwb_locations
    pos = tag.data.position
    uwb_locations[int(tag.data.armarker.markerid)] = [pos.x, pos.y, pos.z]

def arena_get_uwb_locations():
    global uwb_locations
    print("GETTING UWB LOCATIONS FROM ARENA")
    for i in scene.all_objects:
        if ('armarker' in scene.all_objects[i].data and scene.all_objects[i].data.armarker.markertype == "uwb"):
            scene.all_objects[i].update_handler = arena_uwb_location_update
            arena_uwb_location_update(scene.all_objects[i])
    print(uwb_locations)

def arena_target_location_update(target):
    pos = target.data.position
    yaw = target.data.rotation.y
    print("***************************************************")
    print("TARGET UPDATE:", pos, yaw)
    if (pos.x == 0 and pos.y == 0 and pos.z == 0):
        land()
    else:
        set_target(pos.x, pos.z, -pos.y, yaw=yaw)

def arena_get_target_location():
    if "drone_target" in scene.all_objects and scene.all_objects["drone_target"]:
        scene.all_objects["drone_target"].update_handler = arena_target_location_update
    else:
        print("No target found, waypoint flying disabled")

if enable_uwb:
    pf_lock = threading.Lock()
    pf = ParticleFilterLoc()
    uwb = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)

    uwb.write(b"AT+RATE 10\n")
    # if not enable_arena:
    if True:
        uwb_locations = {
            7: [1.988, 1.749, 2.608],
            8: [2.966, 0.799, -1.851],
            9: [-1.66, 0.168, 2.005],
            10: [-2.78, 2.804, -2.076],
            11: [1.494, 2.052, -3.628],
            13: [-0.324, 3.137, -0.44]
        }


####################################
# Functions - MAVLink
####################################


def uwb_loop():
    global uwb, uwb_locations, threads_should_exit, pfStatus
    #First line is usually unreadable
    uwb.readline()
    while not threads_should_exit:
        line = uwb.readline()
        if (len(line) > 5 and line[0] != ord('#')):
            line = line[0:-2].replace(b' ', b'').decode('utf-8')
            #ID, RANGE, RSSI, TIMESTAMP
            curData = line.split(',')
            if (len(curData) < 4):
                continue
            try:
                int(curData[0])
            except ValueError:
                continue
            if int(curData[0]) in uwb_locations.keys():
                x, y, z = uwb_locations[int(curData[0])]
                with pf_lock:
                    try:
                        # print("RANGE", np.float32(x), np.float32(-z), np.float32(y), np.float32(float(curData[1])))
                        pfLocFile.write(f"UWB,{time.time()},{int(curData[0])},{str(x)},{str(-z)},{str(y)},{str(float(curData[1]))},0.1\n")
                        pf.depositRange(np.float32(x), np.float32(-z), np.float32(y), np.float32(float(curData[1])), 0.1)
                        pfStatus |= 1
                    except:
                        pass

def mavlink_loop():
    '''a main routine for a thread; reads data from a mavlink connection,
    calling callbacks based on message type received.
    '''
    global conn, mavlink_callbacks, threads_should_exit
    print("Mavlink")
    interesting_messages = list(mavlink_callbacks.keys())
    while not threads_should_exit:
        m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if m is None:
            continue
        # print(m.get_type())
        mavlink_callbacks[m.get_type()](m)

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_estimate_message():
    global current_time_us, H_aeroRef_aeroBody, reset_counter, current_confidence_level, realsense_resetting
    with drone_lock:
        if H_aeroRef_aeroBody is not None:
            cur_pos = [H_aeroRef_aeroBody[0][3], H_aeroRef_aeroBody[1][3], H_aeroRef_aeroBody[2][3]]

            # Setup angle data
            rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))

            rpy_rad[0] = m.pi - rpy_rad[0]
            if rpy_rad[0] > m.pi: rpy_rad[0] -= 2*m.pi

            # Setup covariance data, which is the upper right triangle of the covariance matrix, see here: https://files.gitter.im/ArduPilot/VisionProjects/1DpU/image.png
            # Attemp #01: following this formula https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/src/base_realsense_node.cpp#L1406-L1411
            cov_pose    = linear_accel_cov * pow(10, 3 - int(3*current_confidence_level/100))
            cov_twist   = angular_vel_cov  * pow(10, 1 - int(3*current_confidence_level/100))

            if realsense_resetting:
                cov_pose *= 100
                cov_twist *= 100
            
            covariance  = np.array([cov_pose, 0, 0, 0, 0, 0,
                                       cov_pose, 0, 0, 0, 0,
                                          cov_pose, 0, 0, 0,
                                            cov_twist, 0, 0,
                                               cov_twist, 0,
                                                  cov_twist])

            # print(f"Sending: {cur_pos[0]}, {cur_pos[1]}, {cur_pos[2]}")
            # Send the message
            conn.mav.vision_position_estimate_send(
                current_time_us,            # us Timestamp (UNIX time or time since system boot)
                cur_pos[0],   # Global X position
                cur_pos[1],   # Global Y position
                cur_pos[2],   # Global Z position
                rpy_rad[0],                 # Roll angle
                rpy_rad[1],                 # Pitch angle
                rpy_rad[2],                 # Yaw angle
                covariance,                 # Row-major representation of pose 6x6 cross-covariance matrix
                reset_counter               # Estimate reset counter. Increment every time pose estimate jumps.
            )

            s = str(time.time()) + "," + str(cur_pos[0]) + "," + str(cur_pos[1]) + "," + str(cur_pos[2]) + "," + \
                        str(rpy_rad[0]) + "," + str(rpy_rad[1]) + "," + str(rpy_rad[2]) + "\n"
            cameraFile.write(s)

# https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
def send_vision_speed_estimate_message():
    global current_time_us, V_aeroRef_aeroBody, reset_counter, current_confidence_level
    with drone_lock:
        if V_aeroRef_aeroBody is not None:

            # Attemp #01: following this formula https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/src/base_realsense_node.cpp#L1406-L1411
            cov_pose    = linear_accel_cov * pow(10, 3 - int(3*current_confidence_level/100))
            covariance  = np.array([cov_pose,   0,          0,
                                    0,          cov_pose,   0,
                                    0,          0,          cov_pose])
            
            # Send the message
            conn.mav.vision_speed_estimate_send(
                current_time_us,            # us Timestamp (UNIX time or time since system boot)
                V_aeroRef_aeroBody[0][3],   # Global X speed
                V_aeroRef_aeroBody[1][3],   # Global Y speed
                V_aeroRef_aeroBody[2][3],   # Global Z speed
                covariance,                 # covariance
                reset_counter               # Estimate reset counter. Increment every time pose estimate jumps.
            )


# Send a mavlink SET_GPS_GLOBAL_ORIGIN message (http://mavlink.org/messages/common#SET_GPS_GLOBAL_ORIGIN), which allows us to use local position information without a GPS.
def set_default_global_origin():
    with drone_lock:
        print("SENDING GLOBAL ORIGIN")
        conn.mav.set_gps_global_origin_send(
            1,
            home_lat, 
            home_lon,
            home_alt
        )

# Send a mavlink SET_HOME_POSITION message (http://mavlink.org/messages/common#SET_HOME_POSITION), which allows us to use local position information without a GPS.
def set_default_home_position():
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    with drone_lock:
        conn.mav.set_home_position_send(
            1,
            home_lat, 
            home_lon,
            home_alt,
            x,
            y,
            z,
            q,
            approach_x,
            approach_y,
            approach_z
        )

def set_mode(mode):
    mode_map = conn.mode_mapping()
    if mode in mode_map:
        mode = mode_map.get(mode)
    else:
        print("MODE NOT FOUND")
        return
    conn.mav.command_long_send(
        1,
        1,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        1,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode,
        0,
        0,
        0,
        0,
        0
    )


def set_target(north, east, down, yaw=0, timeout=None):
    alt = (-down)
    with drone_lock:
        conn.mav.set_position_target_local_ned_send(
              1,
              1,
              0,
              mavutil.mavlink.MAV_FRAME_LOCAL_NED,
              0b0000111111111000,
              north, east, down,
              0,0,0,
              0,0,0,
              0,0)
        # conn.mav.mission_item_int_send(
        #     1,
        #     1,
        #     0,
        #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        #     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        #     2,
        #     1,
        #     0,
        #     0,
        #     0,
        #     yaw,
        #     int(north*METER_TO_LATLNG),
        #     int(east*METER_TO_LATLNG),
        #     alt
        # )

    thresh = 0.3
    def condition():

        g_n = current_global_pos[0]
        g_e = current_global_pos[1]
        g_d = current_global_pos[2]
        # print(g_n, g_e, g_d)

        l2 = m.sqrt((g_n - north)**2 + (g_e - east) ** 2 + (g_d + down) ** 2)
        # print(l2)
        return l2 < thresh
        # return abs(current_global_pos[0] - north) < thresh and \
        #         abs(current_global_pos[1] - east) < thresh and \
        #         abs(current_global_pos[2] + down) < thresh # down is plus since it's negative altitude
    wait_for(condition, timeout=timeout)
        
#speed_type:
#   0 is airspeed
#   1 is groundspeed
#   2 is climbspeed
#   3 is descentspeed
def set_drone_speed(speed_type, speed):
    conn.mav.command_long_send(
        1,
        1,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        speed_type,
        speed,
        -1,
        0,
        0,
        0,
        0
    )

def takeoff(alt):
    set_mode("LOITER")
    conn.arducopter_arm()
    time.sleep(0.5)
    conn.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, alt)
    def check_alt():
        return current_global_pos[2] > alt - 0.2
    wait_for(check_alt)
    set_mode("GUIDED")
    

def land():
    global arena_drone, landed
    set_mode("LAND")
    def check_alt():
        return current_global_pos[2] < 0.3
    wait_for(check_alt)
    time.sleep(1)
    landed = True
    if enable_arena:
        scene.delete_object(arena_drone)
    conn.arducopter_disarm()
    
def wait_for(condition, interval=0.1, timeout=None):
    startTime = time.time()
    while True:
        if main_loop_should_quit:
            return
        if condition():
            return
        if timeout != None and (time.time() - startTime > timeout):
            return
        time.sleep(0.1)

def reboot():
    conn.mav.command_long_send(
        1,
        1,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0
    )

# Listen to attitude data to acquire heading when compass data is enabled
def att_msg_callback(value):
    global heading_north_yaw, arena_attitude
    arduAttFile.write(str(time.time()) + "," + str(value.roll) + "," + str(value.pitch) + "," + str(value.yaw) + "\n")
    # print(value)
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        progress("INFO: Received first ATTITUDE message with heading yaw %.2f degrees" % m.degrees(heading_north_yaw))
    if enable_arena:
        arena_attitude[0] = value.roll*RAD_TO_DEG
        arena_attitude[1] = -value.yaw*RAD_TO_DEG
        arena_attitude[2] = value.pitch*RAD_TO_DEG

def local_pos_callback(value):
    print(value.x,value.y,value.z)

def global_msg_callback(value):
    global H_aeroRef_aeroBody, current_global_pos
    current_global_pos = [value.lat / METER_TO_LATLNG, value.lon / METER_TO_LATLNG, value.alt / 1000]
    arduGlobalFile.write(str(time.time()) + "," + str(value.lat / METER_TO_LATLNG) + "," + str(value.lon / METER_TO_LATLNG) + "," + str(-value.alt / 1000) + "\n")
    # with pf_lock:
    #     pf.depositVio(np.float64(time.time()), np.float32(value.lat / METER_TO_LATLNG), np.float32(-value.lon / METER_TO_LATLNG), np.float32(value.alt / 1000), 0)
    
    # print(f"GLOBAL: {str(value.lat / METER_TO_LATLNG)}, {str(value.lon / METER_TO_LATLNG)}, {str(-value.alt / 1000)}")
    
    #vx, vy, vz, hdg

def local_msg_callback(value):
    print(value)

def home_msg_callback(value):
    global home_msg_received
    print(value)
    if not home_msg_received:
        alt_offset = value.altitude / 1000
        print("ALT_OFFSET", alt_offset)
        home_msg_received = True

def imu_msg_callback(value):
    # print(value)
    imuFile.write(f"{time.time()},{value.xacc},{value.yacc},{value.zacc},{value.xgyro},{value.ygyro},{value.zgyro},{value.xmag},{value.ymag},{value.zmag}\n")

def batt_msg_callback(value):
    global arena_batt
    #print(value.voltages[0])
    arena_batt = value.voltages[0] / 4500

def mode_msg_callback(value):
    print(value)

#######################################
# Functions - T265
#######################################

def increment_reset_counter():
    global reset_counter
    if reset_counter >= 255:
        reset_counter = 1
    reset_counter += 1

# List of notification events: https://github.com/IntelRealSense/librealsense/blob/development/include/librealsense2/h/rs_types.h
# List of notification API: https://github.com/IntelRealSense/librealsense/blob/development/common/notifications.cpp
def realsense_notification_callback(notif):
    progress("INFO: T265 event: " + notif)
    if notif.get_category() is rs.notification_category.pose_relocalization:
        increment_reset_counter()

def realsense_connect():
    global pipe, pose_sensor, rs_cfg

    if (debug_enable):
        rs.log_to_console(rs.log_severity.info)
    
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Build config object before requesting data
    rs_cfg = rs.config()
    ctx = rs.context()

    # Enable the stream we are interested in
    rs_cfg.enable_stream(rs.stream.pose) # Positional data

    # Configure callback for relocalization event
    device = rs_cfg.resolve(pipe).get_device()
    pose_sensor = device.first_pose_sensor()
    pose_sensor.set_option(rs.option.enable_pose_jumping, 0)
    pose_sensor.set_option(rs.option.enable_relocalization, 0)
    pose_sensor.set_option(rs.option.enable_map_preservation, 0)
    pose_sensor.set_option(rs.option.enable_mapping, 0)

    # Hard reset the sensor in an attempt to combat disconnects
    # pose_sensor.hardware_reset()

    pose_sensor.set_notifications_callback(realsense_notification_callback)

    # Start streaming with requested config
    pipe.start(rs_cfg)

#######################################
# Functions - Miscellaneous
#######################################

# Monitor user input from the terminal and perform action accordingly
def user_input_monitor():
    global scale_factor, threads_should_exit, enable_auto_set_ekf_home, home_msg_received, drone_ready, conn
    home_msg_received = False
    while not threads_should_exit and not home_msg_received:
        print('Set EKF home with default GPS location')
        set_default_global_origin()
        set_default_home_position()
        time.sleep(5) # Wait a short while for FCU to start working
    print("DRONE READY")
    drone_ready = True

    while not threads_should_exit:
        # Add new action here according to the key pressed.
        # Enter: Set EKF home when user press enter
        try:
            c = input()
            if c == "":
                print('Set EKF home with default GPS location')
                set_default_global_origin()
                set_default_home_position()
            else:
                print("Got keyboard input %s" % c)
        except IOError: pass


#######################################
# Main code starts here
#######################################

try:
    progress("INFO: pyrealsense2 version: %s" % str(rs.__version__))
except Exception:
    # fail silently
    pass

progress("INFO: Starting Vehicle communications")
conn = mavutil.mavlink_connection(
    connection_string,
    autoreconnect = True,
    source_system = 1,
    source_component = 93,
    baud=connection_baudrate,
    force_connected=True,
)

mavlink_callbacks = {
    'ATTITUDE': att_msg_callback,
    'GLOBAL_POSITION_INT': global_msg_callback,
    'MODE':mode_msg_callback,
    'HOME_POSITION':home_msg_callback,
    'RAW_IMU':imu_msg_callback,
    'BATTERY_STATUS':batt_msg_callback,
    'LOCAL_POSITION_NED':local_pos_callback
}

# connecting and configuring the camera is a little hit-and-miss.
# Start a timer and rely on a restart of the script to get it working.
# Configuring the camera appears to block all threads, so we can't do
# this internally.

signal.setitimer(signal.ITIMER_REAL, 5)  # seconds...

print('Connecting to camera...')
realsense_connect()
print('Camera connected.')

signal.setitimer(signal.ITIMER_REAL, 0)  # cancel alarm

# Try getting a frame:
signal.setitimer(signal.ITIMER_REAL, 5)  # seconds...
pipe.try_wait_for_frames(3000)
print('T265 sending frames.')
signal.setitimer(signal.ITIMER_REAL, 0)  # cancel alarm


#########################
# Signal handling
#########################

# gracefully terminate the script if an interrupt signal (e.g. ctrl-c)
# is received.  This is considered to be abnormal termination.
main_loop_should_quit = False
def sigint_handler(sig, frame):
    global main_loop_should_quit, threads_should_exit
    main_loop_should_quit = True
    threads_should_exit = True
    if (enable_arena):
        time.sleep(1)
        scene.stop_tasks()
signal.signal(signal.SIGINT, sigint_handler)

# gracefully terminate the script if a terminate signal is received
# (e.g. kill -TERM).  
def sigterm_handler(sig, frame):
    global main_loop_should_quit, threads_should_exit
    main_loop_should_quit = True
    threads_should_exit = True
    global exit_code
    exit_code = 0
    if (enable_arena):
        time.sleep(1)
        scene.stop_tasks()

signal.signal(signal.SIGTERM, sigterm_handler)


##################################
# MAIN EVENT LOOP
##################################

def main():
    print("STARTING MAIN THREAD")
    global main_loop_should_quit, pipe, H_aeroRef_aeroBody, H_aeroRef_T265Ref, H_T265body_aeroBody, V_aeroRef_aeroBody, current_confidence_level, heading_north_yaw, prev_data, pfStatus, reset_counter, realsense_resetting, current_time_us
    if compass_enabled == 1:
        time.sleep(1) # Wait a short while for yaw to be correctly initiated
    try:
        while not main_loop_should_quit:
            # Wait for the next set of frames from the camera
            status, frames = pipe.try_wait_for_frames(500)
            if not status: 
                print("--------------------------")
                print("    REALSENSE DATA BAD    ")
                print("--------------------------")
                set_mode("STABILIZE")
                pipe.stop()
                increment_reset_counter()
                realsense_resetting = True
                time.sleep(1)
                signal.setitimer(signal.ITIMER_REAL, 5)  # 5 second timeout
                realsense_connect()
                signal.setitimer(signal.ITIMER_REAL, 0)  # cancel
                realsense_resetting = False
                set_mode("GUIDED")
                continue

            # Fetch pose frame
            pose = frames.get_pose_frame()

            # Process data
            if pose:
                with drone_lock:
                    # Store the timestamp for MAVLink messages
                    current_time_us = int(round(time.time() * 1000000))

                    # Pose data consists of translation and rotation
                    data = pose.get_pose_data()
                    # Confidence level value from T265: 0-3, remapped to 0 - 100: 0% - Failed / 33.3% - Low / 66.6% - Medium / 100% - High  
                    current_confidence_level = float(data.tracker_confidence * 100 / 3)  

                    # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
                    H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z]) 
                    H_T265Ref_T265body[0][3] = data.translation.x * scale_factor
                    H_T265Ref_T265body[1][3] = data.translation.y * scale_factor
                    H_T265Ref_T265body[2][3] = data.translation.z * scale_factor

                    # Transform to aeronautic coordinates (body AND reference frame!)
                    H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody))

                    # Calculate GLOBAL XYZ speed (speed from T265 is already GLOBAL)
                    V_aeroRef_aeroBody = tf.quaternion_matrix([1,0,0,0])
                    V_aeroRef_aeroBody[0][3] = data.velocity.x
                    V_aeroRef_aeroBody[1][3] = data.velocity.y
                    V_aeroRef_aeroBody[2][3] = data.velocity.z
                    V_aeroRef_aeroBody = H_aeroRef_T265Ref.dot(V_aeroRef_aeroBody)

                    # Check for pose jump and increment reset_counter
                    if prev_data != None:
                        delta_translation = [data.translation.x - prev_data.translation.x, data.translation.y - prev_data.translation.y, data.translation.z - prev_data.translation.z]
                        delta_velocity = [data.velocity.x - prev_data.velocity.x, data.velocity.y - prev_data.velocity.y, data.velocity.z - prev_data.velocity.z]
                        position_displacement = np.linalg.norm(delta_translation)
                        speed_delta = np.linalg.norm(delta_velocity)

                        # Pose jump is indicated when position changes abruptly. The behavior is not well documented yet (as of librealsense 2.34.0)
                        jump_threshold = 0.1 # in meters, from trials and errors, should be relative to how frequent is the position data obtained (200Hz for the T265)
                        jump_speed_threshold = 20.0 # in m/s from trials and errors, should be relative to how frequent is the velocity data obtained (200Hz for the T265)
                        if (position_displacement > jump_threshold) or (speed_delta > jump_speed_threshold):
                            print('VISO jump detected')
                            if position_displacement > jump_threshold:
                                print("Position jumped by: %s" % position_displacement)
                            elif speed_delta > jump_speed_threshold:
                                print("Speed jumped by: %s" % speed_delta)
                            increment_reset_counter()
                        
                    prev_data = data

                    # Take offsets from body's center of gravity (or IMU) to camera's origin into account
                    if body_offset_enabled == 1:
                        H_body_camera = tf.euler_matrix(0, 0, 0, 'sxyz')
                        H_body_camera[0][3] = body_offset_x
                        H_body_camera[1][3] = body_offset_y
                        H_body_camera[2][3] = body_offset_z
                        H_camera_body = np.linalg.inv(H_body_camera)
                        H_aeroRef_aeroBody = H_body_camera.dot(H_aeroRef_aeroBody.dot(H_camera_body))

                    # Realign heading to face north using initial compass data
                    if compass_enabled == 1:
                        H_aeroRef_aeroBody = H_aeroRef_aeroBody.dot( tf.euler_matrix(0, 0, heading_north_yaw, 'sxyz'))

                    with pf_lock:
                        # print("VIO: ", np.float32(H_aeroRef_aeroBody[0][3]), np.float32(-H_aeroRef_aeroBody[1][3]), np.float32(-H_aeroRef_aeroBody[2][3]))
                        pfLocFile.write(f"VIO,{str(time.time())},{str(-H_aeroRef_aeroBody[0][3])},{str(H_aeroRef_aeroBody[1][3])},{str(-H_aeroRef_aeroBody[2][3])},0\n")
                        pf.depositVio(np.float64(time.time()), np.float32(-H_aeroRef_aeroBody[0][3]), np.float32(H_aeroRef_aeroBody[1][3]), np.float32(-H_aeroRef_aeroBody[2][3]), 0)
                        pfStatus |= 2

    except Exception as e:
        progress(e)

    except:
        print('ERROR IN SCRIPT')  
        print("Unexpected error: %s" % sys.exc_info()[0])

    finally:
        print('Closing the script...')
        # start a timer in case stopping everything nicely doesn't work.
        signal.setitimer(signal.ITIMER_REAL, 5)  # seconds...
        pipe.stop()

#################################
# Scheduler
#################################
if enable_arena:
    # scene.run_once(arena_get_uwb_locations)
    scene.run_once(arena_get_target_location)
    scene.run_forever(update_arena_pos, 1000/update_arena_hz)

mavlink_thread = threading.Thread(target=mavlink_loop)
mavlink_thread.start()

if enable_uwb:
    print("STARTING UWB")
    uwb_thread = threading.Thread(target=uwb_loop)
    uwb_thread.start()


# Send MAVlink messages in the background at pre-determined frequencies
sched = BackgroundScheduler()
if enable_msg_vision_position_estimate and vision_position_estimate_msg_hz > 0:
    print("SCHEDULING POSITION EVERY: " + str(1/vision_position_estimate_msg_hz))
    sched.add_job(send_vision_position_estimate_message, 'interval', seconds = 1/vision_position_estimate_msg_hz)

if enable_msg_vision_speed_estimate and vision_speed_estimate_msg_hz > 0:
    sched.add_job(send_vision_speed_estimate_message, 'interval', seconds = 1/vision_speed_estimate_msg_hz)

# A separate thread to monitor user input
if enable_user_keyboard_input:
    user_keyboard_input_thread = threading.Thread(target=user_input_monitor)
    user_keyboard_input_thread.daemon = True
    user_keyboard_input_thread.start()
    print("INFO: Press Enter to set EKF home at default location")

main_thread = threading.Thread(target=main)
main_thread.daemon = True
main_thread.start()

sched.start()

def wait_drone_ready():
    def drone_is_ready():
        return drone_ready
    return wait_for(drone_is_ready, 1)

def no_box():
    global flying_box
    print("Start no box")

    while not main_loop_should_quit:
        if main_loop_should_quit:
            return

def fly_box():
    global flying_box
    print("Start")
    flying_box = True
    def is_stopped():
        global flying_box
        return flying_box

    while not main_loop_should_quit:
        set_target(0, 0, -1.5, 0)
        #wait_for(is_stopped)
        time.sleep(3)
        if main_loop_should_quit:
            return

        set_target(1.5, 0, -1.5, 0)
        #wait_for(is_stopped)
        time.sleep(3)
        if main_loop_should_quit:
            return

        set_target(1.5, -1.5, -1.5, 0)
        #wait_for(is_stopped)
        time.sleep(3)
        if main_loop_should_quit:
            return

        set_target(0, -1.5, -1.5, 0)
        #wait_for(is_stopped)
        time.sleep(3)
        if main_loop_should_quit:
            return

#Main blocking portion

if enable_flying:
    set_mode("GUIDED")
    print("START1")
    wait_drone_ready()
    print("START")
    time.sleep(1)

    set_mode("GUIDED")
    set_drone_speed(0, 32)
    set_drone_speed(1, 32)
    set_drone_speed(2, 32)
    set_drone_speed(3, 32)
    time.sleep(1)
    print("TAKEOFF")
    takeoff(1.5)

    # box_thread = threading.Thread(target=fly_box)
    # box_thread.start()
    box_thread = threading.Thread(target=no_box)
    box_thread.start()

if enable_arena:
    scene.run_tasks()
else:
    device.run_tasks()
    # while not main_loop_should_quit:
    #     time.sleep(0.5)

main_loop_should_quit = True
threads_should_exit = True

#Cleanup
sched.shutdown()
mavlink_thread.join()
if enable_uwb:
    uwb_thread.join()
    uwb.close()

if enable_arena:
    scene.disconnect()
conn.close()
cameraFile.close()
arduGlobalFile.close()
arduAttFile.close()
imuFile.close()
pfOutFile.close()
pfLocFile.close()

print("INFO: Realsense pipeline and vehicle object closed.")
sys.exit(exit_code)
