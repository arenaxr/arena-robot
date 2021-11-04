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

from time import sleep
from apscheduler.schedulers.background import BackgroundScheduler
from pymavlink import mavutil

from particlefilter import ParticleFilterLoc
from arena import *

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
connection_timeout_sec_default = 5

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
home_lat = 0    # Somewhere random
home_lon = 0     # Somewhere random
home_alt = 0       # Somewhere random

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
    scene = Scene(host="arenaxr.org", scene="ARENA-Drone", realm="realm", namespace="astrasse")
    arena_drone = Box(object_id="arena_drone", depth=0.4, width=0.4, height=0.07, position=Position(0, 0, 0), scale=Scale(1, 1, 1), color=(128, 0, 172), material={"opacity":0.5})
    scene.add_object(arena_drone)

def update_arena_pos():
    global arena_drone, arena_attitude, arena_batt, pfStatus
    with pf_lock:
        if pfStatus < 3:
            return
        pf_data = pf.getTagLoc()
    print("ARENA", pf_data[2], pf_data[4], -pf_data[3], pf_data[5])
    pfOutFile.write(f"{str(pf_data[1])},{str(pf_data[2])},{str(pf_data[3])},{str(pf_data[4])},{str(pf_data[5])}, \n")
    arena_drone.update_attributes(position=Position(pf_data[2], pf_data[4], -pf_data[3]), rotation=Rotation(arena_attitude[0], arena_attitude[1]+pf_data[5]*RAD_TO_DEG, arena_attitude[2]))#, battery=Attribute(battery=arena_batt))
    scene.update_object(arena_drone)

uwb_locations = {}

def arena_uwb_location_update(tag):
    global uwb_locations
    pos = tag.data.position
    uwb_locations[int(tag.data.armarker.markerid)] = [pos.x, pos.y, pos.z]

def arena_get_uwb_locations():
    global uwb_locations
    print("Get loc")
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
    target = scene.all_objects["drone_target"]
    if target:
        target.update_handler = arena_target_location_update
    else:
        print("No target found, waypoint flying disabled")

if enable_uwb:
    pf_lock = threading.Lock()
    pf = ParticleFilterLoc()
    uwb = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)

    uwb.write(b"AT+RATE 10\n")
    if not enable_arena:
        uwb_locations = {
            7: [1.828, 1.749, 2.558],
            8: [2.806, 0.799, -1.901],
            9: [-1.82, 0.168, 1.955],
            10: [-2.94, 2.804, -2.126],
            11: [1.334, 2.052, -3.678]
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
                        pfLocFile.write(f"UWB,{str(x)},{str(-z)},{str(y)},{str(float(curData[1]))},0.1\n")
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
        mavlink_callbacks[m.get_type()](m)

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_estimate_message():
    global current_time_us, H_aeroRef_aeroBody, reset_counter, current_confidence_level
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


def set_target(north, east, down, yaw=0):
    alt = (-down) - alt_offset
    with drone_lock:
        conn.mav.mission_item_int_send(
            1,
            1,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,
            1,
            0,
            0,
            0,
            yaw,
            int(north*METER_TO_LATLNG),
            int(east*METER_TO_LATLNG),
            alt
        )

    thresh = 0.1
    def condition():
        return abs(current_global_pos[0] - north) < thresh and \
                abs(current_global_pos[1] - east) < thresh and \
                abs(current_global_pos[2] + down) < thresh # down is plus since it's negative altitude
    wait_for(condition)
        
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
    set_mode("GUIDED")
    conn.arducopter_arm()
    time.sleep(0.5)
    conn.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, alt)
    def check_alt():
        return current_global_pos[2] > alt - 0.2
    wait_for(check_alt)
    

def land():
    set_mode("LAND")
    def check_alt():
        return current_global_pos[2] < 0.3
    wait_for(check_alt)
    time.sleep(1)
    conn.arducopter_disarm()
    
def wait_for(condition, interval=0.1):
    while True:
        if main_loop_should_quit:
            return
        if condition():
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
    print(value)
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        progress("INFO: Received first ATTITUDE message with heading yaw %.2f degrees" % m.degrees(heading_north_yaw))
    if enable_arena:
        arena_attitude[0] = value.roll*RAD_TO_DEG
        arena_attitude[1] = -value.yaw*RAD_TO_DEG
        arena_attitude[2] = value.pitch*RAD_TO_DEG

def global_msg_callback(value):
    global H_aeroRef_aeroBody, current_global_pos
    current_global_pos = [value.lat / METER_TO_LATLNG, value.lon / METER_TO_LATLNG, value.alt / 1000]
    arduGlobalFile.write(str(time.time()) + "," + str(value.lat / METER_TO_LATLNG) + "," + str(value.lon / METER_TO_LATLNG) + "," + str(-value.alt / 1000) + "\n")
    # with pf_lock:
    #     pf.depositVio(np.float64(time.time()), np.float32(value.lat / METER_TO_LATLNG), np.float32(-value.lon / METER_TO_LATLNG), np.float32(value.alt / 1000), 0)
    print(f"GLOBAL: {str(value.lat / METER_TO_LATLNG)}, {str(value.lon / METER_TO_LATLNG)}, {str(-value.alt / 1000)}")
    #vx, vy, vz, hdg

def home_msg_callback(value):
    global home_msg_received
    print(value)
    alt_offset = value.altitude / 1000
    print("ALT_OFFSET", alt_offset)
    home_msg_received = True

def imu_msg_callback(value):
    imuFile.write(f"{value.xacc},{value.yacc},{value.xgyro},{value.ygyro},{value.zgyro}\n")
    pass

def batt_msg_callback(value):
    global arena_batt
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
    'BATTERY_STATUS':batt_msg_callback
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
print('Camera sending frames.')
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
    global main_loop_should_quit, pipe, H_aeroRef_aeroBody, H_aeroRef_T265Ref, H_T265body_aeroBody, V_aeroRef_aeroBody, current_confidence_level, heading_north_yaw, prev_data, pfStatus, reset_counter
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
                signal.setitimer(signal.ITIMER_REAL, 5)  # 5 second timeout
                realsense_connect()
                signal.setitimer(signal.ITIMER_REAL, 0)  # cancel
                relasense_resetting = False
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
                        pfLocFile.write(f"VIO,{str(time.time())}, {str(H_aeroRef_aeroBody[0][3])}, {str(-H_aeroRef_aeroBody[1][3])}, {str(-H_aeroRef_aeroBody[2][3])}, 0\n")
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
    scene.run_once(arena_get_uwb_locations)
    scene.run_once(arena_get_target_location)
    scene.run_forever(update_arena_pos, 1000/update_arena_hz)

mavlink_thread = threading.Thread(target=mavlink_loop)
mavlink_thread.start()

if enable_uwb:
    uwb_thread = threading.Thread(target=uwb_loop)
    uwb_thread.start()

# Send MAVlink messages in the background at pre-determined frequencies
sched = BackgroundScheduler()
if enable_msg_vision_position_estimate and vision_position_estimate_msg_hz > 0:
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

def fly_box():
    print("Start")
    while not main_loop_should_quit:
        set_target(0, 0, -2, 0)
        time.sleep(1)
        if main_loop_should_quit:
            return

        set_target(2, 0, -2, 0)
        time.sleep(1)
        if main_loop_should_quit:
            return

        set_target(2, -2, -2, 0)
        time.sleep(1)
        if main_loop_should_quit:
            return

        set_target(0, -2, -2, 0)
        time.sleep(1)
        if main_loop_should_quit:
            return

#Main blocking portion
if enable_arena:
    wait_drone_ready()
    set_drone_speed(0, 32)
    set_drone_speed(1, 32)
    set_drone_speed(2, 32)
    set_drone_speed(3, 32)
    time.sleep(1)
    takeoff(2)

    box_thread = threading.Thread(target=fly_box)
    box_thread.start()

    scene.run_tasks()
    main_loop_should_quit = True
    threads_should_exit = True
else:
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
    takeoff(2)

    box_thread = threading.Thread(target=fly_box)
    box_thread.start()
    
    while not main_loop_should_quit:
        time.sleep(0.5)

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