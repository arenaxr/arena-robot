#!/usr/bin/env python3
from enum import Enum, auto
from arena import *

scene = Scene(host="arenaxr.org", scene="ARENA-drone", realm="realm", namespace="pnaseck")

HEIGHT = 1.5

@scene.run_once
def arena_init():
    global drone_target, pushpin, drone_target_ind

    drone_target = Box(
        object_id="drone_target",
        position=(0,HEIGHT,0),
        scale=(0.1,0.1,0.1),
        color=(204,0,0),
        clickable=True,
        persist=True
    )

    pushpin = Cylinder(
        object_id="pushpin",
        position=(999, 999, 999),
        rotation=(0, 0, 0),
        scale=(.1, .02, .1),
        color=(0, 120, 0),
        persist=True,
        clickable=True,
        evt_handler=click_handler
    )

    drone_target_ind = Cylinder(
        object_id="drone_target_ind",
        position=(999, 999, 999),
        rotation=(0, 0, 0),
        scale=(.1, HEIGHT*0.95, .1),
        material=Material(color="#007000", transparent=True, opacity=0.5),
        persist=True
    )

    origin = Box(
        object_id="origin",
        position=(0, 0, 0),
        scale=(.1, .1, .1),
        color=(120, 120, 120),
        persist=True,
        clickable=True,
        evt_handler=click_handler
    )

    scene.add_object(drone_target)
    scene.add_object(drone_target_ind)
    scene.add_object(pushpin)
    scene.add_object(origin)

    scene.on_msg_callback = on_message


done = False

def click_handler(scene, evt, msg):
    global done
    if done: return

    if evt.type == "mousedown":
        pos = pushpin.data.position

        if abs(pos["x"]) < 0.4 and abs(pos["z"]) < 0.4:
            wp = (0, 0, 0)
            drone_target_ind.update_attributes(position=Position(999, 999, 999), persist=True)
            scene.update_object(drone_target_ind)
            done = True
            pushpin.data.position = Position(999, 999, 999)
            scene.update_object(pushpin)
        else:
            wp = (pos["x"], HEIGHT, pos["z"])
            drone_target_ind.update_attributes(position=Position(wp[0], HEIGHT*0.5, wp[2]), persist=True)
            scene.update_object(drone_target_ind)

        drone_target.update_attributes(position=Position(*wp), persist=True)
        scene.update_object(drone_target)

def quaternion_mult(q,r):
    return [r[0]*q[0]-r[1]*q[1]-r[2]*q[2]-r[3]*q[3],
            r[0]*q[1]+r[1]*q[0]-r[2]*q[3]+r[3]*q[2],
            r[0]*q[2]+r[1]*q[3]+r[2]*q[0]-r[3]*q[1],
            r[0]*q[3]-r[1]*q[2]+r[2]*q[1]+r[3]*q[0]]

def point_rotation_by_quaternion(point,q):
    r = [0]+point
    q_conj = [q[0],-1*q[1],-1*q[2],-1*q[3]]
    return quaternion_mult(quaternion_mult(q,r),q_conj)[1:]

def on_message(scene, evt, msg):
    oid = msg.get("object_id")

    if done: return
    if not oid.startswith("camera"): return
    # Get the position and orientation of the user's head
    pos = msg["data"]["position"]
    rot = msg["data"]["rotation"]
    rot = [rot["w"], rot["x"], rot["y"], rot["z"]]

    # Compute a raycast from the user's head to the ground plane
    vec = point_rotation_by_quaternion([0, 0, -1], rot)
    res = (0 - pos["y"]) / vec[1]
    x = pos["x"] + res*vec[0]
    z = pos["z"] + res*vec[2]

    if abs(x) < 0.4 and abs(z) < 0.4:
        pushpin.data.color = Color(220,40,40)
    else:
        pushpin.data.color = Color(63,150,210)

    pushpin.data.position = Position(x, 0.01, z)
    scene.update_object(pushpin)

scene.run_tasks()

