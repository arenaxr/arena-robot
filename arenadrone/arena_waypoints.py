#!/usr/bin/env python3
from arena import *

scene = Scene(host="arenaxr.org", scene="ARENA-drone", realm="realm", namespace="pnaseck")

positions = [
    [0, 1.5, 0],
    [1.5, 1.5, 0],
    [1.5, 1.5, -1.5],
    [0, 1.5, -1.5]
]
current = 0
mode = "INIT"

drone_target = Box(
    object_id="drone_target",
    position=(0,0,0),
    scale=(0.1,0.1,0.1),
    color=(204,0,0),
    clickable=True
)

def land(scene, evt, msg):
    global mode
    if mode == "FLY":
        mode = "GOTO_LAND"

@scene.run_forever(interval_ms=4000)
def move_target():
    global current, positions, mode
    print(mode)

    next_mode = mode
    if mode == "INIT":
        pos = positions[0]
        scene.update_object(drone_target)
        next_mode = "FLY"
    elif mode == "FLY":
        pos = positions[current]
        current += 1
        if current >= len(positions):
            current = 0
    elif mode == "GOTO_LAND":
        pos = [0, 1.5, 0]
        next_mode = "LAND"
    elif mode == "LAND":
        pos = [0, 0, 0]
        next_mode = "DELETE"
    elif mode == "DELETE":
        return

    if mode == "DELETE":
        scene.delete_object(drone_target)
    else:
        drone_target.update_attributes(position=Position(pos[0], pos[1], pos[2]))
        scene.update_object(drone_target, clickable=True, evt_handler=land)

    mode = next_mode

scene.run_tasks()
