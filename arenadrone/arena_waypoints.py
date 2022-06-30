#!/usr/bin/env python3
from enum import Enum, auto

from arena import *

scene = Scene(host="mqtt.arenaxr.org", scene="ARENA-drone", realm="realm", namespace="pnaseck")

HEIGHT = 1.5
positions = [
    [1.5, HEIGHT, 0],
    [1.5, HEIGHT, -1.5],
    [0, HEIGHT, -1.5],
    [0, HEIGHT, 0],
]
current_pos = 0

class Mode(Enum):
    INIT = auto()
    FLY = auto()
    GOTO_LAND = auto()
    LAND = auto()
    DELETE = auto()
    DONE = auto()
mode = Mode.INIT

added = False

drone_target = Box(
    object_id="drone_target",
    position=(0,HEIGHT,0),
    scale=(0.1,0.1,0.1),
    color=(204,0,0),
    clickable=True,
    persist=True
)

def click(scene, evt, msg):
    global mode
    next_mode = mode
    if evt["type"] == "mousedown":
        if mode == Mode.INIT:
            next_mode = Mode.FLY
        elif mode == Mode.FLY:
            next_mode = Mode.GOTO_LAND
    mode = next_mode

@scene.run_forever(interval_ms=4000)
def move_target():
    global current_pos, positions, mode, HEIGHT, added 
    print(mode.name)

    next_mode = mode
    if mode == Mode.INIT:
        pos = [0, HEIGHT, 0]
        scene.update_object(drone_target,persist=True)
        next_mode = Mode.INIT
    elif mode == Mode.FLY:
        pos = positions[current_pos]
        current_pos += 1
        if current_pos >= len(positions):
            current_pos = 0
        next_mode = Mode.FLY
    elif mode == Mode.GOTO_LAND:
        pos = [0, HEIGHT, 0]
        next_mode = Mode.LAND
    elif mode == Mode.LAND:
        pos = [0, 0, 0]
        next_mode = Mode.DELETE
    elif mode == Mode.DELETE:
        next_mode = Mode.DONE
    elif mode == Mode.DONE:
        next_mode = Mode.DONE
    else:
        raise Exception("Unknown mode", mode)

    if not added: 
        scene.add_object(drone_target)
        added = True
        
    if mode == Mode.DELETE:
        scene.delete_object(drone_target)
    elif mode != Mode.DONE:
        drone_target.update_attributes(position=Position(pos[0], pos[1], pos[2]), persist=True)
        scene.update_object(drone_target, clickable=True, evt_handler=click)

    mode = next_mode

scene.run_tasks()
