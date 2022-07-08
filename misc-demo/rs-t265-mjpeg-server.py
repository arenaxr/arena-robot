#!/usr/bin/env python
"""
rs-t265-mjpeg-server.py
Created by John Pedraza and Perry Naseck on 7/8/22.
Based off of:
https://github.com/devenpatel2/aiohttp_mjpeg/tree/935d66e07f3ddcf2437822de1c08fccd8533cd10

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import argparse
from asyncio import sleep
from sys import exit

import numpy as np
import pyrealsense2.pyrealsense2 as rs
from aiohttp import MultipartWriter, web
from cv2 import imencode


class StreamHandler:
    def __init__(self, cam):
        self._cam = cam

    async def __call__(self, request):
        my_boundary = 'image-boundary'
        response = web.StreamResponse(
            status=200,
            reason='OK',
            headers={
                'Content-Type': 'multipart/x-mixed-replace;boundary={}'.format(my_boundary)
            }
        )
        await response.prepare(request)
        while True:
            frame = await self._cam.get_frame()
            with MultipartWriter('image/jpeg', boundary=my_boundary) as mpwriter:
                mpwriter.append(frame, {
                    'Content-Type': 'image/jpeg'
                })
                try:
                    await mpwriter.write(response, close_boundary=False)
                except ConnectionResetError :
                    print("Client connection closed")
                    break
            await response.write(b"\r\n")


class MjpegServer:
    def __init__(self, host='0.0.0.0', port='8265'):
        self._port = port
        self._host = host
        self._app = web.Application()
        self._cam_routes = []

    async def root_handler(self, request):
        text = 'Available streams:\n\n'
        for route in self._cam_routes:
            text += f"{route} \n"
        return web.Response(text=text)

    def add_stream(self, route, cam):
        route = f"/{route}"
        self._cam_routes.append(route)
        assert hasattr(cam, 'get_frame'), "arg 'cam' should have a 'get_frame' method"
        self._app.router.add_route("GET", f"{route}", StreamHandler(cam))

    def start(self):
        self._app.router.add_route("GET", "/", self.root_handler)
        web.run_app(self._app, host=self._host, port=self._port)

    def stop(self):
        pass


class T265Camera:
    def __init__(self, idx, pipeline):
        self._idx = idx
        self.pipeline = pipeline

    @property
    def identifier(self):
        return self._idx

    # The camera class should contain a "get_frame" method
    async def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        camera = frames.get_fisheye_frame(self._idx)
        frame = np.asanyarray(camera.get_data())
        frame = imencode('.jpg', frame)[1]
        await sleep(1 / 25)
        return frame.tobytes()

    def stop(self):
        pass


def main():
    parser = argparse.ArgumentParser(description='Run T265 MJPEG server.')
    parser.add_argument('--host', type=str, default="0.0.0.0",
                        help='host to listen on')
    parser.add_argument('--port', type=int, default=8265,
                        help='port to listen on')
    args = parser.parse_args()

    # Instantiate Server
    server = MjpegServer(host=args.host, port=args.port)

    # Reset T265
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        dev.hardware_reset()

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable fisheye streams
    config.enable_stream(rs.stream.fisheye, 1)
    config.enable_stream(rs.stream.fisheye, 2)

    # Start streaming
    pipeline.start(config)

    try:
        # create lookup of routes and different camera objects
        cams = {
            "cam1": T265Camera(1, pipeline),
            "cam2": T265Camera(2, pipeline)
        }

        for route, cam in cams.items():
            # add routes
            server.add_stream(route, cam)

        try:
            # start server
            server.start()
        except KeyboardInterrupt:
            print("Keyboard Interrupt, exiting...")
        finally:
            server.stop()
            for cam in cams.values():
                cam.stop()
    finally:
        pipeline.stop()
    return 0

if __name__ == "__main__":
    RES = main()
    exit(RES)


def main() -> int:
    """Run an ARENA-robot service."""
    parser = argparse.ArgumentParser(description='Run an ARENA-robot service.')
    parser.add_argument('service_config_file',  type=str,
                        help='service file device path')
    args = parser.parse_args()

    with open(args.service_config_file, encoding="utf8") as file:
        service_config = json.load(file)

    service_type = service_config.get("service_type", "unknown")
    if "service_type" in service_config:
        del service_config["service_type"]

    service_class = service.SERVICE_TYPE_MAP[service_type]

    service_instance = service_class(**service_config)
    if service_instance.async_service:
        (asyncio.get_event_loop()
            .run_until_complete(service_instance.async_setup()))
    else:
        service_instance.setup()
    service_instance.start()
