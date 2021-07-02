#!/usr/bin/env python
"""
arena_video_call_chrome.py: Connect to an ARENA video call in Chrome.

Created by Perry Naseck on 7/1/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

# NOTE: This example requires the Selenium Chrome Driver
#       On Raspberry Pi you can get it with:
#         sudo apt-get install chromium-chromedriver
#       Works well on Rapsbian Lite.

import atexit
import sys

from arena import Scene
from selenium.webdriver import Chrome, ChromeOptions

from arenarobot.video_call import VideoCall


def main() -> int:
    """Start a video call example."""
    if len(sys.argv) < 5:
        print('Not enough arguments')
        print(f'{__file__} <host> <realm> <namespace> <scene>')
        return 1

    args = {
        'host': sys.argv[1],
        'realm': sys.argv[2],
        'namespace': sys.argv[3],
        'scene': sys.argv[4]
    }

    options = ChromeOptions()
    options.headless = True
    options.add_argument("--use-fake-ui-for-media-stream=1")
    options.add_argument("--enable-accelerated-video-decode")
    options.add_argument("--enable-gpu-rasterization")
    options.add_argument("--ignore-gpu-blocklist")

    scene = Scene(host=args['host'], scene=args['scene'], realm=args['realm'],
                  namespace=args['namespace'], video=True)

    call = VideoCall(scene, Chrome, options)
    atexit.register(call.close)

    call.open()
    call.video_mute(False)
    call.set_name("RVR")

    scene.run_tasks()

    return 0


if __name__ == "__main__":
    RES = main()
    sys.exit(RES)
