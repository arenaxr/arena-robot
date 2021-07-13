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

import sys
from platform import node

from arena import Scene
from selenium.webdriver import Chrome, ChromeOptions

from arenarobot.video_call import VideoCall


def main() -> int:
    """Start a video call example."""
    if len(sys.argv) < 5:
        print('Not enough arguments')
        print(f'{__file__} <host> <realm> <namespace> <scene> [object_id]')
        return 1

    args = {
        'host': sys.argv[1],
        'realm': sys.argv[2],
        'namespace': sys.argv[3],
        'scene': sys.argv[4],
        'object_id': None
    }
    if len(sys.argv) > 5:
        args['object_id'] = sys.argv[5]

    options = ChromeOptions()
    options.headless = True
    options.add_argument("--use-fake-ui-for-media-stream=1")
    options.add_argument("--enable-accelerated-video-decode")
    options.add_argument("--enable-gpu-rasterization")
    options.add_argument("--ignore-gpu-blocklist")

    scene = Scene(host=args['host'], scene=args['scene'], realm=args['realm'],
                  namespace=args['namespace'], video=True)

    call = VideoCall(scene, Chrome, options)

    call.open()
    call.change_display_name(node())  # Hostname
    call.set_audio_mute(True)
    call.set_video_mute(False)

    call.wait_for_join()  # Wait for call to finish joining

    print('Jitsi User ID:', call.get_user_id())
    print('Jitsi Display Name:', call.get_display_name())
    print('Jitsi Room Name:', call.get_room_name())
    print('Jitsi Audio Muted:', call.get_audio_mute())
    print('Jitsi Video Muted:', call.get_video_mute())
    print('Jitsi Video Track Label:', call.get_video_track_label())
    print('Jitsi Video Resolution:', call.get_video_resolution())

    @scene.run_once
    def update_video_object():
        if args['object_id'] is not None:
            print(f"Updating object '{args['object_id']}' to use new video")
            obj = scene.get_persisted_obj(args['object_id'])
            call.update_object_material_src(obj)

    scene.run_tasks()

    return 0


if __name__ == "__main__":
    RES = main()
    sys.exit(RES)
