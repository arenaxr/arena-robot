"""
__init__.py: Definitions for the video call client.

Created by Perry Naseck on 7/1/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

# Some code originally from:
# https://github.com/peppelinux/videodrone/blob/97f867bd39d9dfa4c4335487074e77a855858cd1/src/videodrone/drones/__init__.py#L20

import subprocess  # nosec B404
from functools import partial
from os import setpgrp

from arena import Scene
from selenium.webdriver import common
from selenium.webdriver.remote.webdriver import WebDriver

_selenium_orig_start = common.service.Service.start


class VideoCall():
    """Video Call class for the ARENA."""

    _selenium_start_orig = staticmethod(common.service.Service.start)
    _subprocess_popen_orig = staticmethod(subprocess.Popen)

    def __init__(self, scene: Scene, browser: WebDriver, options=None):
        """Initialize the video call class."""
        self.scene = scene
        self.browser = browser
        self.options = options
        self.instance = None

    def __exit__(self, exception_type, exception_value, traceback):
        """Safely exit 'with' statements."""
        self.close()

    def __del__(self):
        """Safely exit if class deleted."""
        self.close()

    def is_open(self) -> bool:
        """Return if the browser is running."""
        return self.instance is not None

    @staticmethod
    def _selenium_start(*args, **kwargs):
        """
        Start Selenium but ignore handlers like SIGINT.

        Modified from https://stackoverflow.com/a/62430234
        This allows for manually closing Selenium on CTRL+C so that the Jitsi
        call can be properly hung up.
        """
        try:
            subprocess.Popen = partial(subprocess.Popen,
                                       preexec_fn=setpgrp)
            VideoCall._selenium_start_orig(*args, **kwargs)
        finally:
            subprocess.Popen = VideoCall._subprocess_popen_orig

    def open(self):
        """Start the browser."""
        token = self.scene.remote_auth_token['token']
        url = 'https://jitsi0.andrew.cmu.edu:8443/'
        url += f'{self.scene.namespace}_{self.scene.scene}'
        print(f"arena-robot VideoCall: opening {url}")
        url += f'?jwt={token}'
        url += '#config.channelLastN=0&config.resolution=1080'

        # Temporarily override the start function to not pass SIGINT
        try:
            common.service.Service.start = self._selenium_start
            self.instance = self.browser(options=self.options)
        finally:
            common.service.Service.start = self._selenium_start_orig

        self.instance.get(url)

    def set_name(self, name: str):
        """Set the Jitsi display name."""
        script = f"APP.conference.changeLocalDisplayName('{name}');"
        self.instance.execute_script(script)

    def video_mute(self, mute: bool = True):
        """Set the Jitsi video mute state."""
        script = f"APP.conference.muteVideo({str(mute).lower()});"
        self.instance.execute_script(script)

    def audio_mute(self, mute: bool = True):
        """Set the Jitsi audio mute state."""
        script = f"APP.conference.muteAudio({str(mute).lower()});"
        self.instance.execute_script(script)

    def close(self):
        """Close and quit the browser."""
        print("arena-robot VideoCall: closing")
        if self.instance is not None:
            self.instance.execute_script('APP.conference.hangup();')
            self.instance.quit()
            self.instance = None
