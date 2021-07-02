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

# import time

from arena import Scene
from selenium.webdriver.remote.webdriver import WebDriver

# from selenium.webdriver.support.ui import WebDriverWait


class VideoCall():
    """Video Call class for the ARENA."""

    def __init__(self, scene: Scene, browser: WebDriver, options=None):
        """Initialize the video call class."""
        self.scene = scene
        self.browser = browser
        self.options = options
        self.instance = None

    def is_open(self) -> bool:
        """Return if the browser is running."""
        return self.instance is not None

    def open(self):
        """Start the browser."""
        token = self.scene.remote_auth_token['token']
        url = 'https://jitsi0.andrew.cmu.edu:8443/'
        url += f'{self.scene.namespace}_{self.scene.scene}'
        print(f"arena-robot VideoCall: opening {url}")
        url += f'?jwt={token}'
        url += '#config.channelLastN=0'
        self.instance = self.browser(options=self.options)
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
            # TODO: proper hangup # pylint: disable=fixme
            # self.instance.execute_script('APP.conference.hangup();')
            # time.sleep(2000)
            self.instance.quit()
            self.instance = None
