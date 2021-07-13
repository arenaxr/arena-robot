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

from arena import Material, Scene
from selenium.webdriver import common
from selenium.webdriver.remote.webdriver import WebDriver

_selenium_orig_start = common.service.Service.start


class VideoCall():
    """Video Call class for the ARENA."""

    _selenium_start_orig = staticmethod(common.service.Service.start)
    _subprocess_popen_orig = staticmethod(subprocess.Popen)

    def __init__(self, scene: Scene, browser: WebDriver, options=None) -> None:
        """Initialize the video call class."""
        self.scene = scene
        self.browser = browser
        self.options = options
        self.instance = None
        self.joined = False

    def __exit__(self, exception_type, exception_value, traceback) -> None:
        """Safely exit 'with' statements."""
        self.close()

    def __del__(self) -> None:
        """Safely exit if class deleted."""
        self.close()

    def is_open(self) -> bool:
        """Return if the browser is running."""
        return self.instance is not None

    @staticmethod
    def _selenium_start(*args, **kwargs) -> None:
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

    def open(self) -> None:
        """Start the browser."""
        token = self.scene.remote_auth_token['token']
        url = 'https://jitsi0.andrew.cmu.edu:8443/'
        url += f'{self.scene.namespace}_{self.scene.scene}'
        print(f"arena-robot VideoCall: opening {url}")
        url += f'?jwt={token}'
        url += '#config.channelLastN=0'

        # Temporarily override the start function to not pass SIGINT
        try:
            common.service.Service.start = self._selenium_start
            self.instance = self.browser(options=self.options)
        finally:
            common.service.Service.start = self._selenium_start_orig

        self.instance.get(url)

    def wait_for_join(self) -> None:
        """Wait for the call to be joined."""
        if not self.joined:
            script = "return APP.conference.isJoined();"
            while not self.instance.execute_script(script):
                pass
            self.joined = True

    def change_avatar_url(self, url: str) -> None:
        """Set the Jitsi avatar url."""
        script = f"APP.conference.changeLocalAvatarUrl('{url}');"
        self.instance.execute_script(script)

    def change_display_name(self, name: str) -> None:
        """Set the Jitsi display name."""
        script = f"APP.conference.changeLocalDisplayName('{name}');"
        self.instance.execute_script(script)

    def change_email(self, email: str) -> None:
        """Set the Jitsi email."""
        script = f"APP.conference.changeLocalEmail('{email}');"
        self.instance.execute_script(script)

    def get_display_name(self) -> str:
        """Get the Jitsi display name."""
        self.wait_for_join()
        script = "return APP.conference.getLocalDisplayName();"
        return self.instance.execute_script(script)

    def get_room_name(self) -> str:
        """Get the Jitsi room name."""
        script = "return APP.conference.roomName;"
        return self.instance.execute_script(script)

    def get_stats(self) -> dict:
        """Get the Jitsi stats."""
        self.wait_for_join()
        script = "return APP.conference.getStats();"
        return self.instance.execute_script(script)

    def get_user_id(self) -> str:
        """Get the Jitsi user ID."""
        self.wait_for_join()
        script = "return APP.conference.getMyUserId();"
        return self.instance.execute_script(script)

    def get_audio_level(self) -> float:
        """Get the Jitsi audio level."""
        self.wait_for_join()
        script = "return APP.conference.localAudio.audioLevel;"
        return self.instance.execute_script(script)

    def get_audio_track_label(self) -> str:
        """Get the Jitsi audio level."""
        self.wait_for_join()
        script = "return APP.conference.localAudio.track.label;"
        return self.instance.execute_script(script)

    def get_video_resolution(self) -> int:
        """Get the Jitsi video resolution."""
        self.wait_for_join()
        script = "return APP.conference.localVideo.resolution;"
        return self.instance.execute_script(script)

    def get_video_track_label(self) -> str:
        """Get the Jitsi video track."""
        self.wait_for_join()
        script = "return APP.conference.localVideo.track.label;"
        return self.instance.execute_script(script)

    def get_audio_mute(self) -> bool:
        """Get the Jitsi audio mute state."""
        script = "return APP.conference.isLocalAudioMuted();"
        return self.instance.execute_script(script)

    def get_video_mute(self) -> bool:
        """Get the Jitsi video mute state."""
        script = "return APP.conference.isLocalVideoMuted();"
        return self.instance.execute_script(script)

    def set_audio_mute(self, mute: bool = True) -> None:
        """Set the Jitsi audio mute state."""
        script = f"APP.conference.muteAudio({str(mute).lower()});"
        self.instance.execute_script(script)

    def set_video_mute(self, mute: bool = True) -> None:
        """Set the Jitsi video mute state."""
        script = f"APP.conference.muteVideo({str(mute).lower()});"
        self.instance.execute_script(script)

    def update_object_material_src(self, obj) -> None:
        """Update object material source to use video."""
        material_src = f'#video{self.get_user_id()}'
        obj.update_attributes(material=Material(src=material_src))
        self.scene.update_object(obj)

    def close(self) -> None:
        """Close and quit the browser."""
        print("arena-robot VideoCall: closing")
        if self.instance is not None:
            self.instance.execute_script('APP.conference.hangup();')
            self.instance.quit()
            self.instance = None
