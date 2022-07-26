"""
__init__.py: Definitions for controllers.

Created by Perry Naseck on 7/26/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from arenarobot.service.service import ArenaRobotService


class ArenaRobotServiceController(ArenaRobotService):
    """Controller class for the ARENA."""

    DEVICE_INSTANCE_TYPE = "controller"
    DEVICE_INSTANCE_MULTI_TYPE = "unknown"

    def __init__(self, device_instance_prefix="",
                 device_instance_controller_type="unknown",
                 subtopic="",
                 **kwargs):
        """Initialize the controller class."""
        self.device_instance_controller_type = device_instance_controller_type
        device_instance_type = ArenaRobotServiceController.DEVICE_INSTANCE_TYPE
        super().__init__(
            device_instance_type=device_instance_type,
            device_instance_prefix=f"controller_{device_instance_prefix}",
            subtopic=f"controllers/{subtopic}",
            **kwargs
        )

    def publish_msg_base(self):
        """Publish message base."""
        payload = super().publish_msg_base()
        payload.update({
            "device_instance_controller_type":
                self.device_instance_controller_type
        })
        return payload
