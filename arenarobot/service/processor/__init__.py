"""
__init__.py: Definitions for processors.

Created by Perry Naseck on 2/24/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from arenarobot.service.service import ArenaRobotService


class ArenaRobotServiceProcessor(ArenaRobotService):
    """Processor class for the ARENA."""

    DEVICE_INSTANCE_TYPE = "processor"
    DEVICE_INSTANCE_PROCESSOR_TYPE = "unknown"

    def __init__(self, device_instance_prefix="",
                 device_instance_processor_type="unknown",
                 subtopic="",
                 **kwargs):
        """Initialize the processor class."""
        self.device_instance_processor_type = device_instance_processor_type
        device_instance_type = ArenaRobotServiceProcessor.DEVICE_INSTANCE_TYPE
        super().__init__(
            device_instance_type=device_instance_type,
            device_instance_prefix=f"processor_{device_instance_prefix}",
            subtopic=f"processors/{subtopic}",
            **kwargs
        )

    def publish_msg_base(self):
        """Publish message base."""
        payload = super().publish_msg_base()
        payload.update({
            "device_instance_processor_type":
                self.device_instance_processor_type
        })
        return payload
