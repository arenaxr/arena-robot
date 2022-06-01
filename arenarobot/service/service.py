"""
__init__.py: Service class for ARENA-robot.

Created by Perry Naseck on 1/25/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import json
from datetime import datetime

from arena.device import Device
from paho.mqtt.client import MQTTMessage


class ArenaRobotService():
    """
    Service class for the ARENA.

    This class should be created/initialized as normal. Then check if
    async_service is True. If so, async_setup() should be run. If not, then
    setup() should be run. Lastly, run start() when the service should begin
    listening.
    """

    DEVICE_INSTANCE_TYPE = "unknown"

    # pylint: disable=too-many-arguments
    def __init__(self, instance_name: str, subtopic: str,
                 device_instance_type="unknown",
                 device_instance_prefix="",
                 async_service=False, interval_ms=-1):
        """Initialize the service class."""
        self.device_instance_type = device_instance_type
        instance_name = f"service_{device_instance_prefix}{instance_name}"
        self.instance_name = instance_name
        self.async_service_val = async_service
        self.interval_ms = interval_ms
        self.device = Device()
        self.topic = (f"{self.device.realm}/d/{self.device.namespace}/"
                      f"{self.device.device}/{subtopic}")
        self.device.message_callback_add(self.topic, self.msg_rx)

        print(f'Using topic {self.topic}')

    @property
    def async_service(self):
        """Assert if this is an async service."""
        return self.async_service_val

    @staticmethod
    def decode_payload(msg: MQTTMessage) -> dict:
        """Decode MQTTMessage as JSON."""
        try:
            payload_str = msg.payload.decode("utf-8", "ignore")
            payload = json.loads(json.loads(payload_str))
            return payload
        except json.JSONDecodeError as error:
            print("Malformed payload, ignoring:")
            print(error)
            return None

    def msg_rx(self, client, userdata, msg: MQTTMessage):
        """Receive messages."""

    def setup(self):
        """Set up sensor."""
        self.publish({"status": "initialized"})

    async def async_setup(self):
        """Set up sensor in async."""
        self.setup()

    def publish_msg_base(self):
        """Publish message base."""
        payload = {
            "device_instance_type": self.device_instance_type,
            "device_instance_name": self.instance_name,
            "timestamp": datetime.now().isoformat()[:-3]+"Z"
        }
        return payload

    def publish(self, msg):
        """Publish messages."""
        payload = self.publish_msg_base()
        payload.update({
            "msg": msg
        })
        payload = json.dumps(payload)
        self.device.publish(self.topic, payload)

    def fetch(self):
        """Fetch data."""
        self.publish({"data": "unknown"})

    async def async_fetch(self):
        """Fetch data in async."""
        self.fetch()

    def start(self):
        """Start fetching data."""
        self.publish({"status": "starting"})
        if self.async_service:
            self.device.run_async(self.async_fetch)
        elif self.interval_ms < 0:
            self.device.run_once(self.fetch)
        else:
            self.device.run_forever(self.fetch, interval_ms=self.interval_ms)
        self.device.run_tasks()
