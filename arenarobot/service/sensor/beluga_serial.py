"""
beluga_serial.py: Definition for Beluga UWB modules over serial.

Created by Perry Naseck on 1/27/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import time
from typing import Optional

from aioserial import AioSerial

from arenarobot.service.sensor import ArenaRobotServiceSensor


# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceSensorBelugaSerial(ArenaRobotServiceSensor):
    """Beluga serial sensor class for the ARENA."""

    DEVICE_INSTANCE_SENSOR_TYPE = "beluga_serial"

    # pylint: disable=too-many-arguments
    # pylint: disable=too-many-statements
    def __init__(self, dev_path: str, beluga_id: int, baudrate: int = 115200,
                 beluga_rate: int = 10, beluga_channel: Optional[int] = None,
                 beluga_tx_power: Optional[int] = None,
                 beluga_timeout: Optional[int] = None,
                 beluga_twr_mode: Optional[int] = None,
                 beluga_led_mode: Optional[int] = None,
                 **kwargs):
        """Initialize the Beluga serial sensor class."""
        self.dev_path = dev_path
        self.baudrate = baudrate
        self.beluga_id = beluga_id
        self.beluga_rate = beluga_rate
        self.beluga_channel = beluga_channel
        self.beluga_tx_power = beluga_tx_power
        self.beluga_timeout = beluga_timeout
        self.beluga_twr_mode = beluga_twr_mode
        self.beluga_led_mode = beluga_led_mode

        self.aioserial_instance = None

        sensor_type = (ArenaRobotServiceSensorBelugaSerial
                       .DEVICE_INSTANCE_SENSOR_TYPE)
        super().__init__(
            device_instance_sensor_type=sensor_type,
            device_instance_prefix="beluga_serial_",
            async_service=True, **kwargs
        )

    async def async_setup(self):
        """Set up Beluga serial sensor."""
        self.aioserial_instance = AioSerial(port=self.dev_path,
                                            baudrate=self.baudrate)

        def msg_in_clear():
            # Wait and clear all of the messages that come in
            # Python seems bad at clearing the buffers the first time
            for _ in range(2):
                while self.aioserial_instance.in_waiting > 0:
                    self.aioserial_instance.reset_input_buffer()
                time.sleep(1)

        async def wait_for_msg(wait_msg: str):
            while True:
                msg_bytes = await self.aioserial_instance.readline_async()
                msg = msg_bytes.decode(errors="ignore")
                if msg.startswith(wait_msg):
                    break

        msg_in_clear()

        set_id_msg = "AT+RESET\n".encode()
        await self.aioserial_instance.write_async(set_id_msg)
        await wait_for_msg("Reset OK")
        time.sleep(1)

        set_id_msg = "AT+REBOOT\n".encode()
        await self.aioserial_instance.write_async(set_id_msg)

        msg_in_clear()

        set_id_msg = f"AT+ID {self.beluga_id}\n".encode()
        await self.aioserial_instance.write_async(set_id_msg)
        await wait_for_msg("OK")

        set_boot_mode_msg = "AT+BOOTMODE 0\n".encode()
        await self.aioserial_instance.write_async(set_boot_mode_msg)
        await wait_for_msg("Bootmode: 0 OK")

        if self.beluga_rate is not None:
            set_rate_msg = f"AT+RATE {self.beluga_rate}\n".encode()
            await self.aioserial_instance.write_async(set_rate_msg)
            await wait_for_msg(f"Rate: {self.beluga_rate} OK")

        if self.beluga_channel is not None:
            set_channel_msg = f"AT+CHANNEL {self.beluga_channel}\n".encode()
            await self.aioserial_instance.write_async(set_channel_msg)
            await wait_for_msg("OK")

        if self.beluga_tx_power is not None:
            set_tx_power_msg = f"AT+TXPOWER {self.beluga_tx_power}\n".encode()
            await self.aioserial_instance.write_async(set_tx_power_msg)
            await wait_for_msg("OK")

        if self.beluga_timeout is not None:
            set_timeout_msg = f"AT+TIMEOUT {self.beluga_timeout}\n".encode()
            await self.aioserial_instance.write_async(set_timeout_msg)
            await wait_for_msg("OK")

        set_stream_mode_msg = "AT+STREAMMODE 0\n".encode()
        await self.aioserial_instance.write_async(set_stream_mode_msg)
        await wait_for_msg("OK")

        if self.beluga_twr_mode is not None:
            set_twr_mode_msg = f"AT+TWRMODE {self.beluga_twr_mode}\n".encode()
            await self.aioserial_instance.write_async(set_twr_mode_msg)
            await wait_for_msg("OK")

        if self.beluga_led_mode is not None:
            set_led_mode_msg = f"AT+LEDMODE {self.beluga_led_mode}\n".encode()
            await self.aioserial_instance.write_async(set_led_mode_msg)
            await wait_for_msg("OK")

        start_ble_msg = "AT+STARTBLE\n".encode()
        await self.aioserial_instance.write_async(start_ble_msg)
        await wait_for_msg("OK")
        time.sleep(0.25)

        start_uwb_msg = "AT+STARTUWB\n".encode()
        await self.aioserial_instance.write_async(start_uwb_msg)
        await wait_for_msg("OK")

        time.sleep(1)
        await super().async_setup()

    async def async_fetch(self):
        """Fetch Beluga serial data."""
        self.aioserial_instance.reset_input_buffer()
        while True:
            msg_bytes = await self.aioserial_instance.readline_async()
            msg = msg_bytes.decode(errors="ignore")
            if msg[0] == "#":
                # Skip publishing comment messages
                continue
            msg_split = msg.split(", ")
            if len(msg_split) < 4:
                # Skip garbled messages
                continue
            rx_id, rx_range, rx_rssi, rx_timestamp = msg_split
            data = {
                "rx_nodes": [{
                    "src": int(self.beluga_id),
                    "dst": int(rx_id),
                    "distance": float(rx_range),
                    "ble_rssi": int(rx_rssi),
                    "counter": int(rx_timestamp)
                }]
            }
            self.publish({"data": data})
