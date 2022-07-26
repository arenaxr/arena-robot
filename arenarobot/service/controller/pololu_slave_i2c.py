"""
pololu_slave_i2c.py: Interface for Pololu Slave I2C devices.

Created by Perry Naseck on 6/30/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

# This is essentially a port of the Pololu code from smbus to periphery with
# dynamic buffer format configuration:
# https://github.com/pololu/pololu-rpi-slave-arduino-library/blob/master/pi/a_star.py

from struct import calcsize, pack, unpack
from time import sleep
from typing import Sequence

from paho.mqtt.client import MQTTMessage
from periphery import I2C, I2CError

from arenarobot.service.controller import ArenaRobotServiceController


# pylint: disable=too-many-instance-attributes
class ArenaRobotServiceControllerPololuSlaveI2C(ArenaRobotServiceController):
    """Pololu Slave I2C device class for the ARENA."""

    DEVICE_INSTANCE_CONTROLLER_TYPE = "pololu_slave_i2c"

    # pylint: disable=too-many-branches
    def __init__(self, dev_path: str,
                 slave_data_types: Sequence[dict],
                 slave_addr: int = 20,
                 **kwargs):
        """Initialize the Pololu Slave I2C device controller class."""
        self.dev_path = dev_path
        self.slave_addr = slave_addr
        self.slave_data_types = slave_data_types

        for name, data_type in self.slave_data_types.items():
            if len(name) < 1:
                raise ValueError("Data type names must not be blank!")
            if "format" not in data_type:
                raise ValueError("Data type missing format char!")
            if not isinstance(data_type["format"], str):
                raise ValueError("Data type format not a str!")
            if len(data_type["format"]) != 1:
                raise ValueError("Data type format option should be one char!")
            if "num_items" not in data_type:
                raise ValueError("Data type missing num_items!")
            if (not isinstance(data_type["num_items"], int) or
                    data_type["num_items"] < 1):
                raise ValueError("Data type num_items must be int at least 1!")
            if "write" not in data_type:
                raise ValueError("Data type missing write!")
            if not isinstance(data_type["write"], bool):
                raise ValueError("Data type write not a bool!")
            if data_type["write"]:
                if "initial" not in data_type:
                    raise ValueError("Writable data type missing initial!")
                if data_type["format"] == "s":
                    if not isinstance(data_type["initial"], str):
                        raise ValueError("Data type initial string not a str!")
                    intitial_bytes = bytes(data_type["initial"], "utf-8")
                    if len(intitial_bytes) != data_type["num_items"]:
                        raise ValueError("Data type initial wrong length!")
                elif len(data_type["initial"]) != data_type["num_items"]:
                    if not isinstance(data_type["initial"], list):
                        raise ValueError("Data type initial list not a list!")
                    raise ValueError("Data type initial wrong length!")

        self.data_types_write = []
        self.i2c_bus = None
        self.struct_format_str = "<"  # Little endian
        self.struct_size = 0

        controller_type = (ArenaRobotServiceControllerPololuSlaveI2C
                           .DEVICE_INSTANCE_CONTROLLER_TYPE)
        super().__init__(
            device_instance_controller_type=controller_type,
            device_instance_prefix="pololu_slave_i2c_",
            **kwargs
        )

    def setup(self):
        """Set up Pololu Slave I2C controller."""
        for name, data_type in self.slave_data_types.items():
            if data_type["write"]:
                self.data_types_write.append(name)
            data_type["addr"] = self.struct_size
            data_type["full_format"] = (
                str(data_type["num_items"]) + data_type["format"])
            data_type["size"] = calcsize(data_type["format"])
            data_type["full_size"] = calcsize(data_type["full_format"])
            self.struct_size += data_type["full_size"]
            self.struct_format_str += data_type["full_format"]

        self.i2c_bus = I2C(self.dev_path)

        # write start byte and read and ignore first message
        self.i2c_bus.transfer(self.slave_addr, [I2C.Message([0], read=False)])
        sleep(0.0001)
        self.i2c_bus.transfer(
            self.slave_addr,
            [I2C.Message([0x0]*self.struct_size, read=True)])
        sleep(0.0001)

        # reset initials
        for name in self.data_types_write:
            initial = self.slave_data_types[name]["initial"]
            self.write_val(name, initial)
            sleep(0.0001)

        super().setup()

    def write_val(self, param_name: str, value, index: int = 0):
        """Write value to Pololu Slave I2C device."""
        if param_name not in self.data_types_write:
            raise ValueError("Param not found or not in write list!")
        data_type = self.slave_data_types[param_name]
        addr = data_type["addr"]
        format_char = data_type["format"]
        packed_format = format_char
        num_items = data_type["num_items"]

        if index > 0 and format_char == "s":
            raise ValueError("Indices cannot be used for strings!")
        if index > 0 and num_items <= 1:
            raise ValueError("Index too high for this data type!")
        if format_char == "s" and not isinstance(value, str):
            raise ValueError("String value must be a string!")
        if isinstance(value, Sequence) and len(value) > num_items - index:
            raise ValueError("Too many items in value list!")
        if isinstance(value, Sequence) and len(value) == 0:
            raise ValueError("Value list is empty!")

        addr += data_type["size"] * index
        if format_char == "s":
            packed_format = data_type["full_format"]
            value = (bytes(value, "utf-8"),)
        elif not isinstance(value, Sequence):
            value = (value,)
        else:
            packed_format = data_type["format"] * len(value)

        packed = list(pack(packed_format, *value))
        msgs = [I2C.Message([addr, *packed])]
        self.i2c_bus.transfer(self.slave_addr, msgs)

    def fetch(self):
        """Fetch Pololu Slave I2C data."""
        # Set up for read
        msgs = [I2C.Message([0x0], read=False)]
        try:
            self.i2c_bus.transfer(self.slave_addr, msgs)
        except I2CError as err:
            print(err)
            self.publish({"data": {"error": str(err)}})
            print("continuing")
            return

        # Required delay, see Pololu a_star.py for explanation. This was
        # to also be required in this implementation. Otherwise, the first
        # few fields would be unintentionally written to.
        sleep(0.0001)

        # Read
        msgs = [I2C.Message([0x0]*self.struct_size, read=True)]
        try:
            self.i2c_bus.transfer(self.slave_addr, msgs)
        except I2CError as err:
            print(err)
            self.publish({"data": {"error": str(err)}})
            print("continuing")
            return
        buffer_in = unpack(self.struct_format_str, bytes(msgs[0].data))
        data = {}
        index = 0
        for name in self.slave_data_types:
            num_items = self.slave_data_types[name]["num_items"]
            format_char = self.slave_data_types[name]["format"]
            if format_char == "s":  # bytes from a string are special in struct
                data[name] = buffer_in[index].decode("utf-8")
                index += 1
            elif num_items == 1:
                data[name] = buffer_in[index]
                index += num_items
            else:
                data[name] = []
                for count in range(num_items):
                    data[name].append(buffer_in[index + count])
                index += num_items
        self.publish({"data": {"pololu_device": data}})

    # pylint: disable=unused-argument
    def msg_rx(self, client, userdata, msg: MQTTMessage):
        """Receive messages to controll the Pololu I2C device."""
        payload = self.decode_payload(msg)
        if payload is None:
            return

        # Only receive commands destined for this instance
        if ("target_device_instance_name" not in payload or
                payload["target_device_instance_name"] != self.instance_name):
            return
        if "msg" not in payload:
            print("Skipping missing msg for",
                  payload["target_device_instance_name"],
                  payload)
            return
        if "data" not in payload["msg"]:
            print("Skipping missing data for",
                  payload["target_device_instance_name"],
                  payload["msg"])
            return
        data = payload["msg"]["data"]

        if "pololu_device_commands" not in data:
            print("Skipping missing commands for",
                  payload["target_device_instance_name"],
                  payload["msg"])
            return

        for command in data["pololu_device_commands"]:
            try:
                name = command["name"]
                value = command["value"]
                if "index" in command:
                    self.write_val(name, value, command["index"])
                else:
                    self.write_val(name, value)
            except ValueError as err:
                print(err)
                self.publish({"data": {"error": str(err)}})
            except Exception as err:  # pylint: disable=broad-except
                print(err)
                self.publish({"data": {"error": str(err)}})
                print("continuing")
            sleep(0.0001)
