#!/usr/bin/env python
"""
licosa_py.py: Get data from a LiCosa serial device.

Created by Perry Naseck on 2/22/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import argparse
import asyncio
from dataclasses import asdict

from aioserial import AioSerial

from licosa_py import LiCosaPacketParser

async def licosa(device="/dev/ttyAMA2", baudrate=230400):
    aioserial_instance = AioSerial(port=args.dev_path, baudrate=args.baudrate)
    pkt = LiCosaPacketParser()

    while True:
        msg_byte = await aioserial_instance.read_async(size=1)

        pkt_out = pkt.next_byte(msg_byte)
        if pkt_out is not False:
            data = asdict(pkt_out)
            print(data)

