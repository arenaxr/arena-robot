#!/usr/bin/env python
"""
licosa_py_basic.py: Get data from a LiCosa serial device.

Created by Perry Naseck on 2/22/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import argparse
import asyncio
from dataclasses import asdict

# from aioserial import AioSerial
from serial import Serial

from licosa_py.parser import LiCosaPacketParser

async def main():
    parser = argparse.ArgumentParser(
        description='Get data from a LiCosa serial device.')
    parser.add_argument('dev_path', metavar='/dev/*', type=str,
                        help='serial device path')
    parser.add_argument('baudrate', nargs='?', type=int, default=230400,
                        help='serial device baudrate')
    args = parser.parse_args()

    # aioserial_instance = AioSerial(port=args.dev_path, baudrate=args.baudrate)
    serial_instance = Serial(port=args.dev_path, baudrate=args.baudrate)
    pkt = LiCosaPacketParser()

    while True:
        # msg_byte = await aioserial_instance.read_async(size=1)
        msg_byte = serial_instance.read(size=1)

        pkt_out = pkt.next_byte(msg_byte)
        if pkt_out is not False:
            data = asdict(pkt_out)
            print(data)

if __name__=="__main__":
    res = asyncio.run(main())
    exit(res)
