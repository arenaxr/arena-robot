#!/usr/bin/env python3
"""
arena_robot_service.py: Run an ARENA-robot service.

Created by Perry Naseck on 1/25/22.

Copyright (c) 2022, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import argparse
import asyncio
import json
import sys

from arenarobot import service


def main() -> int:
    """Run an ARENA-robot service."""
    parser = argparse.ArgumentParser(description='Run an ARENA-robot service.')
    parser.add_argument('service_config_file',  type=str,
                        help='service file device path')
    args = parser.parse_args()

    with open(args.service_config_file, encoding="utf8") as file:
        service_config = json.load(file)

    service_type = service_config.get("service_type", "unknown")
    if "service_type" in service_config:
        del service_config["service_type"]

    service_class = service.SERVICE_TYPE_MAP[service_type]

    service_instance = service_class(**service_config)
    if service_instance.async_service:
        (asyncio.get_event_loop()
            .run_until_complete(service_instance.async_setup()))
    else:
        service_instance.setup()
    service_instance.start()

    return 0


if __name__ == "__main__":
    RES = main()
    sys.exit(RES)
