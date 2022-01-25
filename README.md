ARENA-robot
===========
Robot integration runtime for the ARENA.

## Installation
Requires Python3.6+
```bash
pip install git+https://github.com/conix-center/ARENA-robot#egg=arena-robot
```

Or for development, clone the repo and from inside the directory:
```bash
pip install -e .
```

The `vl53l5cx_py` module requires Linux i2c interface (`linux/i2c.h` and `linux/i2c-dev.h`).

## Modules

### arenarobot

Services that stream to MQTT may be started with the `arena-robot-service` command. See [`examples/service_vl53l5cx_lidar_1.json`](./examples/service_vl53l5cx_lidar_1.json) for a service file example. All services require the following arguments:

 - `service_type`: Required. Currently only `sensor_vl53l5cx` is supported.
 - `instance_name`: Required. The name of this running service.
 - `subtopic`: Required. The device subtopic to publish to. For sensors this will always be prefixed with `sensors/`.
 - `interval_ms` For services that run repeatedly, this is the interval. Set to 0 by default, which will fire the service once.

#### sensor_vl53l5cx
This is a lidar sensor that connects through I2C and addresses by toggling its LPn pins. It additionally supports the following arguments:

 - `dev_path`: Required. Path to the I2C dev file device. For example, the first I2C port on the Raspberry Pi is `/dev/i2c-1`.
 - `gpio_path`: Required. Path to the GPIO dev file device. For example, the first I2C port on the Raspberry Pi is `/dev/gpiochip0`.
 - `lpn_pins`: Required. Array of pins (integers) on `gpio_path` connected to Lpn pins on the sensor. The array must be present, and at least one pin must be specified or the service won't initialized any sensors.

### arenavideocall

After installing the repo, you should now have the `arena-video-call-chrome` command. You can also run it with `python3 -m arena-video-call-chrome`. It uses the [standard arena-py scene environment variables](https://arena.conix.io/content/python/#running-from-the-command-line).

The first argument sets the surface that the video should be mapped to. For example, to map to an object called `avideobox`:
```bash
MQTTH=arenaxr.org NAMESPACE=namespace SCENE=scene arena-video-call-chrome avideobox
```

### vl53l5cx_py

C shared library
```bash
gcc -fPIC -shared -o vl53l5cx.so -Ivl53l5cx_py/include -Ivl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/platform -Ivl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/uld-driver/inc vl53l5cx_py/src/*.c vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/platform/*.c vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/uld-driver/src/*.c -l
```

## License

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
