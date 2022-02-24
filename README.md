ARENA-robot
===========
Robot integration runtime for the ARENA.

## Installation
Requires Python3.8+ (it probably runs fine on 3.6 and 3.7, but you will need your own NumPy wheels)

From [PyPi](https://pypi.org/project/arena-robot/):
```bash
pip install arena-robot
```

Latest development:
```bash
pip install git+https://github.com/conix-center/ARENA-robot#egg=arena-robot
```

Or for development, clone the repo and from inside the directory:
```bash
pip install -e .
```

## Modules

### arenarobot

Services that stream to MQTT may be started with the `arena-robot-service` command. See [`examples/service_vl53l5cx_lidar_1.json`](./examples/service_vl53l5cx_lidar_1.json) for a service file example. All services require the following arguments:

 - `service_type`: Required. Currently only `sensor_vl53l5cx` is supported.
 - `instance_name`: Required. The name of this running service.
 - `subtopic`: Required. The device subtopic to publish to. For sensors this will always be prefixed with `sensors/`.
 - `interval_ms` For services that run repeatedly, this is the interval. Set to -1 by default, which will fire the service once. Set to 0 to fire as fast as possible. This option is ignored for async-based services.

#### sensor_beluga_serial
This is a [Beluga](https://github.com/WiseLabCMU/Beluga) UWB sensor connected via serial:

 - `dev_path`: Required. Path to the serial dev file device. For example, a Beluga USB serial device commonly shows up on Linux as `/dev/ttyACM[0-9]+`. Hardware serial ports are often `/dev/ttyS[0-9]+`. For USB devices, it is best practice to use the full serial device symlink like `/dev/serial/by-id/usb-SEGGER_J-Link_000760025484-if00` so that the connection order does not affect the device assignment. If multiple USB devices of the same type are connected, then the `/dev/serial/by-path/.*` syminks should be used to select the serial device by USB port.
 - `beluga_id`: Required. ID to assign to Beluga.
 - `baudrate`: Optional. Defaults to 115200.
 - `beluga_rate`: Optional. Defaults to 10. Rate in ms that Beluga will query and respond.
 - `beluga_channel`: Optional. Defaults to unset. See Beluga documentation.
 - `beluga_tx_power`: Optional. Defaults to unset. See Beluga documentation.
 - `beluga_timeout`: Optional. Defaults to unset. See Beluga documentation.
 - `beluga_twr_mode`: Optional. Defaults to unset. See Beluga documentation.
 - `beluga_led_mode`: Optional. Defaults to unset. See Beluga 

#### sensor_licosa_serial
This is a LiCosa IMU and lidar sensor connected via serial:

 - `dev_path`: Required. Path to the serial dev file device. For example, a LiCosa USB serial device commonly shows up on Linux as `/dev/ttyACM[0-9]+`. Hardware serial ports are often `/dev/ttyS[0-9]+`. For USB devices, it is best practice to use the full serial device symlink like `/dev/serial/by-id/usb-Raspberry_Pi_Pico_E4616C840F602C27-if00` so that the connection order does not affect the device assignment. If multiple USB devices of the same type are connected, then the `/dev/serial/by-path/.*` syminks should be used to select the serial device by USB port.
 - `baudrate`: Optional. Defaults to 230400.

#### sensor_vl53l5cx
This is a lidar sensor that connects through I2C and addresses by toggling its LPn pins. It additionally supports the following arguments:

 - `dev_path`: Required. Path to the I2C dev file device. For example, the first I2C port on the Raspberry Pi is `/dev/i2c-1`.
 - `gpio_path`: Required. Path to the GPIO dev file device. For example, the first I2C port on the Raspberry Pi is `/dev/gpiochip0`.
 - `rst_pin`: Required. Pin (integer) on `gpio_path` connected to the `I2C_RST` pin on all of the sensors.
 - `lpn_pins`: Required. Array of pins (integers) on `gpio_path` connected to `Lpn` pins on each sensor. The array must be present, and at least one pin must be specified or the service won't initialized any sensors. Sensors will be initialized and addressed in te order if this array.

### arenavideocall

After installing the repo, you should now have the `arena-video-call-chrome` command. You can also run it with `python3 -m arena-video-call-chrome`. It uses the [standard arena-py scene environment variables](https://arena.conix.io/content/python/#running-from-the-command-line).

The first argument sets the surface that the video should be mapped to. For example, to map to an object called `avideobox`:
```bash
MQTTH=arenaxr.org NAMESPACE=namespace SCENE=scene arena-video-call-chrome avideobox
```

### licosa_py

The `licosa_py` module depends only on NumPy. It includes a packet parser but not the serial interface; you will need to provide this yourself. See [`examples/licosa_py_basic.py`](./examples/licosa_py_basic.py) for an example.

### vl53l5cx_py

The `vl53l5cx_py` module requires Linux I2C interface (`linux/i2c.h` and `linux/i2c-dev.h`). It will issue a `RuntimeError` on other platforms if it is attempted to initialize. 

C shared library
```bash
gcc -fPIC -shared -o vl53l5cx.so -Ivl53l5cx_py/include -Ivl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/platform -Ivl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/uld-driver/inc vl53l5cx_py/src/*.c vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/platform/*.c vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/uld-driver/src/*.c -l
```

## License

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
