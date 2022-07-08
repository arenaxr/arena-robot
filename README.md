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

Services that stream to MQTT may be started with the `arena-robot-service` command. See [`examples/service_sensor_vl53l5cx_lidar_1.json`](./examples/service_sensor_vl53l5cx_lidar_1.json) for a service file example. All services require the following arguments:

 - `service_type`: Required. Currently only `sensor_vl53l5cx` is supported.
 - `instance_name`: Required. The name of this running service.
 - `subtopic`: Required. The device subtopic to publish to. For sensors, this will always be prefixed with `sensors/`. For processors, this will always be prefixed with `processors/`.
 - `interval_ms` For services that run repeatedly, this is the interval. Set to -1 by default, which will fire the service once. Set to 0 to fire as fast as possible. This option is ignored for async-based services.

A service can be run with the `arenaxr.org` instance of ARENA with:
```bash
MQTTH=mqtt.arenaxr.org DEVICE=mydevicename arena-robot-service my_service.json
```
A complete set of example services may be found in the [`examples`](./examples/) directory.

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
 - `beluga_led_mode`: Optional. Defaults to unset. See Beluga documentation.

#### sensor_licosa_serial
This is a LiCosa IMU and lidar sensor connected via serial:

 - `dev_path`: Required. Path to the serial dev file device. For example, a LiCosa USB serial device commonly shows up on Linux as `/dev/ttyACM[0-9]+`. Hardware serial ports are often `/dev/ttyS[0-9]+`. For USB devices, it is best practice to use the full serial device symlink like `/dev/serial/by-id/usb-Raspberry_Pi_Pico_E4616C840F602C27-if00` so that the connection order does not affect the device assignment. If multiple USB devices of the same type are connected, then the `/dev/serial/by-path/.*` syminks should be used to select the serial device by USB port.
 - `baudrate`: Optional. Defaults to 230400.

#### sensor_t265
This is an Intel T265:

 - `timeout_ms`: Optional. Defaults to 500. Timeout in milliseconds for waiting for pose frames from the sensor.
 - `enable_pose_jumping`: Optional. Defaults to False. Enable position jumping. See [librealsense docs](https://github.com/IntelRealSense/librealsense/blob/d19829788008b8e000870895a068f0c43d58895a/doc/t265.md#are-there-any-t265-specific-options).
 - `enable_relocalization`: Optional. Defaults to False. Enable appearance based relocalization. See [librealsense docs](https://github.com/IntelRealSense/librealsense/blob/d19829788008b8e000870895a068f0c43d58895a/doc/t265.md#are-there-any-t265-specific-options).
 - `enable_mapping`: Optional. Defaults to False. Enable an internal map. See [librealsense docs](https://github.com/IntelRealSense/librealsense/blob/d19829788008b8e000870895a068f0c43d58895a/doc/t265.md#are-there-any-t265-specific-options).
 - `enable_map_preservation`: Optional. Defaults to False. Preserve map from the previous run.
 - `enable_dynamic_calibration`: Optional. Defaults to True. Enable dynamic calibration.

#### processor_t265
This is an Intel T265 transformation processor. This service runs initially to set up callback listeners, so the `interval_ms` should always be `-1`. The available options are:

 - `sensor_t265_topic`: Required. Full path to the topic with the T265 data. Note that the processor will ignore sensor messages that are not from a `sensor_t265` instance.
 - `sensor_t265_instance_name`: Optional. Defaults to None. Name of the T265 instance to process messages from. This is useful if multiple T265 instances are publishing to a topic and you would like to ignore all but one of them.
 - `camera_orientation`: Optional. Defaults to 0. The orientation options are:
   - `0`: Forward, USB port to the right
   - `1`: Downfacing, USB port to the right
   - `2`: 45 degree forward
   - `3`: Upward facing, USB port to the right
 - `scale_factor`: Optional. Defaults to 1. Scale factor to apply to body transformations.
 - `jump_threshold_translation`: Optional. Defaults to 0.1. Position jump detection threshold in meters. This value should be relative to how frequent is the position data is obtained (200Hz for the T265).
 - `jump_threshold_velocity`: Optional. Defaults to 20. Velocity jump detection threshold in meters/second. This value should be relative to how frequent is the velocity data is obtained (200Hz for the T265).

#### sensor_vl53l5cx
This is a lidar sensor that connects through I2C and addresses by toggling its LPn pins. It additionally supports the following arguments:

 - `dev_path`: Required. Path to the I2C dev file device. For example, the first I2C port on the Raspberry Pi is `/dev/i2c-1`.
 - `gpio_path`: Required. Path to the GPIO dev file device. For example, the first I2C port on the Raspberry Pi is `/dev/gpiochip0`.
 - `rst_pin`: Required. Pin (integer) on `gpio_path` connected to the `I2C_RST` pin on all of the sensors.
 - `lpn_pins`: Required. Array of pins (integers) on `gpio_path` connected to `Lpn` pins on each sensor. The array must be present, and at least one pin must be specified or the service won't initialized any sensors. Sensors will be initialized and addressed in te order if this array.

#### processor_apriltag_detector
This processor reads frames, detects Apriltags, and returns camera pose given a known set of Apriltag locations.

    - `video_file`: Required. Where frames should be read from. e.g. `/dev/video*`
    - `camera_resolution`: Required. 2-element list. [horizontal resolution, vertical resolution]
    - `camera_params`: Required. Camera parameters fx, fy, cx, cy
    - `dist_params`: Required. Distortion parameters k1, k2, p1, p2, k3
    - `apriltag_locations`: Required. Dictionary with Apriltag IDs as keys and 2D-list tag poses as values. The 2D-list should be a 4x4 matrix. The uppermost left 3x3 matrix should represent the rotation matrix of the Apriltag. The rightmost column, from top to bottom, contains the x, y, and z coordinates of the Apriltag's translation. The bottom row should be [0, 0, 0, 1] to allow for transformation-related matrix operations.

The following parameters are for the [Python bindings for the Apriltags library](https://github.com/duckietown/lib-dt-apriltags)
    
    - `apriltag_family`: Optional. Defaults to 'tag36h11'. See above documentation.
    - `tag_size`: Optional. Defaults to 0.15. See above documentation.
    - `num_detector_threads`: Optional. Defaults to 1. See above documentation.
    - `quad_decimate`: Optional. Defaults to 2.0. See above documentation.
    - `quad_sigma`: Optional. Defaults to 0.0. See above documentation.
    - `refine_edges`: Optional. Defaults to 1. See above documentation.
    - `decode_sharpening`: Optional. Defaults to 0.25. See above documentation.

### arenavideocall

After installing the repo, you should now have the `arena-video-call-chrome` command. You can also run it with `python3 -m arena-video-call-chrome`. It uses the [standard arena-py scene environment variables](https://arena.conix.io/content/python/#running-from-the-command-line).

The first argument sets the surface that the video should be mapped to. For example, to map to an object called `avideobox`:
```bash
MQTTH=mqtt.arenaxr.org NAMESPACE=namespace SCENE=scene arena-video-call-chrome avideobox
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
