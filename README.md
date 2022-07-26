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
pip install git+https://github.com/arenaxr/arena-robot#egg=arena-robot
```

Or for development, clone the repo and from inside the directory:
```bash
pip install -e .
```

## Modules

### arenarobot

Services may be one of the following types:

 - `sensor`: Primarily takes in data from a hardware device and publishes the data to its own topic. These are usually sensors (ex. IMU, lidar, etc.).
 - `processor`: Primarily takes in data from another MQTT topic (typically a `sensor`, `controller`, or another `processor`) and publishes transformed data to its own topic. These are usually more computationally-heavy tasks that have been decouples from the sensor service or they are transformations that take in data from multiple sensors, processors, or controllers (ex. VIO, SLAM, navigation, etc.).
 - `controller`: Primarily takes in data on its own topic and publishes data on its own topic. Controllers usually interact with hardware devices that constantly take in and produce data (ex. a robot, drone, etc.)

Services that stream to MQTT may be started with the `arena-robot-service` command. See [`examples/service_sensor_vl53l5cx_lidar_1.json`](./examples/service_sensor_vl53l5cx_lidar_1.json) for a service file example. All services require the following arguments:

 - `service_type`: Required. See below for service types.
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

NOTE: This requires an extras package: `pip install arena-robot[realsense]`

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
This processor reads frames, detects Apriltags, and returns camera pose given a known set of Apriltag locations. All services require the following arguments:

- `video_file`: Required. Where frames should be read from. e.g. `/dev/video0` (v4l) or `http://localhost:8265/cam1` (mjpeg)
- `camera_resolution`: Required. 2-element list. [horizontal resolution, vertical resolution]
- `camera_params`: Required. Camera parameters fx, fy, cx, cy
- `dist_params`: Required. Distortion parameters k1, k2, p1, p2, k3
- `apriltag_locations`: Required. Dictionary with Apriltag IDs as keys and 2D-list tag poses as values. The 2D-list should be a 4x4 matrix. The uppermost left 3x3 matrix should represent the rotation matrix of the Apriltag. The rightmost column, from top to bottom, contains the x, y, and z coordinates of the Apriltag's translation. The bottom row should be [0, 0, 0, 1] to allow for transformation-related matrix operations.

The following parameters are for the [Python bindings for the Apriltags library](https://github.com/duckietown/lib-dt-apriltags):
    
- `apriltag_family`: Optional. Defaults to 'tag36h11'. See above documentation.
- `tag_size`: Optional. Defaults to 0.15. See above documentation.
- `num_detector_threads`: Optional. Defaults to 1. See above documentation.
- `quad_decimate`: Optional. Defaults to 2.0. See above documentation.
- `quad_sigma`: Optional. Defaults to 0.0. See above documentation.
- `refine_edges`: Optional. Defaults to 1. See above documentation.
- `decode_sharpening`: Optional. Defaults to 0.25. See above documentation.

NOTE: This requires an extras package: `pip install arena-robot[apriltag_detector]`

#### controller_pololu_slave_i2c
This controller allows for controlling and receiving data from a [Pololu slave device](https://github.com/pololu/pololu-rpi-slave-arduino-library) over I2C. This service has been abstracted to work with any data configuration provided by the I2C driver on the Arduino side.

Flash the Pololu sketch to the A-Star or Romi. Examples for both boards may be found [here](https://github.com/pololu/pololu-rpi-slave-arduino-library/tree/master/examples), but you should not download these directly. So, prepare the hardware like so:

1. You will need to install either the [`AStar32U4` Arduino Library](https://github.com/pololu/a-star-32u4-arduino-library) or the [`Romi32U4` Arduino Library](https://github.com/pololu/romi-32u4-arduino-library) insalled in the Arduino IDE.
2. Install the [`PololuRPiSlave` Arduino Library](https://github.com/pololu/pololu-rpi-slave-arduino-library) and load the example for your board via the Arduino example menu.
3. Using `raspi-config`, enable I2C (if using a Raspberry Pi).
4. Add `dtparam=i2c_arm_baudrate=400000` to `/boot/config.txt` (if using a Raspberry Pi).
5. Reboot

The example service configuration found at [`examples/service_controller_pololu_slave_i2c_romi.json`](./examples/service_controller_pololu_slave_i2c_romi.json) matches the `Data` struct defined in the Romi example found [here](https://github.com/pololu/pololu-rpi-slave-arduino-library/blob/d002d901acdecd5c23955a3595150e00cff1fb09/examples/RomiRPiSlaveDemo/RomiRPiSlaveDemo.ino) (note: this is a permalink to a specific Git version of this file).

`interval_ms` should be set to `100` ms by default, matching the Pololu polling speed. Anything faster is not guaranteed to work. All services require the following arguments:

 - `dev_path`: Required. Path to the I2C dev file device. For example, the first I2C port on the Raspberry Pi is `/dev/i2c-1`.
- `slave_data_types`: Required. Dictionary of objects for each data type found in the Pololu `Data` struct in the order they are defined in the struct. Each top level key in this dictionary is the friendly name for the data type. Each object for these keys may contain:
  - `format`: Required. A single string character matching a [Python Struct Format Character](https://docs.python.org/3/library/struct.html#format-characters). Repeated characters can be accomplished with the `num_items` field. This also ensure that strings are handled properly.
  - `num_items`: Required. The number of items this data type has. Multiple items will be returned as a list in MQTT with this key. Must be at least `1`.
  - `write`: Required. Bool of whether or not this field may be written to.
  - `initial`: Required if `write` is `true`. For more fields, this is an array of initial values. Number of items in list should match `num_items`. If `format` is `s`, then this should be a string with the number of characters of `num_items` (note that strings like `\u0000` count as one character).

The controller receives commands on its own MQTT topic. It will only processor commands that are well-formed and targeted for the specific instance. Commands consist of a standard ARENA-robot message with a key called `pololu_device_commands` in `data`. Commands will be executed in the order listed. This key consists of an array of objects, each containing:

 - `name`: Required. The name of the field to be set. This matches the key for each object in `slave_data_types`.
 - `value`: Required. Either a list of items (for fields with `num_items` greater than `1`) or a single value to set. For string fields, this must always be a string (not a list).
 - `index`: Optional. An integer offset for setting values. This allows values in the middle at at the end of a field with `num_items` greater than `1` to be set without setting the earlier values.

For example, to turn on the red LED on an instance called `service_controller_pololu_slave_i2c_pololu_slave_i2c_1_romi` from the command line:
```bash
arenaxr_pub -t realm/d/myuser/mydevice/controllers/robot -m '"{\"target_device_instance_name\": \"service_controller_pololu_slave_i2c_pololu_slave_i2c_1_romi\", \"msg\": {\"data\": {\"pololu_device_commands\": [{\"name\": \"led_red\", \"value\": true}]}}}"'
```
```json
{
  "target_device_instance_name": "service_controller_pololu_slave_i2c_pololu_slave_i2c_1_romi",
  "msg": {
    "data": {
      "pololu_device_commands": [
        {"name": "led_red", "value": true}
      ]
    }
  }
}
```

To play a tune:
```bash
arenaxr_pub -t realm/d/myuser/mydevice/controllers/robot -m '"{\"target_device_instance_name\": \"service_controller_pololu_slave_i2c_pololu_slave_i2c_1_romi\", \"msg\": {\"data\": {\"pololu_device_commands\": [{\"name\": \"notes\", \"value\": \"l16ceg>c\"}, {\"name\": \"notes_play\", \"value\": true}]}}}"'
```
```json
{
  "target_device_instance_name": "service_controller_pololu_slave_i2c_pololu_slave_i2c_1_romi",
  "msg": {
    "data": {
      "pololu_device_commands": [
        {"name": "notes", "value": "l16ceg>c"},
        {"name": "notes_play", "value": true}
      ]
    }
  }
}
```

To set both motors (causing a spin):
```bash
arenaxr_pub -t realm/d/myuser/mydevice/controllers/robot -m '"{\"target_device_instance_name\": \"service_controller_pololu_slave_i2c_pololu_slave_i2c_1_romi\", \"msg\": {\"data\": {\"pololu_device_commands\": [{\"name\": \"motors\", \"value\": [-512, 512]}]}}}"'
```
```json
{
  "target_device_instance_name": "service_controller_pololu_slave_i2c_pololu_slave_i2c_1_romi",
  "msg": {
    "data": {
      "pololu_device_commands": [
        {"name": "motors", "value": [-512, 512]}
      ]
    }
  }
}
```

To set only the second motor (makes use of the `index` field):
```bash
arenaxr_pub -t realm/d/myuser/mydevice/controllers/robot -m '"{\"target_device_instance_name\": \"service_controller_pololu_slave_i2c_pololu_slave_i2c_1_romi\", \"msg\": {\"data\": {\"pololu_device_commands\": [{\"name\": \"motors\", \"value\": [60], \"index\": 1}]}}}"'
```
```json
{
  "target_device_instance_name": "service_controller_pololu_slave_i2c_pololu_slave_i2c_1_romi",
  "msg": {
    "data": {
      "pololu_device_commands": [
        {"name": "motors", "value": [60], "index": 1}
      ]
    }
  }
}
```

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
