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

## Commands

### Video Call

After installing the repo, you should now have the `arena-robot-video-call-chrome` command. You can also run it with `python3 -m arena-robot-video-call-chrome`. It uses the [standard arena-py scene environment variables](https://arena.conix.io/content/python/#running-from-the-command-line).

The first argument sets the surface that the video should be mapped to. For example, to map to an object called `avideobox`:
```bash
MQTTH=arenaxr.org NAMESPACE=namespace SCENE=scene arena-robot-video-call-chrome avideobox
```

## License

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
