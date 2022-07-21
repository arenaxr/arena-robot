#!/usr/bin/env python
"""
setup.py: Package setup file.

Created by Perry Naseck on 6/28/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

from glob import glob
from Cython.Build import cythonize
import pathlib
from setuptools import setup, Extension, find_packages
import numpy
from sys import platform

here = pathlib.Path(__file__).parent.resolve()

long_description = (here / 'README.md').read_text(encoding='utf-8')

extensions = []

# vl53l5cx_py only available on Linux
if platform.startswith('linux'):
    extensions.append(Extension("vl53l5cx_py.driver",
              (["./vl53l5cx_py/cython/vl53l5cx_py_wrapper.pyx"] +
               glob("./vl53l5cx_py/src/*.c") +
               glob("./vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/uld-driver/src/*.c") +
               glob("./vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/platform/*.c")),
              include_dirs=["./vl53l5cx_py/include",
                            "./vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/uld-driver/inc",
                            "./vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/platform",
                            numpy.get_include()]
    ))
else:
    extensions.append(Extension("vl53l5cx_py.driver",
              (["./vl53l5cx_py/cython/vl53l5cx_py_wrapper_disable.pyx"]),
              include_dirs=[]
    ))

setup(
    name="arena-robot",
    description="Robot integration runtime for the ARENA",
    long_description=long_description,
    long_description_content_type='text/markdown',
    license='BSD-3-Clause',
    author="Conix Research Center",
    author_email="info@conix.io",
    python_requires='>=3.8, <4',
    packages=find_packages(),
    ext_modules=cythonize(extensions, language_level="3"),
    zip_safe=False,
    platforms=["any"],
    url="https://github.com/arenaxr/arena-robot",
    project_urls={
        'Bug Reports': 'https://github.com/arenaxr/arena-robot/issues',
        'Source': 'https://github.com/arenaxr/arena-robot',
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Framework :: Robot Framework',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'Intended Audience :: Telecommunications Industry',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: C',
        'Programming Language :: Cython',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3 :: Only',
        'Topic :: Scientific/Engineering',
        'Topic :: System :: Distributed Computing',
        'Topic :: System :: Hardware'
    ],
    keywords='arena robot ar vr robotics drone lidar vl53l5cx stm selenium jitsi video videocall conference',
    entry_points = {
        'console_scripts': [
            'arena-video-call-chrome=arenavideocall.scripts.arena_video_call_chrome:main',
            'arena-robot-service=arenarobot.scripts.arena_robot_service:main'
        ],
    },
    install_requires=[
        'aioserial>=1,<2',
        'arena-py>=0.1.43,<1',
        'scipy>=1.8',
        'numpy>=1,<2',
        'pyserial>=3,<4',
        'python-periphery>=2,<3',
        'selenium>=3,<4',
        'setuptools_scm>=6.0',
        'slam3d>=0,<1',
        'transformations==2021.6.6'
    ],
    extras_require={
        'dev': [
            'bandit>=1,<2',
            'flake8>=3,<4',
            'flake8-bugbear>=21,<22',
            'isort>=5,<6',
            'pydocstyle>=6,<7',
            'pylint>=2,<3'
        ],
        'realsense': [
            'pyrealsense2>=2,<3'
        ],
        'apriltag_detector': [
            'dt-apriltags>=3,<4',
            # issue with pylint later
            # see https://github.com/opencv/opencv-python/issues/570#issuecomment-1151984301
            'opencv-python==4.5.5.64'
        ]
    }
)
