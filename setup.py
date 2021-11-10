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

here = pathlib.Path(__file__).parent.resolve()

long_description = (here / 'README.md').read_text(encoding='utf-8')

extensions = [
    Extension("vl53l5cx_py",
              (["./vl53l5cx_py/cython/vl53l5cx_py_wrapper.pyx"] +
               glob("./vl53l5cx_py/src/*.c") +
               glob("./vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/uld-driver/src/*.c") +
               glob("./vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/platform/*.c")),
              include_dirs=["./vl53l5cx_py/include",
                            "./vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/uld-driver/inc",
                            "./vl53l5cx_py/VL53L5CX_Linux_driver_1.1.2/user/platform",
                            numpy.get_include()]
    )
]

setup(
    name="arena-robot",
    description="Robot integration runtime for the ARENA",
    long_description=long_description,
    long_description_content_type='text/markdown',
    license='BSD-3-Clause',
    author="Conix Research Center",
    author_email="info@conix.io",
    python_requires='>=3.6, <4',
    packages=find_packages(),
    ext_modules=cythonize(extensions),
    zip_safe=False,
    platforms=["any"],
    url="https://github.com/conix-center/ARENA-robot",
    project_urls={
        'Bug Reports': 'https://github.com/conix-center/ARENA-robot/issues',
        'Source': 'https://github.com/conix-center/ARENA-robot',
    },
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Framework :: Robot Framework',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'Intended Audience :: Telecommunications Industry',
        'License :: OSI Approved :: BSD License',
        'Operating System :: POSIX :: Linux',
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
        'console_scripts': ['arena-video-call-chrome=arenavideocall.scripts.arena_video_call_chrome:main'],
    },
    install_requires=[
        'arena-py>=0.1.42,<1',
        'numpy>=1,<2',
        'selenium>=3,<4',
        'setuptools_scm>=6.0',
        'slam3d>=0,<1'
    ],
    extras_require={
        'dev': [
            'bandit>=1,<2',
            'flake8>=3,<4',
            'flake8-bugbear>=21,<22',
            'isort>=5,<6',
            'pydocstyle>=6,<7',
            'pylint>=2,<3'
        ]
    }
)
