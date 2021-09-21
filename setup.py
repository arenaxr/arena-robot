#!/usr/bin/env python
"""
setup.py: Package setup file.

Created by Perry Naseck on 6/28/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

import pathlib
from setuptools import setup, find_packages

here = pathlib.Path(__file__).parent.resolve()

long_description = (here / 'README.md').read_text(encoding='utf-8')

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
    platforms=["any"],
    url="https://github.com/conix-center/ARENA-robot",
    project_urls={
        'Bug Reports': 'https://github.com/conix-center/ARENA-robot/issues',
        'Source': 'https://github.com/conix-center/ARENA-robot',
    },
    classifiers=[
        'Development Status :: 1 - Planning',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3 :: Only',
        'Topic :: Scientific/Engineering',
        'Framework :: Robot Framework'
    ],
    keywords='arena robot ar vr robotics',
    install_requires=[
        # 'arena-py>=0.1.42,<1',
        'arena-py @ git+https://git@github.com/conix-center/arena-py@5baf5f5bc3a6d332192afc515488a9a96e6b761c#egg=arena-py',  # for jitsi host, until next release
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
