"""
__init__.py: Package definition file.

Created by Perry Naseck on 6/28/21.

Copyright (c) 2021, The CONIX Research Center
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.
"""

try:
    from _version import version
except ImportError:
    try:
        from setuptools_scm import get_version
        version = get_version()  # pylint: disable=invalid-name
    except (ImportError, LookupError):
        version = 'dev'  # pylint: disable=invalid-name
__version__ = version
__author__ = "The CONIX Research Center"

__license__ = "BSD 3-Clause"
__maintainer__ = "WiSE Lab, Carnegie Mellon University"
