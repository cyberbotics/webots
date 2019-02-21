# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Open Source Robotics Foundation, Inc. nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior
#    written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Checks metapackages for compliance with REP-0127.

Reference: http://ros.org/reps/rep-0127.html#metapackage
"""

from __future__ import print_function

import os

from catkin_pkg.cmake import configure_file
from catkin_pkg.cmake import get_metapackage_cmake_template_path

__author__ = 'William Woodall'
__email__ = 'william@osrfoundation.org'
__maintainer__ = 'William Woodall'

DEFINITION_URL = 'http://ros.org/reps/rep-0127.html#metapackage'


class InvalidMetapackage(Exception):

    def __init__(self, msg, path, package):
        self.path = path
        self.package = package
        Exception.__init__(self, "Metapackage '%s': %s" % (package.name, msg))


def get_expected_cmakelists_txt(metapackage_name):
    """
    Return the expected boilerplate CMakeLists.txt file for a metapackage.

    :param metapackage_name: name of the metapackage
    :type metapackage_name: str
    :returns: expected CMakeLists.txt file
    :rtype: str
    """
    env = {
        'name': metapackage_name,
        'metapackage_arguments': ''
    }
    return configure_file(get_metapackage_cmake_template_path(), env)


def has_cmakelists_txt(path):
    """
    Return True if the given path contains a CMakeLists.txt, otherwise False.

    :param path: path to folder potentially containing CMakeLists.txt
    :type path: str
    :returns: True if path contains CMakeLists.txt, else False
    :rtype: bool
    """
    cmakelists_txt_path = os.path.join(path, 'CMakeLists.txt')
    return os.path.isfile(cmakelists_txt_path)


def get_cmakelists_txt(path):
    """
    Fetch the CMakeLists.txt from a given path.

    :param path: path to the folder containing the CMakeLists.txt
    :type path: str
    :returns: contents of CMakeLists.txt file in given path
    :rtype: str
    :raises OSError: if there is no CMakeLists.txt in given path
    """
    cmakelists_txt_path = os.path.join(path, 'CMakeLists.txt')
    with open(cmakelists_txt_path, 'r') as f:
        return f.read()


def has_valid_cmakelists_txt(path, metapackage_name):
    """
    Return True if the given path contains a valid CMakeLists.txt, otherwise False.

    A valid CMakeLists.txt for a metapackage is defined by REP-0127

    :param path: path to folder containing CMakeLists.txt
    :type path: str
    :param metapackage_name: name of the metapackage being tested
    :type metapackage_name: str
    :returns: True if the path contains a valid CMakeLists.txt, else False
    :rtype: bool
    :raises OSError: if there is no CMakeLists.txt in given path
    """
    cmakelists_txt = get_cmakelists_txt(path)
    expected = get_expected_cmakelists_txt(metapackage_name)
    return cmakelists_txt == expected


def validate_metapackage(path, package):
    """
    Validate the given package (catkin_pkg.package.Package) as a metapackage.

    This validates the metapackage against the definition from REP-0127

    :param path: directory of the package being checked
    :type path: str
    :param package: package to be validated
    :type package: :py:class:`catkin_pkg.package.Package`
    :raises InvalidMetapackage: if package is not a valid metapackage
    :raises OSError: if there is not package.xml at the given path
    """
    # Is there actually a package at the given path, else raise
    # Cannot do package_exists_at from catkin_pkg.packages because of circular dep
    if not os.path.isdir(path) or not os.path.isfile(os.path.join(path, 'package.xml')):
        raise OSError("No package.xml found at path: '%s'" % path)
    # Is it a metapackage, else raise
    if not package.is_metapackage():
        raise InvalidMetapackage('No <metapackage/> tag in <export> section of package.xml', path, package)
    # Is there a CMakeLists.txt, else raise
    if not has_cmakelists_txt(path):
        raise InvalidMetapackage('No CMakeLists.txt', path, package)
    # Is the CMakeLists.txt correct, else raise
    if not has_valid_cmakelists_txt(path, package.name):
        raise InvalidMetapackage("""\
Invalid CMakeLists.txt
Expected:
<<<%s>>>
Got:
<<<%s>>>""" % (get_expected_cmakelists_txt(package.name), get_cmakelists_txt(path)), path, package
        )
    # Does it buildtool depend on catkin, else raise
    if not package.has_buildtool_depend_on_catkin():
        raise InvalidMetapackage('No buildtool dependency on catkin', path, package)
    # Does it have only run depends, else raise
    if package.has_invalid_metapackage_dependencies():
        raise InvalidMetapackage(
            'Has build, buildtool, and/or test depends, but only run depends are allowed (except buildtool catkin)',
            path, package)
