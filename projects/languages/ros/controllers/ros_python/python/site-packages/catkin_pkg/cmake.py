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

from __future__ import print_function

import os
import re


def get_metapackage_cmake_template_path():
    """
    Return the location of the metapackage CMakeLists.txt CMake template.

    :returns: ``str`` location of the metapackage CMakeLists.txt CMake template
    """
    rel_path = os.path.join('templates', 'metapackage.cmake.in')
    return os.path.join(os.path.dirname(__file__), rel_path)


def configure_file(template_file, environment):  # noqa: D402
    """
    Evaluate a .in template file used in CMake with configure_file().

    :param template_file: path to the template, ``str``
    :param environment: dictionary of placeholders to substitute,
      ``dict``
    :returns: string with evaluates template
    :raises: KeyError for placeholders in the template which are not
      in the environment
    """
    with open(template_file, 'r') as f:
        template = f.read()
        return configure_string(template, environment)


def configure_string(template, environment):
    """
    Substitute variables enclosed by @ characters.

    :param template: the template, ``str``
    :param environment: dictionary of placeholders to substitute,
      ``dict``
    :returns: string with evaluates template
    :raises: KeyError for placeholders in the template which are not
      in the environment
    """
    def substitute(match):
        var = match.group(0)[1:-1]
        return environment[var]
    return re.sub('\@[a-zA-Z0-9_]+\@', substitute, template)
