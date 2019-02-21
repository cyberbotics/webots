# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Open Source Robotics Foundation, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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
Common functions that can be used to mark spaces, e.g. build and devel, to indicate which tools previously built the space.

This allows the tools to detect cross tool talk and avoid it where appropriate
"""


from __future__ import print_function

import os

SPACE_BUILT_BY_MARKER_FILENAME = '.built_by'


def get_previous_tool_used_on_the_space(space_path):
    """
    Return the tool used to build the space at the given path, or None.

    Returns None if the path does not exist or if there is no built by file.

    :param str space_path: path to the space in question.
    :returns: str identifying the tool used to build the space or None.
    """
    if os.path.isdir(space_path):
        marker_path = os.path.join(space_path, SPACE_BUILT_BY_MARKER_FILENAME)
        if os.path.isfile(marker_path):
            with open(marker_path, 'r') as f:
                return f.read().strip()
    return None


def mark_space_as_built_by(space_path, tool_name):
    """
    Place a marker file in the space at the given path, telling who built it.

    The path to the marker is created if necessary.

    :param str space_path: path to the space which should be marked.
    :param str tool_name: name of the tool doing the marking.
    :raises: OSError, others, when trying to create the folder.
    """
    if not os.path.isdir(space_path):
        # Might fail if it's a file already or for permissions.
        os.makedirs(space_path)
    marker_path = os.path.join(space_path, SPACE_BUILT_BY_MARKER_FILENAME)
    with open(marker_path, 'w') as f:
        f.write(tool_name)
