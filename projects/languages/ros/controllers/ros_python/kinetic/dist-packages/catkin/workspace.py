# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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

from __future__ import print_function
import os

CATKIN_MARKER_FILE = '.catkin'


def get_workspaces():
    """
    Based on CMAKE_PREFIX_PATH return all catkin workspaces

    :param _environ: environment module to use, ``dict``
    """
    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    paths = [path for path in os.environ.get(env_name, '').split(os.pathsep) if path]
    # remove non-workspace paths
    workspaces = [path for path in paths if os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE))]
    return workspaces


def get_source_paths(workspace):
    """
    reads catkin workspace files and returns the list of all declared
    source paths

    :param workspace: path to catkin workspace folder, ``str``
    """
    # determine source spaces
    filename = os.path.join(workspace, CATKIN_MARKER_FILE)
    if not os.path.isfile(filename):
        raise ValueError('Not a catkin workspace: "%s", missing file %s' % (workspace, filename))
    with open(filename) as f:
        data = f.read()

    source_paths = []
    if data != '':
        real_source_paths = set([])
        for path in data.split(';'):
            real_path = os.path.realpath(path)
            if real_path not in real_source_paths:
                source_paths.append(path)
                real_source_paths.add(real_path)
    return source_paths
