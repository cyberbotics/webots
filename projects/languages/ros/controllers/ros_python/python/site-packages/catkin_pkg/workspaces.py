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

"""Library to provided logic for chained workspaces."""

from __future__ import print_function

import os

CATKIN_WORKSPACE_MARKER_FILE = '.catkin_workspace'


def get_spaces(paths=None):
    """
    Return a list of spaces based on the CMAKE_PREFIX_PATH or passed in list of workspaces.

    It resolves the source space for each devel space and ignores non-catkin paths.
    :param paths_to_order: list of paths
    :param prefix_paths: list of prefixes, must not end with '/'
    """
    if paths is None:
        if 'CMAKE_PREFIX_PATH' not in os.environ:
            raise RuntimeError('Neither the environment variable CMAKE_PREFIX_PATH is set nor was a list of paths passed.')
        paths = os.environ['CMAKE_PREFIX_PATH'].split(os.pathsep) if os.environ['CMAKE_PREFIX_PATH'] else []

    spaces = []
    for path in paths:
        marker = os.path.join(path, '.catkin')
        # ignore non catkin paths
        if not os.path.exists(marker):
            continue
        spaces.append(path)

        # append source spaces
        with open(marker, 'r') as f:
            data = f.read()
            if data:
                spaces += data.split(';')
    return spaces


def order_paths(paths_to_order, prefix_paths):
    """
    Return a list containing all items of paths_to_order ordered by list of prefix_paths, compared as strings.

    :param paths_to_order: list of paths
    :param prefix_paths: list of prefixes, must not end with '/'
    """
    # the ordered paths contains a list for each prefix plus one more which contains paths which do not match one of the prefix_paths
    ordered_paths = [[] for _ in range(len(prefix_paths) + 1)]

    for path in paths_to_order:
        # put each directory into the slot where it matches the prefix, or last otherwise
        index = 0
        for prefix in prefix_paths:
            if path == prefix or path.startswith(prefix + os.sep) or (os.altsep and path.startswith(prefix + os.altsep)):
                break
            index += 1
        ordered_paths[index].append(path)

    # flatten list of lists
    return [j for i in ordered_paths for j in i]


def ensure_workspace_marker(base_path):
    """
    Create workspace marker file at path if not existing.

    :param path: target folder
    """
    if not os.path.exists(os.path.join(base_path, CATKIN_WORKSPACE_MARKER_FILE)):
        with open(os.path.join(base_path, CATKIN_WORKSPACE_MARKER_FILE), 'a') as fhand:
            fhand.write('# This file currently only serves to mark the location of a catkin workspace for tool integration\n')
