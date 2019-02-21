#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#
# Revision $Id$

"""
Warning: do not use this library.  It is unstable and most of the routines
here have been superceded by other libraries (e.g. rospkg).  These
routines will likely be *deleted* in future releases.
"""

import os
import sys
import re

import roslib.packages
import roslib.stack_manifest

import rospkg

ROS_ROOT=rospkg.environment.ROS_ROOT
ROS_PACKAGE_PATH=rospkg.environment.ROS_PACKAGE_PATH

STACK_FILE = 'stack.xml'
ROS_STACK = 'ros'

class ROSStackException(Exception): pass
class InvalidROSStackException(ROSStackException): pass

def stack_of(pkg, env=None):
    """
    @param env: override environment variables
    @type  env: {str: str}
    @return: name of stack that pkg is in, or None if pkg is not part of a stack
    @rtype: str
    @raise roslib.packages.InvalidROSPkgException: if pkg cannot be located
    """
    if env is None:
        env = os.environ
    pkg_dir = roslib.packages.get_pkg_dir(pkg, ros_root=env[ROS_ROOT], ros_package_path=env.get(ROS_PACKAGE_PATH, None))
    d = pkg_dir
    while d and os.path.dirname(d) != d:
        stack_file = os.path.join(d, STACK_FILE)
        if os.path.exists(stack_file):
            #TODO: need to resolve issues regarding whether the
            #stack.xml or the directory defines the stack name
            return os.path.basename(d)
        d = os.path.dirname(d)
        
def get_stack_dir(stack, env=None):
    """
    Get the directory of a ROS stack. This will initialize an internal
    cache and return cached results if possible.
    
    This routine is not thread-safe to os.environ changes.
    
    @param env: override environment variables
    @type  env: {str: str}
    @param stack: name of ROS stack to locate on disk
    @type  stack: str
    @return: directory of stack.
    @rtype: str
    @raise InvalidROSStackException: if stack cannot be located.
    """
    _init_rosstack(env=env)
    try:
        return _rosstack.get_path(stack)
    except rospkg.ResourceNotFound:
        # preserve old signature
        raise InvalidROSStackException(stack)

_rosstack = None
_ros_paths = None

def _init_rosstack(env=None):
    global _rosstack, _ros_paths
    if env is None:
        env = os.environ
    ros_paths = rospkg.get_ros_paths(env)
    if ros_paths != _ros_paths:
        _ros_paths = ros_paths
        _rosstack = rospkg.RosStack(ros_paths)
    
def list_stacks(env=None):
    """
    Get list of all ROS stacks. This uses an internal cache.

    This routine is not thread-safe to os.environ changes.

    @param env: override environment variables
    @type  env: {str: str}
    @return: complete list of stacks names in ROS environment
    @rtype: [str]
    """
    _init_rosstack(env=env)
    return _rosstack.list()

def list_stacks_by_path(path, stacks=None, cache=None):
    """
    List ROS stacks within the specified path.

    Optionally, a cache dictionary can be provided, which will be
    updated with the stack->path mappings. list_stacks_by_path() does
    NOT returned cached results -- it only updates the cache.
    
    @param path: path to list stacks in
    @type  path: str
    @param stacks: list of stacks to append to. If stack is
      already present in stacks, it will be ignored.
    @type  stacks: [str]
    @param cache: (optional) stack path cache to update. Maps stack name to directory path.
    @type  cache: {str: str}
    @return: complete list of stack names in ROS environment. Same as stacks parameter.
    @rtype: [str]
    """
    if stacks is None:
        stacks = []
    MANIFEST_FILE = rospkg.MANIFEST_FILE
    basename = os.path.basename
    for d, dirs, files in os.walk(path, topdown=True):
        if STACK_FILE in files:
            stack = basename(d)
            if stack not in stacks:
                stacks.append(stack)
                if cache is not None:
                    cache[stack] = d
            del dirs[:]
            continue #leaf
        elif MANIFEST_FILE in files:
            del dirs[:]
            continue #leaf     
        elif 'rospack_nosubdirs' in files:
            del dirs[:]
            continue  #leaf
        # remove hidden dirs (esp. .svn/.git)
        [dirs.remove(di) for di in dirs if di[0] == '.']
        for sub_d in dirs:
            # followlinks=True only available in Python 2.6, so we
            # have to implement manually
            sub_p = os.path.join(d, sub_d)
            if os.path.islink(sub_p):
                stacks.extend(list_stacks_by_path(sub_p, cache=cache))
    return stacks

# #2022
def expand_to_packages(names, env=None):
    """
    Expand names into a list of packages. Names can either be of packages or stacks.

    @param names: names of stacks or packages
    @type  names: [str]
    @return: ([packages], [not_found]). expand_packages() returns two
    lists. The first is of packages names. The second is a list of
    names for which no matching stack or package was found. Lists may have duplicates.
    @rtype: ([str], [str])
    """
    if env is None:
        env = os.environ
    ros_paths = rospkg.get_ros_paths(env)
    rospack = rospkg.RosPack(ros_paths)
    rosstack = rospkg.RosStack(ros_paths)
    return rospkg.expand_to_packages(names, rospack, rosstack)

def get_stack_version(stack, env=None):
    """
    @param env: override environment variables
    @type  env: {str: str}

    @return: version number of stack, or None if stack is unversioned.
    @rtype: str
    """
    _init_rosstack(env=env)
    return _rosstack.get_stack_version(stack)

def get_stack_version_by_dir(stack_dir):
    """
    Get stack version where stack_dir points to root directory of stack.
    
    @param env: override environment variables
    @type  env: {str: str}

    @return: version number of stack, or None if stack is unversioned.
    @rtype: str
    """
    # REP 109: check for <version> tag first, then CMakeLists.txt
    manifest_filename = os.path.join(stack_dir, STACK_FILE)
    if os.path.isfile(manifest_filename):
        m = roslib.stack_manifest.parse_file(manifest_filename)
        if m.version:
            return m.version
    
    cmake_filename = os.path.join(stack_dir, 'CMakeLists.txt')
    if os.path.isfile(cmake_filename):
        with open(cmake_filename) as f:
            return _get_cmake_version(f.read())
    else:
        return None

def _get_cmake_version(text):
    for l in text.split('\n'):
        if l.strip().startswith('rosbuild_make_distribution'):
            x_re = re.compile(r'[()]')
            lsplit = x_re.split(l.strip())
            if len(lsplit) < 2:
                raise ReleaseException("couldn't find version number in CMakeLists.txt:\n\n%s"%l)
            return lsplit[1]
