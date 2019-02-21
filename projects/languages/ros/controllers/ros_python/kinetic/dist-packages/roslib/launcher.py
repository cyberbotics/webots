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

"""
Python path loader for python scripts and applications. Paths are
derived from dependency structure declared in ROS manifest files.
"""

import os
import sys

import rospkg

# bootstrapped keeps track of which packages we've loaded so we don't
# update the path multiple times
_bootstrapped = []
# _rospack is our cache of ROS package data
_rospack = rospkg.RosPack()

def get_depends(package, rospack):
    vals = rospack.get_depends(package, implicit=True)
    return [v for v in vals if not rospack.get_manifest(v).is_catkin]

def load_manifest(package_name, bootstrap_version="0.7"):
    """
    Update the Python sys.path with package's dependencies

    :param package_name: name of the package that load_manifest() is being called from, ``str``
    """
    if package_name in _bootstrapped:
        return
    sys.path = _generate_python_path(package_name, _rospack) + sys.path
    
def _append_package_paths(manifest_, paths, pkg_dir):
    """
    Added paths for package to paths
    :param manifest_: package manifest, ``Manifest``
    :param pkg_dir: package's filesystem directory path, ``str``
    :param paths: list of paths, ``[str]``
    """
    exports = manifest_.get_export('python','path')
    if exports:
        for export in exports:
            if ':' in export:
                export = export.split(':')
            else:
                export = [export]
            for e in export:
                paths.append(e.replace('${prefix}', pkg_dir))
    else:
        dirs = [os.path.join(pkg_dir, d) for d in ['src', 'lib']]
        paths.extend([d for d in dirs if os.path.isdir(d)])
    
def _generate_python_path(pkg, rospack):
    """
    Recursive subroutine for building dependency list and python path
    :raises: :exc:`rospkg.ResourceNotFound` If an error occurs while attempting to load package or dependencies
    """
    if pkg in _bootstrapped:
        return []

    # short-circuit if this is a catkin-ized package
    m = rospack.get_manifest(pkg)
    if m.is_catkin:
        _bootstrapped.append(pkg)
        return []

    packages = get_depends(pkg, rospack) 
    packages.append(pkg)

    paths = []
    try:
        for p in packages:
            m = rospack.get_manifest(p)
            d = rospack.get_path(p)
            _append_package_paths(m, paths, d)
            _bootstrapped.append(p)
    except:
        if pkg in _bootstrapped:
            _bootstrapped.remove(pkg)
        raise
    return paths
