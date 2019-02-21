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
# $Author$

"""
Warning: do not use this library.  It is unstable and most of the routines
here have been superceded by other libraries (e.g. rospkg).  These
routines will likely be *deleted* in future releases.
"""

import sys
import os
import getopt

import roslib.packages

MANIFEST_FILE = 'manifest.xml'

import roslib.manifestlib
# re-export symbols for backwards compatibility
from roslib.manifestlib import ManifestException, Depend, Export, ROSDep, VersionControl

class Manifest(roslib.manifestlib._Manifest):
    """
    Object representation of a ROS manifest file
    """
    __slots__ = []
    def __init__(self):
        """
        Initialize new empty manifest.
        """
        super(Manifest, self).__init__('package')
        
    def get_export(self, tag, attr):
        """
        @return: exports that match the specified tag and attribute, e.g. 'python', 'path'
        @rtype: [L{Export}]
        """
        return [e.get(attr) for e in self.exports if e.tag == tag if e.get(attr) is not None]

def _manifest_file_by_dir(package_dir, required=True, env=None):
    """
    @param package_dir: path to package directory
    @type  package_dir: str
    @param env: environment dictionary
    @type  env: dict
    @param required: require that the directory exist
    @type  required: bool
    @return: path to manifest file of package
    @rtype:  str
    @raise InvalidROSPkgException: if required is True and manifest file cannot be located
    """
    if env is None:
        env = os.environ
    try:
        p = os.path.join(package_dir, MANIFEST_FILE)
        if not required and not os.path.exists(p):
            return p
        if not os.path.isfile(p):
            raise roslib.packages.InvalidROSPkgException("""
Package '%(package_dir)s' is improperly configured: no manifest file is present.
"""%locals())
        return p
    except roslib.packages.InvalidROSPkgException as e:
        if required:
            raise

def manifest_file(package, required=True, env=None):
    """
    @param package str: package name
    @type  package: str
    @param env: override os.environ dictionary
    @type  env: dict
    @param required: require that the directory exist
    @type  required: bool
    @return: path to manifest file of package
    @rtype: str
    @raise InvalidROSPkgException: if required is True and manifest file cannot be located
    """
    # ros_root needs to be determined from the environment or else
    # everything breaks when trying to launch nodes via ssh where the
    # path isn't setup correctly.
    if env is None:
        env = os.environ
    d = roslib.packages.get_pkg_dir(package, required, ros_root=env['ROS_ROOT']) 
    return _manifest_file_by_dir(d, required=required, env=env)

def load_manifest(package):
    """
    Load manifest for specified package.
    @param pacakge: package name
    @type  package: str
    @return: Manifest instance
    @rtype: L{Manifest}
    @raise InvalidROSPkgException: if package is unknown
    """
    return parse_file(manifest_file(package))
    
def parse_file(file):
    """
    Parse manifest.xml file
    @param file: manifest.xml file path
    @type  file: str
    @return: Manifest instance
    @rtype: L{Manifest}
    """
    return roslib.manifestlib.parse_file(Manifest(), file)

def parse(string, filename='string'):
    """
    Parse manifest.xml string contents
    @param string: manifest.xml contents
    @type  string: str
    @return: Manifest instance
    @rtype: L{Manifest}
    """
    v = roslib.manifestlib.parse(Manifest(), string, filename)
    if v.version:
        raise ManifestException("<version> tag is not valid in a package manifest.xml file")
    return v
