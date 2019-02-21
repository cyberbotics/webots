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

import os
import sys
import subprocess
import roslib.exceptions
import rospkg

if sys.hexversion > 0x03000000: #Python3
    python3 = True
else:
    python3 = False

import warnings
warnings.warn("roslib.rospack is deprecated, please use rospkg", stacklevel=2)

def rospackexec(args):
    """
    @return: result of executing rospack command (via subprocess). string will be strip()ed.
    @rtype: str
    @raise roslib.exceptions.ROSLibException: if rospack command fails
    """
    rospack_bin = 'rospack'
    if python3:
        val = subprocess.Popen([rospack_bin] + args, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0]
        val = val.decode().strip()
    else:
        val = (subprocess.Popen([rospack_bin] + args, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0] or '').strip()        
    if val.startswith('rospack:'): #rospack error message
        raise roslib.exceptions.ROSLibException(val)
    return val

def rospack_depends_on_1(pkg):
    """
    @param pkg: package name
    @type  pkg: str
    @return: A list of the names of the packages which depend directly on pkg
    @rtype: list
    """
    return rospackexec(['depends-on1', pkg]).split()

def rospack_depends_on(pkg):
    """
    @param pkg: package name
    @type  pkg: str
    @return: A list of the names of the packages which depend on pkg
    @rtype: list
    """
    return rospackexec(['depends-on', pkg]).split()

def rospack_depends_1(pkg):
    """
    @param pkg: package name
    @type  pkg: str
    @return: A list of the names of the packages which pkg directly depends on
    @rtype: list    
    """
    return rospackexec(['deps1', pkg]).split()

def rospack_depends(pkg):
    """
    @param pkg: package name
    @type  pkg: str
    @return: A list of the names of the packages which pkg depends on
    @rtype: list    
    """
    return rospackexec(['deps', pkg]).split()

def rospack_plugins(pkg):
    """
    @param pkg: package name
    @type  pkg: str
    @return: A list of the names of the packages which provide a plugin for pkg
    @rtype: list    
    """
    val = rospackexec(['plugins', '--attrib=plugin', pkg])
    if val:
      return [tuple(x.split(' ')) for x in val.split('\n')]
    else:
      return []

def rosstackexec(args):
    """
    @return: result of executing rosstack command (via subprocess). string will be strip()ed.
    @rtype:  str
    @raise roslib.exceptions.ROSLibException: if rosstack command fails
    """
    rosstack_bin = 'rosstack'
    if python3:
        val = subprocess.Popen([rosstack_bin] + args, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0]
        val = val.decode().strip()
    else:
        val = (subprocess.Popen([rosstack_bin] + args, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0] or '').strip()
    if val.startswith('rosstack:'): #rospack error message
        raise roslib.exceptions.ROSLibException(val)
    return val

def rosstack_depends_on(s):
    """
    @param s: stack name
    @type  s: str
    @return: A list of the names of the stacks which depend on s
    @rtype: list
    """
    return rosstackexec(['depends-on', s]).split()

def rosstack_depends_on_1(s):
    """
    @param s: stack name
    @type  s: str
    @return: A list of the names of the stacks which depend directly on s
    @rtype: list
    """
    return rosstackexec(['depends-on1', s]).split()

def rosstack_depends(s):
    """
    @param s: stack name
    @type  s: str
    @return: A list of the names of the stacks which s depends on 
    @rtype: list
    """
    return rosstackexec(['depends', s]).split()

def rosstack_depends_1(s):
    """
    @param s: stack name
    @type  s: str
    @return: A list of the names of the stacks which s depends on directly
    @rtype: list
    """
    return rosstackexec(['depends1', s]).split()
