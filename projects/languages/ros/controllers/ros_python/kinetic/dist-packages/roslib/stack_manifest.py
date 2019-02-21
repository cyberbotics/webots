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

STACK_FILE = 'stack.xml'

import roslib.manifestlib
# re-export symbols so that external code does not have to import manifestlib as well
from roslib.manifestlib import ManifestException, StackDepend

class StackManifest(roslib.manifestlib._Manifest):
    """
    Object representation of a ROS manifest file
    """
    __slots__ = []
    def __init__(self):
        """
        Create an empty stack manifest instance.
        """
        super(StackManifest, self).__init__('stack')
        
def _stack_file_by_dir(stack_dir, required=True):
    """
    @param stack_dir: path to stack directory
    @type  stack_dir: str
    @param required: require that the directory exist
    @type  required: bool
    @return: path to manifest file of stack
    @rtype: str
    @raise InvalidROSPkgException: if required is True and manifest file cannot be located
    """
    try:
        p = os.path.join(stack_dir, STACK_FILE)
        if not required and not os.path.exists(p):
            return p
        if not os.path.isfile(p):
            raise roslib.stacks.InvalidROSStackException("""
Stack '%(stack_dir)s' is improperly configured: no manifest file is present.
"""%locals())
        return p
    except roslib.stacks.InvalidROSStackException as e:
        if required:
            raise

def stack_file(stack, required=True):
    """
    @param stack: stack name
    @type  stack: str
    @param required: require that the directory exist
    @type  required: bool
    @return: path to manifest file of stack
    @rtype:  str
    @raise InvalidROSPkgException: if required is True and manifest file cannot be located
    """
    d = roslib.stacks.get_stack_dir(stack)
    return _stack_file_by_dir(d, required)
        
def parse_file(file):
    """
    Parse stack.xml file
    @param file: stack.xml file path
    @param file: str
    @return: StackManifest instance
    @rtype:  L{StackManifest}
    """
    return roslib.manifestlib.parse_file(StackManifest(), file)

def parse(string, filename='string'):
    """
    Parse stack.xml string contents
    @param string: stack.xml contents
    @type  string: str
    @return: StackManifest instance
    @rtype:  L{StackManifest}
    """
    s = roslib.manifestlib.parse(StackManifest(), string, filename)
    #TODO: validate
    return s
