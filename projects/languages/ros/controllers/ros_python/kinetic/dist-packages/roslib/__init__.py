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
roslib is largely deprecated starting in the ROS Fuerte release.

roslib has a very important role in all Python code written for ROS:
it contains the L{load_manifest()} method, which updates the
PYTHONPATH based on a set of ROS Package manifest.xml files. 

Beyond the important load_manifest() call, most of the rest of roslib
consists of low-level libraries that 99% of ROS users need not
interact with. These libraries are primarily to support higher-level
ROS Python libraries, such as the rospy client library, as well as
numerous ROS tools (e.g. rostopic).

"""

__version__ = '1.7.0'

from roslib.launcher import load_manifest

# this import is necessary due to a bug in purge_build.py in our
# debian assets.
import roslib.stacks

_is_interactive = False
def set_interactive(interactive):
    """
    General API for a script specifying that it is being run in an
    interactive environment. Many libraries may wish to change their
    behavior based on being interactive (e.g. disabling signal
    handlers on Ctrl-C).

    @param interactive: True if current script is being run in an interactive shell
    @type  interactive: bool
    """
    global _is_interactive
    _is_interactive = interactive

def is_interactive():
    """
    General API for a script specifying that it is being run in an
    interactive environment. Many libraries may wish to change their
    behavior based on being interactive (e.g. disabling signal
    handlers on Ctrl-C).

    @return: True if interactive flag has been set
    @rtype: bool
    """
    return _is_interactive

