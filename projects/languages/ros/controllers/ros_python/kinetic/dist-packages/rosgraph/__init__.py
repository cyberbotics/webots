# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

import sys

from . rosenv import get_master_uri, ROS_MASTER_URI, ROS_NAMESPACE, ROS_HOSTNAME, ROS_IP, ROS_IPV6
from . masterapi import Master, MasterFailure, MasterError, MasterException
from . masterapi import is_online as is_master_online

# bring in names submodule
from . import names

def myargv(argv=None):
    """
    Remove ROS remapping arguments from sys.argv arguments.
    
    :returns: copy of sys.argv with ROS remapping arguments removed, ``[str]``
    """
    if argv is None:
        argv = sys.argv
    return [a for a in argv if not names.REMAP in a]

__all__ = ['myargv',
        'get_master_uri', 'ROS_MASTER_URI', 'ROS_NAMESPACE', 'ROS_HOSTNAME', 'ROS_IP', 'ROS_IPV6',
        'Master', 'MasterFailure', 'MasterError', 'MasterException',
        'is_master_online']

