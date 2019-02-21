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
# Copyright (c) 2008, Willow Garage, Inc.
# Revision $Id$

"""
ROS client library for Python.
See U{http://ros.org/wiki/rospy}
@author: Ken Conley (kwc)
"""

# import symbols into rospy namespace
# NOTE: there are much better ways to configure python module
# dictionaries, but the rospy codebase isn't quite in shape for that
# yet

from std_msgs.msg import Header

from .client import spin, myargv, init_node, \
    get_published_topics, \
    wait_for_message, \
    get_master, \
    on_shutdown, \
    get_param, get_param_names, set_param, delete_param, has_param, search_param,\
    DEBUG, INFO, WARN, ERROR, FATAL
from .timer import sleep, Rate, Timer
from .core import is_shutdown, signal_shutdown, \
    get_node_uri, get_ros_root, \
    logdebug, logwarn, loginfo, logout, logerr, logfatal, \
    logdebug_throttle, logwarn_throttle, loginfo_throttle, logerr_throttle, logfatal_throttle, \
    parse_rosrpc_uri
from .exceptions import *
from .msg import AnyMsg
from .msproxy import MasterProxy
from .names import get_name, get_caller_id, get_namespace, resolve_name, remap_name
from .rostime import Time, Duration, get_rostime, get_time
from .service import ServiceException

# - use tcp ros implementation of services
from .impl.tcpros_service import Service, ServiceProxy, wait_for_service
from .topics import Message, SubscribeListener, Publisher, Subscriber

## \defgroup validators Validators
## \defgroup clientapi Client API

__all__ = [
    'Header',
    'spin',
    'myargv',
    'init_node',
    'get_master',
    'get_published_topics',
    'wait_for_service',
    'on_shutdown',
    'get_param',
    'get_param_names',
    'set_param',
    'delete_param',
    'has_param',
    'search_param',
    'sleep',
    'Rate',
    'DEBUG',
    'INFO',
    'WARN',
    'ERROR',
    'FATAL'
    'is_shutdown',
    'signal_shutdown',
    'get_node_uri',
    'get_ros_root',
    'logdebug',
    'logwarn', 'loginfo',
    'logout', 'logerr', 'logfatal',
    'logdebug_throttle',
    'logwarn_throttle', 'loginfo_throttle',
    'logerr_throttle', 'logfatal_throttle',
    'parse_rosrpc_uri',
    'MasterProxy',
    'NodeProxy',    
    'ROSException',
    'ROSSerializationException',
    'ROSInitException',
    'ROSInterruptException',
    'ROSInternalException',
    'TransportException',
    'TransportTerminated',
    'TransportInitError',
    'AnyMsg', 'Message',
    'get_name',
    'get_caller_id',
    'get_namespace',
    'resolve_name',
    'remap_name',
    'Time', 'Duration', 'get_rostime', 'get_time',
    'ServiceException', 
    'Service', 'ServiceProxy',
    'SubscribeListener', 'Publisher', 'Subscriber',
    ]
