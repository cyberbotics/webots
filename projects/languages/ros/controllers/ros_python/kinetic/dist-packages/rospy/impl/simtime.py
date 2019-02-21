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

"""Internal-use: Support for simulated clock."""

import traceback

import rosgraph
from rosgraph_msgs.msg import Clock

import rospy.core
import rospy.rostime
import rospy.topics

# ROS clock topics and parameter config
_ROSCLOCK = '/clock'
_USE_SIMTIME = '/use_sim_time'

# Subscriber handles for /clock and /time
_rostime_sub = None
_rosclock_sub = None

def _is_use_simtime():
    # in order to prevent circular dependencies, this does not use the
    # builtin libraries for interacting with the parameter server, at least
    # until I reorganize the client vs. internal APIs better.
    master_uri = rosgraph.get_master_uri()
    m = rospy.core.xmlrpcapi(master_uri)
    code, msg, val = m.getParam(rospy.names.get_caller_id(), _USE_SIMTIME)
    if code == 1 and val:
        return True
    return False
    
from rospy.rostime import _set_rostime
def _set_rostime_clock_wrapper(time_msg):
    _set_rostime(time_msg.clock)
def _set_rostime_time_wrapper(time_msg):
    _set_rostime(time_msg.rostime)
    
def init_simtime():
    """
    Initialize the ROS time system by connecting to the /time topic
    IFF the /use_sim_time parameter is set.
    """    
    import logging
    logger = logging.getLogger("rospy.simtime")
    try:
        if not _is_use_simtime():
            logger.info("%s is not set, will not subscribe to simulated time [%s] topic"%(_USE_SIMTIME, _ROSCLOCK))
        else:
            global _rostime_sub, _clock_sub
            if _rostime_sub is None:
                logger.info("initializing %s core topic"%_ROSCLOCK)
                _clock_sub = rospy.topics.Subscriber(_ROSCLOCK, Clock, _set_rostime_clock_wrapper, queue_size=1)
                logger.info("connected to core topic %s"%_ROSCLOCK)

                _set_rostime(rospy.rostime.Time(0, 0))
        rospy.rostime.set_rostime_initialized(True)
        return True
    except Exception as e:
        logger.error("Unable to initialize %s: %s\n%s", _ROSCLOCK, e, traceback.format_exc())
        return False
