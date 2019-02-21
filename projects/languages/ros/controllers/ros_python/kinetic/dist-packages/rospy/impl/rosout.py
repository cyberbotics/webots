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

"""Internal use: support for /rosout logging in rospy"""

import logging
import sys
import traceback

import rospy.names 

from rospy.core import get_caller_id
from rospy.exceptions import ROSException
from rospy.topics import Publisher, Subscriber
from rospy.rostime import Time

from rospy.impl.registration import get_topic_manager

#Log message for rosout
from rosgraph_msgs.msg import Log

_ROSOUT = '/rosout'
_rosout_pub = None

_rospy_to_logging_levels = {
    Log.DEBUG: logging.DEBUG,
    Log.INFO: logging.INFO,
    Log.WARN: logging.WARNING,
    Log.ERROR: logging.ERROR,
    Log.FATAL: logging.CRITICAL,
}

def init_rosout():
    logger = logging.getLogger("rospy.rosout")
    try:
        global _rosout_pub
        if _rosout_pub is None:
            logger.info("initializing %s core topic"%_ROSOUT)
            _rosout_pub = Publisher(_ROSOUT, Log, latch=True, queue_size=100)
            logger.info("connected to core topic %s"%_ROSOUT)
        return True
    except Exception as e:
        logger.error("Unable to initialize %s: %s\n%s", _ROSOUT, e, traceback.format_exc())
        return False

_in_rosout = False
## log an error to the /rosout topic
def _rosout(level, msg, fname, line, func):
    global _in_rosout
    try:
        if _rosout_pub is not None:
            # protect against infinite recursion
            if not _in_rosout:
                try:
                    _in_rosout = True
                    msg = str(msg)
                    topics = get_topic_manager().get_topics()
                    l = Log(level=level, name=str(rospy.names.get_caller_id()), msg=str(msg), topics=topics, file=fname, line=line, function=func)
                    l.header.stamp = Time.now()
                    _rosout_pub.publish(l)
                finally:
                    _in_rosout = False
    except Exception as e:
        #traceback.print_exc()
        # don't use logerr in this case as that is recursive here
        logger = logging.getLogger("rospy.rosout")        
        logger.error("Unable to report rosout: %s\n%s", e, traceback.format_exc())
        return False

_logging_to_rospy_levels = {
      logging.DEBUG: Log.DEBUG,
      logging.INFO: Log.INFO,
      logging.WARNING: Log.WARN,
      logging.ERROR: Log.ERROR,
      logging.CRITICAL: Log.FATAL,
      }

class RosOutHandler(logging.Handler):
   def emit(self, record):
      _rosout(_logging_to_rospy_levels[record.levelno], record.getMessage(),
            record.filename, record.lineno, record.funcName)

## Load loggers for publishing to /rosout
## @param level int: Log level. Loggers >= level will be loaded.
def load_rosout_handlers(level):
    logger = logging.getLogger('rosout')
    logger.addHandler(RosOutHandler())
    if level != None:
        logger.setLevel(_rospy_to_logging_levels[level])
