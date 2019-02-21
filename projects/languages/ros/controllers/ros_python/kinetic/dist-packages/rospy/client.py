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
Additional ROS client API methods.
"""

import logging
import os
import socket
import struct
import sys
from threading import Lock
import time
import random
import yaml

import rosgraph
import rosgraph.names

import roslib

import rospy.core
from rospy.core import logwarn, loginfo, logerr, logdebug
import rospy.exceptions
import rospy.names
import rospy.rostime

import rospy.impl.init
import rospy.impl.rosout 
import rospy.impl.simtime

TIMEOUT_READY = 15.0 #seconds

# log level constants
from rosgraph_msgs.msg import Log
from roscpp.srv import GetLoggers, GetLoggersResponse, SetLoggerLevel, SetLoggerLevelResponse
from roscpp.msg import Logger
from rospy.impl.tcpros_service import Service
DEBUG = Log.DEBUG
INFO = Log.INFO
WARN = Log.WARN
ERROR = Log.ERROR
FATAL = Log.FATAL

def myargv(argv=None):
    """
    Remove ROS remapping arguments from sys.argv arguments.
    @return: copy of sys.argv with ROS remapping arguments removed
    @rtype: [str]
    """
    if argv is None:
        argv = sys.argv
    return [a for a in argv if not rosgraph.names.REMAP in a]

def load_command_line_node_params(argv):
    """
    Load node param mappings (aka private parameters) encoded in
    command-line arguments, e.g. _foo:=bar. See also rosgraph.names.load_mappings.
    @param argv: command-line arguments
    @param argv: [str]
    @return: param->value remappings. 
    @rtype: {str: val}
    @raises: ROSInitException
    """
    try:
        mappings = {}
        for arg in argv:
            if rosgraph.names.REMAP in arg:
                src, dst = [x.strip() for x in arg.split(rosgraph.names.REMAP)]
                if src and dst:
                    if len(src) > 1 and src[0] == '_' and src[1] != '_':
                        mappings[src[1:]] = yaml.load(dst)
        return mappings
    except Exception as e:
        raise rospy.exceptions.ROSInitException("invalid command-line parameters: %s"%(str(e)))

def on_shutdown(h):
    """
    Register function to be called on shutdown. This function will be
    called before Node begins teardown.
    @param h: Function with zero args to be called on shutdown.
    @type  h: fn()
    """
    rospy.core.add_client_shutdown_hook(h)
    
def spin():
    """
    Blocks until ROS node is shutdown. Yields activity to other threads.
    @raise ROSInitException: if node is not in a properly initialized state
    """
    
    if not rospy.core.is_initialized():
        raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
    logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())        
    try:
        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.5)
    except KeyboardInterrupt:
        logdebug("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')

_logging_level_names = {
      logging.DEBUG:    'DEBUG',
      logging.INFO:     'INFO',
      logging.WARNING:  'WARN',
      logging.ERROR:    'ERROR',
      logging.CRITICAL: 'FATAL',
      }

def _get_loggers(request):
    """
    ROS service handler to get the levels of all active loggers.
    """
    ret = GetLoggersResponse()
    for n in logging.Logger.manager.loggerDict.keys():
       level = logging.getLogger(n).getEffectiveLevel()
       level = _logging_level_names[level]
       ret.loggers.append(Logger(n, level))
    return ret
    
_names_to_logging_levels = {
      'DEBUG':    logging.DEBUG,
      'INFO':     logging.INFO,
      'WARN':     logging.WARNING,
      'ERROR':    logging.ERROR,
      'FATAL':    logging.CRITICAL,
      }

def _set_logger_level(request):
    """
    ROS service handler to set the logging level for a particular logger
    """
    level = request.level.upper()
    if level in _names_to_logging_levels:
        logger = logging.getLogger(request.logger)
        logger.setLevel(_names_to_logging_levels[level])
    else:
       logging.getLogger('rospy').error("Bad logging level: %s"%level)
    ret = SetLoggerLevelResponse()
    return ret

def _init_node_params(argv, node_name):
    """
    Uploads private params to the parameter server. Private params are specified
    via command-line remappings.

    @raises: ROSInitException
    """

    # #1027: load in param name mappings
    params = load_command_line_node_params(argv)
    for param_name, param_value in params.items():
        logdebug("setting param %s to %s"%(param_name, param_value))
        set_param(rosgraph.names.PRIV_NAME + param_name, param_value)

_init_node_args = None

def init_node(name, argv=None, anonymous=False, log_level=None, disable_rostime=False, disable_rosout=False, disable_signals=False, xmlrpc_port=0, tcpros_port=0):
    """
    Register client node with the master under the specified name.
    This MUST be called from the main Python thread unless
    disable_signals is set to True. Duplicate calls to init_node are
    only allowed if the arguments are identical as the side-effects of
    this method are not reversible.

    @param name: Node's name. This parameter must be a base name,
        meaning that it cannot contain namespaces (i.e. '/')
    @type  name: str
    
    @param argv: Command line arguments to this program, including
        remapping arguments (default: sys.argv). If you provide argv
        to init_node(), any previously created rospy data structure
        (Publisher, Subscriber, Service) will have invalid
        mappings. It is important that you call init_node() first if
        you wish to provide your own argv.
    @type  argv: [str]

    @param anonymous: if True, a name will be auto-generated for the
        node using name as the base.  This is useful when you wish to
        have multiple instances of the same node and don't care about
        their actual names (e.g. tools, guis). name will be used as
        the stem of the auto-generated name. NOTE: you cannot remap
        the name of an anonymous node.  
    @type anonymous: bool

    @param log_level: log level for sending message to /rosout and log
        file, which is INFO by default. For convenience, you may use
        rospy.DEBUG, rospy.INFO, rospy.ERROR, rospy.WARN, rospy.FATAL
    @type  log_level: int
    
    @param disable_signals: If True, rospy will not register its own
        signal handlers. You must set this flag if (a) you are unable
        to call init_node from the main thread and/or you are using
        rospy in an environment where you need to control your own
        signal handling (e.g. WX). If you set this to True, you should
        call rospy.signal_shutdown(reason) to initiate clean shutdown.

        NOTE: disable_signals is overridden to True if
        roslib.is_interactive() is True.
        
    @type  disable_signals: bool
    
    @param disable_rostime: for internal testing only: suppresses
        automatic subscription to rostime
    @type  disable_rostime: bool

    @param disable_rosout: for internal testing only: suppress
        auto-publication of rosout
    @type  disable_rostime: bool

    @param xmlrpc_port: If provided, it will use this port number for the client
        XMLRPC node. 
    @type  xmlrpc_port: int

    @param tcpros_port: If provided, the TCPROS server will listen for
        connections on this port
    @type  tcpros_port: int

    @raise ROSInitException: if initialization/registration fails
    @raise ValueError: if parameters are invalid (e.g. name contains a namespace or is otherwise illegal)
    """
    if argv is None:
        argv = sys.argv
    else:
        # reload the mapping table. Any previously created rospy data
        # structure does *not* reinitialize based on the new mappings.
        rospy.names.reload_mappings(argv)

    if not name:
        raise ValueError("name must not be empty")

    # this test can be eliminated once we change from warning to error in the next check
    if rosgraph.names.SEP in name:
        raise ValueError("namespaces are not allowed in node names")

    global _init_node_args

    # #972: allow duplicate init_node args if calls are identical
    # NOTE: we don't bother checking for node name aliases (e.g. 'foo' == '/foo').
    if _init_node_args:
        if _init_node_args != (name, argv, anonymous, log_level, disable_rostime, disable_signals):
            raise rospy.exceptions.ROSException("rospy.init_node() has already been called with different arguments: "+str(_init_node_args))
        else:
            return #already initialized

    # for scripting environments, we don't want to use the ROS signal
    # handlers
    disable_signals = disable_signals or roslib.is_interactive()
    _init_node_args = (name, argv, anonymous, log_level, disable_rostime, disable_signals)
        
    if not disable_signals:
        # NOTE: register_signals must be called from main thread
        rospy.core.register_signals() # add handlers for SIGINT/etc...
    else:
        logdebug("signal handlers for rospy disabled")

    # check for name override
    mappings = rospy.names.get_mappings()
    if '__name' in mappings:
        name = mappings['__name']
        if anonymous:
            logdebug("[%s] WARNING: due to __name setting, anonymous setting is being changed to false"%name)
            anonymous = False
        
    if anonymous:
        # not as good as a uuid/guid, but more readable. can't include
        # hostname as that is not guaranteed to be a legal ROS name
        name = "%s_%s_%s"%(name, os.getpid(), int(time.time()*1000))

    # check for legal base name once all changes have been made to the name
    if not rosgraph.names.is_legal_base_name(name):
        import warnings
        warnings.warn("'%s' is not a legal ROS base name. This may cause problems with other ROS tools."%name, stacklevel=2)

    # use rosgraph version of resolve_name to avoid remapping
    resolved_node_name = rosgraph.names.resolve_name(name, rospy.core.get_caller_id())
    rospy.core.configure_logging(resolved_node_name)
    # #1810
    rospy.names.initialize_mappings(resolved_node_name)
    
    logger = logging.getLogger("rospy.client")
    logger.info("init_node, name[%s], pid[%s]", resolved_node_name, os.getpid())
            
    # node initialization blocks until registration with master
    node = rospy.impl.init.start_node(os.environ, resolved_node_name, port=xmlrpc_port, tcpros_port=tcpros_port) 
    rospy.core.set_node_uri(node.uri)
    rospy.core.add_shutdown_hook(node.shutdown)    
    
    if rospy.core.is_shutdown():
        logger.warn("aborting node initialization as shutdown has been triggered")
        raise rospy.exceptions.ROSInitException("init_node interrupted before it could complete")

    # upload private params (set via command-line) to parameter server
    _init_node_params(argv, name)

    rospy.core.set_initialized(True)

    if not disable_rosout:
        rospy.impl.rosout.init_rosout()
        rospy.impl.rosout.load_rosout_handlers(log_level)

    if not disable_rostime:
        if not rospy.impl.simtime.init_simtime():
            raise rospy.exceptions.ROSInitException("Failed to initialize time. Please check logs for additional details")
    else:
        rospy.rostime.set_rostime_initialized(True)

    logdebug("init_node, name[%s], pid[%s]", resolved_node_name, os.getpid())    
    # advertise logging level services
    Service('~get_loggers', GetLoggers, _get_loggers)
    Service('~set_logger_level', SetLoggerLevel, _set_logger_level)


#_master_proxy is a MasterProxy wrapper
_master_proxy = None
_master_proxy_lock = Lock()

def get_master(env=os.environ):
    """
    Get a remote handle to the ROS Master.
    This method can be called independent of running a ROS node,
    though the ROS_MASTER_URI must be declared in the environment.

    @return: ROS Master remote object
    @rtype: L{rospy.MasterProxy}
    @raise Exception: if server cannot be located or system cannot be
    initialized
    """
    global _master_proxy, _master_proxy_lock
    if _master_proxy is None:
        with _master_proxy_lock:
            if _master_proxy is None:
                _master_proxy = rospy.msproxy.MasterProxy(
                    rosgraph.get_master_uri())
    return _master_proxy

#########################################################
# Topic helpers

def get_published_topics(namespace='/'):
    """
    Retrieve list of topics that the master is reporting as being published.

    @return: List of topic names and types: [[topic1, type1]...[topicN, typeN]]
    @rtype: [[str, str]]
    """
    code, msg, val = get_master().getPublishedTopics(namespace)
    if code != 1:
        raise rospy.exceptions.ROSException("unable to get published topics: %s"%msg)
    return val

class _WFM(object):
    def __init__(self):
        self.msg = None
    def cb(self, msg):
        if self.msg is None:
            self.msg = msg
            
def wait_for_message(topic, topic_type, timeout=None):
    """
    Receive one message from topic.
    
    This will create a new subscription to the topic, receive one message, then unsubscribe.

    @param topic: name of topic
    @type  topic: str
    @param topic_type: topic type
    @type  topic_type: L{rospy.Message} class
    @param timeout: timeout time in seconds
    @type  timeout: double
    @return: Message
    @rtype: L{rospy.Message}
    @raise ROSException: if specified timeout is exceeded
    @raise ROSInterruptException: if shutdown interrupts wait
    """
    wfm = _WFM()
    s = None
    try:
        s = rospy.topics.Subscriber(topic, topic_type, wfm.cb)
        if timeout is not None:
            timeout_t = time.time() + timeout
            while not rospy.core.is_shutdown() and wfm.msg is None:
                rospy.rostime.wallsleep(0.01)
                if time.time() >= timeout_t:
                    raise rospy.exceptions.ROSException("timeout exceeded while waiting for message on topic %s"%topic)

        else:
            while not rospy.core.is_shutdown() and wfm.msg is None:
                rospy.rostime.wallsleep(0.01)            
    finally:
        if s is not None:
            s.unregister()
    if rospy.core.is_shutdown():
        raise rospy.exceptions.ROSInterruptException("rospy shutdown")
    return wfm.msg


#########################################################
# Param Server Access

_param_server = None
_param_server_lock = Lock()
def _init_param_server():
    """
    Initialize parameter server singleton
    """
    global _param_server, _param_server_lock
    if _param_server is None:
        with _param_server_lock:
            if _param_server is None:
                _param_server = get_master()
    return _param_server_lock
        
# class and singleton to distinguish whether or not user has passed us a default value
class _Unspecified(object): pass
_unspecified = _Unspecified()

def get_param(param_name, default=_unspecified):
    """
    Retrieve a parameter from the param server

    NOTE: this method is thread-safe.
    
    @param default: (optional) default value to return if key is not set
    @type  default: any
    @return: parameter value
    @rtype: XmlRpcLegalValue
    @raise ROSException: if parameter server reports an error
    @raise KeyError: if value not set and default is not given
    """
    try:
        _init_param_server()
        return _param_server[param_name] #MasterProxy does all the magic for us
    except KeyError:
        if default != _unspecified:
            return default
        else:
            raise

def get_param_names():
    """
    Retrieve list of parameter names.

    NOTE: this method is thread-safe.
    
    @return: parameter names
    @rtype: [str]
    @raise ROSException: if parameter server reports an error
    """
    _init_param_server()
    code, msg, val = _param_server.getParamNames() #MasterProxy does all the magic for us
    if code != 1:
        raise rospy.exceptions.ROSException("Unable to retrieve parameter names: %s"%msg)
    else:
        return val

def set_param(param_name, param_value):
    """
    Set a parameter on the param server

    NOTE: this method is thread-safe.
    If param_value is a dictionary it will be treated as a parameter
    tree, where param_name is the namespace. For example:::
      {'x':1,'y':2,'sub':{'z':3}}
    will set param_name/x=1, param_name/y=2, and param_name/sub/z=3.
    Furthermore, it will replace all existing parameters in the
    param_name namespace with the parameters in param_value. You must
    set parameters individually if you wish to perform a union update.

    @param param_name: parameter name
    @type  param_name: str
    @param param_value: parameter value
    @type  param_value: XmlRpcLegalValue
    @raise ROSException: if parameter server reports an error
    """
    # #2202
    if not rosgraph.names.is_legal_name(param_name):
        import warnings
        warnings.warn("'%s' is not a legal ROS graph resource name. This may cause problems with other ROS tools"%param_name, stacklevel=2)

    _init_param_server()
    _param_server[param_name] = param_value #MasterProxy does all the magic for us

def search_param(param_name):
    """
    Search for a parameter on the param server

    NOTE: this method is thread-safe.
    
    @param param_name: parameter name
    @type  param_name: str
    @return: key of matching parameter or None if no matching parameter. 
    @rtype: str
    @raise ROSException: if parameter server reports an error
    """
    _init_param_server()
    return _param_server.search_param(param_name)
    
def delete_param(param_name):
    """
    Delete a parameter on the param server

    NOTE: this method is thread-safe.
    
    @param param_name: parameter name
    @type  param_name: str
    @raise KeyError: if parameter is not set    
    @raise ROSException: if parameter server reports an error
    """    
    _init_param_server()
    del _param_server[param_name] #MasterProxy does all the magic for us

def has_param(param_name):
    """
    Test if parameter exists on the param server

    NOTE: this method is thread-safe.
    
    @param param_name: parameter name
    @type  param_name: str
    @raise ROSException: if parameter server reports an error
    """
    _init_param_server()
    return param_name in _param_server #MasterProxy does all the magic for us
