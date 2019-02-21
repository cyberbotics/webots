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
Internal use: ROS Node (Slave) API. 

The Node API is implemented by the L{ROSHandler}.

API return convention: (statusCode, statusMessage, returnValue)

 - statusCode: an integer indicating the completion condition of the method. 
 - statusMessage: a human-readable string message for debugging
 - returnValue: the return value of the method; method-specific.

Current status codes: 

 - -1: ERROR: Error on the part of the caller, e.g. an invalid parameter
 - 0: FAILURE: Method was attempted but failed to complete correctly.
 - 1: SUCCESS: Method completed successfully.

Individual methods may assign additional meaning/semantics to statusCode.
"""

import os
import sys
import itertools
import logging
import socket
import threading
import traceback
import time
import errno

try:
    #py3k
    import urllib.parse as urlparse
except ImportError:
    import urlparse

from rosgraph.xmlrpc import XmlRpcHandler

import rospy.names
import rospy.rostime

import rospy.impl.tcpros

from rospy.core import *
from rospy.impl.paramserver import get_param_server_cache
from rospy.impl.registration import RegManager, get_topic_manager
from rospy.impl.validators import non_empty, ParameterInvalid

# Return code slots
STATUS = 0
MSG = 1
VAL = 2

# pseudo-validators ###############################
# these validators actually return tuples instead of a function and it is up to a custom
# validator on the class itself to perform the validation
def is_publishers_list(paramName):
    return ('is_publishers_list', paramName)

_logger = logging.getLogger("rospy.impl.masterslave")

LOG_API = True

def apivalidate(error_return_value, validators=()):
    """
    ROS master/slave arg-checking decorator. Applies the specified
    validator to the corresponding argument and also remaps each
    argument to be the value returned by the validator.  Thus,
    arguments can be simultaneously validated and canonicalized prior
    to actual function call.
    @param error_return_value: API value to return if call unexpectedly fails
    @param validators: sequence of validators to apply to each
      arg. None means no validation for the parameter is required. As all
      api methods take caller_id as the first parameter, the validators
      start with the second param.
    @type  validators: sequence
    """
    def check_validates(f):
        assert len(validators) == f.__code__.co_argcount - 2, "%s failed arg check"%f #ignore self and caller_id
        def validated_f(*args, **kwds):
            if LOG_API:
                _logger.debug("%s%s", f.__name__, str(args[1:]))
                #print "%s%s"%(f.func_name, str(args[1:]))
            if len(args) == 1:
                _logger.error("%s invoked without caller_id paramter"%f.__name__)
                return -1, "missing required caller_id parameter", error_return_value
            elif len(args) != f.__code__.co_argcount:
                return -1, "Error: bad call arity", error_return_value

            instance = args[0]
            caller_id = args[1]
            if not isinstance(caller_id, str):
                _logger.error("%s: invalid caller_id param type", f.__name__)
                return -1, "caller_id must be a string", error_return_value
            
            newArgs = [instance, caller_id] #canonicalized args
            try:
                for (v, a) in zip(validators, args[2:]):
                    if v:
                        try:
                            #simultaneously validate + canonicalized args
                            if type(v) == list or type(v) == tuple:
                                newArgs.append(instance._custom_validate(v[0], v[1], a, caller_id))
                            else:
                                newArgs.append(v(a, caller_id)) 
                        except ParameterInvalid as e:
                            _logger.error("%s: invalid parameter: %s", f.__name__, str(e) or 'error')
                            return -1, str(e) or 'error', error_return_value
                    else:
                        newArgs.append(a)

                if LOG_API:
                    retval = f(*newArgs, **kwds)
                    _logger.debug("%s%s returns %s", f.__name__, args[1:], retval)
                    return retval
                else:
                    code, msg, val = f(*newArgs, **kwds)
                    if val is None:
                        return -1, "Internal error (None value returned)", error_return_value
                    return code, msg, val
            except TypeError as te: #most likely wrong arg number
                _logger.error(traceback.format_exc())
                return -1, "Error: invalid arguments: %s"%te, error_return_value
            except Exception as e: #internal failure
                _logger.error(traceback.format_exc())
                return 0, "Internal failure: %s"%e, error_return_value
        validated_f.__name__ = f.__name__
        validated_f.__doc__ = f.__doc__ #preserve doc
        return validated_f
    return check_validates


class ROSHandler(XmlRpcHandler):
    """
    Base handler for both slave and master nodes. API methods
    generally provide the capability for establishing point-to-point
    connections with other nodes.
    
    Instance methods are XML-RPC API methods, so care must be taken as
    to what is added here. 
    """
    
    def __init__(self, name, master_uri):
        """
        Base constructor for ROS nodes/masters
        @param name: ROS name of this node
        @type  name: str
        @param master_uri: URI of master node, or None if this node is the master
        @type  master_uri: str
        """
        super(ROSHandler, self).__init__()
        self.masterUri = master_uri
        self.name = name
        self.uri = None
        self.done = False

        # initialize protocol handlers. The master will not have any.
        self.protocol_handlers = []
        handler = rospy.impl.tcpros.get_tcpros_handler()
        if handler is not None:
            self.protocol_handlers.append(handler)
            
        self.reg_man = RegManager(self)

    ###############################################################################
    # INTERNAL 

    def _is_registered(self):
        """
        @return: True if slave API is registered with master.
        @rtype: bool
        """
        if self.reg_man is None:
            return False
        else:
            return self.reg_man.is_registered()
        

    def _ready(self, uri):
        """
        @param uri: XML-RPC URI
        @type  uri: str
        callback from ROSNode to inform handler of correct i/o information
        """
        _logger.info("_ready: %s", uri)
        self.uri = uri
        #connect up topics in separate thread
        if self.reg_man:
            t = threading.Thread(target=self.reg_man.start, args=(uri, self.masterUri))
            rospy.core._add_shutdown_thread(t)
            t.start()

    def _custom_validate(self, validation, param_name, param_value, caller_id):
        """
        Implements validation rules that require access to internal ROSHandler state.
        @param validation: name of validation rule to use
        @type  validation: str
        @param param_name: name of parameter being validated
        @type  param_name: str
        @param param_value str: value of parameter
        @type  param_value: str
        @param caller_id: value of caller_id parameter to API method
        @type  caller_id: str
        @raise ParameterInvalid: if the parameter does not meet validation
        @return: new value for parameter, after validation
        """
        if validation == 'is_publishers_list':
            if not type(param_value) == list:
                raise ParameterInvalid("ERROR: param [%s] must be a list"%param_name)
            for v in param_value:
                if not isinstance(v, str):
                    raise ParameterInvalid("ERROR: param [%s] must be a list of strings"%param_name)
                parsed = urlparse.urlparse(v)
                if not parsed[0] or not parsed[1]: #protocol and host
                    raise ParameterInvalid("ERROR: param [%s] does not contain valid URLs [%s]"%(param_name, v))
            return param_value
        else:
            raise ParameterInvalid("ERROR: param [%s] has an unknown validation type [%s]"%(param_name, validation))

    ## static map for tracking which arguments to a function should be remapped
    #  { methodName : [ arg indices ]
    _remap_table = { } 

    @classmethod
    def remappings(cls, methodName):
        """
        @internal
        @param cls: class to register remappings on
        @type  cls: Class: class to register remappings on    
        @return: parameters (by pos) that should be remapped because they are names
        @rtype: list
        """
        if methodName in cls._remap_table:
            return cls._remap_table[methodName]
        else:
            return []
    
    ###############################################################################
    # UNOFFICIAL/PYTHON-ONLY API

    @apivalidate('')
    ## (Python-Only API) Get the XML-RPC URI of this server
    ## @param self
    ## @param caller_id str: ROS caller id    
    ## @return [int, str, str]: [1, "", xmlRpcUri]
    def getUri(self, caller_id):
        return 1, "", self.uri

    @apivalidate('')
    ## (Python-Only API) Get the ROS node name of this server
    ## @param self
    ## @param caller_id str: ROS caller id    
    ## @return [int, str, str]: [1, "", ROS node name]
    def getName(self, caller_id):
        return 1, "", self.name


    ###############################################################################
    # EXTERNAL API

    @apivalidate([])
    def getBusStats(self, caller_id):
        """
        Retrieve transport/topic statistics
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return: [publishStats, subscribeStats, serviceStats]::
           publishStats: [[topicName, messageDataSent, pubConnectionData]...[topicNameN, messageDataSentN, pubConnectionDataN]]
               pubConnectionData: [connectionId, bytesSent, numSent, connected]* . 
           subscribeStats: [[topicName, subConnectionData]...[topicNameN, subConnectionDataN]]
               subConnectionData: [connectionId, bytesReceived, dropEstimate, connected]* . dropEstimate is -1 if no estimate. 
           serviceStats: not sure yet, probably akin to [numRequests, bytesReceived, bytesSent] 
        """
        pub_stats, sub_stats = get_topic_manager().get_pub_sub_stats()
        #TODO: serviceStats
        return 1, '', [pub_stats, sub_stats, []]

    @apivalidate([])
    def getBusInfo(self, caller_id):
        """
        Retrieve transport/topic connection information
        @param caller_id: ROS caller id    
        @type  caller_id: str
        """
        return 1, "bus info", get_topic_manager().get_pub_sub_info()
    
    @apivalidate('')
    def getMasterUri(self, caller_id):
        """
        Get the URI of the master node.
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return: [code, msg, masterUri]
        @rtype: [int, str, str]
        """
        if self.masterUri:
            return 1, self.masterUri, self.masterUri
        else:
            return 0, "master URI not set", ""

    def _shutdown(self, reason=''):
        """
        @param reason: human-readable debug string
        @type  reason: str
        """
        if not self.done:
            self.done = True
            if reason:
                _logger.info(reason)
            if self.protocol_handlers:
                for handler in self.protocol_handlers:
                    handler.shutdown()
                del self.protocol_handlers[:]
                self.protocol_handlers = None
            return True
        
    @apivalidate(0, (None, ))
    def shutdown(self, caller_id, msg=''):
        """
        Stop this server
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param msg: a message describing why the node is being shutdown.
        @type  msg: str
        @return: [code, msg, 0]
        @rtype: [int, str, int]
        """
        if msg:
            print("shutdown request: %s"%msg)
        else:
            print("shutdown requst")
        if self._shutdown('external shutdown request from [%s]: %s'%(caller_id, msg)):
            signal_shutdown('external shutdown request from [%s]: [%s]'%(caller_id, msg))
        return 1, "shutdown", 0

    @apivalidate(-1)
    def getPid(self, caller_id):
        """
        Get the PID of this server
        @param caller_id: ROS caller id
        @type  caller_id: str
        @return: [1, "", serverProcessPID]
        @rtype: [int, str, int]
        """
        return 1, "", os.getpid()

    ###############################################################################
    # PUB/SUB APIS

    @apivalidate([])
    def getSubscriptions(self, caller_id):
        """
        Retrieve a list of topics that this node subscribes to.
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return: list of topics this node subscribes to.
        @rtype: [int, str, [ [topic1, topicType1]...[topicN, topicTypeN]]]
        """
        return 1, "subscriptions", get_topic_manager().get_subscriptions()

    @apivalidate([])
    def getPublications(self, caller_id):
        """
        Retrieve a list of topics that this node publishes.
        @param caller_id: ROS caller id    
        @type  caller_id: str
        @return: list of topics published by this node.
        @rtype: [int, str, [ [topic1, topicType1]...[topicN, topicTypeN]]]
        """
        return 1, "publications", get_topic_manager().get_publications()
    
    def _connect_topic(self, topic, pub_uri): 
        """
        Connect subscriber to topic.
        @param topic: Topic name to connect.
        @type  topic: str
        @param pub_uri: API URI of topic publisher.
        @type  pub_uri: str
        @return: [code, msg, numConnects]. numConnects is the number
           of subscribers connected to the topic.
        @rtype: [int, str, int]
        """
        caller_id = rospy.names.get_caller_id()
        sub = get_topic_manager().get_subscriber_impl(topic)
        if not sub:
            return -1, "No subscriber for topic [%s]"%topic, 0
        elif sub.has_connection(pub_uri):
            return 1, "_connect_topic[%s]: subscriber already connected to publisher [%s]"%(topic, pub_uri), 0
        
        #Negotiate with source for connection
        # - collect supported protocols
        protocols = []
        for h in self.protocol_handlers: #currently only TCPROS
            protocols.extend(h.get_supported())
        if not protocols:
            return 0, "ERROR: no available protocol handlers", 0

        _logger.debug("connect[%s]: calling requestTopic(%s, %s, %s)", topic, caller_id, topic, str(protocols))
        # 1) have to preserve original (unresolved) params as this may
        #    go outside our graph
        # 2) xmlrpclib doesn't give us any way of affecting the
        #    timeout other than affecting the global timeout. We need
        #    to set a timeout to prevent infinite hangs. 60 seconds is
        #    a *very* long time. All of the rospy code right now sets
        #    individual socket timeouts, but this could potentially
        #    affect user code.
        socket.setdefaulttimeout(60.)
        success = False
        interval = 0.5  # seconds
        # while the ROS node is not shutdown try to get the topic information
        # and retry on connections problems after some wait
        # Abort the retry if the we get a Connection Refused since at that point
        # we know for sure the URI is invalid
        while not success and not is_shutdown():
            try:
                code, msg, result = \
                      xmlrpcapi(pub_uri).requestTopic(caller_id, topic, protocols)
                success = True
            except Exception as e:
                if getattr(e, 'errno', None) == errno.ECONNREFUSED:
                    code = -errno.ECONNREFUSED
                    msg = str(e)
                    break
                elif not is_shutdown():
                    _logger.debug("Retrying for %s" % topic)
                    if interval < 30.0:
                        # exponential backoff (maximum 32 seconds)
                        interval = interval * 2
                    time.sleep(interval)

        #Create the connection (if possible)
        if code <= 0:
            _logger.debug("connect[%s]: requestTopic did not succeed %s, %s", pub_uri, code, msg)
            return code, msg, 0
        elif not result or type(protocols) != list:
            return 0, "ERROR: publisher returned invalid protocol choice: %s"%(str(result)), 0
        _logger.debug("connect[%s]: requestTopic returned protocol list %s", topic, result)
        protocol = result[0]
        for h in self.protocol_handlers:
            if h.supports(protocol):
                return h.create_transport(topic, pub_uri, result)
        return 0, "ERROR: publisher returned unsupported protocol choice: %s"%result, 0

    @apivalidate(-1, (global_name('parameter_key'), None))
    def paramUpdate(self, caller_id, parameter_key, parameter_value):
        """
        Callback from master of current publisher list for specified topic.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param parameter_key str: parameter name, globally resolved
        @type  parameter_key: str
        @param parameter_value New parameter value
        @type  parameter_value: XMLRPC-legal value
        @return: [code, status, ignore]. If code is -1 ERROR, the node
        is not subscribed to parameter_key
        @rtype: [int, str, int]
        """
        try:
            get_param_server_cache().update(parameter_key, parameter_value)
            return 1, '', 0
        except KeyError:
            return -1, 'not subscribed', 0

    @apivalidate(-1, (is_topic('topic'), is_publishers_list('publishers')))
    def publisherUpdate(self, caller_id, topic, publishers):
        """
        Callback from master of current publisher list for specified topic.
        @param caller_id: ROS caller id
        @type  caller_id: str
        @param topic str: topic name
        @type  topic: str
        @param publishers: list of current publishers for topic in the form of XMLRPC URIs
        @type  publishers: [str]
        @return: [code, status, ignore]
        @rtype: [int, str, int]
        """
        if self.reg_man:
            for uri in publishers:
                self.reg_man.publisher_update(topic, publishers)
        return 1, "", 0
    
    _remap_table['requestTopic'] = [0] # remap topic 
    @apivalidate([], (is_topic('topic'), non_empty('protocols')))
    def requestTopic(self, caller_id, topic, protocols):
        """
        Publisher node API method called by a subscriber node.
   
        Request that source allocate a channel for communication. Subscriber provides
        a list of desired protocols for communication. Publisher returns the
        selected protocol along with any additional params required for
        establishing connection. For example, for a TCP/IP-based connection,
        the source node may return a port number of TCP/IP server. 
        @param caller_id str: ROS caller id    
        @type  caller_id: str
        @param topic: topic name
        @type  topic: str
        @param protocols: list of desired
         protocols for communication in order of preference. Each
         protocol is a list of the form [ProtocolName,
         ProtocolParam1, ProtocolParam2...N]
        @type  protocols: [[str, XmlRpcLegalValue*]]
        @return: [code, msg, protocolParams]. protocolParams may be an
        empty list if there are no compatible protocols.
        @rtype: [int, str, [str, XmlRpcLegalValue*]]
        """
        if not get_topic_manager().has_publication(topic):
            return -1, "Not a publisher of [%s]"%topic, []
        for protocol in protocols: #simple for now: select first implementation 
            protocol_id = protocol[0]
            for h in self.protocol_handlers:
                if h.supports(protocol_id):
                    _logger.debug("requestTopic[%s]: choosing protocol %s", topic, protocol_id)
                    return h.init_publisher(topic, protocol)
        return 0, "no supported protocol implementations", []

