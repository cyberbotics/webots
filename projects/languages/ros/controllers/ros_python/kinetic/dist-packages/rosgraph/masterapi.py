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
#
# Revision $Id: masterapi.py 9672 2010-05-11 21:57:40Z kwc $
"""
Python adapter for calling ROS Master API. While it is trivial to call the 
Master directly using XML-RPC, this API provides a safer abstraction in the event
the Master API is changed.
"""

try:
    from xmlrpc.client import ServerProxy  # Python 3.x
except ImportError:
    from xmlrpclib import ServerProxy  # Python 2.x

from . names import make_caller_id
from . rosenv import get_master_uri
from . network import parse_http_host_and_port

class MasterException(Exception):
    """
    Base class of ROS-master related errors.
    """
    pass

class MasterFailure(MasterException):
    """
    Call to Master failed. This generally indicates an internal error
    in the Master and that the Master may be in an inconsistent state.
    """
    pass

class MasterError(MasterException):
    """
    Master returned an error code, which indicates an error in the
    arguments passed to the Master.
    """
    pass

# backwards compat
ROSMasterException = MasterException
Error = MasterError
Failure = MasterFailure

def is_online(master_uri=None):
    """
    @param master_uri: (optional) override environment's ROS_MASTER_URI
    @type  master_uri: str
    @return: True if Master is available
    """
    return Master('rosgraph', master_uri=master_uri).is_online()

class Master(object):
    """
    API for interacting with the ROS master. Although the Master is
    relatively simple to interact with using the XMLRPC API, this
    abstraction layer provides protection against future updates. It
    also provides a streamlined API with builtin return code checking
    and caller_id passing.
    """
    
    def __init__(self, caller_id, master_uri=None):
        """
        :param caller_id: name of node to use in calls to master, ``str``
        :param master_uri: (optional) override default ROS master URI, ``str``
        :raises: :exc:`ValueError` If ROS master uri not set properly
        """

        if master_uri is None:
            master_uri = get_master_uri()
        self._reinit(master_uri)

        self.caller_id = make_caller_id(caller_id) #resolve
        if self.caller_id[-1] == '/':
            self.caller_id = self.caller_id[:-1]
        
    def _reinit(self, master_uri):
        """
        Internal API for reinitializing this handle to be a new master

        :raises: :exc:`ValueError` If ROS master uri not set
        """
        if master_uri is None:
            raise ValueError("ROS master URI is not set")
        # #1730 validate URL for better error messages
        try:
            parse_http_host_and_port(master_uri)
        except ValueError:
            raise ValueError("invalid master URI: %s"%(master_uri))

        self.master_uri = master_uri
        self.handle = ServerProxy(self.master_uri)
        
    def is_online(self):
        """
        Check if Master is online.

        NOTE: this is not part of the actual Master API. This is a convenience function.
        
        @param master_uri: (optional) override environment's ROS_MASTER_URI
        @type  master_uri: str
        @return: True if Master is available
        """
        try:
            self.getPid()
            return True
        except:
            return False

    def _succeed(self, args):
        """
        Check master return code and return the value field.
        
        @param args: master return value
        @type  args: (int, str, XMLRPCLegalValue)
        @return: value field of args (master return value)
        @rtype: XMLRPCLegalValue
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        code, msg, val = args
        if code == 1:
            return val
        elif code == -1:
            raise Error(msg)
        else:
            raise Failure(msg)            

    ################################################################################
    # PARAM SERVER

    def deleteParam(self, key):
        """
        Parameter Server: delete parameter
        @param key: parameter name
        @type  key: str
        @return: 0
        @rtype: int
        """
        return self._succeed(self.handle.deleteParam(self.caller_id, key))
        
    def setParam(self, key, value):
        """
        Parameter Server: set parameter.  NOTE: if value is a
        dictionary it will be treated as a parameter tree, where key
        is the parameter namespace. For example:::
          {'x':1,'y':2,'sub':{'z':3}}

        will set key/x=1, key/y=2, and key/sub/z=3. Furthermore, it
        will replace all existing parameters in the key parameter
        namespace with the parameters in value. You must set
        parameters individually if you wish to perform a union update.
        
        @param key: parameter name
        @type  key: str
        @param value: parameter value.
        @type  value: XMLRPCLegalValue
        @return: 0
        @rtype: int
        """
        return self._succeed(self.handle.setParam(self.caller_id, key, value))

    def getParam(self, key):
        """
        Retrieve parameter value from server.
        @param key: parameter to lookup. If key is a namespace,
        getParam() will return a parameter tree.
        @type  key: str
        getParam() will return a parameter tree.

        @return: parameterValue. If key is a namespace,
            the return value will be a dictionary, where each key is a
            parameter in that namespace. Sub-namespaces are also
            represented as dictionaries.
        @rtype: XMLRPCLegalValue
        """
        return self._succeed(self.handle.getParam(self.caller_id, key))

    def searchParam(self, key):
        """
        Search for parameter key on parameter server. Search starts in caller's namespace and proceeds
        upwards through parent namespaces until Parameter Server finds a matching key.

        searchParam's behavior is to search for the first partial match.
        For example, imagine that there are two 'robot_description' parameters::
          
           /robot_description
             /robot_description/arm
             /robot_description/base
           /pr2/robot_description
             /pr2/robot_description/base

        If I start in the namespace /pr2/foo and search for
        'robot_description', searchParam will match
        /pr2/robot_description. If I search for 'robot_description/arm'
        it will return /pr2/robot_description/arm, even though that
        parameter does not exist (yet).

        @param key: parameter key to search for.
        @type  key: str
        @return: foundKey
        @rtype: str
        """
        return self._succeed(self.handle.searchParam(self.caller_id, key))

    def subscribeParam(self, caller_api, key):
        """
        Retrieve parameter value from server and subscribe to updates to that param. See
        paramUpdate() in the Node API. 
        @param key: parameter to lookup.
        @type  key: str
        @param caller_api: API URI for paramUpdate callbacks.
        @type  caller_api: str
        @return: parameterValue. parameterValue is an empty dictionary if the parameter has not been set yet.
        @rtype: XMLRPCLegalValue
        """
        return self._succeed(self.handle.subscribeParam(self.caller_id, caller_api, key))

    def unsubscribeParam(self, caller_api, key):
        """
        Retrieve parameter value from server and subscribe to updates to that param. See
        paramUpdate() in the Node API. 
        @param key: parameter to lookup.
        @type  key: str
        @param caller_api: API URI for paramUpdate callbacks.
        @type  caller_api: str
        @return: numUnsubscribed. If numUnsubscribed is zero it means that the caller was not subscribed to the parameter.
        @rtype: int
        """        
        return self._succeed(self.handle.unsubscribeParam(self.caller_id, caller_api, key))

    def hasParam(self, key):
        """
        Check if parameter is stored on server. 
        @param key: parameter to check
        @type  key: str
        @return: [code, statusMessage, hasParam]
        @rtype: [int, str, bool]
        """
        return self._succeed(self.handle.hasParam(self.caller_id, key))

    def getParamNames(self):
        """
        Get list of all parameter names stored on this server.
        This does not adjust parameter names for caller's scope.
        
        @return: [code, statusMessage, parameterNameList]
        @rtype: [int, str, [str]]
        """
        return self._succeed(self.handle.getParamNames(self.caller_id))     
            
        
    ################################################################################
        
    def getPid(self):
        """
        Get the PID of this server
        @return: serverProcessPID
        @rtype: int
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.getPid(self.caller_id))

    def getUri(self):
        """
        Get the URI of this Master
        @return: masterUri
        @rtype: str
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.getUri(self.caller_id))
    
    def registerService(self, service, service_api, caller_api):
        """
        Register the caller as a provider of the specified service.
        @param service str: Fully-qualified name of service 
        @param service_api str: Service URI 
        @param caller_api str: XML-RPC URI of caller node 
        @return: ignore
        @rtype: int
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """        
        return self._succeed(self.handle.registerService(self.caller_id, service, service_api, caller_api))
    
    def lookupService(self, service):
        """
        Lookup all provider of a particular service.
        @param service: fully-qualified name of service to lookup.
        @type: service: str
        @return (int, str, str): (code, message, serviceUrl). service URL is provides
           and address and port of the service.  Fails if there is no provider.
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.lookupService(self.caller_id, service))
    

    def unregisterService(self, service, service_api):
        """
        Unregister the caller as a provider of the specified service.
        @param service: Fully-qualified name of service
        @type  service: str
        @param service_api: API URI of service to unregister. Unregistration will only occur if current
           registration matches.
        @type  service_api: str
        @return: (code, message, numUnregistered). Number of unregistrations (either 0 or 1).
           If this is zero it means that the caller was not registered as a service provider.
           The call still succeeds as the intended final state is reached.
        @rtype: (int, str, int)
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.unregisterService(self.caller_id, service, service_api))
    

    def registerSubscriber(self, topic, topic_type, caller_api):
        """
        Subscribe the caller to the specified topic. In addition to receiving
        a list of current publishers, the subscriber will also receive notifications
        of new publishers via the publisherUpdate API.        
        @param topic str: Fully-qualified name of topic to subscribe to. 
        @param topic_type: Datatype for topic. Must be a package-resource name, i.e. the .msg name.
        @type  topic_type: str
        @param caller_api: XML-RPC URI of caller node for new publisher notifications
        @type  caller_api: str
        @return: (code, message, publishers). Publishers is a list of XMLRPC API URIs
           for nodes currently publishing the specified topic.
        @rtype: (int, str, list(str))
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.registerSubscriber(self.caller_id, topic, topic_type, caller_api))
    

    def unregisterSubscriber(self, topic, caller_api):
        """
        Unregister the caller as a publisher of the topic.
        @param topic: Fully-qualified name of topic to unregister.
        @type  topic: str
        @param caller_api: API URI of service to unregister. Unregistration will only occur if current
        @type  caller_api: str
           registration matches.    
        @return: (code, statusMessage, numUnsubscribed). 
          If numUnsubscribed is zero it means that the caller was not registered as a subscriber.
          The call still succeeds as the intended final state is reached.
        @rtype: (int, str, int)
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.unregisterSubscriber(self.caller_id, topic, caller_api))
    
    def registerPublisher(self, topic, topic_type, caller_api):
        """
        Register the caller as a publisher the topic.
        @param topic: Fully-qualified name of topic to register.
        @type  topic: str
        @param topic_type: Datatype for topic. Must be a
        package-resource name, i.e. the .msg name.
        @type  topic_type: str
        @param caller_api str: ROS caller XML-RPC API URI
        @type  caller_api: str
        @return: subscriberApis.
        List of current subscribers of topic in the form of XMLRPC URIs.
        @rtype: [str]
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.registerPublisher(self.caller_id, topic, topic_type, caller_api))
    
    def unregisterPublisher(self, topic, caller_api):
        """
        Unregister the caller as a publisher of the topic.
        @param topic: Fully-qualified name of topic to unregister.
        @type  topic: str
        @param caller_api str: API URI of service to
           unregister. Unregistration will only occur if current
           registration matches.
        @type  caller_api: str
        @return: numUnregistered. 
           If numUnregistered is zero it means that the caller was not registered as a publisher.
           The call still succeeds as the intended final state is reached.
        @rtype: int
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """            
        return self._succeed(self.handle.unregisterPublisher(self.caller_id, topic, caller_api))        

    def lookupNode(self, node_name):
        """
        Get the XML-RPC URI of the node with the associated
        name/caller_id.  This API is for looking information about
        publishers and subscribers. Use lookupService instead to lookup
        ROS-RPC URIs.
        @param node: name of node to lookup
        @type  node: str
        @return: URI
        @rtype: str
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.lookupNode(self.caller_id, node_name))        
        
    def getPublishedTopics(self, subgraph):
        """
        Get list of topics that can be subscribed to. This does not return topics that have no publishers.
        See L{getSystemState()} to get more comprehensive list.
        @param subgraph: Restrict topic names to match within the specified subgraph. Subgraph namespace
           is resolved relative to the caller's namespace. Use '' to specify all names.
        @type  subgraph: str
        @return: [[topic1, type1]...[topicN, typeN]]
        @rtype: [[str, str],]
        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.getPublishedTopics(self.caller_id, subgraph))        
    
    def getTopicTypes(self): 
        """
        Retrieve list topic names and their types.

        New in ROS 1.2.

        @rtype: (int, str, [[str,str]] )
        @return: (code, statusMessage, topicTypes). topicTypes is a list of [topicName, topicType] pairs.
        """
        return self._succeed(self.handle.getTopicTypes(self.caller_id))
    
    def getSystemState(self): 
        """
        Retrieve list representation of system state (i.e. publishers, subscribers, and services).
        @rtype: [[str,[str]], [str,[str]], [str,[str]]]
        @return: systemState

           System state is in list representation::
             [publishers, subscribers, services].
        
           publishers is of the form::
             [ [topic1, [topic1Publisher1...topic1PublisherN]] ... ]
        
           subscribers is of the form::
             [ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]
        
           services is of the form::
             [ [service1, [service1Provider1...service1ProviderN]] ... ]

        @raise rosgraph.masterapi.Error: if Master returns ERROR.
        @raise rosgraph.masterapi.Failure: if Master returns FAILURE.
        """
        return self._succeed(self.handle.getSystemState(self.caller_id))
