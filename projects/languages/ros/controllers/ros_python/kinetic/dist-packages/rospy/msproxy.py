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
Master/Slave XML-RPC Wrappers.

The L{MasterProxy} simplifies usage of master/slave
APIs by automatically inserting the caller ID and also adding python
dictionary accessors on the parameter server.
"""

from threading import Lock

import rospy.core
import rospy.exceptions
import rospy.names

import rospy.impl.paramserver
import rospy.impl.masterslave

_master_arg_remap = { 
    'deleteParam': [0], # remap key
    'setParam': [0], # remap key
    'getParam': [0], # remap key
    'searchParam': [0], # remap key
    'subscribeParam': [0], # remap key
    'unsubscribeParam': [0], # remap key
    'hasParam': [0], # remap key
    'registerService': [0], # remap service
    'lookupService': [0], # remap service
    'unregisterService': [0], # remap service
    'registerSubscriber': [0], # remap topic
    'unregisterSubscriber': [0], # remap topic    
    'registerPublisher': [0], # remap topic   
    'unregisterPublisher': [0], # remap topic   
    'lookupNode': [0], # remap node
    'getPublishedTopics': [0], # remap subgraph
    }
    
class MasterProxy(object):
    """
    Convenience wrapper for ROS master API and XML-RPC
    implementation. The Master API methods can be invoked on this
    object and will be forwarded appropriately. Names in arguments
    will be remapped according to current node settings. Provides
    dictionary-like access to parameter server, e.g.::
    
      master[key] = value

    All methods are thread-safe.
    """

    def __init__(self, uri):
        """
        Constructor for wrapping a remote master instance.
        @param uri: XML-RPC URI of master
        @type  uri: str
        """
        self.target = rospy.core.xmlrpcapi(uri)        
        self._lock = Lock()

    def __getattr__(self, key): #forward api calls to target
        if key in _master_arg_remap:
            remappings = _master_arg_remap[key]
        else:
            remappings = rospy.impl.masterslave.ROSHandler.remappings(key)
        def wrappedF(*args, **kwds):
            args = [rospy.names.get_caller_id(),]+list(args)
            #print "Remap indicies", remappings
            for i in remappings:
                i = i + 1 #callerId does not count
                #print "Remap %s => %s"%(args[i], rospy.names.resolve_name(args[i]))
                args[i] = rospy.names.resolve_name(args[i])
            with self._lock:
                f = getattr(self.target, key)
                return f(*args, **kwds)
        return wrappedF

    def __getitem__(self, key):
        """
        Fetch item from parameter server and subscribe to future updates so that
        values can be cached.
        @param key: parameter key
        @type key: str
        @raise KeyError: if key is not set
        """
        #NOTE: remapping occurs here!
        resolved_key = rospy.names.resolve_name(key)
        if 1: # disable param cache
            with self._lock:
                code, msg, value = self.target.getParam(rospy.names.get_caller_id(), resolved_key)
            if code != 1: #unwrap value with Python semantics
                raise KeyError(key)
            return value

        try:
            # check for value in the parameter server cache
            return rospy.impl.paramserver.get_param_server_cache().get(resolved_key)
        except KeyError:
            # first access, make call to parameter server
            with self._lock:
                code, msg, value = self.target.subscribeParam(rospy.names.get_caller_id(), rospy.core.get_node_uri(), resolved_key)
            if code != 1: #unwrap value with Python semantics
                raise KeyError(key)
            # set the value in the cache so that it's marked as subscribed
            rospy.impl.paramserver.get_param_server_cache().set(resolved_key, value)
            return value
        
    def __setitem__(self, key, val):
        """
        Set parameter value on Parameter Server
        @param key: parameter key
        @type key: str
        @param val: parameter value
        @type val: XMLRPC legal value
        """
        with self._lock:
            self.target.setParam(rospy.names.get_caller_id(), rospy.names.resolve_name(key), val)
        
    def search_param(self, key):
        """
        Search for a parameter matching key on the parameter server
        @return: found key or None if search did not succeed
        @rtype: str
        @raise ROSException: if parameter server reports an error
        """
        # #1810 searchParam has to use unresolved form of mappings
        mappings = rospy.names.get_mappings()
        if key in mappings:
            key = mappings[key]
        with self._lock:
            code, msg, val = self.target.searchParam(rospy.names.get_caller_id(), key)
        if code == 1:
            return val
        elif code == -1:
            return None
        else:
            raise rospy.exceptions.ROSException("cannot search for parameter: %s"%msg)
        
    def __delitem__(self, key):
        """
        Delete parameter key from the parameter server.
        @raise KeyError: if key is not set
        @raise ROSException: if parameter server reports an error
        """
        resolved_key = rospy.names.resolve_name(key)
        with self._lock:
            code, msg, _ = self.target.deleteParam(rospy.names.get_caller_id(), resolved_key)
        if code == -1:
            raise KeyError(key)
        elif code != 1:
            raise rospy.exceptions.ROSException("cannot delete parameter: %s"%msg)
        elif 0: #disable parameter cache
            # set the value in the cache so that it's marked as subscribed
            rospy.impl.paramserver.get_param_server_cache().delete(resolved_key)

    def __contains__(self, key):
        """
        Check if parameter is set on Parameter Server
        @param key: parameter key
        @type key: str
        @raise ROSException: if parameter server reports an error
        """        
        with self._lock:
            code, msg, value = self.target.hasParam(rospy.names.get_caller_id(), rospy.names.resolve_name(key))
        if code != 1:
            raise rospy.exceptions.ROSException("cannot check parameter on server: %s"%msg)
        return value
        
    def __iter__(self):
        """
        @raise ROSException: if parameter server reports an error
        """
        with self._lock:
            code, msg, value = self.target.getParamNames(rospy.names.get_caller_id())
        if code == 1:
            return value.__iter__()
        else:
            raise rospy.exceptions.ROSException("cannot retrieve parameter names: %s"%msg)
