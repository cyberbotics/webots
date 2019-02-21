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
Base classes for rospy transports

These are the base underlying transport implementations, i.e.
TCP/IP connections, etc...

For topic implementations, see L{topics}
"""

import threading

# we need ids for the transports so we can send the IDs instead of
# full connection details
_transport_id = 0
_id_lock = threading.Lock()
def _nextId():
    global _transport_id
    try:
        _id_lock.acquire()
        _transport_id += 1
        return _transport_id
    finally:
        _id_lock.release()

INBOUND = 'i'
OUTBOUND = 'o'
BIDIRECTIONAL = 'b'

## Base API of Transport implementations
class Transport(object):
    transport_type = 'UNKNOWN'
    
    ## @param self
    ## @param direction str: INBOUND | OUTBOUND | BIDIRECTIONAL
    ## @param name str
    def __init__(self, direction, name='unnamed'):
        self.name       = name
        self.direction  = direction
        self.done       = False
        self.cleanup_cb = None
        self.endpoint_id = ''

        #STATS
        self.id           = _nextId() 
        self.stat_bytes   = 0
        # Number of messages that have passed through this transport
        self.stat_num_msg = 0         
    
        # Endpoint Details (IP, Port)
        self.local_endpoint = (0, 0)
        self.remote_endpoint = (0, 0)

    def fileno(self):
        """
        Get a file descriptor for select() if available
        """
        return None
    
    ## callback function to invoke when this connection is
    ## closed. Function will be passed this transport as an argument.
    ## @param self
    ## @param cleanup_callback fn(Transport): callback for when connection is closed
    def set_cleanup_callback(self, cleanup_callback):
        self.cleanup_cb = cleanup_callback

    ## terminate i/o. Leaf subclasses should override and call this implementation
    ## @param self
    def close(self):
        self.done = True
        if self.cleanup_cb:
            self.cleanup_cb(self)

    ## Write raw data to transport
    ## @throws TransportInitialiationError could not be initialized
    ## @throws TransportTerminated no longer open for publishing
    def write_data(self, data):
        raise Exception("not implemented")

    ## Implements the getTransportInfo() from roscpp
    ## Similar to getTransportInfo() in 'libros/transport/transport_tcp.cpp'
    def get_transport_info(self):
        raise NotImplementedError

## Shell class to hold stats about transport that is being killed off.
## This allows the information to stick around but the original Tranport to be gc'd
class DeadTransport(Transport):

    ## @param self
    ## @param transport str: transport name    
    def __init__(self, transport):
        super(DeadTransport, self).__init__(
            transport.direction, transport.name)
        self.transport_type = transport.transport_type #class property
        self.id           = transport.id
        self.stat_bytes   = transport.stat_bytes
        self.stat_num_msg = transport.stat_num_msg
        self.done         = True
        self.endpoint_id  = transport.endpoint_id
        self.local_endpoint = transport.local_endpoint
        self.remote_endpoint = transport.remote_endpoint

    ## @param self
    def get_transport_info(self):
        return "Closed %s connection on port %s to [%s:%s]" % (self.transport_type, self.local_endpoint[1], self.remote_endpoint[0], self.remote_endpoint[1])

## ProtocolHandler interface: implements topic communication for a
## particular protocol(s).  In order to understand the methods of this
## API, it is important to understand how topic communication is
## established:
##
## When a subscriber is notified of a new topic publisher, it contacts
## the publisher to establish a connection. The subscriber gathers a
## list of supported protocols (e.g. [['TCPROS'], ['MEMMAP']]) from
## its protocol handlers (L{get_supported}) and then passes these to
## the publisher.  Each of these protocols is actual a list,
## e.g. ['MPI', LaneWidth, BusSpeed], since a protocol may have
## associated parameters. This is considered the start of the
## 'negotiation phase'.
##        
##    subscriber -> pub.requestTopic(protocols)
##
## The Publisher selects a protocol from the lists and tells the
## appropriate protocol handler to prepare the outbound connection:
##                
##    pub.requestTopic() -> pub.protocol_handler.init_publisher(selected_protocol)
##
## The protocol handler will return a new set of parameters
## representing connection parameters, e.g. [TCPROS, address,
## port]. These new parameters are passed back to the subscriber,
## which tells its protocol handler to establish the connection.
##                
##    subscriber -> subscriber.protocol_handler.create_transport(protocolParams)               
class ProtocolHandler(object): #interface

    ## shutdown any resources associated with handling this protocol
    ## @param self
    def shutdown(self):
        pass

    ## Create a new Transport using the specified \a protocol_params
    ## returned from the Publisher \a pub_uri.
    ## @param self
    ## @param protocol_params [[str, val*]]: parameter list from Publisher. Actual
    ##   contents are protocol-specified.
    ## @param pub_uri str: publisher API URI
    ## @param topic str: topic name
    ## @return int, str, int: code, message, debug
    def create_transport(self, topic, pub_uri, protocol_params):
        raise Exception("interface impl")

    ## @param self
    ## @param protocol str: protocol name. Must match string identifier used in
    ##    negotiation.
    ## @return bool: True if this handler supports the specified protocol"""
    def supports(self, protocol):
        return False
    
    ## This method is called on subscribers and returns the protocol list
    ## @param self
    ## @return [[str, val*]]: list of supported protocol params. Each set of protocol params is a
    ##     list where the first element is the string identifier for the protocol.
    def get_supported(self):
        return []
        
    ## Prepare a transport based on one of the supported protocols
    ## declared by a Subscriber. Subscribers supply a list of
    ## supported protocols, of which one is selected by the Publisher
    ## and passed to init_publisher(). init_publisher is responsible
    ## for initializing the publisher based on the selection.
    ## @param self
    ## @param topic str: name of topic
    ## @param protocol: selected protocol parameters from the Subscriber.
    ## @return (int, str, list): (code, statusMessage, params). params
    ## is protocol specific. These params will be sent to the Subscriber
    ## so that it can create_transport().
    def init_publisher(self, topic, protocol): 
        raise Exception("interface impl")
    
