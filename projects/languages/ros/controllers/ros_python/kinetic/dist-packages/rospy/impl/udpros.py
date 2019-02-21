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

from __future__ import print_function

"""
UDPROS connection protocol.
"""
## UDPROS connection protocol.
#  http://ros.org/wiki/ROS/UDPROS
# 

import rosgraph.network

import rospy.impl.registration
import rospy.impl.transport

def get_max_datagram_size():
    #TODO
    return 1024

class UDPROSHandler(rospy.transport.ProtocolHandler):
    """
    rospy protocol handler for UDPROS. Stores the datagram server if necessary.
    """
    
    def __init__(self, port=0):
        """
        ctor
        """
        self.port = port
        self.buff_size = get_max_datagram_size()
        
    def init_server(self):
        """
        Initialize and start the server thread, if not already initialized.
        """
        if self.server is not None:
            return
        if rosgraph.network.use_ipv6():
            s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
        else:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
        s.bind((rosgraph.network.get_bind_address(), self.port))
        if self.port == 0:
            self.port = s.getsockname()[1]
        self.server = s
        threading.start_new_thread(self.run, ())

    def run(self):
        buff_size = self.buff_size
        try:
            while not rospy.core.is_shutdown():
                data = self.server.recvfrom(self.buff_size)
                print("received packet")
                #TODO
        except:
            #TODO: log
            pass

    def shutdown(self):
        if self.sock is not None:
            self.sock.close()

    def create_transport(self, topic_name, pub_uri, protocol_params):
        """
        Connect to topic resolved_name on Publisher pub_uri using UDPROS.
        @param resolved_name str: resolved topic name
        @type  resolved_name: str
        @param pub_uri: XML-RPC URI of publisher 
        @type  pub_uri: str
        @param protocol_params: protocol parameters to use for connecting
        @type protocol_params: [XmlRpcLegal]
        @return: code, message, debug
        @rtype: (int, str, int)
        """

        #Validate protocol params = [UDPROS, address, port, headers]
        if type(protocol_params) != list or len(protocol_params) != 4:
            return 0, "ERROR: invalid UDPROS parameters", 0
        if protocol_params[0] != UDPROS:
            return 0, "INTERNAL ERROR: protocol id is not UDPROS: %s"%id, 0

        #TODO: get connection_id and buffer size from params
        id, dest_addr, dest_port, headers = protocol_params

        self.init_server()
        
        #TODO: parse/validate headers

        sub = rospy.registration.get_topic_manager().get_subscriber_impl(topic_name)
        # Create Transport
        
        # TODO: create just a single 'connection' instance to represent
        # all UDP connections. 'connection' can take care of unifying
        # publication if addresses are the same
        transport = UDPTransport(protocol, topic_name, sub.receive_callback) 
        
        # Attach connection to _SubscriberImpl
        if sub.add_connection(transport): #pass udp connection to handler
            return 1, "Connected topic[%s]. Transport impl[%s]"%(topic_name, transport.__class__.__name__), dest_port
        else:
            transport.close()
            return 0, "ERROR: Race condition failure: duplicate topic subscriber [%s] was created"%(topic_name), 0

    def supports(self, protocol):
        """
        @param protocol: name of protocol
        @type protocol: str
        @return: True if protocol is supported
        @rtype: bool
        """
        return protocol == UDPROS
    
    def get_supported(self):
        """
        Get supported protocols
        """
        return [[UDPROS]]
        
    def init_publisher(self, topic_name, protocol_params): 
        """
        Initialize this node to start publishing to a new UDP location.
        
        @param resolved_name: topic name
        @type  resolved__name: str
        
        @param protocol_params: requested protocol
          parameters. protocol[0] must be the string 'UDPROS'
        @type  protocol_params: [str, value*]
        @return: (code, msg, [UDPROS, addr, port])
        @rtype: (int, str, list)
        """

        if protocol_params[0] != UDPROS:
            return 0, "Internal error: protocol does not match UDPROS: %s"%protocol, []
        #TODO
        _, header, host, port, max_datagram_size = protocol_params
        #TODO: connection_id, max_datagraph_size
        return 1, "ready", [UDPROS]

    def topic_connection_handler(self, sock, client_addr, header):
        """
        Process incoming topic connection. Reads in topic name from
        handshake and creates the appropriate L{TCPROSPub} handler for the
        connection.
        @param sock: socket connection
        @type sock: socket.socket
        @param client_addr: client address
        @type client_addr: (str, int)
        @param header: key/value pairs from handshake header
        @type header: dict
        @return: error string or None 
        @rtype: str
        """
        for required in ['topic', 'md5sum', 'callerid']:
            if not required in header:
                return "Missing required '%s' field"%required
        else:
            resolved_topic_name = header['topic']
            md5sum = header['md5sum']
            tm = rospy.registration.get_topic_manager()
            topic = tm.get_publisher_impl(resolved_topic_name)
            if not topic:
                return "[%s] is not a publisher of  [%s]. Topics are %s"%(rospy.names.get_caller_id(), resolved_topic_name, tm.get_publications())
            elif md5sum != rospy.names.TOPIC_ANYTYPE and md5sum != topic.data_class._md5sum:

                actual_type = topic.data_class._type

                # check to see if subscriber sent 'type' header. If they did, check that
                # types are same first as this provides a better debugging message
                if 'type' in header:
                    requested_type = header['type']
                    if requested_type != actual_type:
                        return "topic types do not match: [%s] vs. [%s]"%(requested_type, actual_type)
                else:
                    # defaults to actual type
                    requested_type = actual_type

                return "Client [%s] wants topic [%s] to have datatype/md5sum [%s/%s], but our version has [%s/%s] Dropping connection."%(header['callerid'], resolved_topic_name, requested_type, md5sum, actual_type, topic.data_class._md5sum)

            else:
                #TODO:POLLING if polling header is present, have to spin up receive loop as well

                # #1334: tcp_nodelay support from subscriber option
                if 'tcp_nodelay' in header:
                    tcp_nodelay = True if header['tcp_nodelay'].strip() == '1' else False
                else:
                    tcp_nodelay = self.tcp_nodelay_map.get(resolved_topic_name, False)

                _configure_pub_socket(sock, tcp_nodelay)
                protocol = TCPROSPub(resolved_topic_name, topic.data_class, is_latch=topic.is_latch, headers=topic.headers)
                transport = TCPROSTransport(protocol, resolved_topic_name)
                transport.set_socket(sock, header['callerid'])
                transport.write_header()
                topic.add_connection(transport)
            
    

## UDPROS communication routines
class UDPROSTransport(rospy.transport.Transport):
    transport_type = 'UDPROS'
    
    def __init__(self, protocol, name, header):
        """
        ctor
        @param name: topic name    
        @type  name: str:
        @param protocol: protocol implementation    
        @param protocol: UDPROSTransportProtocol 
        @param header: handshake header if transport handshake header was
        already read off of transport.
        @type  header: dict
        @throws TransportInitError: if transport cannot be initialized according to arguments
        """
        super(UDPROSTransport, self).__init__(protocol.direction, name=name)
        if not name:
            raise TransportInitError("Unable to initialize transport: name is not set")

        self.done = False
        self.header = header
            
    def send_message(self, msg, seq):
        """
        Convenience routine for services to send a message across a
        particular connection. NOTE: write_data is much more efficient
        if same message is being sent to multiple connections. Not
        threadsafe.
        @param msg: message to send
        @type  msg: Msg
        @param seq: sequence number for message
        @type  seq: int
        @raise TransportException: if error occurred sending message
        """
        # this will call write_data(), so no need to keep track of stats
        serialize_message(self.write_buff, seq, msg)
        self.write_data(self.write_buff.getvalue())
        self.write_buff.truncate(0)

    def write_data(self, data):
        """
        Write raw data to transport
        @raise TransportInitialiationError: could not be initialized
        @raise TransportTerminated: no longer open for publishing
        """
        # TODO
        # - cut into packets
        # write to address
        pass
    
    def receive_once(self):
        """
        block until messages are read off of socket
        @return: list of newly received messages
        @rtype: [Msg]
        @raise TransportException: if unable to receive message due to error
        """
        pass

    ## Receive messages until shutdown
    ## @param self
    ## @param msgs_callback fn([msg]): callback to invoke for new messages received    
    def receive_loop(self, msgs_callback):
        pass
    
    ## close i/o and release resources
    def close(super):
        self(UDPROSTransport, self).close()
        #TODO
        self.done = True
    
_handler = UDPROSHandler()

def get_handler():
    return _handler
