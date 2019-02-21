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
# Revision $Id: xmlrpc.py 15336 2011-11-07 20:43:00Z kwc $

from __future__ import print_function

"""
Common XML-RPC for higher-level libraries running XML-RPC libraries in
ROS. In particular, this library provides common handling for URI
calculation based on ROS environment variables.

The common entry point for most libraries is the L{XmlRpcNode} class.
"""

import logging
import select
import socket

try:
    import _thread
except ImportError:
    import thread as _thread

import traceback

try:
    from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler #Python 3.x
except ImportError:
    from SimpleXMLRPCServer import SimpleXMLRPCServer #Python 2.x
    from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler #Python 2.x

try:
    import socketserver
except ImportError:
    import SocketServer as socketserver

import rosgraph.network

def isstring(s):
    """Small helper version to check an object is a string in a way that works
    for both Python 2 and 3
    """
    try:
        return isinstance(s, basestring)
    except NameError:
        return isinstance(s, str)

class SilenceableXMLRPCRequestHandler(SimpleXMLRPCRequestHandler):
    def log_message(self, format, *args):
        if 0:
            SimpleXMLRPCRequestHandler.log_message(self, format, *args)
    
class ThreadingXMLRPCServer(socketserver.ThreadingMixIn, SimpleXMLRPCServer):
    """
    Adds ThreadingMixin to SimpleXMLRPCServer to support multiple concurrent
    requests via threading. Also makes logging toggleable.
    """
    def __init__(self, addr, log_requests=1):
        """
        Overrides SimpleXMLRPCServer to set option to allow_reuse_address.
        """
        # allow_reuse_address defaults to False in Python 2.4.  We set it 
        # to True to allow quick restart on the same port.  This is equivalent 
        # to calling setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
        self.allow_reuse_address = True
        # Increase request_queue_size to handle issues with many simultaneous
        # connections in OSX 10.11
        self.request_queue_size = min(socket.SOMAXCONN, 128)
        if rosgraph.network.use_ipv6():
            logger = logging.getLogger('xmlrpc')
            # The XMLRPC library does not support IPv6 out of the box
            # We have to monipulate private members and duplicate
            # code from the constructor.
            # TODO IPV6: Get this into SimpleXMLRPCServer 
            SimpleXMLRPCServer.__init__(self, addr, SilenceableXMLRPCRequestHandler, log_requests,  bind_and_activate=False)
            self.address_family = socket.AF_INET6
            self.socket = socket.socket(self.address_family, self.socket_type)
            logger.info('binding ipv6 xmlrpc socket to' + str(addr))
            # TODO: set IPV6_V6ONLY to 0:
            # self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
            self.server_bind()
            self.server_activate()
            logger.info('bound to ' + str(self.socket.getsockname()[0:2]))
        else:
            SimpleXMLRPCServer.__init__(self, addr, SilenceableXMLRPCRequestHandler, log_requests)

    def handle_error(self, request, client_address):
        """
        override ThreadingMixin, which sends errors to stderr
        """
        if logging and traceback:
            logger = logging.getLogger('xmlrpc')
            if logger:
                logger.error(traceback.format_exc())
    
# class ForkingXMLRPCServer(socketserver.ForkingMixIn, SimpleXMLRPCServer):
#     """
#     Adds ThreadingMixin to SimpleXMLRPCServer to support multiple concurrent
#     requests via forking. Also makes logging toggleable.
#     """
#     def __init__(self, addr, request_handler=SilenceableXMLRPCRequestHandler, log_requests=1):
#         SimpleXMLRPCServer.__init__(self, addr, request_handler, log_requests)
    

class XmlRpcHandler(object):
    """
    Base handler API for handlers used with XmlRpcNode. Public methods will be 
    exported as XML RPC methods.
    """

    def _ready(self, uri):
        """
        callback into handler to inform it of XML-RPC URI
        """
        pass
    
    def _shutdown(self, reason):
        """
        callback into handler to inform it of shutdown
        """
        pass

class XmlRpcNode(object):
    """
    Generic XML-RPC node. Handles the additional complexity of binding
    an XML-RPC server to an arbitrary port. 
    XmlRpcNode is initialized when the uri field has a value.
    """

    def __init__(self, port=0, rpc_handler=None, on_run_error=None):
        """
        XML RPC Node constructor
        :param port: port to use for starting XML-RPC API. Set to 0 or omit to bind to any available port, ``int``
        :param rpc_handler: XML-RPC API handler for node, `XmlRpcHandler`
        :param on_run_error: function to invoke if server.run() throws
          Exception. Server always terminates if run() throws, but this
          enables cleanup routines to be invoked if server goes down, as
          well as include additional debugging. ``fn(Exception)``
        """
        super(XmlRpcNode, self).__init__()

        self.handler = rpc_handler
        self.uri = None # initialize the property now so it can be tested against, will be filled in later
        self.server = None
        if port and isstring(port):
            port = int(port)
        self.port = port
        self.is_shutdown = False
        self.on_run_error = on_run_error

    def shutdown(self, reason):
        """
        Terminate i/o connections for this server.

        :param reason: human-readable debug string, ``str``
        """
        self.is_shutdown = True
        if self.server:
            server = self.server
            handler = self.handler
            self.handler = self.server = self.port = self.uri = None
            if handler:
                handler._shutdown(reason)
            if server:
                server.socket.close()
                server.server_close()
                
    def start(self):
        """
        Initiate a thread to run the XML RPC server. Uses thread.start_new_thread.
        """
        _thread.start_new_thread(self.run, ())

    def set_uri(self, uri):
        """
        Sets the XML-RPC URI. Defined as a separate method as a hood
        for subclasses to bootstrap initialization. Should not be called externally.

        :param uri: XMLRPC URI, ``str``
        """
        self.uri = uri
        
    def run(self):
        try:
            self._run()
        except Exception as e:
            if self.is_shutdown:
                pass
            elif self.on_run_error is not None:
               self.on_run_error(e)
            else:
                raise

    # separated out for easier testing
    def _run_init(self):
        logger = logging.getLogger('xmlrpc')            
        try:
            log_requests = 0
            port = self.port or 0 #0 = any

            bind_address = rosgraph.network.get_bind_address()
            logger.info("XML-RPC server binding to %s:%d" % (bind_address, port))
            
            self.server = ThreadingXMLRPCServer((bind_address, port), log_requests)
            self.port = self.server.server_address[1] #set the port to whatever server bound to
            if not self.port:
                self.port = self.server.socket.getsockname()[1] #Python 2.4

            assert self.port, "Unable to retrieve local address binding"

            # #528: semi-complicated logic for determining XML-RPC URI
            # - if ROS_IP/ROS_HOSTNAME is set, use that address
            # - if the hostname returns a non-localhost value, use that
            # - use whatever rosgraph.network.get_local_address() returns
            uri = None
            override = rosgraph.network.get_address_override()
            if override:
                uri = 'http://%s:%s/'%(override, self.port)
            else:
                try:
                    hostname = socket.gethostname()
                    if hostname and not hostname == 'localhost' and not hostname.startswith('127.') and hostname != '::':
                        uri = 'http://%s:%s/'%(hostname, self.port)
                except:
                    pass
            if not uri:
                uri = 'http://%s:%s/'%(rosgraph.network.get_local_address(), self.port)
            self.set_uri(uri)
            
            logger.info("Started XML-RPC server [%s]", self.uri)

            self.server.register_multicall_functions()
            self.server.register_instance(self.handler)

        except socket.error as e:
            if e.errno == 98:
                msg = "ERROR: Unable to start XML-RPC server, port %s is already in use"%self.port
            else:
                msg = "ERROR: Unable to start XML-RPC server: %s" % e.strerror
            logger.error(msg)
            print(msg)
            raise #let higher level catch this

        if self.handler is not None:
            self.handler._ready(self.uri)
        logger.info("xml rpc node: starting XML-RPC server")
        
    def _run(self):
        """
        Main processing thread body.
        :raises: :exc:`socket.error` If server cannot bind
        
        """
        self._run_init()
        while not self.is_shutdown:
            try:
                self.server.serve_forever()
            except (IOError, select.error) as e:
                # check for interrupted call, which can occur if we're
                # embedded in a program using signals.  All other
                # exceptions break _run.
                if self.is_shutdown:
                    pass
                elif e.errno != 4:
                    self.is_shutdown = True
                    logging.getLogger('xmlrpc').error("serve forever IOError: %s, %s"%(e.errno, e.strerror))
                    
